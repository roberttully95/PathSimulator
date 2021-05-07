classdef Simulator < handle
    %SIMULATOR Simulator base class. All of the code that is common between different algorithms
    %should be written here. This includes plotting the paths, reading data from input files, etc.
    
    % Simulation
    properties
        speedup         % Determines the speedup factor of running the code in real-time.
        file            % Complete path to the .json file.
        triangles       % Array of triangles that form the map
        simData         % Simulation-level parameters / information
        vehicles        % Array of vehicles.
        dT              % The time difference between consecutive timesteps.
        t               % Holds the current time of the simulation
        plotMode        % Determines now the data will be plotted.
        handle          % The handle for plotting vehicle objects
        distances       % The distances between each vehicle (Lower Triangular Matrix)
        mapAxis         % The axis for plotting the map.
        dataAxis        % The map for plotting vehicle-level data.
        triMethod       % Defines the triangulation method in use for the simulation
        LOG_PATH        % Path to the current simulation's logging directory.
    end
    
    properties (Dependent)
        hasAxis         % Determines if there has been a valid axis argument passed to the simulator.
        path1           % The first bounding path.
        path2           % The second bounding path. 
        tEnd            % The latest time that a vehicle can be spawned.
        fSpawn          % The frequency at which vehicles are spawned into the map.
        seed            % The random number seed that allows for reproducability of simulations.
        velocity        % The nominal velocity of the vehicles in the simulation.
        vehicleRadius   % The radius of the vehicle in meters.
        omegaMax        % The max turn rate of the vehicle in rads / s.
        Kp              % Proportional heading control gain.
        Ki              % Integral heading control gain.
        Kd              % Derivative heading control gain.
        entryEdge       % The edge that the vehicles will enter from.
        exitEdge        % The edge that the vehicles will exit from.
        nVehicles       % The number of vehicles in the simulation
        nTriangles      % The number of triangles in the simulation.
        activeVehicles  % The list of active vehicle indices
        nActiveVehicles % The number of active vehicles in the map.
        avgClosestDist  % Returns the average closest distance between adjacent vehicles.
        avgDist         % Returns the average distance between all vehicles.
        finished        % Flag that contains the completion state of the simulation.
        collDist        % The collision distance between two vehicles
        corridorBounds  % Stores the bounding coordinates of the corridor
    end
    
    % Time Logging
    properties
        TIME_PATH               % Path to the current simulation's time spreadsheet.
        TIME                    % Records the simulation time
        NUM_ACTIVE_VEHICLES     % Records the number of active vehicles at each time step.
        AVERAGE_CLOSEST_DIST    % Records the average closest distance between adjacent vehicles
        AVERAGE_DIST            % Records the average distance between all vehicles
    end
    
    % Vehicle Logging
    properties 
        VEHICLE_PATH            % Path to the current simulation's vehicle (event) spreadsheet.
        END_CONDITION           % Stores the end condition for the vehicle (reached goal, collision etc.)
        END_TIME                % Stores the time at which the end condition was achieved.
        END_POSE                % Stores the vehicle pose at the time the end condition was achieved
        DURATION                % Stores the amount of time the vehicle was active for
        DIST_TRAVELLED          % Stores the distance travelled by the vehicle in the simulation
        COLLIDED_WITH           % Stores the index of the vehicle collided with (if appropriate)
    end
    
    % PUBLIC METHODS
    methods
        
        function this = Simulator(path, triMethod, plot)
            %INIT Initializer for the simulator class.
            
            % Parse input data
            this.triMethod = triMethod;
            this.plotMode = (nargin == 3) * plot;
            
            % Set file
            [~, filename, fileext] = fileparts(path);
            this.file = strcat(cd, '\files\', filename, fileext);
            
            % Create logging directorys
            this.LOG_PATH = strcat(cd, "\logs\",  filename);
            this.TIME_PATH = strcat(this.LOG_PATH, "\time.xls");
            this.VEHICLE_PATH = strcat(this.LOG_PATH, "\vehicles.xls");
            [~, ~, ~] = mkdir(this.LOG_PATH);
            
            % Read simulation data from input file.
            this.simData = readJson(this.file);

            % Initialize vehicle file parameters
            this.END_CONDITION = NaN(this.nVehicles, 1);
            this.END_TIME = NaN(this.nVehicles, 1);
            this.END_POSE = NaN(this.nVehicles, 3);
            this.DURATION = NaN(this.nVehicles, 1);
            this.DIST_TRAVELLED = NaN(this.nVehicles, 1);
            this.COLLIDED_WITH = NaN(this.nVehicles, 1);
            
            % Setup random number generator
            rng(this.seed, 'combRecursive');
            
            % Initialize distances
            this.distances = NaN(this.nVehicles, this.nVehicles);
            
            % Set params
            this.t = 0;
            this.speedup = 1;
            
            this.initPlot();
            
            % Define the triangulation
            this.triangulate();
            
            % Plot region
            this.initPlot();
            this.plotTriangles();
            
            % Initialize Vehicles
            this.initVehicles();
            
            % Start first vehicle
            this.placeVehicle(1);
        end
        
        function triangulate(this)
            %TRIANGULATE Creates a triangulation based on the provided triangulation enumeration
            %value
            
            switch this.triMethod
                case Triangulation.Closest
                    this.triangles = closestTriangulation(this.path1, this.path2);
                case Triangulation.ConstantTurnRadius
                    this.triangles = ConstTurnRadiusTriangulation(this.path1, this.path2, this.velocity, this.omegaMax);
            end
        end
        
        function initVehicles(this)
            %INITVEHICLES Randomly initializes vehicles along the entry edge of the region. Does not
            %initialize the heading of the vehicles, however. The initialized heading is determined
            %by the triangulation method in the derived method.
            
            % Get the controller
            controller = HeadingController(this.dT, this.Kp, this.Ki, this.Kd, this.omegaMax);
            
            % Create array of spawn times
            t0 = 0:(1/this.fSpawn):this.tEnd;
            
            % Determine number of vehicles to be created.
            n = length(t0);
            
            % Create vehicles array.
            this.vehicles = Vehicle.empty(0, n);
            for i = 1:n
                this.vehicles(i) = Vehicle(NaN, NaN, NaN, this.velocity, t0(i), this.vehicleRadius, this.omegaMax, controller);
            end
        end
        
        function placeVehicle(this, i)
            %PLACEVEHICLE Places a vehicle into the map in a deconflicted state.
            
            % Get the current vehicle positions
            pos = reshape([this.vehicles(:).pos], 2, this.nVehicles)';
            
            % Direction of entry edge.
            v1 = this.entryEdge(1, :);
            v2 = this.entryEdge(2, :);
                        
            % Init
            validLocation = false;
            while ~validLocation
                
                % Create random number
                r = rand;
                
                % Generate point
                pt = v1 + r * (v2 - v1);
                
                % Ensure it is not collided with walls
                if norm(pt - v1) < this.vehicleRadius || norm(pt - v2) < this.vehicleRadius
                    continue;
                end
                
                % Ensure it is not collided with existing vehicle
                dists = sqrt((pos(:, 1) - pt(1)).^2 + (pos(:, 2) - pt(2)).^2);
                if any(dists < this.collDist)
                    continue;
                end
                
                % At this point, it is valid
                validLocation = true;
            end
            
            % Assign
            this.vehicles(i).x = pt(1);
            this.vehicles(i).y = pt(2);
            this.vehicles(i).active = true;
            
            % Determine the vehicle's initial triangle index and heading
            for j = 1:this.nTriangles
                p = polyshape(this.triangles(j).x, this.triangles(j).y);
                if isinterior(p, pt(1), pt(2))
                    this.vehicles(i).triangleIndex = j;
                    this.vehicles(i).th = atan2(this.triangles(j).dir(2), this.triangles(j).dir(1));
                    break;
                end
                if j == this.nTriangles
                    error("Vehicle not located in any map triangle!")
                end
            end
            
            % NOTE: Cannot assign the vehicle heading until the
            % triangulation has been performed.
        end
        
        function initPlot(this)
            %INITPLOT Plots both bounding paths for the region along with edges joining the start
            %and end vertices of the paths.
            
            if this.hasAxis
                
                % Initialize plot map.
                fig = figure(1);
                this.mapAxis = axes(fig);

                % Setup axis
                cla(this.mapAxis);
                this.mapAxis.DataAspectRatio = [1, 1, 1];
                hold(this.mapAxis , 'on');

                % Plot the paths
                this.path1.plot(this.mapAxis, 'b');
                this.path2.plot(this.mapAxis, 'b');

                % Plot the entry and exit lines.
                plot(this.mapAxis, this.entryEdge(:, 1), this.entryEdge(:, 2), 'g', 'LineWidth', 1.5)
                plot(this.mapAxis, this.exitEdge(:, 1), this.exitEdge(:, 2), 'r', 'LineWidth', 1.5)

                % Label Axes
                xlabel(this.mapAxis, "x");
                ylabel(this.mapAxis, "y");
                title(this.mapAxis, "Map Region")
                
                % Set bounds
                xlim(this.corridorBounds(1:2))
                ylim(this.corridorBounds(3:4))
                
                % Set data axis
                if this.plotMode == 2
                    fig = figure(2);
                    this.dataAxis = axes(fig);
                end
                
            end
        end
        
        function plotTriangles(this)
            %PLOTTRIANGLES Plots the triangles on the map plot.
            
            if this.hasAxis
                for i = 1:size(this.triangles, 2)
                    % Plot triangle
                    tri = this.triangles(i);
                    tri.plot(this.mapAxis, 'g');

                    % Get centroid
                    v = tri.centroid;

                    % Get length 
                    len = (1/6) * tri.dirLength;

                    % Plot arrows
                    arrows(this.mapAxis, v(1), v(2), len, 90 - atan2(tri.dir(2), tri.dir(1)) * (180 / pi))
                end
            end
        end
        
        function propogate(this)
            % PROPOGATE Propogates the simulation by 'dT' seconds.
            
            % Propogate
            for j = 1:this.nActiveVehicles
                i = this.activeVehicles(j);
                
                % Get desired heading
                triangle = this.triangles(this.vehicles(i).triangleIndex);
                thDesired = atan2(triangle.dir(2), triangle.dir(1));
                
                % Propogate vehicle
                this.vehicles(i).propogate(this.dT, thDesired);
                
                % Update triangle
                if ~triangle.containsPt(this.vehicles(i).pos)
                    this.vehicles(i).triangleIndex = triangle.nextIndex;
                end
            end
            this.t = this.t + this.dT;
            this.pause();
            
            %%%%%%%%%%%%%
            % POSTCHECK %
            %%%%%%%%%%%%%
            this.postPropogationUpdate();
            
            % Plot vehicles in their new state
            this.plotVehicles();
        end
        
        function pause(this)
            % PAUSE Pauses the simulation for a specified time.
            if this.hasAxis
                pause(this.dT / this.speedup);
            end
        end
        
        function plotVehicles(this)
            %PLOTVEHICLES Plots the currently active vehicles in the map.
            % Get position of active vehicles.
            
            if this.hasAxis 
                % Get locations
                i = this.activeVehicles;
                x = [this.vehicles(i).x];
                y = [this.vehicles(i).y];
                r = [this.vehicles(i).r];
                
                % Plot data
                hold(this.mapAxis, 'on');
                delete(this.handle)
                
                % Plot the circle
                this.handle = viscircles([x', y'], r');
                
            end
        end
        
        function terminateVehicle(this, i, cond, coll)
            %TERMINATEVEHICLE Terminates a vehicle.
            
            % Inform the vehicles that they are finished
            this.vehicles(i).terminate(this.t);
            
            % Remove vehicles from the distance matrix
            this.distances(i, :) = NaN;
            this.distances(:, i) = NaN;
            
            % Log the vehicle info
            this.END_CONDITION(i) = cond;
            this.END_TIME(i) = this.vehicles(i).tEnd;
            this.END_POSE(i, :) = this.vehicles(i).pose;
            if this.vehicles(i).tEnd - this.vehicles(i).tInit < this.dT
                this.DURATION(i) = 0;
            else
                this.DURATION(i) = this.vehicles(i).tEnd - this.vehicles(i).tInit;
            end
            if cond == 1
                this.COLLIDED_WITH(i) = coll;
            else
                this.COLLIDED_WITH(i) = 0;
            end
            this.DIST_TRAVELLED(i) = this.vehicles(i).distTravelled;
        end
                
        function postPropogationUpdate(this)
            %POSTPROPOGATIONUPDATE After all vehicles have been propogated,
            %call this function to check for collisions that occurred
            %during the propogating step and to update the distance matrix.
            
            % Step 1: Terminate vehicles that reached goal during the last
            % time step.
            active = this.activeVehicles;
            iEnded = find(isnan([this.vehicles(active).triangleIndex]));
            for i = 1:size(iEnded, 2)
                this.terminateVehicle(active(iEnded(i)), 0);
            end
            
            % Step 2: Terminate vehicles that collided with the wall.
            active = this.activeVehicles;
            positions  = reshape([this.vehicles(active).pos], 2, this.nActiveVehicles)';
            for i = 1:size(positions, 1)
                dirEdge = [this.triangles(this.vehicles(active(i)).triangleIndex).directionEdge];
                [d, ~] = distToLineSegment(dirEdge, positions(i, :));
                if d < this.vehicles(active(i)).r
                    this.terminateVehicle(active(i), 2);
                end
            end
            
            % Step 3: Terminate vehicles that collided with each other and
            % update distance matrix
            this.updateDistances();
            
            % Step 4: Initialize vehicle before the next time step begins
            iInit = find(abs(this.t - [this.vehicles(:).tInit]) < 0.1 * this.dT);
            for i = 1:length(iInit)
                this.placeVehicle(iInit(i));
            end
            
            % Step 5: Log time data
            this.TIMELOG();
        end
        
        function updateDistances(this)
            %UPDATEDISTANCES Updates the distance matrix based on the current distance between all
            %vehicles.

            % Find indices of vehicles that are active
            indices = this.activeVehicles;
            n = size(indices, 2);
            
            % Create list
            iList = 1;
            list = NaN(n*n, 2);
            for i = 1:n
                ii = indices(i);
                for j = 1:n
                    jj = indices(j);
                    % Keep upper triangular
                    if ii <= jj 
                        continue;
                    end
                    list(iList, :) = [ii, jj];
                    iList = iList + 1;
                end
            end
            list(any(isnan(list), 2), :) = [];
            
            % Get dists
            if ~isempty(list)
                
                % Faster to extract out
                vehiclesTemp = this.vehicles;
                
                % Get locations
                src = [[vehiclesTemp(list(:, 1)).x]; [vehiclesTemp(list(:, 1)).y]]';
                dst = [[vehiclesTemp(list(:, 2)).x]; [vehiclesTemp(list(:, 2)).y]]';

                % Calculate dist
                dists = src - dst;
                dists = sqrt(dists(:, 1).^2 + dists(:, 2).^2);

                % Assign all items in parallel
                idx1 = sub2ind(size(this.distances), list(:,1), list(:,2));
                idx2 = sub2ind(size(this.distances), list(:,2), list(:,1)); 
                this.distances(idx1) = dists;
                this.distances(idx2) = dists;
                
                % Check if distances are less than the min distance (Ra + Rb)
                dMin = [vehiclesTemp(list(:, 1)).r]' + [vehiclesTemp(list(:, 2)).r]';
                collided = list(dists < dMin, :);
                
                % Process collision if they occur.
                for i = 1:size(collided, 1)
                    this.terminateVehicle(collided(i, 1), 1, collided(i, 2))
                    this.terminateVehicle(collided(i, 2), 1, collided(i, 1))
                end
            end
        end
        
        function TIMELOG(this)
            %TIMELOG Appends current data to the time logger.
            this.TIME = [this.TIME; this.t];
            this.NUM_ACTIVE_VEHICLES = [this.NUM_ACTIVE_VEHICLES; this.nActiveVehicles];
            this.AVERAGE_CLOSEST_DIST = [this.AVERAGE_CLOSEST_DIST; this.avgClosestDist];
            this.AVERAGE_DIST = [this.AVERAGE_DIST; this.avgDist];
        end
        
        function writeLogFiles(this)
            %WRAPUP Wraps up the simulation
            
            % Write the time log file
            TIME_DATA = [{'Time','NUM_ACTIVE_VEHICLES','AVERAGE_CLOSEST_DIST','AVERAGE_DIST'};...
                num2cell(this.TIME), num2cell(this.NUM_ACTIVE_VEHICLES), num2cell(this.AVERAGE_CLOSEST_DIST), num2cell(this.AVERAGE_DIST)];
            writecell(TIME_DATA, this.TIME_PATH)
            
            % Write the vehicle log file
            VEHICLE_DATA = [{'Vehicle','END_CONDITION','END_TIME','DURATION', 'DIST_TRAVELLED', 'COLLIDED_WITH'};...
                num2cell((1:this.nVehicles)'), num2cell(this.END_CONDITION), num2cell(this.END_TIME), num2cell(this.DURATION), num2cell(this.DIST_TRAVELLED), num2cell(this.COLLIDED_WITH)];
            writecell(VEHICLE_DATA, this.VEHICLE_PATH)          
        end
        
    end
    
    % GETTERS
    methods 
        
        function val = get.hasAxis(this)
            val = (this.plotMode > 0);
        end
        
        function val = get.path1(this)
            val = this.simData.paths(1);
        end
        
        function val = get.path2(this)
            val = this.simData.paths(2);
        end
        
        function val = get.dT(this)
            val = this.simData.properties.deltaT;
        end
        
        function val = get.fSpawn(this)
            val = this.simData.properties.spawnFreq;
        end
        
        function val = get.tEnd(this)
            val = (this.nVehicles - 1)/this.fSpawn;
        end 
        
        function val = get.seed(this)
            val = this.simData.properties.seed;
        end
        
        function val = get.velocity(this)
            val = this.simData.properties.velocity;
        end
        
        function val = get.entryEdge(this)
            val = [this.path1.coords(1, :); this.path2.coords(1, :)];
        end
        
        function val = get.exitEdge(this)
            val = [this.path1.coords(end, :); this.path2.coords(end, :)];
        end
        
        function val = get.nVehicles(this)
            val = this.simData.properties.nVehicles;
        end

        function val = get.vehicleRadius(this)
            val = this.simData.properties.vehicleRadius;
        end
        
        function val = get.omegaMax(this)
            val = this.simData.properties.omegaMax;
        end
        
        function val = get.Kp(this)
            val = this.simData.properties.Kp;
        end
        
        function val = get.Ki(this)
            val = this.simData.properties.Ki;
        end
        
        function val = get.Kd(this)
            val = this.simData.properties.Kd;
        end

        function val = get.nTriangles(this)
            val = size(this.triangles, 2);
        end
                
        function val = get.activeVehicles(this)
            val = find([this.vehicles.active] == 1);
        end
        
        function val = get.nActiveVehicles(this)
            val = size(this.activeVehicles, 2);
        end
        
        function val = get.avgClosestDist(this)
            val = mean(min(this.distances), 'omitnan');
        end
        
        function val = get.avgDist(this)
            val = mean(this.distances, 'all', 'omitnan');
        end
        
        function val = get.finished(this)
            val = (this.nActiveVehicles == 0);
        end
        
        function val = get.collDist(this)
            val = 2 * this.vehicleRadius;
        end
        
        function val = get.corridorBounds(this)
            %CORRIDORBOUNDS Get the bounds of the corridor
            val(1) = min([this.path1.x; this.path2.x]);
            val(2) = max([this.path1.x; this.path2.x]);
            val(3) = min([this.path1.y; this.path2.y]);
            val(4) = max([this.path1.y; this.path2.y]);
        end
        
    end
    
end

