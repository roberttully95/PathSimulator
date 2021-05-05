classdef VelChangeSimulator < handle
    %VELCHANGESIMULATOR Simulator base class. All of the code that is common between different algorithms
    %should be written here. This includes plotting the paths, reading data from input files, etc.
    
    % Simulation
    properties
        speedup     	% Determines the speedup factor of running the code in real-time.
        configPath      % Complete path to the config .json file.
        velPath         % Complete path to the velocity file.
        velArray        % Holds the velocity array for all the vehicles
        velArrayIndex   % Holds the velocity array index.
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
        waypoints       % Stores the waypoints for each vehicle
        
        LOG_PATH        % Path to the current simulation's logging directory.
    end
    
    properties (Dependent)
        hasAxis         % Determines if there has been a valid axis argument passed to the simulator.
        path1           % The first bounding path.
        path2           % The second bounding path. 
        tEnd            % The latest time that a vehicle can be spawned.
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
        separationDist  % The separation distance that the files must maintain
        finished        % Flag that contains the completion state of the simulation.
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
        
        function this = VelChangeSimulator(configPath, velPath, triMethod, plot)
            %VELCHANGESIMULATOR Constructor
            
            % Parse input data
            this.triMethod = triMethod;
            this.plotMode = (nargin == 4) * plot;
            
            % Set file
            [~, filename, fileext] = fileparts(configPath);
            this.configPath = strcat(cd, '\files\', filename, fileext);
            [~, filename, fileext] = fileparts(velPath);
            this.velPath = strcat(cd, '\files\', filename, fileext);
            
            % Create logging directorys
            this.LOG_PATH = strcat(cd, "\logs\",  filename);
            this.TIME_PATH = strcat(this.LOG_PATH, "\time.xls");
            this.VEHICLE_PATH = strcat(this.LOG_PATH, "\vehicles.xls");
            [~, ~, ~] = mkdir(this.LOG_PATH);
            
            % Read simulation data from input file.
            this.simData = readJson(this.configPath);

            % Load the velocity table
            this.loadVelocityArray();
            
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
            
            % Define the triangulation
            this.triangulate();
            
            % Init vehicles
            this.initVehicles();

            % Plot triangles
            this.initPlot();
            this.plotTriangles();
        end
        
        function triangulate(this)
            %TRIANGULATE Creates a triangulation based on the provided triangulation enumeration
            %value
            
            switch this.triMethod
                case Triangulation.Closest
                    this.triangles = closestTriangulation(this.path1, this.path2);
            end
            
        end
        
        function loadVelocityArray(this)
            %LOADVELOCITYARRAY Loads the table of velocity changes
            
            % Read raw table data
            rawTable = readtable(this.velPath, 'ReadVariableNames',false);
            [r, c] = size(rawTable);

            % Convert to array
            rawArray = table2array(rawTable);
            
            % Remove duplicates
            for i = (r-1):-1:1
                
                % Get consecutive entries
                frst = rawArray(i, 1:end-1);
                scnd = rawArray(i + 1, 1:end-1);
                
                % If the entries are the same, then delete the second one.
                if frst == scnd
                    rawArray(i + 1, :) = [];
                end
            end
            
            % Store
            this.velArray = rawArray;
            this.velArrayIndex = 1;
        end
        
        function initVehicles(this)
            %INITVEHICLES Randomly initializes vehicles along the entry edge of the region. Does not
            %initialize the heading of the vehicles, however. The initialized heading is determined
            %by the triangulation method in the derived method.
            
            % Create vehicles array.
            this.vehicles = Vehicle.empty(0, this.nVehicles);
            for i = 1:this.nVehicles
                this.vehicles(i) = Vehicle(NaN, NaN, NaN, this.velArray(this.velArrayIndex, i), 0, this.separationDist, this.omegaMax);
            end
            
            % Init
            for i = 1:this.nVehicles
                this.initVehicle(i);
            end
            
        end
        
        function initVehicle(this, i)
            %INITVEHICLE Initializes a vehicle to a deconflicted state.
            
            % Get the current vehicle positions
            pos = reshape([this.vehicles(:).pos], 2, this.nVehicles)';
            
            % Get the bounds
            xMin = min([this.path1.x; this.path2.x]);
            xMax = max([this.path1.x; this.path2.x]);
            yMin = min([this.path1.y; this.path2.y]);
            yMax = max([this.path1.y; this.path2.y]);
              
            % Init
            k = 1;
            validLocation = false;
            if i == 1
                % If the first vehicle, then it must be in the first
                % triangle
                poly = polyshape(this.triangles(1).x, this.triangles(1).y);
            else
                % Init polygon shape
                poly = polyshape([this.path1.x; flipud(this.path2.x)], [this.path1.y; flipud(this.path2.y)]);
            end
                
            while ~validLocation
                if k == 1000
                    error("Cannot fit specified number of vehicle on the map!");
                end

                % Create random coordinate
                xR = xMin + rand * (xMax - xMin);
                yR = yMin + rand * (yMax - yMin);

                % Generate point
                pt = [xR, yR];

                % Ensure the point is located within the polygon
                if ~isinterior(poly, xR, yR)
                    k = k + 1;
                    continue;
                end

                % Ensure vehicle is greater than sep dist from all vehicles
                dists = sqrt((pos(:, 1) - pt(1)).^2 + (pos(:, 2) - pt(2)).^2);
                if any(dists < this.separationDist )
                    k = k + 1;
                    continue;
                end

                % Ensure vehicle is less than sep dist from any vehicle
                if i ~= 1
                    if ~any(dists < 3*this.separationDist)
                        k = k + 1;
                        continue;
                    end
                end
                
                % At this point, it is valid
                validLocation = true;
                k = 1;
            end
            
            % Assign
            this.vehicles(i).x = pt(1);
            this.vehicles(i).y = pt(2);
            this.vehicles(i).active = true;
            
            % Determine the vehicle's initial triangle index and heading
            for j = 1:this.nTriangles
                p = polyshape(this.triangles(j).x, this.triangles(j).y);
                if isinterior(p, xR, yR)
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
            
            if ~this.hasAxis
                return;
            end
            
            % Initialize plot map.
            fig = figure(1);
            this.mapAxis = axes(fig);

            % Setup axis
            cla(this.mapAxis);
            %this.mapAxis.DataAspectRatio = [1, 1, 1];
            hold(this.mapAxis , 'on');
            axis(this.mapAxis, 'equal');
            
            % Plot the paths
            this.path1.plot(this.mapAxis, 'b');
            this.path2.plot(this.mapAxis, 'b');

            % Plot the entry and exit lines.
            plot(this.mapAxis, this.entryEdge(:, 1), this.entryEdge(:, 2), 'g', 'LineWidth', 1.5)
            plot(this.mapAxis, this.exitEdge(:, 1), this.exitEdge(:, 2), 'r', 'LineWidth', 1.5)

            % Plot the vertices along the paths
            scatter(this.mapAxis, this.path1.x, this.path1.y, 'r');
            scatter(this.mapAxis, this.path2.x, this.path2.y, 'r');

            % Label Axes
            xlabel(this.mapAxis, "x");
            ylabel(this.mapAxis, "y");
            title(this.mapAxis, "Map Region")

            % Set data axis
            if this.plotMode == 2
                fig = figure(2);
                this.dataAxis = axes(fig);
            end
            
        end
        
        function plotTriangles(this)
            %PLOTTRIANGLES Plots the triangles on the map plot.
            
            if ~this.hasAxis
                return;
            end
            
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
            
            % Step 2: Terminate vehicles that collided with each other and
            % update distance matrix
            this.updateDistances();
            
            % Step 3: Update velocities based on the velocity table
            if this.velArrayIndex < size(this.velArray, 1) 
                if this.t > this.velArray(this.velArrayIndex, end)
                    this.velArrayIndex = this.velArrayIndex + 1;
                    for i = 1:this.nVehicles
                        this.vehicles(i).v = this.velArray(this.velArrayIndex, i);
                    end
                end
            end
            
            % Step 4: Log time data
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
                collided = list(dists < this.separationDist, :);
                
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
            
            % Write waypoints
            warning( 'off', 'MATLAB:xlswrite:AddSheet' ) ;
            filename = strcat(this.LOG_PATH, "\waypoints.xls");
            for i = 1:length(this.waypoints)
                writematrix(this.waypoints{i}, filename, 'Sheet', i);
            end
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
            val = size(this.velArray, 2) - 1;
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
        
        function val = get.separationDist(this)
            val = this.simData.properties.separationDist;
        end
        
        function val = get.finished(this)
            val = (this.nActiveVehicles == 0);
        end
        

        
    end
    
end

