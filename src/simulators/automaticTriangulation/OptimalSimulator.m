classdef OptimalSimulator < handle
    %OPTIMALSIMULATOR OptimalSimulator base class.
    
    % Simulation
    properties
        speedup         % Determines the speedup factor of running the code in real-time.
        file            % Complete path to the .json file.
        simData         % Simulation-level parameters / information
        vehicles        % Array of vehicles.
        dT              % The time difference between consecutive timesteps.
        t               % Holds the current time of the simulation
        plotMode        % Determines now the data will be plotted.
        handle          % The handle for plotting vehicle objects
        distances       % The distances between each vehicle (Lower Triangular Matrix)
        mapAxis         % The axis for plotting the map.
        
        turnOrientation % Stores the orientations for each of the turns cc (1) ccw(2)
        
        turnCenters     % Stores the centers of the turns
        turnIndices     % Stores the index of which turn the vehicle is approaching next
        
        dataAxis        % The map for plotting vehicle-level data.
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
        turnRadius      % The turn radius of the vehicles
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
        
        function this = OptimalSimulator(path, plot)
            %INIT Initializer for the simulator class.
            
            % Parse input data
            this.plotMode = (nargin == 2) * plot;
            
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
            
            % Initialize plot
            this.initPlot();

            % Get the turn centers
            this.getTurnCenters();
            for i = 1:length(this.turnCenters)
                viscircles(this.mapAxis, this.turnCenters(i, :), this.velocity / this.omegaMax);
            end
            
            % Initialize Vehicles
            this.initVehicles();
            
            % Start first vehicle
            this.placeVehicle(1);
        end
        
        function initVehicles(this)
            %INITVEHICLES Randomly initializes vehicles along the entry edge of the region. Does not
            %initialize the heading of the vehicles, however. The initialized heading is determined
            %by the triangulation method in the derived method.
            
            % Create array of spawn times
            t0 = 0:(1/this.fSpawn):this.tEnd;
            
            % Determine number of vehicles to be created.
            n = length(t0);
            
            % Create vehicles array.
            this.vehicles = Vehicle.empty(0, n);
            for i = 1:n
                this.vehicles(i) = Vehicle(NaN, NaN, NaN, this.velocity, t0(i), this.vehicleRadius, this.omegaMax);
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
            this.turnIndices(i) = 1;
            this.vehicles(i).th = this.getHeading(1, pt);
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
        
        function getTurnCenters(this)
            %GETTURNCENTERS Gets the centers of curvature of the turns that
            %need to be made.
            
            % Init vars
            pL = this.path1;
            pR = this.path2;
            r = this.velocity / this.omegaMax;
            
            % Combine [Right, Left]
            P = [pR, pL];

            % Length
            n = length(pL.x);
            
            % Get turn directions
            this.turnOrientation = NaN(n - 2, 1);
            for i = 2:(n-1)

                % Get orientation of path 1 and 2
                or1 = orientation(pL.coords(i - 1, :), pL.coords(i, :), pL.coords(i + 1, :));
                or2 = orientation(pR.coords(i - 1, :), pR.coords(i, :), pR.coords(i + 1, :));

                % Assumption 2: Same turn for corresponding path vertices
                if or1 ~= or2
                    error("Paths must have the same turn sequence!")
                end

                % Assign turn
                this.turnOrientation(i - 1) = or1;
            end
            
            % Get centers
            innerCenters = NaN(n - 2, 2);
            for i = 2:(n-1)

                % Get turn direction
                current = this.turnOrientation(i - 1);
                opposite = (current == 1) * 2 + (current == 2) * 1; 

                % Get inside and outside verts
                vInside = P(current).coords(i, :);
                vOutside = P(opposite).coords(i, :);

                % Get bisector of outer apex that points inwards
                vec2 = vecNorm(P(opposite).coords(i - 1, :) - vOutside);
                vec1 = vecNorm(P(opposite).coords(i + 1, :) - vOutside);
                bisector = vectorBisect(vec2, vec1);

                % Get the vectors that point away from the inside apex.
                vec1 = vecNorm(P(current).coords(i + 1, :) - vInside);
                vec2 = vecNorm(P(current).coords(i - 1, :) - vInside);

                % Calculate normals to these edges
                thRot = -pi/2 + pi *(current == 1);
                normal = rotateVec(bisector, -thRot);

                % Determine points in circle perpendicular to edges
                ptA = vInside + r * rotateVec(vec1, -thRot);
                ptB = vInside + r * rotateVec(vec2, thRot);

                % Determine the halfplane values for the left and right bound
                hpA = (vInside - ptA)*normal';
                hpB = (vInside - ptB)*normal';

                % Determine if there is a circle intersection based on the
                % halfplane
                hp = (vInside - vOutside)*normal';
                if hp < hpB
                    innerCenters(i - 1, :) = ptA;
                elseif hp > hpA
                    innerCenters(i - 1, :) = ptB;
                else
                    [innerCenters(i - 1, :), ~] = vectorCircleIntersection(vOutside, bisector, vInside, r);
                end
            end
            
            % Move the inner centers out by a vehicle radius
            for i = 1:length(innerCenters)
                
                % get path
                if this.turnOrientation(i) == 1
                    p = this.path2;
                else
                    p = this.path1;
                end
                
                % get distance from center to vertex
                dir = vecNorm(p.coords(i+1, :) - innerCenters(i, :));
                innerCenters(i, :) = innerCenters(i, :) + this.vehicleRadius * dir;
            end
            
            % Assign
            this.turnCenters = innerCenters;
        end
        
        function propogate(this)
            % PROPOGATE Propogates the simulation by 'dT' seconds.
            
            % Propogate
            for j = 1:this.nActiveVehicles
                i = this.activeVehicles(j);
                
                % Get the desired heading
                thDesired = this.getHeading(this.turnIndices(i), this.vehicles(i).pos);
                
                % Propogate vehicle
                this.vehicles(i).propogate(this.dT, thDesired);
                    
                % Update waypoint
                if this.turnIndices(i) <= length(this.turnOrientation)
                    if norm(this.vehicles(i).pos - this.turnCenters(this.turnIndices(i), :)) < 1.01 * this.turnRadius
                    	this.turnIndices(i) = this.turnIndices(i) + 1;
                    end
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
        
        function th = getHeading(this, i, pt)
            %GETHEADING Returns the commanded heading for the queried
            %vehicle.
            
            % Get number 
            n = length(this.path1.x);
            
            if i == 1
                % If we are dealing with point-circle (BEGINNING)
                if this.turnOrientation(i) == 1
                    % If turn is right turn
                    [~, ~, ~, tangent] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                else
                    % If turn is left turn
                    [~, ~,  tangent, ~] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                end
            elseif i == (n - 1)
                % If we are dealing with circle-point (END)
                cent = [mean([this.path1.x(end), this.path2.x(end)]), mean([this.path1.y(end), this.path2.y(end)])];
                tangent = [pt; cent];
            else
                % If we are dealing with circle-circle
                if this.turnOrientation(i - 1) == 1 && this.turnOrientation(i) == 2
                    % Right turn then left turn
                    [~, ~, tangent, ~] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                elseif this.turnOrientation(i - 1) == 2 && this.turnOrientation(i) == 1
                    % Left turn then right turn
                    [~, ~, ~, tangent] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                elseif this.turnOrientation(i - 1) == 1 && this.turnOrientation(i) == 1
                    % Right turn then right turn
                    [tangent, ~, ~, ~] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                else
                    % Left turn then left turn
                    [~, tangent, ~, ~] = circleCircleTangents(pt, 0, this.turnCenters(i, :), this.turnRadius);
                end
            end
            
            % Get heading
            th = vecHeading(vecNorm(tangent(2, :) - tangent(1, :)));
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
            for i = 1:length(active)
                ii = active(i);
                if this.turnIndices(ii) == size(this.turnCenters, 1) + 1
                   [dist, ~] = distToLineSegment(this.exitEdge, this.vehicles(ii).pos);
                   if dist < this.velocity * this.dT * 1.001
                        this.terminateVehicle(ii, 0);
                   end
                end
            end

            %{
                %TODO: Get boundary around triangles
                % Step 2: Terminate vehicles that collided with the wall.
                active = this.activeVehicles;
                positions  = reshape([this.vehicles(active).pos], 2, this.nActiveVehicles)';
                for i = 1:size(positions, 1)
                    d1 = distToPolyline(this.path1.coords, positions(i, :));
                    d2 = distToPolyline(this.path2.coords, positions(i, :));
                    if d1 < this.vehicles(active(i)).r || d2 < this.vehicles(active(i)).r
                        this.terminateVehicle(active(i), 2);
                    end
                end
            %}
            
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
        
        function val = get.turnRadius(this)
            val = this.velocity / this.omegaMax;
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

