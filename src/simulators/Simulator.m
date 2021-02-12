classdef (Abstract) Simulator < handle
    %SIMULATOR Simulator base class. All of the code that is common between different algorithms
    %should be written here. This includes plotting the paths, reading data from input files, etc.
    
    properties
        file        % Complete path to the .json file.
        triangles   % Array of triangles that form the map
        simData     % Simulation-level parameters / information
        vehicles    % Array of vehicles.
        dT          % The time difference between consecutive timesteps.
        finished    % Flag that contains the completion state of the simulation.
        t           % Holds the current time of the simulation
        hasAxis     % Determines if there has been a valid axis argument passed to the simulator.
        axis        % Contains the axis on which the items will be rendered.
        handle      % The handle for plotting vehicle objects
        distances   % The distances between each vehicle (Lower Triangular Matrix)
    end
    
    properties (Dependent)
        path1       % The first bounding path.
        path2       % The second bounding path. 
        tEnd        % The latest time that a vehicle can be spawned.
        fSpawn      % The frequency at which vehicles are spawned into the map.
        seed        % The random number seed that allows for reproducability of simulations.
        velocity    % The nominal velocity of the vehicles in the simulation.
        entryEdge   % The edge that the vehicles will enter from.
        exitEdge    % The edge that the vehicles will exit from.
        nVehicles   % The number of vehicles in the simulation
        nTriangles  % The number of triangles in the simulation.
    end
    
    methods (Abstract)
        % Abstract methods are methods that have to be implemented by any classes derived from the
        % class.
        triangulate(this)
        propogate(this)
    end
    
    % PUBLIC METHODS
    methods
        
        function init(this, args)
            %INIT Initializer for the simulator class.
            
            % Detect invalid number of arguments.
            n = size(args, 2);
            if n < 1 || n > 2
                error("Invalid number of arguments provided");
            end
            
            % Set file
            this.file = strcat(cd, '\data\', args{1});
            
            % Set axis
            if n == 2
                if isgraphics(args{2})
                    this.axis = args{2};
                else
                    warning("Invalid axis object passed. Continuing without axis.")
                end
            end
            
            % Read simulation data from input file.
            this.simData = readJson(this.file);

            % Setup random number generator
            rng(this.seed, 'combRecursive');
            
            % Initialize Vehicles
            this.initVehicles();
            
            % Initialize distances
            this.distances = NaN(this.nVehicles, this.nVehicles);
            
            % Set params
            this.t = 0;
            this.finished = (this.tEnd == 0);
            
            % Plot region
            this.initPlot();
        end

        function initVehicles(this)
            %INITVEHICLES Randomly initializes vehicles along the entry edge of the region. Does not
            %initialize the heading of the vehicles, however. The initialized heading is determined
            %by the triangulation method in the derived method.
            
            % Create array of spawn times
            t0 = 0:(1/this.fSpawn):this.tEnd;
            
            % Determine number of vehicles to be created.
            n = length(t0);
            
            % Direction of entry edge.
            v1 = this.entryEdge(1, :);
            v2 = this.entryEdge(2, :);
            
            % Direction of entry edge.
            d = v2 - v1;
            
            % Create vehicles array.
            this.vehicles = Vehicle.empty(0, n);
            for i = 1:n
                pt = v1 + d * rand;
                this.vehicles(i) = Vehicle(pt(1), pt(2), 0, this.velocity, t0(i));
            end
        end
        
        function initPlot(this)
            %INITPLOT Plots both bounding paths for the region along with edges joining the start
            %and end vertices of the paths.
            
            if this.hasAxis

                % Setup axis
                cla(this.axis)
                this.axis.DataAspectRatio = [1, 1, 1];
                hold(this.axis , 'on');

                % Plot the paths
                this.path1.plot(this.axis, 'b');
                this.path2.plot(this.axis, 'b');

                % Plot the entry and exit lines.
                plot(this.axis, this.entryEdge(:, 1), this.entryEdge(:, 2), 'g', 'LineWidth', 1.5)
                plot(this.axis, this.exitEdge(:, 1), this.exitEdge(:, 2), 'r', 'LineWidth', 1.5)

                % Plot the vertices along the paths
                scatter(this.axis, this.path1.x, this.path1.y, 'r');
                scatter(this.axis, this.path2.x, this.path2.y, 'r');

                % Label Axes
                xlabel(this.axis, "x");
                ylabel(this.axis, "y");
                title(this.axis, "Map Region")
            end
        end
        
        function plotTriangles(this)
            %PLOTTRIANGLES Plots the triangles on the map plot.
            
            if this.hasAxis
                for i = 1:size(this.triangles, 2)
                    % Plot triangle
                    tri = this.triangles(i);
                    tri.plot(this.axis, 'g');

                    % Get centroid
                    v = tri.centroid;

                    % Get length 
                    len = (1/6) * tri.dirLength;

                    % Plot arrows
                    arrows(this.axis, v(1), v(2), len, 90 - atan2(tri.dir(2), tri.dir(1)) * (180 / pi))
                end
            end
        end

        function updateDistances(this)
            %UPDATEDISTANCES Updates the distance matrix based on the current distance between all
            %vehicles.
            
            % Calculate max index based on the time.
            max = min(floor((this.t * this.fSpawn) + 1), this.nVehicles);
            
            % Iterate through all vehicles.
            for i = 1:max
                for j = 1:max
                    % Keep upper triangular
                    if i <= j 
                        continue;
                    end
                    
                    % Only get distance if both are active
                    if this.t >= this.vehicles(i).tInit && this.t < this.vehicles(i).tEnd && ...
                            this.t >= this.vehicles(j).tInit && this.t < this.vehicles(j).tEnd
                        this.distances(i, j) = norm(this.vehicles(i).pos - this.vehicles(j).pos);
                    else
                        this.distances(i, j) = NaN;
                    end
                    
                end
            end
            
        end
        
        function val = isFinished(this)
            val = this.finished;
        end
    end
    
    % GETTERS
    methods 
        
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
        
        function val = get.nTriangles(this)
            val = size(this.triangles, 2);
        end
        
        function val = get.hasAxis(this)
            val = ~isempty(this.axis);
        end
    end
end

