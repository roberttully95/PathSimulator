classdef (Abstract) Simulator < handle
    %SIMULATOR Simulator base class. All of the code that is common between different algorithms
    %should be written here. This includes plotting the paths, reading data from input files, etc.
    
    properties
        file        % Complete path to the .json file.
        triangles   % Array of triangles that form the map
        simData     % Simulation-level parameters / information
        vehicles    % Array of vehicles.
        dT          % The time difference between consecutive timesteps.
    end
    
    properties (Access = protected)
        finished    % Flag that contains the completion state of the simulation.
        t           % Holds the current time of the simulation
        handle      % The handle for plotting vehicle objects
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
        
        function init(this, file)
            %INIT Initializer for the simulator class.
            
            % Set file
            this.file = strcat(cd, '\data\', file);
            
            % Read simulation data from input file.
            this.simData = readJson(this.file);

            % Setup random number generator
            rng(this.seed, 'combRecursive');
            
            % Initialize Vehicles
            this.initVehicles();
            
            % Set params
            this.t = 0;
            this.finished = (this.tEnd == 0);
            
            % Plot region
            this.initPlot(gca);
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
                hold on;
                scatter(pt(1), pt(2), 'r', '*');
                this.vehicles(i) = Vehicle(pt(1), pt(2), 0, this.velocity, t0(i));
            end
        end
        
        function initPlot(this, ax)
            %INITPLOT Plots both bounding paths for the region along with edges joining the start
            %and end vertices of the paths.
            
            % Setup axis
            cla(ax)
            ax.DataAspectRatio = [1, 1, 1];
            hold(ax , 'on');

            % Plot the paths
            this.path1.plot(ax, 'b');
            this.path2.plot(ax, 'b');

            % Plot the entry and exit lines.
            plot(ax, this.entryEdge(:, 1), this.entryEdge(:, 2), 'g', 'LineWidth', 1.5)
            plot(ax, this.exitEdge(:, 1), this.exitEdge(:, 2), 'r', 'LineWidth', 1.5)

            % Plot the vertices along the paths
            scatter(ax, this.path1.x, this.path1.y, 'r');
            scatter(ax, this.path2.x, this.path2.y, 'r');
            
            % Label Axes
            xlabel(ax, "x");
            ylabel(ax, "y");
            title(ax, "Map Region")
        end
        
        function plotTriangles(this, ax)
            %PLOTTRIANGLES Plots the triangles on the map plot.
            for i = 1:size(this.triangles, 2)
                % Plot triangle
                tri = this.triangles(i);
                tri.plot(gca, 'g');
                
                % Get centroid
                v = tri.centroid;
                
                % Get length 
                len = (1/6) * norm(tri.DirE(1, :) - tri.DirE(2, :));
                
                arrows(ax, v(1), v(2), len, 90 - atan2(this.triangles(i).Dir(2), this.triangles(i).Dir(1)) * (180 / pi))
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
            val = this.simData.properties.duration;
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
            val = size(this.vehicles, 2);
        end
        
        function val = get.nTriangles(this)
            val = size(this.triangles, 2);
        end
    end
end

