classdef (Abstract) Simulator < handle
    %SIMULATOR Simulator base class. All of the code that is common between different algorithms
    %should be written here. This includes plotting the paths, reading data from input files, etc.
    
    properties
        file % Complete path to the .json file.
        
        triangles % THe array of triangles that form the map.
        
        simData % Simulation-level parameters / information
    end
    
    properties (Dependent)
        path1 % The first bounding path.
        path2 % The second bounding path. 
    end
    
    methods (Abstract)
        % Abstract methods are methods that have to be implemented by any classes derived from the
        % class.
        triangulate(this)
    end
    
    % PUBLIC METHODS
    methods (Access = protected)
        
        function init(this, file)
            %INIT Initializer for the simulator class.
            
            % Set file
            this.file = strcat(cd, '\data\', file);
            
            % Read simulation data from input file.
            this.simData = readJson(this.file);

            % Plot region
            this.initPlot(gca);
        end

        function initPlot(this, ax)
            %PLOTREGION Plots both bounding paths for the region along with edges joining the start
            %and end vertices of the paths.
            
            % Setup axis
            cla(ax)
            ax.DataAspectRatio = [1, 1, 1];
            hold(ax , 'on');

            % Plot the paths
            this.path1.plot(ax, 'b');
            this.path2.plot(ax, 'b');

            % Plot the entry and exit lines.
            plot(ax, [this.path1.x(1);  this.path2.x(1)], [this.path1.y(1);  this.path2.y(1)], ...
                'g', 'LineWidth', 1.5)
            plot(ax, [this.path1.x(end);  this.path2.x(end)], [this.path1.y(end);  this.path2.y(end)], ...
                'r', 'LineWidth', 1.5)

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
       
    end
    
    methods 
        function val = get.path1(this)
            val = this.simData.paths(1);
        end
        
        function val = get.path2(this)
            val = this.simData.paths(2);
        end
    end
end

