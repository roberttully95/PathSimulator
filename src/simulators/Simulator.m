classdef Simulator < handle
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        path1
        path2
    end
    
    properties (Access = protected)
        handle
    end
    
    % PUBLIC METHODS
    methods
        
        function this = Simulator(varargin)
            %SIMULATOR Constructor for the simulator class.
            
            % Parse arguments
            if nargin ~= 1
                error("Invalid number of arguments")
            else
                file = strcat(cd, '\data\', varargin{1});
            end

            % Read kml
            data = readKml(file);
            this.path1 = Path(data(1).Lon, data(1).Lat);
            this.path2 = Path(data(2).Lon, data(2).Lat);
            
            % Plot region
            this.plotRegion(gca);
        end

        function plotRegion(this, ax)
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
            plot(ax, [this.path1.x(1);  this.path2.x(1)], [this.path1.y(1);  this.path2.y(1)], 'g')
            plot(ax, [this.path1.x(end);  this.path2.x(end)], [this.path1.y(end);  this.path2.y(end)], 'r')

            % Label Axes
            xlabel(ax, "x");
            ylabel(ax, "y");
            title(ax, "Map Region")
        end
        
    end
end

