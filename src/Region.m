classdef Region < handle
    %REGION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        coords      % vertex coordinates
        thetaCmd    % commanded theta
        velCmd      % commanded velocity
        thetaDotCmd % commanded turn rate
        
        prevIndex   % set previous index
        nextIndex   % set next index
    end
    
    properties (Access = private)
        color       % the color that the region will be plotted
    end
    
    properties (Dependent)
        x
        y
    end
    
    methods
        function this = Region(coords, color)
            %REGION Construct an instance of this class
            %   Detailed explanation goes here
            this.coords = coords;
            
            % Init all to NaN
            this.thetaCmd = NaN;
            this.velCmd = NaN;
            this.thetaDotCmd = NaN;
            
            % Init indices
            this.prevIndex = 0;
            this.nextIndex = NaN;
            
            this.color = color;
        end
        
        function plot(this, ax, color)
            %PLOT Plots the triangle.
            if nargin == 2
                color = 'r';
            end
                        
            % Set hold on
            hold(ax, 'on');
               
            % Patch shape
            patch(this.x, this.y, color, 'FaceColor', this.color, 'FaceAlpha', 0.2);
            
            % Plot arrow
            l = norm(this.coords(1, :) - [mean(this.x), mean(this.y)]) / 2;
            arrows(ax, mean(this.x), mean(this.y), l, 90 - rad2deg(this.thetaCmd));
            
            % Hold off
            hold(ax, 'off');
        end
        
        function val = containsPt(this, pt)
            %CONTAINSPT Determines if a triangle contains a provided point.
            
            val = inpolygon(pt(1), pt(2), this.x, this.y);
        end
    end
    
    methods
        
        function val = get.x(this)
            val = this.coords(:, 1);
        end
        
        function val = get.y(this)
            val = this.coords(:, 2);
        end
        
    end
end

