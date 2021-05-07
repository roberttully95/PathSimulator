classdef Region < handle
    %REGION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        coords % vertex coordinates
         theta  % commanded heading for the region
         vel    % commanded velocity for the region
    end
    
    properties (Dependent)
        x
        y
    end
    
    methods
        function this = Region(coords, theta, vel)
            %REGION Construct an instance of this class
            %   Detailed explanation goes here
            this.coords = coords;
             this.theta = theta;
             this.vel = vel;
        end
        
        function plot(this, ax, color)
            %PLOT Plots the triangle.
            if nargin == 2
                color = 'r';
            end
                        
            % Set hold on
            hold(ax, 'on');
               
            % Patch shape
            patch(this.x, this.y, color, 'FaceAlpha', 0.2);
            
            l = norm(this.coords(1, :) - [mean(this.x), mean(this.y)]) / 2;
            arrows(ax, mean(this.x), mean(this.y), l, 90 - rad2deg(this.theta));
            
            % Hold off
            hold(ax, 'off');
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

