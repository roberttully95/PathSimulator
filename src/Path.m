classdef Path < handle
    %PATH Path class.
    
    properties
        x
        y
    end
    
    properties (Dependent)
        coords
        length
    end
    
    methods
        function this = Path(x, y)
            %PATH Constructor for path object.
            this.x = x;
            this.y = y;
        end
        
        function plot(this, ax, color)
            %PLOT Plots the path on an axis,
            plot(ax, this.x, this.y, color, 'LineWidth', 1.5);
        end

    end
    
    methods
        
        function val = get.coords(this)
            val = [this.x, this.y];
        end
        
        function val = get.length(this)
            val = size(this.x, 1);
        end
    end
    
end

