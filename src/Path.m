classdef Path < handle
    %PATH Summary of this class goes here
    %   Detailed explanation goes here
    
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
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            plot(ax, this.x, this.y, color);
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

