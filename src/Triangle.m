classdef Triangle < handle
    %TRIANGLE Triangle object.
    
    properties
        V1
        V2
        V3
        Entry
        Exit
        DirE
        Dir
        Prev
        Next
    end
    
    properties (Dependent)
        xVals
        yVals
        centroid
    end
    
    methods
        function this = Triangle(A, B, C, type)
            %TRIANGLE Construct an instance of this class
            %   Detailed explanation goes here
            
            % If the user entered the edges.
            if strcmp(type, 'Edges')
                
                % Set edges
                this.Entry = A;
                this.Exit = B;
                this.DirE = C;

                % Set Vertices
                this.V1 = A(1, :);
                this.V2 = A(2, :);
                this.V3 = B(2, :);
                
            elseif strcmp(type, 'Vertices')
                
                % Set Vertices
                this.V1 = A;
                this.V2 = B;
                this.V3 = C;
                
                % Set Edges
                this.Entry = [A; B];
                this.Exit = [B; C];
                this.DirE = [A; C];   
            end
           
            % Set Direction
            dirAbs = this.DirE(2, :) - this.DirE(1, :); 
            this.Dir = dirAbs / norm(dirAbs);
            
            % Check Colinear
            pointsAreCollinear = @(xy) rank(xy(2:end,:) - xy(1,:)) == 1;
            if pointsAreCollinear([this.V1; this.V2; this.V3])
                error('Triangle vertices are collinear.')
            end
               
            % Init indexes
            this.Prev = 0;
            this.Next = NaN;
        end
        
        function plot(this, ax, color)
            if nargin == 2
                color = 'r';
            end
                        
            % Set hold on
            hold(ax, 'on');
               
            % Patch shape
            patch(this.xVals, this.yVals, color, 'FaceAlpha', 0.2);
            
            % Plot entry edge
            line([this.V1(1), this.V2(1)], [this.V1(2), this.V2(2)],...
                'LineWidth',1,...
                'Color','k')
            
            % Plot exit edge
            line([this.V2(1), this.V3(1)], [this.V2(2), this.V3(2)],...
                'LineWidth',1,...
                'Color','k')
            
            % Plot direction edge
            line([this.V3(1), this.V1(1)], [this.V3(2), this.V1(2)],...
                'LineWidth',1,...
                'Color','k')
            
            % Hold off
            hold(ax, 'off');
        end

        function val = containsPt(this, pt)
            
            d1 = cross2d(pt - this.V2, this.V1 - this.V2);
            d2 = cross2d(pt - this.V3, this.V2 - this.V3);
            d3 = cross2d(pt - this.V1, this.V3 - this.V1);

            has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

            val = ~(has_neg && has_pos);
        end

        function xVals = get.xVals(this)
            xVals = [this.V1(1), this.V2(1), this.V3(1)];
        end
        
        function yVals = get.yVals(this)
            yVals = [this.V1(2), this.V2(2), this.V3(2)];
        end
        
        function val = get.centroid(this)
            val = mean([this.V1; this.V2; this.V3], 1);
        end
    end
end

