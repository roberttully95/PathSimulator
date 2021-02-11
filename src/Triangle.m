classdef Triangle < handle
    %TRIANGLE Triangle object.
    
    properties
        V1              % Location of vertex 1 [x, y]
        V2              % Location of vertex 2 [x, y]
        V3              % Location of vertex 3 [x, y]
        entryEdge       % The entry edge [V1; V2]
        exitEdge        % The exit edge [V2; V3]
        directionEdge   % The direction edge [V1; V3]
        prevIndex       % The index of the previous triangle in the triangles array
        nextIndex       % The index of the next triangle in the triangles array
    end
    
    properties (Dependent)
        x           % The x coordinates of the vertices
        y           % The y coordinates of the vertices 
        centroid    % The centroid of the triangle
        dir         % The normalized direction vector for the triangle
        entryLength % Length of the entry edge
        exitLength  % Length of the exit edge
        dirLength   % Length of the direction edge
    end
    
    methods
        function this = Triangle(A, B, C, type)
            %TRIANGLE Construct a triangle instance. Triangle can be constructed by either passing
            %the vertices with the tag 'Vertices' or by passing the edges with the tag 'Edges'
            
            % If the user entered the edges.
            if strcmp(type, 'Edges')
                
                % Set edges
                this.entryEdge = A;
                this.exitEdge = B;
                this.directionEdge = C;

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
                this.entryEdge = [A; B];
                this.exitEdge = [B; C];
                this.directionEdge = [A; C];   
            end
            
            % Check Colinear
            pointsAreCollinear = @(xy) rank(xy(2:end,:) - xy(1,:)) == 1;
            if pointsAreCollinear([this.V1; this.V2; this.V3])
                error('Triangle vertices are collinear.')
            end
               
            % Init indexes
            this.prevIndex = 0;
            this.nextIndex = NaN;
        end
        
        function plot(this, ax, color)
            if nargin == 2
                color = 'r';
            end
                        
            % Set hold on
            hold(ax, 'on');
               
            % Patch shape
            patch(this.x, this.y, color, 'FaceAlpha', 0.2);
            
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

        function xVals = get.x(this)
            xVals = [this.V1(1), this.V2(1), this.V3(1)];
        end
        
        function yVals = get.y(this)
            yVals = [this.V1(2), this.V2(2), this.V3(2)];
        end
        
        function val = get.centroid(this)
            val = mean([this.V1; this.V2; this.V3], 1);
        end
        
        function val = get.entryLength(this)
            val = norm(this.entryEdge(1,:) - this.entryEdge(2,:));
        end
        
        function val = get.exitLength(this)
            val = norm(this.exitEdge(1,:) - this.exitEdge(2,:));
        end
        
        function val = get.dirLength(this)
            val = norm(this.directionEdge(1,:) - this.directionEdge(2,:));
        end
        
        function val = get.dir(this)
            dirAbs = this.directionEdge(2, :) - this.directionEdge(1, :); 
            val = dirAbs / norm(dirAbs);
        end
    end
end

