function triangles = ConstTurnRadiusTriangulation(pA, pB, vMax, thetaDot)
%CONSTTURNRADIUSTRIANGULATION 
%TODO: Rename simulation since the turn radius is not necessarily the same,
%but it is used to determine the radii.
%
% ASSUMPTIONS: 
%   1: Same number of vertices for left and right path
%   2: If turn on left path is 'left', the corresponding turn on right path
%      is also 'left'
%

    % Combine [Left, Right]
    P = [pB, pA];

    % Length
    n = length(pA.x);

    % Turn Radius
    r = vMax / thetaDot;
    
    % Assumption 1: Same number of vertices for left and right path
    if length(pB.x) ~= n
        error("Paths must be of the same length!");
    end

    %%%%%%%%%%%%%%%%%%%%%%% 
    % GET TURN DIRECTIONS %
    %%%%%%%%%%%%%%%%%%%%%%%
    turns = NaN(n - 2, 1);
    for i = 2:(n-1)
        
        % Get orientation of path 1 and 2
        or1 = orientation(pA.coords(i - 1, :), pA.coords(i, :), pA.coords(i + 1, :));
        or2 = orientation(pB.coords(i - 1, :), pB.coords(i, :), pB.coords(i + 1, :));

        % Assumption 2: Same turn for corresponding path vertices
        if or1 ~= or2
            error("Paths must have the same turn sequence!")
        end
        
        % Assign turn
        turns(i - 1) = or1;
    end
    
    %%%%%%%%%%%%%%%%%%%%
    % GET TURN CENTERS %
    %%%%%%%%%%%%%%%%%%%%
    centers = NaN(n - 2, 2);
    for i = 2:(n-1)
        % Get turn direction
        inside = turns(i-1);
        outside = (inside == 1) * 2 + (inside == 2) * 1; 
        
        % Get inside and outside verts
        vInside = P(inside).coords(i, :);
        vOutside = P(outside).coords(i, :);

        % Get bisector of outer apex that points inwards
        vec2 = vecNorm(P(outside).coords(i - 1, :) - vOutside);
        vec1 = vecNorm(P(outside).coords(i + 1, :) - vOutside);
        bisector = vectorBisect(vec2, vec1);

        % Get the vectors that point away from the inside apex.
        vec1 = vecNorm(P(inside).coords(i + 1, :) - vInside);
        vec2 = vecNorm(P(inside).coords(i - 1, :) - vInside);

        % Calculate normals to these edges
        thRot = -pi/2 + pi *(inside == 1);
        normal = rotateVec(bisector, -thRot);
    
        % Determine points in circle perpendicular to edges
        pA = vInside + r * rotateVec(vec1, -thRot);
        pB = vInside + r * rotateVec(vec2, thRot);

        % Determine the halfplane values for the left and right bound
        hpA = (vInside - pB)*normal';
        hpB = (vInside - pA)*normal';
        
        % Determine if there is a circle intersection based on the
        % halfplane
        hp = (vInside - vOutside)*normal';
        if hp < hpB
            centers(i - 1, :) = pA;
        elseif hp > hpA
            centers(i - 1, :) = pB;
        else
            [centers(i - 1, :), ~] = vectorCircleIntersection(vOutside, bisector, vInside, r);
        end
    end
    
    %%%%%%%%%%%%%%%%%%
    % GET MAX RADIUS %
    %%%%%%%%%%%%%%%%%%
    radii = NaN(n - 2, 1);
    for i = 2:(n-1)
        
        % Get turn direction
        current = turns(i - 1);
        opposite = (current == 1) * 2 + (current == 2) * 1;
        
        % Get candidate lines
        line1 = [P(opposite).coords(i, :); P(opposite).coords(i - 1, :)];
        line2 = [P(opposite).coords(i, :); P(opposite).coords(i + 1, :)];
        
        % Get distances
        d1 = distToLineSegment(line1, centers(i - 1, :));
        d2 = distToLineSegment(line2, centers(i - 1, :));
        
        % Assign radius
        radii(i - 1, :) = min(d1, d2);
    end
    
    %%%%%%%%%%%%%%
    % GET WIDTHS %
    %%%%%%%%%%%%%%
    widths = radii - r;
    
    %%%%%%%%%%%%%%%
    % TRIANGULATE %
    %%%%%%%%%%%%%%%
    
end

function or = orientation(p1, p2, p3, varargin)
%ORIENTATION Determines the orientation of a triplet of point
%   0: Points are colinear (not turn)
%   1: Points are clockwise (right turn)
%   2: Points are counter-clockwise (left turn)
    
    % Tolerance
    if nargin == 4
        tol = varargin{4};
    else 
        tol = 10^-8;
    end

    % Get val
    val = (p2(2) - p1(2)) * (p3(1) - p2(1)) - (p2(1) - p1(1)) * (p3(2) - p2(2));
    
    % Colinear
    if abs(val) < tol
        or = 0;
        return;
    end
    
    % Get orientation
    or = 2 - (val > 0);
end