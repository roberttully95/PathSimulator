function triangles = ConstTurnRadiusTriangulation(pL, pR, vMax, thetaDot)
%CONSTTURNRADIUSTRIANGULATION 
%TODO: Rename simulation since the turn radius is not necessarily the same,
%but it is used to determine the radii.
%
% ASSUMPTIONS: 
%   1: Same number of vertices for left and right path
%   2: If turn on left path is 'left', the corresponding turn on right path
%      is also 'left'
%

    % Combine [Right, Left]
    P = [pR, pL];

    % Length
    n = length(pL.x);

    % Turn Radius
    r = vMax / thetaDot;
    
    % Assumption 1: Same number of vertices for left and right path
    if length(pR.x) ~= n
        error("Paths must be of the same length!");
    end

    %%%%%%%%%%%%%%%%%%%%%%% 
    % GET TURN DIRECTIONS %
    %%%%%%%%%%%%%%%%%%%%%%%
    turns = NaN(n - 2, 1);
    for i = 2:(n-1)
        
        % Get orientation of path 1 and 2
        or1 = orientation(pL.coords(i - 1, :), pL.coords(i, :), pL.coords(i + 1, :));
        or2 = orientation(pR.coords(i - 1, :), pR.coords(i, :), pR.coords(i + 1, :));

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
        current = turns(i-1);
        opposite = (current == 1) * 2 + (current == 2) * 1; 
        
        % Get inside and outside verts
        vInside = P(current).coords(i, :);
        vOutside = P(opposite).coords(i, :);

        % Get bisector of outer apex that points inwards
        vec2 = vecNorm(P(opposite).coords(i - 1, :) - vOutside);
        vec1 = vecNorm(P(opposite).coords(i + 1, :) - vOutside);
        bisector = vectorBisect(vec2, vec1);

        % Get the vectors that point away from the inside apex.
        vec1 = vecNorm(P(current).coords(i + 1, :) - vInside);
        vec2 = vecNorm(P(current).coords(i - 1, :) - vInside);

        % Calculate normals to these edges
        thRot = -pi/2 + pi *(current == 1);
        normal = rotateVec(bisector, -thRot);
    
        % Determine points in circle perpendicular to edges
        ptA = vInside + r * rotateVec(vec1, -thRot);
        ptB = vInside + r * rotateVec(vec2, thRot);

        % Determine the halfplane values for the left and right bound
        hpA = (vInside - ptA)*normal';
        hpB = (vInside - ptB)*normal';
        
        % Determine if there is a circle intersection based on the
        % halfplane
        hp = (vInside - vOutside)*normal';
        if hp < hpB
            centers(i - 1, :) = ptA;
        elseif hp > hpA
            centers(i - 1, :) = ptB;
        else
            [centers(i - 1, :), ~] = vectorCircleIntersection(vOutside, bisector, vInside, r);
        end
    end
    
    %%%%%%%%%%%%%%%%%%
    % GET MAX RADIUS %
    %%%%%%%%%%%%%%%%%%
    outerRadii = NaN(n - 2, 1);
    for i = 2:(n-1)
        
        % Get turn direction
        current = turns(i - 1);
        opposite = (current == 1) * 2 + (current == 2) * 1;
        
        % Get line segments from the opposite path
        points = P(opposite).coords;
        lines = cell(size(points, 1) - 1, 1);
        for j = 2:size(points, 1)
            lines{j - 1} = [points(j - 1, :); points(j, :)];
        end
        
        % Get dists to each line segment
        dists = NaN(size(points, 1) - 1, 1);
        for j = 1:size(lines, 1)
            dists(j) = distToLineSegment(lines{j}, centers(i - 1, :));
        end
        outerRadii(i - 1, :) = min(dists);
        
        %viscircles(centers(i - 1, :),  min(d1, d2));
    end
    
    % Plot circles
    viscircles(centers, outerRadii, 'Color', 'r');
    viscircles(centers, repmat(r, size(outerRadii)), 'Color', 'g');
    
    %%%%%%%%%%%%%%%%
    % GET TANGENTS %
    %%%%%%%%%%%%%%%%
    for i = 1:(n-1)
        
        if i == 1
            % If we are dealing with point-circle
            if turns(i) == 1
                % If turn is right turn
                [~, ~, ~, insideTangent] = circleCircleTangents(pR.coords(i, :), 0, centers(i, :), r);
                [~, ~, ~, outsideTangent] = circleCircleTangents(pL.coords(i, :), 0, centers(i, :), outerRadii(i, :));
            else
                % If turn is left turn
                [~, ~,  insideTangent, ~] = circleCircleTangents(pL.coords(i, :), 0, centers(i, :), r);
                [~, ~,  outsideTangent, ~] = circleCircleTangents(pR.coords(i, :), 0, centers(i, :), outerRadii(i, :));
            end
        elseif i == (n-1)
            % If we are dealing with circle-point
            if turns(i - 1) == 1
                % If turn is right turn
                [~, ~, insideTangent, ~] = circleCircleTangents(centers(i - 1, :), r, pR.coords(end, :), 0);
                [~, ~, outsideTangent, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), pL.coords(end, :), 0);
            else
                % If turn is left turn
                [~, ~, ~, insideTangent] = circleCircleTangents(centers(i - 1, :), r, pL.coords(end, :), 0);
                [~, ~, ~, outsideTangent] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), pR.coords(end, :), 0);
            end
        else
            % If we are dealing with circle-circle
            if turns(i - 1) == 1 && turns(i) == 2
                % Right turn then left turn
                [~, ~,  insideTangent, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), outerRadii(i, :));
                [~, ~, outsideTangent, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), r);
            elseif turns(i - 1) == 2 && turns(i) == 1
                % Left turn then right turn
                [~, ~, ~,  insideTangent] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), outerRadii(i, :));
                [~, ~, ~, outsideTangent] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), r);
            elseif turns(i - 1) == 1 && turns(i) == 1
                % Right turn then right turn (TEST)
                [ insideTangent, ~, ~, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), r);
                [outsideTangent, ~, ~, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), outerRadii(i, :));
            else
                % Left turn then left turn (TEST)
                [~,  insideTangent, ~, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), r);
                [~, outsideTangent, ~, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), outerRadii(i, :));
            end
        end
        plot(insideTangent(:, 1), insideTangent(:, 2),'k','LineWidth', 4);
        plot(outsideTangent(:, 1), outsideTangent(:, 2),'k','LineWidth', 4);
    end
    
    %%%%%%%%%%%%%%
    % GET WIDTHS %
    %%%%%%%%%%%%%%
    widths = outerRadii - r;
    
    %%%%%%%%%%%%%%%
    % TRIANGULATE %
    %%%%%%%%%%%%%%%
    
end

