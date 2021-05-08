function triangles = ConstVelocityTriangulation(pL, pR, vMax, thetaDot)
%CONSTVELOCITYTRIANGULATION 
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
    rightTangents = cell(n-1, 1);
    leftTangents = cell(n-1, 1);
    for i = 1:(n-1)
        
        if i == 1
            % If we are dealing with point-circle
            if turns(i) == 1
                % If turn is right turn
                [~, ~, ~, currR] = circleCircleTangents(pR.coords(i, :), 0, centers(i, :), r);
                [~, ~, ~, currL] = circleCircleTangents(pL.coords(i, :), 0, centers(i, :), outerRadii(i, :));
            else
                % If turn is left turn
                [~, ~,  currL, ~] = circleCircleTangents(pL.coords(i, :), 0, centers(i, :), r);
                [~, ~,  currR, ~] = circleCircleTangents(pR.coords(i, :), 0, centers(i, :), outerRadii(i, :));
            end
        elseif i == (n-1)
            % If we are dealing with circle-point
            if turns(i - 1) == 1
                % If turn is right turn
                [~, ~, currR, ~] = circleCircleTangents(centers(i - 1, :), r, pR.coords(end, :), 0);
                [~, ~, currL, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), pL.coords(end, :), 0);
            else
                % If turn is left turn
                [~, ~, ~, currL] = circleCircleTangents(centers(i - 1, :), r, pL.coords(end, :), 0);
                [~, ~, ~, currR] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), pR.coords(end, :), 0);
            end
        else
            % If we are dealing with circle-circle
            if turns(i - 1) == 1 && turns(i) == 2
                % Right turn then left turn
                [~, ~, currR, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), outerRadii(i, :));
                [~, ~, currL, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), r);
            elseif turns(i - 1) == 2 && turns(i) == 1
                % Left turn then right turn
                [~, ~, ~, currL] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), outerRadii(i, :));
                [~, ~, ~, currR] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), r);
            elseif turns(i - 1) == 1 && turns(i) == 1
                % Right turn then right turn (TEST)
                [currR, ~, ~, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), r);
                [currL, ~, ~, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), outerRadii(i, :));
            else
                % Left turn then left turn (TEST)
                [~, currL, ~, ~] = circleCircleTangents(centers(i - 1, :), r, centers(i, :), r);
                [~, currR, ~, ~] = circleCircleTangents(centers(i - 1, :), outerRadii(i - 1, :), centers(i, :), outerRadii(i, :));
            end
        end
        
        % Assign
        rightTangents{i} = currR;
        leftTangents{i} = currL;
    end
    
    % Plot tangents
    for i = 1:length(rightTangents)
        plot(rightTangents{i}(:, 1), rightTangents{i}(:, 2),'r','LineWidth', 4);
        plot(leftTangents{i}(:, 1), leftTangents{i}(:, 2),'g','LineWidth', 4);
    end
    
    % Get quadrilateral regions
    for i = 1:length(turns)
        
        % Get turn direction
        current = turns(i);
        opposite = (current == 1) * 2 + (current == 2) * 1; 
        
        % Get the coordinates of the start and end of the turn vertices for
        % the turn
        rEnd = rightTangents{i}(2, :);
        rStart = rightTangents{i + 1}(1, :);
        lEnd = leftTangents{i}(2, :);
        lStart = leftTangents{i + 1}(1, :);
        
        % Create the incoming and outgoing edges
        incomingEdge = [rEnd; lEnd];
        outgoingEdge = [rStart; lStart];
        
        % Get the intersection point
        intPt = lineLineIntersection(incomingEdge, outgoingEdge);
        
        % Calculate the length of the triangle edges to cover the entire
        % region
        vec1 = vecNorm(incomingEdge(opposite, :) - intPt);
        vec2 = vecNorm(outgoingEdge(opposite, :) - intPt);
        
        % Just assume 2 x rBig (FIXME)
        pt1 = intPt + 2 * outerRadii(i) * vec1;
        pt2 = intPt + 2 * outerRadii(i) * vec2;
        
        % Plot the triangle
        p = polyshape([pt1; pt2; intPt]);
        plot(p);
    end

    %%%%%%%%%%%%%%%
    % TRIANGULATE %
    %%%%%%%%%%%%%%%
end

