function regions = ConstTurnRadiusTriangulation(pL, pR, r)
%CONSTTURNRADIUSTRIANGULATION 
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
    innerCenters = NaN(n - 2, 2);
    outerCenters = NaN(n - 2, 2);
    for i = 2:(n-1)
        
        % Get turn direction
        current = turns(i - 1);
        opposite = (current == 1) * 2 + (current == 2) * 1; 
        
        % Get inside and outside verts
        vInside = P(current).coords(i, :);
        vOutside = P(opposite).coords(i, :);

        % Get bisector of outer apex that points inwards
        vec2 = vecNorm(P(opposite).coords(i - 1, :) - vOutside);
        vec1 = vecNorm(P(opposite).coords(i + 1, :) - vOutside);
        bisector = vectorBisect(vec2, vec1);
        th = acos(max(min(dot(vec1, vec2)/(norm(vec1)*norm(vec2)), 1), -1));
        len = r / sin(th / 2);
        outerCenters(i - 1, :) = vOutside + bisector * len;
        
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
            innerCenters(i - 1, :) = ptA;
        elseif hp > hpA
            innerCenters(i - 1, :) = ptB;
        else
            [innerCenters(i - 1, :), ~] = vectorCircleIntersection(vOutside, bisector, vInside, r);
        end
    end
    
    % Plot inner and outer circles
    %for i = 1:size(innerCenters, 1)
    %    viscircles(innerCenters(i, :), r, 'Color', 'r');
    %    viscircles(outerCenters(i, :), r, 'Color', 'g');
    %end
    
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
                [~, ~, ~, currR] = circleCircleTangents(pR.coords(i, :), 0, innerCenters(i, :), r);
                [~, ~, ~, currL] = circleCircleTangents(pL.coords(i, :), 0, outerCenters(i, :), r);
            else
                % If turn is left turn
                [~, ~,  currL, ~] = circleCircleTangents(pL.coords(i, :), 0, innerCenters(i, :), r);
                [~, ~,  currR, ~] = circleCircleTangents(pR.coords(i, :), 0, outerCenters(i, :), r);
            end
        elseif i == (n-1)
            % If we are dealing with circle-point
            if turns(i - 1) == 1
                % If turn is right turn
                [~, ~, currR, ~] = circleCircleTangents(outerCenters(i - 1, :), r, pR.coords(end, :), 0);
                [~, ~, currL, ~] = circleCircleTangents(innerCenters(i - 1, :), r, pL.coords(end, :), 0);
            else
                % If turn is left turn
                [~, ~, ~, currL] = circleCircleTangents(innerCenters(i - 1, :), r, pL.coords(end, :), 0);
                [~, ~, ~, currR] = circleCircleTangents(outerCenters(i - 1, :), r, pR.coords(end, :), 0);
            end
        else
            % If we are dealing with circle-circle
            if turns(i - 1) == 1 && turns(i) == 2
                % Right turn then left turn
                [~, ~, currR, ~] = circleCircleTangents(innerCenters(i - 1, :), r, outerCenters(i, :), r);
                [~, ~, currL, ~] = circleCircleTangents(outerCenters(i - 1, :), r, innerCenters(i, :), r);
            elseif turns(i - 1) == 2 && turns(i) == 1
                % Left turn then right turn
                [~, ~, ~, currL] = circleCircleTangents(innerCenters(i - 1, :), r, outerCenters(i, :), r);
                [~, ~, ~, currR] = circleCircleTangents(outerCenters(i - 1, :), r, innerCenters(i, :), r);
            elseif turns(i - 1) == 1 && turns(i) == 1
                % Right turn then right turn
                [currR, ~, ~, ~] = circleCircleTangents(innerCenters(i - 1, :), r, innerCenters(i, :), r);
                [currL, ~, ~, ~] = circleCircleTangents(outerCenters(i - 1, :), r, outerCenters(i, :), r);
            else
                % Left turn then left turn
                [~, currL, ~, ~] = circleCircleTangents(innerCenters(i - 1, :), r, innerCenters(i, :), r);
                [~, currR, ~, ~] = circleCircleTangents(outerCenters(i - 1, :), r, outerCenters(i, :), r);
            end
        end
        
        % Assign
        rightTangents{i} = currR;
        leftTangents{i} = currL;
    end
    
    % Plot tangents
    %for i = 1:length(rightTangents)
    %    plot(rightTangents{i}(:, 1), rightTangents{i}(:, 2),'r','LineWidth', 4);
    %    plot(leftTangents{i}(:, 1), leftTangents{i}(:, 2),'g','LineWidth', 4);
    %end
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % GET STRAIGHT REGIONS %
    %%%%%%%%%%%%%%%%%%%%%%%%
    straightRegions = Region.empty(2 * length(rightTangents), 0);
    for i = 1:length(rightTangents)
        % Create two separate triangles for each straight region. 
        %   1: 'insideStart -> insideEnd -> outsideEnd'
        %   2: 'insideStart -> outsideStart -> outsideEnd'
        insideStart = rightTangents{i}(1, :);
        insideEnd = rightTangents{i}(2, :);
        outsideStart = leftTangents{i}(1, :);
        outsideEnd = leftTangents{i}(2, :);

        % Initialize regions with coordinates
        straightRegions(2*i - 1) = Region([outsideStart; insideStart; insideEnd], 'g');
        straightRegions(2*i) = Region([outsideStart; outsideEnd; insideEnd], 'g');
        
        % Get the heading directions
        dir1 = vecNorm(insideEnd - insideStart);
        dir2 = vecNorm(outsideEnd - outsideStart);
        
        % Set the commanded heading
        straightRegions(2*i - 1).thetaCmd = atan2(dir1(2), dir1(1));
        straightRegions(2*i).thetaCmd = atan2(dir2(2), dir2(1));
    end
    
    
    %%%%%%%%%%%%%%%%%%%%
    % GET TURN REGIONS %
    %%%%%%%%%%%%%%%%%%%%
    turnRegions = Region.empty(3 * length(turns), 0);
    for i = 1:length(turns)
        
        % Get turn direction
        current = turns(i);
        opposite = (current == 1) * 2 + (current == 2) * 1; 
        
        % Get the vertices
        if turns(i) == 1
            % Right turn
            insideStart = rightTangents{i}(2, :);
            insideEnd = rightTangents{i + 1}(1, :);
            outsideStart = leftTangents{i}(2, :);
            outsideEnd = leftTangents{i + 1}(1, :);
        else
            % Left turn
            outsideStart = rightTangents{i}(2, :);
            outsideEnd = rightTangents{i + 1}(1, :);
            insideStart = leftTangents{i}(2, :);
            insideEnd = leftTangents{i + 1}(1, :);
        end
        
        % Make triangles
        turnRegions(3*i - 2) = Region([outsideStart; insideStart; P(opposite).coords(i + 1, :)], 'r');
        turnRegions(3*i - 1) = Region([insideStart; P(opposite).coords(i + 1, :); insideEnd], 'r');
        turnRegions(3*i) = Region([P(opposite).coords(i + 1, :); insideEnd; outsideEnd], 'r');
        
        % Set the commanded heading
        turnRegions(3*i - 2).thetaCmd = straightRegions(2*i + 1).thetaCmd;
        turnRegions(3*i - 1).thetaCmd = straightRegions(2*i + 1).thetaCmd;
        turnRegions(3*i).thetaCmd = straightRegions(2*i + 1).thetaCmd;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % CREATE TRIANGULATION %
    %%%%%%%%%%%%%%%%%%%%%%%%
    regions = Region.empty(length(straightRegions) + length(turnRegions), 0);
    for i = 1:length(rightTangents)
        j = 5*(i - 1);
        
        % For the first sequence, we don't have a turn
        if i ~= 1
            % Create objects
            regions(j - 2) = turnRegions(3*(i -1) - 2);
            regions(j - 1) = turnRegions(3*(i -1) - 1);
            regions(j) = turnRegions(3*(i -1));
        end
        
        % For each sequence, we have turn followed by straight
        regions(j + 1) = straightRegions(2*i - 1);
        regions(j + 2) = straightRegions(2*i);
    end
    for i = 1:size(regions, 2)
        regions(i).prevIndex = i - 1;
        regions(i).nextIndex = i + 1;
    end
    regions(1).prevIndex = 0;
    regions(end).nextIndex = NaN;
end