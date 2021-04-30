function [dist, proj] = distToLineSegment(line, p)
%DISTTOLINESEGMENT Determines the shortest distance betweeen a point and a
%line segment.

    % Define params
    v = line(1, :);
    w = line(2, :);
    
    % Get line length
    l2 = norm(v - w)* norm(v-w);
    
    % Deal with the instance in which the line is a point.
    if l2 == 0
        dist = norm(v - p);
        return;
    end

    % Consider the line extending the segment, parameterized as v + t (w - v).
    % We find projection of point p onto the line. 
    % It falls where t = [(p-v) . (w-v)] / |w-v|^2
    % We clamp t from [0,1] to handle points outside the segment vw.
    t = max(0, min(1, dot(p - v, w - v) / l2));
    proj = v + t * (w - v);
    
    dist = norm(p - proj);
end