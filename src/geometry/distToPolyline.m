function dMin = distToPolyline(path, pt)
%DISTTOPOLYLINE Summary of this function goes here
%   Detailed explanation goes here

    % Init min distance
    dMin = Inf;

    % Iterate through points
    for i = 2:size(path, 1)
        d = distToLineSegment(path(i - 1:i, :), pt);
        if d < dMin
            dMin = d;
        end
    end

end

