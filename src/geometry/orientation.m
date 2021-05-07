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