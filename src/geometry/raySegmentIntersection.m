function pt = raySegmentIntersection(origin, dir, line)
%RAYSEGMENTINTERSECTION Summary of this function goes here
%   Detailed explanation goes here

    v1 = origin - line(1, :);
    v2 = line(2, :) - line(1, :);
    v3 = [-dir(2), dir(1)];
    
    dotProd = dot(v2, v3);
    if abs(dotProd) < 1e-6
        pt = [NaN, NaN];
        return
    end
    
    t1 = cross2d(v2, v1) / dotProd;
    t2 = dot(v1, v3) / dotProd;
    
    if (t1 >= 0 && (t2 >= 0 && t2 <= 1))
        pt = origin + t1 * dir;
        return;
    end
    
    pt = [NaN, NaN];
end

