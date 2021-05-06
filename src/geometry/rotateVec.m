function vec = rotateVec(vec, th)
%ROTATEVEC Rotates a vector by a specified angle

    vec = ([cos(th), -sin(th); sin(th), cos(th)] * vec')';
end

