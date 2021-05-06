function bisector = vectorBisect(v1, v2)
%VECTORBISECT Bisects two input vectors and returns the direction vector of
%the bisector.

    % Normalize
    v1 = vecNorm(v1);
    v2 = vecNorm(v2);

    % Get the bisector
    bisector = vecNorm((v1 + v2)/2);
end

