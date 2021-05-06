function [P1,P2] = vectorCircleIntersection(VO,VD, C, R)
%VECTORCIRCLEINTERSECTION Determines the points of intersection between a
%vector and a circle.
%   INPUTS:
%      VO: The origin of the vector
%      VD: The direction of the vector
%       C: The center of the circle.
%       R: The radius of the circle
%
%   OUTPUTS:
%      P1: The intersection pt farthest away from the vector origin.
%      P2: The intersection pt closest to the vector origin.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Get line joining vector origin and circle center
    OM = VO - C;

    B = 2 * dot(VD, OM);
    C = dot(OM, OM) - R^2;
    
    Q = sqrt(B^2 - 4*C)/2;
    B = (-B/2);
    
    P1 = VD * (B + Q) + VO;
    P2 = VD * (B - Q) + VO;
end

