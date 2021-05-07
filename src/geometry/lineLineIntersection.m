function intPt = lineLineIntersection(line1, line2)
%LINELINEINTERSECTION Summary of this function goes here
%   Detailed explanation goes here

    % Extract
    L1_x1 = line1(1, 1);
    L1_y1 = line1(1, 2);
    L1_x2 = line1(2, 1);
    L1_y2 = line1(2, 2);
    L2_x1 = line2(1, 1);
    L2_y1 = line2(1, 2);
    L2_x2 = line2(2, 1);
    L2_y2 = line2(2, 2);

    % Compute several intermediate quantities
    Dx12 = L1_x1-L1_x2;
    Dx34 = L2_x1-L2_x2;
    Dy12 = L1_y1-L1_y2;
    Dy34 = L2_y1-L2_y2;
    Dx24 = L1_x2-L2_x2;
    Dy24 = L1_y2-L2_y2;

    % Solve for t and s parameters
    ts = [Dx12 -Dx34; Dy12 -Dy34] \ [-Dx24; -Dy24];

    % Take weighted combinations of points on the line
    intPt = (ts(1)*[L1_x1; L1_y1] + (1-ts(1))*[L1_x2; L1_y2])';
end

