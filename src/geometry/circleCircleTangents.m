function [LL,RR, LR, RL] = circleCircleTangents(c1, r1, c2, r2)
%CIRCLECIRCLETANGENTS Summary of this function goes here
%   Detailed explanation goes here

    % Compute cross tangent s-t
    c21 = c2 - c1;
    d = norm(c2 - c1);
    rSum = r1 + r2;
    c21 = c21/d;
    
    % Inner Tangents
    if rSum >= d
        warning('Not possible to generate inner tangents!');
        LR = [NaN, NaN; NaN, NaN];
        RL = [NaN, NaN; NaN, NaN];
    else
        cInner = rSum/d;
        QLR = [c21', [-c21(2); c21(1)]]; 
        QxLR = QLR*[cInner; sqrt(1 - cInner^2)];
        LR1 = c1' + QxLR*r1;
        LR2 = c2' - QxLR*r2;
        LR = [LR1'; LR2'];

        QRL = [c21', [c21(2); -c21(1)]]; 
        QxRL = QRL*[cInner; sqrt(1 - cInner^2)];
        RL1 = c1' + QxRL*r1;
        RL2 = c2' - QxRL*r2;
        RL = [RL1'; RL2'];
    end
    
    
    % Outer Tangents
    cOuter = (r1 - r2)/d;
    QRR = [c21', [c21(2); -c21(1)]]; 
    QxRR = QRR*[cOuter; sqrt(1 - cOuter^2)];
    RR1 = c1' + QxRR*r1;
    RR2 = c2' + QxRR*r2;
    RR = [RR1'; RR2'];
    
    QLL = [c21', [-c21(2); c21(1)]]; 
    QxLL = QLL*[cOuter; sqrt(1 - cOuter^2)];
    LL1 = c1' + QxLL*r1;
    LL2 = c2' + QxLL*r2;
    LL = [LL1'; LL2'];
    
    %{
    % need to plot circle
    phi = linspace(0,2*pi,361);
    cs = cos(phi);
    ss = sin(phi);
    plotcirclefun = @(xy,r) plot(xy(1)+r*cs, xy(2)+r*ss, 'k', 'linewidth', 3);
    
    % Graphical output
    figure(1)
    plotcirclefun(c1', r1);
    plotcirclefun(c2', r2);
    plot([LR1(1) LR2(1)], [LR1(2) LR2(2)], 'k', 'linewidth', 3);
    plot([RL1(1) RL2(1)], [RL1(2) RL2(2)], 'k', 'linewidth', 3);
    plot([RR1(1) RR2(1)], [RR1(2) RR2(2)], 'k', 'linewidth', 3);
    plot([LL1(1) LL2(1)], [LL1(2) LL2(2)], 'k', 'linewidth', 3);
    %}
end

