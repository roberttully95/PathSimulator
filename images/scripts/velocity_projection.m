axis equal;
hold on;

% Create a velocity vector
v = [sqrt(2)/2, sqrt(2)/2];
quiver(v(1), v(2), 'LineWidth', 2, 'Color', 'b');

% Create the flow arrow
q = [1, 0.2]/norm([1, 0.2]);
quiver(q(1), q(2), 'LineWidth', 2, 'Color', 'r');

% Angle between
thetaDeg = acosd(max(min(dot(q,v)/(norm(q)*norm(v)),1),-1));

% Projection
p = dot(q, v)/(norm(v)^2)*q;
quiver(p(1), p(2), 'LineWidth', 2, 'Color', 'g');

% dashed line
p1 = 1 + 0.9*v;
p2 = 1 + 0.9*p;
plot([p1(1), p2(1)], [p1(2), p2(2)], 'Color', 'k', 'LineStyle', '--');

% Add Text
text(1.75, 1.2, 'Q', 'Color','red','FontSize',14);
text(1.375, 1.13, 'proj_{Q}V', 'Color','g','FontSize',14);
text(1.3,  1.4, 'V', 'Color','blue','FontSize',14);

% Add angle
text(1.15,  1.1, '\theta', 'FontSize', 24);