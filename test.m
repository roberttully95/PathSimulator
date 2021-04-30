%%%%%%%%%%%%%%%%%%%%%
% Define parameters %
%%%%%%%%%%%%%%%%%%%%%
dT = 0.1;
tMax = 40;
Kp = Inf;
Ki = 0;
Kd = 0;
thDotMax = pi/4;
%%%%%%%%%%%%%%%%%%%%%
hold on;

% Define time series
t = 0:dT:tMax;

% Create controller
controller = HeadingController(dT, Kp, Ki, Kd, thDotMax);

% Initialize theta dot and theta
thDelta = NaN(1, length(t));
th = NaN(1, length(t));
thDelta(1) = 0;
th(1) = 0;

% Iterate through time
for i = 2:length(t)
    thDelta(i) = controller.calculate(pi, th(i - 1));
    th(i) = th(i - 1) + thDelta(i);
end

% Plot the target
yline(pi, 'g');

% Plot
plot(t, thDelta, 'b');
plot(t, th, 'r');