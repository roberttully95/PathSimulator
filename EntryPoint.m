beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
triangleJsonFile = 'triangleTest1.json';
pathJsonFile = 'pathTest1.json';

% Initialize simulators.
sim1 = ManualTriangleSimulator(triangleJsonFile, 1);
while ~sim1.finished
    sim1.propogate();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%