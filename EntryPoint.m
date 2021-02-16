beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
triangleJsonFile = 'triangleTest1.json';
pathJsonFile = 'curvedPath.json';

% Path Simulator
pathsim = SimulatorExample(pathJsonFile, 1);
pathsim.speedup = 10;
while ~pathsim.finished
    pathsim.propogate();
end

% Initialize simulators.
sim1 = ManualTriangleSimulator(triangleJsonFile, 1);
while ~sim1.finished
    sim1.propogate();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%