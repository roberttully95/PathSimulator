beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
triangleJsonFile = 'triangleTest1.json';
pathJsonFile = 'curvedPath.json';

% PARAMS
plotMap = 0;

% Path Simulator
pathsim = SimulatorExample(pathJsonFile, plotMap);
pathsim.speedup = 10;
while ~pathsim.finished
    pathsim.propogate();
end

% Initialize simulators.
%sim1 = ManualTriangleSimulator(triangleJsonFile, plotMap);
%while ~sim1.finished
%    sim1.propogate();
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%