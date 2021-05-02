beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
triangleJsonFile = 'triangleTest1.json';
pathJsonFile = 'curvedPath.json';

% Paramts
plotMap = 0;
simSpeedup = 10000;

% Path Simulator
pathsim = ClosestTriangulationSimulator(pathJsonFile, plotMap);
pathsim.speedup = simSpeedup;
while ~pathsim.finished
    pathsim.propogate();
end
pathsim.writeLogFiles();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize simulators.
%sim1 = ManualTriangleSimulator(triangleJsonFile, plotMap);
%while ~sim1.finished
%    sim1.propogate();
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%