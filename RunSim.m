beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
%triangleJsonFile = 'triangleTest1.json';
pathJsonFile = 'curvedPath.json';

% Parameters
plotMap = 1;
simSpeedup = 10;

% Path Simulator
pathsim = ClosestTriangulationSimulator(pathJsonFile, plotMap);
pathsim.speedup = simSpeedup;
while ~pathsim.finished
    pathsim.propogate();
end
pathsim.writeLogFiles();

% Send email
%sendEmail('files/login.ini', 'Simulation Finished', 'The simulation has finished succsefully');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize simulators.
%sim1 = ManualTriangleSimulator(triangleJsonFile, plotMap);
%while ~sim1.finished
%    sim1.propogate();
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%