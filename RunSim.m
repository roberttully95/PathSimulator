beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pathJsonFile = 'curvedPath.json';
%triangulation = Triangulation.Closest;
triangulation = Triangulation.ConstantVelocity;
plotMap = 1;
simSpeedup = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pathsim = Simulator(pathJsonFile, triangulation, plotMap);
pathsim.speedup = simSpeedup;
while ~pathsim.finished
    pathsim.propogate();
end
pathsim.writeLogFiles();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Send email
%sendEmail('files/login.ini', 'Simulation Finished', 'The simulation has finished succsefully');