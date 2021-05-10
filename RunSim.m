beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pathJsonFile = 'corridor.json';
plotMap = 1;
simSpeedup = 10000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run simulation with two at a time
pathsim = OptimalSimulator(pathJsonFile, plotMap);
pathsim.speedup = simSpeedup;
while ~pathsim.finished
    pathsim.propogate();
end
pathsim.writeLogFiles();

% Run simulation with two at a time
pathsim = Simulator(pathJsonFile, Triangulation.ConstantTurnRadius, plotMap);
pathsim.speedup = simSpeedup;
while ~pathsim.finished
    pathsim.propogate();
end
pathsim.writeLogFiles();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Send email
%sendEmail('files/login.ini', 'Simulation Finished', 'The simulation has finished succsefully');