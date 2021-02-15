beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
jsonFile = 'jsonTest1.json';

% Initialize simulator.
sim = SimulatorExample(jsonFile, gca);
while ~sim.finished
    sim.propogate();
    pause(sim.dT);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%