beep off
addpath(genpath("src"))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify file location
jsonFile = 'jsonTest1.json';

% Initialize simulator.
sim = SimulatorExample(jsonFile);
while ~sim.isFinished()
    sim.propogate();
    pause(sim.dT);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%