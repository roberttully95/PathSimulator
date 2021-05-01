beep off

% Specify the name of the simulation we are analyzing
simUnderTest = 'curvedPath';

% Generate the path to the time and vehicle files
timeLogFile = append(cd, '\logs\', simUnderTest, '\time.xls');
vehicleLogFile = append(cd, '\logs\', simUnderTest, '\vehicles.xls');