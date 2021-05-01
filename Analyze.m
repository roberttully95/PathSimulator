beep off

% Specify the name of the simulation we are analyzing
simUnderTest = 'curvedPath';

%%%%%%%%%%%%%
% LOAD DATA %
%%%%%%%%%%%%%

% Generate the path to the time and vehicle files
timeLogFile = append(cd, '\logs\', simUnderTest, '\time.xls');
vehicleLogFile = append(cd, '\logs\', simUnderTest, '\vehicles.xls');

% Read data into excel tables
timeData = readtable(timeLogFile);
vehicleData = readtable(vehicleLogFile);

%%%%%%%%%%%%%%
% Preprocess %
%%%%%%%%%%%%%%

% VEHICLE DATA

% Step 1: Remove vehicles with no duration and the vehicles they collided with
selfIndices = find(vehicleData.DURATION == 0 & vehicleData.END_CONDITION == 1);
collIndices = vehicleData{selfIndices, 'COLLIDED_WITH'};
combined = [selfIndices; collIndices];
vehicleData(combined, :) = [];

% Step 2: Remove vehicles with no duration that collided with wall
collIndices = find(vehicleData.DURATION == 0);
vehicleData(collIndices, :) = [];

%%%%%%%%%%%
% ANALYZE %
%%%%%%%%%%%

% Get the percentage of vehicles that made it to the end
numSuccess = sum(vehicleData.END_CONDITION == 0);
successRate = numSuccess / height(vehicleData);
fprintf("Success Rate: %.2f %%\n", successRate * 100);

% Plot the times taken
subplot(2, 1, 1)
durationDist = fitdist(vehicleData.DURATION, 'Normal');
x = (durationDist.mu - 4*durationDist.sigma):(durationDist.sigma/100):(durationDist.mu + 4*durationDist.sigma);
y = normpdf(x, durationDist.mu, durationDist.sigma);
plot(x, y)
xline(durationDist.mu, 'g', 'LineWidth', 2);
title('Vehicle Duration Distribution')
xlabel('Duration (s)')
ylabel('Probability Density')

% Plot the average vehicle velocities
subplot(2, 1, 2)
avgVelocityDist = fitdist(vehicleData.DIST_TRAVELLED ./ vehicleData.DURATION , 'Normal');
x = (avgVelocityDist.mu - 4*avgVelocityDist.sigma):(avgVelocityDist.sigma/100):(avgVelocityDist.mu + 4*avgVelocityDist.sigma);
y = normpdf(x, avgVelocityDist.mu, avgVelocityDist.sigma);
plot(x, y)
xline(avgVelocityDist.mu, 'g', 'LineWidth', 2);
title('Average Vehicle Velocity Distribution')
xlabel('Velocity (m / s)')
ylabel('Probability Density')