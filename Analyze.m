beep off

% Specify the name of the simulation we are analyzing
simUnderTest = 'corridor';

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
initCollisionEvents = sum(vehicleData.DURATION == 0 & vehicleData.END_CONDITION == 1);
if initCollisionEvents > 0
    warning("Collision events occurred during vehicle initialization!")
end

%%%%%%%%%%%
% ANALYZE %
%%%%%%%%%%%

% Get the percentage of vehicles that made it to the end
numSuccess = sum(vehicleData.END_CONDITION == 0);
successRate = numSuccess / height(vehicleData);
fprintf("Success Rate: %.2f %%\n", successRate * 100);

% Plot the times taken
subplot(3, 1, 1)
durationDist = fitdist(vehicleData.DURATION, 'Normal');
x = (durationDist.mu - 4*durationDist.sigma):(durationDist.sigma/100):(durationDist.mu + 4*durationDist.sigma);
y = normpdf(x, durationDist.mu, durationDist.sigma);
plot(x, y)
xline(durationDist.mu, 'g', 'LineWidth', 2);
title('Vehicle Duration Distribution')
xlim([x(1), x(end)])
xlabel('Duration (s)')
ylabel('Probability Density')

% Plot the average vehicle velocities
subplot(3, 1, 2)
avgVelocityDist = fitdist(vehicleData.DIST_TRAVELLED ./ vehicleData.DURATION , 'Normal');
x = 0:(avgVelocityDist.mu/500):2*avgVelocityDist.mu;
y = normpdf(x, avgVelocityDist.mu, avgVelocityDist.sigma);
plot(x, y)
xline(avgVelocityDist.mu, 'g', 'LineWidth', 2);
title('Average Vehicle Velocity Distribution')
xlim([x(1), x(end)])
xlabel('Velocity (m / s)')
ylabel('Probability Density')

% Plot the average vehicle distance travelled
subplot(3, 1, 3)
avgDistTravelled = fitdist(vehicleData.DIST_TRAVELLED, 'Normal');
x = (avgDistTravelled.mu - 4*avgDistTravelled.sigma):(avgDistTravelled.sigma/100):(avgDistTravelled.mu + 4*avgDistTravelled.sigma);
y = normpdf(x, avgDistTravelled.mu, avgDistTravelled.sigma);
plot(x, y)
xline(avgDistTravelled.mu, 'g', 'LineWidth', 2);
title('Distance Travelled Distribution')
xlim([x(1), x(end)])
xlabel('Distance (m)')
ylabel('Probability Density')

