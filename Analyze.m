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

% Count the number of each type of collision
numV2Vcollisions = sum(vehicleData.END_CONDITION == 1);
numV2Wallcollisions = sum(vehicleData.END_CONDITION == 2);
numSuccess = sum(vehicleData.END_CONDITION == 0);
fprintf("%.0f vehicle-to-vehicle collisions involving %.0f cars\n", numV2Vcollisions / 2, numV2Vcollisions);
fprintf("%.0f vehicle-to-map collisions\n", numV2Wallcollisions);
fprintf("%.0f vehicle exited the map safely for a success rate of %0.2f%%\n", numSuccess, 100 * numSuccess / height(vehicleData));

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

