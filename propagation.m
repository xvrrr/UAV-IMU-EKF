% There is NOTHING you need to change in this file!
clear;
% load imu data and measurement data
imu_data = readtable("imu.csv");
l_imu = height(imu_data);
x_log = zeros(l_imu, 15);

% Initialize state vector and covariance matrix
x = zeros(15, 1);  % [euler, position, velocity, bg, ba]
P = eye(15);       % Initial covariance matrix

% Time step
dt = 0.005;
imu_idx = 1;
mea_idx = 1;

% EKF Loop
while imu_idx <= l_imu
    % Prediction Step
    acc = imu_data(imu_idx, 2:4).Variables';
    omega = imu_data(imu_idx, 5:7).Variables';
    [x, ] = predict(x, acc, omega, dt);
    x_log(imu_idx, :) = x';
    imu_idx = imu_idx + 1;
end

% Plot results (you can adapt this based on your specific state variables)
time = imu_data.time - imu_data.time(1);
figure("Name", "Imu propagation");
title_list = ["Euler Angle \psi", "Euler Angle \theta", "Euler Angle \phi", "Position x", "Position y", "Position z"];
plot_ord = [1, 3, 5, 2, 4, 6];
dim_ord = [7, 8, 9, 1, 2, 3];
for i=1:6
subplot(3, 2, plot_ord(i));
plot(time, x_log(:, dim_ord(i)), 'LineWidth', 2);
xlabel('Time');
if i > 3
    ylabel('Position (m)')
else
    ylabel('Angle (rad)')
end
title(title_list(i));
grid("on");
end