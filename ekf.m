clear;
% Arrays to store estimates
x_estimates = [];

% load imu data and measurement data
imu_data = readtable("imu.csv");
measurement_data = readtable("measurement.csv");

l_imu = height(imu_data);
l_mea = height(measurement_data);

% Initialize state vector and covariance matrix
x = zeros(15, 1);           % state vector: [position, velocity, euler, gyro bias, acc bias]
P = eye(15);                % Initial covariance matrix

% Define process and measurement noise covariances
cov_w = 0.01;
cov_a = 0.01;
cov_bg = 0.01;
cov_ba = 0.01;
sig_w = diag([cov_a, cov_a, cov_a, cov_w, cov_w, cov_w, cov_bg, cov_bg, cov_bg, cov_ba, cov_ba, cov_ba]);    % Process noise covariance
R = diag([0.01, 0.01, 0.01]);       % Measurement noise covariance

% Define the measurement function (h)
H = zeros([3, 15]);
H(1:3, 1:3) = eye(3);

% Time step
dt = 0.005;

% EKF Loop
imu_idx = 1;
mea_idx = 1;
while imu_idx <= l_imu && mea_idx <= l_mea
    if imu_data.time(imu_idx) < measurement_data.time(mea_idx)
        % Prediction Step
        acc = imu_data(imu_idx, 2:4).Variables';
        omega = imu_data(imu_idx, 5:7).Variables';
        [x, Fx, Fw] = predict(x, acc, omega, dt);
%*******************************************************************************************
% **************************** WRITE YOUR CODE HERE ****************************************
% Predict your covariance estimate here with matrix Fx, Fw, P and sig_w
        % P: The covariance of state vector, which is an [15 x 15] matrix
        % Fx: The Jacobian of state transform function by state, which is an [15 x 15] matrix
        % Fw: The Jacobian of state transform function by noise, which is an [12 x 12] matrix
        % sig_w: The covariance of Noise, which is an [12 x 12] matrix

        P = Fx*P*Fx'+Fw*sig_w*Fw';
        % tips: A code like [P = A * P] where P both exsits on the left and right side of equation is ok!
% **************************** WRITE YOUR CODE HERE ****************************************
%*******************************************************************************************
        imu_idx = imu_idx + 1;
    else
        % Update Step (Measurement Update)
        z = measurement_data(mea_idx, 2:4).Variables';
%*******************************************************************************************
% **************************** WRITE YOUR CODE HERE ****************************************
% calculate the kalman Gain, updated state, and updated state covariance here
        % H: the measurement matrix, which is an [3 x 15] matrix
        % x: the state, which is an [15 x 1] vector
        % P: The covariance of state vector, which is an [15 x 15] matrix
        % Fx: The Jacobian of state transform function by state, which is an [15 x 15] matrix
        % Fw: The Jacobian of state transform function by noise, which is an [12 x 12] matrix
        % sig_w: The covariance of Noise, which is an [12 x 12] matrix
        % R: The covariance of measurement Noise, which is an [3 x 3] matrix
        
        K = P*H'/(H*P*H'+R);
        x = x+K*(z-H*x);
        P = (eye(15)-K*H)*P;
        % tips: A code like [P = A * P] where P both exsits on the left and right side of equation is ok!
% **************************** WRITE YOUR CODE HERE ****************************************
%*******************************************************************************************
        mea_idx = mea_idx + 1;
        % Store the estimates and measurements
        x_estimates = [x_estimates; x'];
    end
end

% Plot results (you can adapt this based on your specific state variables)
time = measurement_data.time - measurement_data.time(1);
figure("Name", "Measurements");
plot(measurement_data.time - measurement_data.time(1), [measurement_data.position_x, measurement_data.position_y, measurement_data.position_z], 'LineWidth', 2);
grid("on");
legend("x", "y", "z");
xlabel("Time (s)");
ylabel("Position (m)");
figure("Name", "Estimation");
title_list = ["Euler Angle \psi", "Euler Angle \theta", "Euler Angle \phi", "Position x", "Position y", "Position z"];
plot_ord = [1, 3, 5, 2, 4, 6];
dim_ord = [7, 8, 9, 1, 2, 3];
for i=1:6
    subplot(3, 2, plot_ord(i));
    plot(time, x_estimates(:, dim_ord(i)), 'LineWidth', 2);
    xlabel('Time');
    title(title_list(i));
    grid("on");
        if i > 3
        hold on
        plot(time, measurement_data(:,i-2).Variables, '--', 'LineWidth', 2);
        ylabel('Position (m)')
        legend("Estimation", "Measurement");
    else
        ylabel('Angle (rad)')
        legend("Estimation");
    end
end
figure("Name", "Bias & g");
subplot(2, 1, 1);
title("b_g")
plot(time, x_estimates(:, 13:15), 'LineWidth', 2);
grid("on");
legend("x", "y", "z");
xlabel("Time (s)");
ylabel("Gyro Bias (rad/s)");
subplot(2, 1, 2);
title("b_a");
plot(time, x_estimates(:, 10:12), 'LineWidth', 2);
grid("on");
legend("x", "y", "z");
xlabel("Time (s)");
ylabel("Acc Bias (m/s^2)");