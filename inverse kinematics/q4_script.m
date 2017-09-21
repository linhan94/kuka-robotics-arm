% Close all open figures and clear all data
clc;
clear;
close all;

% Load the trajectory for 4R-planar manipulator
load('data_q4.mat');

% Trajectories data are in this form: (x, y, z, rx, ry, rz). You can import
% the positions (x, y, z) directly to the transformations. For rotation
% matrix, you need to use rodrigues function, e.g. rodrigues([rx, ry, rz]).
% It will give you a rotation matrix for a transformation.
number_of_data = size(youbot_traj, 1);

%Initialise the error vector 
%(first row: translation error, second row: rotation error)
ERROR_closedform = zeros(2, number_of_data);
ERROR_jacob = zeros(2, number_of_data);
ERROR_ikine = zeros(2, number_of_data);

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);

%Construct the robot
Youbot = SerialLink([L1, L2, L3, L4, L5]);

% Inverse kinematics - Closed form method
tic;
theta_closedform = inverse_kine_closedform(youbot_traj, Youbot, 'q4');
cost_closedform = toc;

% Inverse kinematics - Iterative method
tic;
theta_jacob = inverse_kine_jacobian(youbot_traj, Youbot, 'q4');
cost_jacob = toc;

% Inverse kinematics - Iterative method (Peter Corke's toolbox)
tic;
theta_ikine = zeros(number_of_data, 5); %Comment this line
cost_ikine = toc;

T_EST_closedform = Youbot.fkine(theta_closedform);
T_EST_jacob = Youbot.fkine(theta_jacob);
T_EST_ikine = Youbot.fkine(theta_ikine);

%Currently this function returns a vector of random numbers. Implement the
%function to calculate pose error.
[ERROR_closedform(1, :), ERROR_closedform(2, :)] = calculate_pose_error(T_EST_closedform, youbot_traj);
[ERROR_jacob(1, :), ERROR_jacob(2, :)] = calculate_pose_error(T_EST_jacob, youbot_traj);
[ERROR_ikine(1, :), ERROR_ikine(2, :)] = calculate_pose_error(T_EST_ikine, youbot_traj);

disp('Printing trajectory for closed form method....');
%Plot the desired path and expected path together. (jacobian)
figure(1)
plot3(youbot_traj(:, 1), youbot_traj(:, 2), youbot_traj(:, 3));
view(0, 90);
hold on
Youbot.plot(theta_closedform, 'trail', 'r-')
hold off

disp('Printing trajectory for Jacobian method....');
%Plot the desired path and expected path together. (jacobian)
figure(2)
plot3(youbot_traj(:, 1), youbot_traj(:, 2), youbot_traj(:, 3));
view(0, 90);
hold on
Youbot.plot(theta_jacob, 'trail', 'r-')
hold off

disp('Printing trajectory for ikine method....');
%Plot the desired path and expected path together. (ikine)
figure(3)
plot3(youbot_traj(:, 1), youbot_traj(:, 2), youbot_traj(:, 3));
view(0, 90);
hold on
Youbot.plot(theta_ikine, 'trail', 'r-')
hold off

%Plot error
figure(4),
plot(1:number_of_data, ERROR_closedform(1, :), 'r:', 1:number_of_data, ERROR_jacob(1, :), 'g:', 1:number_of_data, ERROR_ikine(1, :), 'b:', 'LineWidth', 2), 
title('Euclidean error between estimation and ground truth'), grid on,
xlabel('data number'), ylabel('error (metre)'), legend('Closed form', 'Jacobian', 'Robotics Toolbox Solver')

%Plot error
figure(5),
plot(1:number_of_data, ERROR_closedform(2, :), 'r:', 1:number_of_data, ERROR_jacob(2, :), 'g:', 1:number_of_data, ERROR_ikine(2, :), 'b:', 'LineWidth', 2), 
title('Rotation error between estimation and ground truth'), grid on,
xlabel('data number'), ylabel('error (deg)'), legend('Closed form', 'Jacobian', 'Robotics Toolbox Solver')

fprintf('Computational cost of Jacobian method: %.4f, Average error (translation): %.4f, Average error (rotation): %.4f\n', cost_closedform, mean(ERROR_closedform(1, :)), mean(ERROR_closedform(2, :)));
fprintf('Computational cost of Jacobian method: %.4f, Average error (translation): %.4f, Average error (rotation): %.4f\n', cost_jacob, mean(ERROR_jacob(1, :)), mean(ERROR_jacob(2, :)));
fprintf('Computational cost of ikine function: %.4f, Average error (translation): %.4f, Average error (rotation): %.4f\n', cost_ikine, mean(ERROR_ikine(1, :)), mean(ERROR_ikine(2, :)));