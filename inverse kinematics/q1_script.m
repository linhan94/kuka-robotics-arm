% Close all open figures and clear all data
clc;
clear;
close all;

% Load the trajectory for 4R-planar manipulator
load('data_q1.mat');

number_of_data = size(planar_traj, 1);

%Initialise the error vector
ERROR_CCD = zeros(1, number_of_data);
ERROR_jacob = zeros(1, number_of_data);
ERROR_ikine = zeros(1, number_of_data);

%Specify the length
L = 1;

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link([0, 0, L, 0, 0], 'standard');
L2 = Link([0, 0, L, 0, 0], 'standard');
L3 = Link([0, 0, L, 0, 0], 'standard');
L4 = Link([0, 0, L, 0, 0], 'standard');

%Construct the robot
planar4Rrobot = SerialLink([L1, L2, L3, L4]);

% Inverse kinematics - CCD
tic;
theta_CCD = inverse_kine_CCD(planar_traj, planar4Rrobot); 
cost_CCD = toc;

% Inverse kinematics - Iterative method
tic;
theta_jacob = inverse_kine_jacobian(planar_traj, planar4Rrobot, 'q1');
cost_jacob = toc;

% Inverse kinematics - Iterative method (Peter Corke's toolbox)
tic;
theta_ikine = zeros(number_of_data, 4); %Comment this line
% Use identity matrix as the orientation for the whole trajectory and build
% the transformations for ikine function using transl. You may add more 
% options to make the solver more stable, but you also need to justify your 
% options as well.
cost_ikine = toc;

T_EST_CCD = planar4Rrobot.fkine(theta_CCD);
T_EST_jacob = planar4Rrobot.fkine(theta_jacob);
T_EST_ikine = planar4Rrobot.fkine(theta_ikine);

%Currently this function returns a vector of random numbers. Implement the
%function to calculate pose error.
[ERROR_CCD, ~] = calculate_pose_error(T_EST_CCD, planar_traj);
[ERROR_jacob, ~] = calculate_pose_error(T_EST_jacob, planar_traj);
[ERROR_ikine, ~] = calculate_pose_error(T_EST_ikine, planar_traj);

disp('Printing trajectory for CCD method....');
%Plot the desired path and expected path together.
figure(1)
plot3(planar_traj(:, 1), planar_traj(:, 2), zeros(number_of_data, 1));
view(0, 90);
hold on
planar4Rrobot.plot(theta_CCD, 'trail', 'r-')
hold off

disp('Printing trajectory for jacobian method....');
%Plot the desired path and expected path together.
figure(2)
plot3(planar_traj(:, 1), planar_traj(:, 2), zeros(number_of_data, 1));
view(0, 90);
hold on
planar4Rrobot.plot(theta_jacob, 'trail', 'r-')
hold off

disp('Printing trajectory for ikine method....');
%Plot the desired path and expected path together.
figure(3)
plot3(planar_traj(:, 1), planar_traj(:, 2), zeros(number_of_data, 1));
view(0, 90);
hold on
planar4Rrobot.plot(theta_ikine, 'trail', 'r-')
hold off

%Plot error
figure(4),
plot(1:number_of_data, ERROR_CCD, 'r:', 1:number_of_data, ERROR_jacob, 'g:', 1:number_of_data, ERROR_ikine, 'b:', 'LineWidth', 2), 
title('Euclidean error between estimation and ground truth'), grid on,
xlabel('data number'), ylabel('error (metre)'), legend('CCD', 'Jacobian', 'Robotics Toolbox Solver')

fprintf('Computational cost of CCD algorithm: %.4f, Average error: %.4f\n', cost_CCD, mean(ERROR_CCD));
fprintf('Computational cost of Jacobian method: %.4f, Average error: %.4f\n', cost_jacob, mean(ERROR_jacob));
fprintf('Computational cost of ikine function: %.4f, Average error: %.4f\n', cost_ikine, mean(ERROR_ikine));