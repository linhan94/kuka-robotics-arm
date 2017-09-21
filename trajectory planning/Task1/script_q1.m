clc;
clear;
close all;

load('data_q1.mat');

%Implement these three functions. These functions should output N x 5
%matrices. Note that N depends on the number of sample between each given
%pose which you have to tune properly. When N is too high, you are 
%oversampling the trajectory which may give you bad memory
%allocation in real YouBot system and if it is too low, the generated
%motion can be jerky. We are going to run the trajectory in the YouBot next 
%year. You can try running your trajectory using the code we have given you 
%in the last coursework in VREP or use the Robotics Toolbox. 

[joint_q1a_data, velocity_q1a] = create_trajectory_1a(data_q1a);
[joint_q1b_data, velocity_q1b] = create_trajectory_1b(data_q1b);
[joint_q1c_data, velocity_q1c] = create_trajectory_1c(data_q1c);

joint_profile = [joint_q1c_data];% joint_q1b_data; joint_q1c_data];
velo_profile = [velocity_q1c];% velocity_q1b; velocity_q1c];

save('youbot_joint_data.mat', 'joint_profile');
save('youbot_velocity_data.mat', 'velo_profile');