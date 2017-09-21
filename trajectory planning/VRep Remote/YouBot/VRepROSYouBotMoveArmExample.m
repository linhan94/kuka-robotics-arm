%% UCL MSc Robotics - ROS Network/Vrep RemoteAPI
% Author: Kevin Tchaka
% Date: 04/11/2016
% Summary:
% Script showing how to create a ros network and use it in conjunction with
% vrep's remoteApi to manipulate a YouBot.

clear all

%% Step 1. Initialize ros network nodes and remote vrep connection
% This script creates a simple ros network consisting of three
% nodes, three topics and three pairs of publishers and subscribers
YouBotSetUp

%% Step 2. Send a trajectory to follow to the vrep node
% The 'Trajectory' function takes a matrix formed by a set of joint
% positions organized by rows and sent to the virtual devices
% chronologically to have it follow a set trajectory. The second argument
% of this function is the publisher to the '/poseSet' topic.
jointData = load('youbot_joint_data.mat');
LGripper = [linspace(0, 0.02, 100)'; linspace(0.02, 0, 100)'];
RGripper = [linspace(0, -0.02, 100)'; linspace(-0.02, 0, 100)'];
GripperData = [LGripper, RGripper];
GripperData = repmat(GripperData, [4, 1]);
GripperData = GripperData(1:length(jointData.joint_profile), :);
jointData.data = [jointData.joint_profile, GripperData];
jointData.data(1:end, 1:5) = 0; %We set this to zeros just to see the gripper motion.
JPM = jointData.joint_profile;
clear jointData;
Trajectory(JPM, setPoseArmPub);

%% Step 3. Get Joint Position from vrep node
% this function takes as argument the publisher to the '/poseRequest' topic 
% then the subscribing node will publish the returned positions to the 
% '/poseReturn' topic. the result will be visible in the global variable 
% jointPos.
GetJointPosition(getArmPosePub);
pause(7);

%% Step 5. Free all ros and vrep variables
FreeNetwork

clear armJointPos wheelJointPos armJoints wheelJoints;
clear JPM ans res;
