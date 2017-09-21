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
JPM = jointData.data;
clear jointData;
Trajectory(JPM, setPosePub);

%% Step 3. Get Joint Position from vrep node
% this function takes as argument the publisher to the '/poseRequest' topic 
% then the subscribing node will publish the returned positions to the 
% '/poseReturn' topic. the result will be visible in the global variable 
% jointPos.
GetJointPosition(getPosePub);
pause(7);

%% Step 5. Free all ros and vrep variables
FreeNetwork

clear jointPos armJoints wheelJoints;
clear JPM ans res;
