%% UCL MSc Robotics - ROS Network/Vrep RemoteAPI
% Author: Kevin Tchaka
% Date: 04/11/2016
% Summary:
% Script showing how to create a ros network and use it in conjunction with
% vrep's remoteApi to manipulate a KUKA Robot Arm.

clear all

%% Step 1. Initialize ros network nodes and remote vrep connection
% This script creates a simple ros network consisting of three
% nodes, three topics and three pairs of publishers and subscribers
SetUpNetwork
pause(7);

%% Step 2. Send set of joint positions to vrep node
% The 'SendJointTargetPosition' function takes as arguments the publisher
% to the '/poseSet' topic, and an array of all joints desired target
% positions. the 'setPosePub' publisher is created and defined in the
% 'SetUpNetwork' script. jointVals contains the desired joint's target
% positions.
MoveLBR800

%% Step 3. Send a trajectory to follow to the vrep node
% The 'Trajectory' function takes a matrix formed by a set of joint
% positions organized by rows and sent to the virtual devices
% chronologically to have it follow a set trajectory. The second argument
% of this function is the publisher to the '/poseSet' topic.
JPM = rand(100, 7);
Trajectory(JPM, setPosePub);

%% Step 4. Get Joint Position from vrep node
% this function takes as argument the publisher to the '/poseRequest' topic 
% then the subscribing node will publish the returned positions to the 
% '/poseReturn' topic. the result will be visible in the global variable 
% jointPos.
GetJointPosition(getPosePub);
pause(7);

%% Step 5. Free all ros and vrep variables
FreeNetwork
