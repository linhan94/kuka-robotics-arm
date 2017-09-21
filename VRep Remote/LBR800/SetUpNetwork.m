%% UCL MSc Robotics - Set Up ROS Network
% Author: kevin Tchaka
% Date: 26/10/2016
% Summary:
% Script showing how to create a simple ros network and set up vrep remote
% API connection

%% Vrep Remote Connection

%this global variable will come in handy when receiving the position from
%the Kuka later
global jointPos;


% Open connection to vrep remote API
vrep = remApi('remoteApi'); % create remote vrep handle
vrep.simxFinish(-1); % close remote simulation just-in-case
% open connection to vrep remote service, this function takes as arguments:
% - the ip address to connect to vrep server
% - the remote port to connect to
% - boolean to indicate whether to block until connected
% - boolean to indicate whether to try to reconnect if connection is lost
% (if true it won't attempt it)
% - time out in milliseconds
% - thread cycle in milliseconds, indicates how often packets are sent over
% the network (default value of 5 recommended by Vrep)
% check vrep config for remote port (default should be 19997, might be
% disabled though in which it will need to be enabled from vrep)
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5); 

% Load the vrep scene file
res = vrep.simxLoadScene(clientID,'sceneExampleKuka7.ttt',1,vrep.simx_opmode_blocking);

%Set joint Target Velocity
SetTargetVelocity(clientID, vrep, 0);

%% ROS Network initialization

%start matlab ros service and create master node
rosinit;

%set up basic nodes and masterHost
masterHost = 'localhost';
userNode = robotics.ros.Node('userNode', masterHost);
vKukaNode = robotics.ros.Node('vKukaNode', masterHost);

%% set-up publishers and subscribers for '/poseSet' topic

%create publisher associate it to node, topic and define message type
%(float array here)
setPosePub = robotics.ros.Publisher(userNode, '/poseSet', 'std_msgs/Float64MultiArray');

%create subscriber associate it with listening node, topic and set callback
%function
setPoseSub = robotics.ros.Subscriber(vKukaNode, '/poseSet', {@MoveJointsCallBack, clientID, vrep});

%% set-up publishers and subscribers for '/poseReturn' topic

%create publisher associate it to node, topic and define message type
%(float array here)
% these should be left untouched.
returnPosePub = robotics.ros.Publisher(vKukaNode, '/poseReturn', 'std_msgs/Float64MultiArray');
returnPoseSub = robotics.ros.Subscriber(userNode, '/poseReturn', @GetJointPosCallBack);

%% set-up publishers and subscribers for '/poseRequest' topic

%create publisher associate it to node, topic and define message type
%(empty here)
getPosePub = robotics.ros.Publisher(userNode, '/poseRequest', 'std_msgs/Empty');

%create subscriber associate it with listening node, topic and set callback
%function
getPoseSub = robotics.ros.Subscriber(vKukaNode, '/poseRequest', {@RequestJointPosCallBack, clientID, vrep, returnPosePub});

%% start vrep simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

