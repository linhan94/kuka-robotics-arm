%% UCL MSc Robotics - CW#3 Setup
% Author: kevin Tchaka
% Date: 08/12/2016
% Summary:
% Script setting up a simple ros network and vrep remote
% API connection for the 3rd coursework

%% Vrep Remote Connection

%these global variable will come in handy when receiving the position from
%the Kuka later

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
res = vrep.simxLoadScene(clientID,'cw3altscene2.ttt',1,vrep.simx_opmode_blocking);

%Set joint Target Velocity
SetTargetVelocity(clientID, vrep, 0);

%% get youbot's joint by type

% Wheel joints - front left, rear left, front right, rear right
wheelJoints = [0,0,0,0];
[~, wheelJoints(1)] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_blocking);
[~, wheelJoints(2)] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_blocking);
[~, wheelJoints(3)] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_blocking);
[~, wheelJoints(4)] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_blocking);

% Arm Joints
armJoints = [0,0,0,0,0];
gripJoints = [0,0];
[~, armJoints(1)] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint0', vrep.simx_opmode_blocking');
[~, armJoints(2)] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint1', vrep.simx_opmode_blocking');
[~, armJoints(3)] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint2', vrep.simx_opmode_blocking');
[~, armJoints(4)] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint3', vrep.simx_opmode_blocking');
[~, armJoints(5)] = vrep.simxGetObjectHandle(clientID, 'youBotArmJoint4', vrep.simx_opmode_blocking');
[~, gripJoints(1)] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint1', vrep.simx_opmode_blocking');
[~, gripJoints(2)] = vrep.simxGetObjectHandle(clientID, 'youBotGripperJoint2', vrep.simx_opmode_blocking');

%% get kinect's depth sensor handle
[~,kinect] = vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_blocking);
%variable to store depth map
global depth;

%% ROS Network initialization

%start matlab ros service and create master node
rosinit;

%set up basic nodes and masterHost
masterHost = 'localhost';
userNode = robotics.ros.Node('userNode', masterHost);
vKukaNode = robotics.ros.Node('vKukaNode', masterHost);
kinectNode = robotics.ros.Node('kinectNode', masterHost);

%% set-up publishers and subscribers for '/poseArmSet' topic

%create publisher associate it to node, topic and define message type
%(float array here)
setPoseArmPub = robotics.ros.Publisher(userNode, '/poseArmSet', 'std_msgs/Float64MultiArray');

%create subscriber associate it with listening node, topic and set callback
%function
setPoseArmSub = robotics.ros.Subscriber(vKukaNode, '/poseArmSet', {@MoveJointsCallBackYB, clientID, vrep, armJoints});

%% set-up publishers and subscribers for '/poseReturn' topic

%create publisher associate it to node, topic and define message type
%(float array here)
% these should be left untouched.
returnPosePub = robotics.ros.Publisher(vKukaNode, '/poseReturn', 'std_msgs/Float64MultiArray');
returnPoseSub = robotics.ros.Subscriber(userNode, '/poseReturn', @GetJointPosCallBack);

%% set-up publishers and subscribers for '/gripperAction' topic

%create publisher associate it to node, topic and define message type
%(float array here)
% these should be left untouched.
closeGripperPub = robotics.ros.Publisher(vKukaNode, '/gripOpen', 'std_msgs/Empty');
closeGripperSub = robotics.ros.Subscriber(userNode, '/gripOpen', {@CloseGripperCallBack, clientID, vrep, gripJoints});
openGripperPub = robotics.ros.Publisher(vKukaNode, '/gripClose', 'std_msgs/Empty');
openGripperSub = robotics.ros.Subscriber(userNode, '/gripClose', {@OpenGripperCallBack, clientID, vrep, gripJoints});

%% set-up publishers and subscribers for '/poseArmRequest' topic

%create publisher associate it to node, topic and define message type
%(empty here)
getArmPosePub = robotics.ros.Publisher(userNode, '/poseArmRequest', 'std_msgs/Empty');

%create subscriber associate it with listening node, topic and set callback

%function
getArmPoseSub = robotics.ros.Subscriber(vKukaNode, '/poseArmRequest', {@RequestJointPosCallBackYB, clientID, vrep, returnPosePub, armJoints});

%% set-up publisher and subscriber for '/depthRequest' topic

%create publisher associate it to node, topic and define message type
%(empty here)
getDepthPub = robotics.ros.Publisher(userNode, '/depthRequest', 'std_msgs/Empty');

%create subscriber associate it with listening node, topic and set callback
%function
getDepthSub = robotics.ros.Subscriber(kinectNode, '/depthRequest', {@DepthMapCallback, clientID, vrep, kinect});

%% start vrep simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
%% Set the depth sensor stream from vrep.
[~, res, depth] = vrep.simxGetVisionSensorDepthBuffer2(clientID, kinect, vrep.simx_opmode_streaming);

