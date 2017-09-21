%% UCL MSc Robotics - CW3 Free Resources
% Author: Kevin Tchaka
% Date: 08/12/2016
% Summary:
% Script freeing all ros network and vrep remoteAPI resources
%% V-rep resources

% Stop v-rep simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
% Close the scene
vrep.simxCloseScene(clientID, vrep.simx_opmode_blocking);
% stop v-rep remote connection
vrep.simxFinish(clientID);
% free v-rep remote handle
vrep.delete();
clear clientID vrep kinect;

%% Ros resources

% clear ros nodes, publishers and subscribers
clear getArmPosePub getArmPoseSub;
clear setPoseArmPub setPoseArmSub;
clear getBasePosePub getBasePoseSub;
clear setPoseBasePub setPoseBaseSub;
clear closeGripperSub closeGripperPub;
clear openGripperPub openGripperSub;
clear gripJoints wheelJointPos armJointPos;
clear returnPosePub returnPoseSub;
clear userNode vKukaNode kinectNode;
clear getDepthPub getDepthSub;
clear masterHost;
clear depth ans res wheelJoints armJoints load;

% shut down ros network and free master node
rosshutdown;