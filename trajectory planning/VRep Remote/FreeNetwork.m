%% UCL MSc Robotics - Free Ros Network/V-Rep Connection
% Author: Kevin Tchaka
% Date: 26/10/2016
% Summary:
% Script showing how to Free all ros network and vrep remoteAPI resources

%% V-rep resources

% Stop v-rep simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
% stop v-rep remote connection
vrep.simxFinish(clientID);
% free v-rep remote handle
vrep.delete();
clear clientID vrep;

%% Ros resources

% clear ros nodes, publishers and subscribers
clear getPosePub getPoseSub; %posePubMsg;
clear setPosePub setPoseSub;
clear returnPosePub returnPoseSub;
clear userNode vKukaNode;
clear masterHost;

% shut down ros network and free master node
rosshutdown;