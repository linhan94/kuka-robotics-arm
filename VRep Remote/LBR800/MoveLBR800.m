%% UCL MSc Robotics - ROS Network/Vrep RemoteAPI
% Author: Kevin Tchaka
% Date: 04/11/2016
% Summary:
% Script demonstrating how to move a LBR_iiwa_7_R800 through some positions
% via vrep's remoteApi.

targetPos1 = [90*pi/180, 90*pi/180, 170*pi/180, -90*pi/180, 90*pi/180, 90*pi/180, 0];
targetPos2 = [-90*pi/180, 90*pi/180, 180*pi/180, -90*pi/180, 90*pi/180, 90*pi/180, 0];
targetPos3 = [0, 0, 0, 0, 0, 0, 0];


SendJointTargetPosition(setPosePub,targetPos1);
pause(7);

SendJointTargetPosition(setPosePub,targetPos2);
pause(7);

SendJointTargetPosition(setPosePub,targetPos3);
pause(7);