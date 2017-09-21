function CloseGripperCallBack(~, msg, clientID, vrep, gripJoints)


if (clientID > -1)
    joints = gripJoints;
    njoints = length(joints);
    
    vrep.simxSetJointTargetVelocity(clientID, joints(2), 0.04, vrep.simx_opmode_oneshot);
    [~,pos] = vrep.simxGetJointPosition(clientID, joints(2), vrep.simx_opmode_blocking);
    pos = pos * -0.5;
    vrep.simxSetJointTargetPosition(clientID, joints(1), pos, vrep.simx_opmode_oneshot);
end