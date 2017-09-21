function MoveJointsCallBack(~, msg, clientID, vrep)
% Callback method to send the desired joint position to virtual robot
% msg is the rosmessage received by the subscriber
% clientID is the ID of the current vrep remote session
% vrep is the handle to the remote vrep API

% retrieve data from message
X = msg.Data;
nvals = length(X);

if (clientID > -1)
    % retrieve collection of joints from v-rep remote API
    [res, joints] = vrep.simxGetObjects(clientID,vrep.sim_object_joint_type,vrep.simx_opmode_blocking);
    
    if (res == vrep.simx_return_ok)
        njoints = length(joints);
        nitems = min(njoints,nvals);
        % Pause v-rep communication to actuate al joints at once
        vrep.simxPauseCommunication(clientID, true);
        % send desired target position for each joint
        for i = 1 : nitems
            vrep.simxSetJointTargetPosition(clientID,joints(i),X(i),vrep.simx_opmode_oneshot);
        end
        %restart v-rep communication
        vrep.simxPauseCommunication(clientID,false);
    end
end
