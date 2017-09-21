function MoveBaseCallback(~, msg, clientID, vrep, wheelJoints)
% Callback method to send the desired joint position to virtual YouBot
% msg is the rosmessage received by the subscriber
% clientID is the ID of the current vrep remote session
% vrep is the handle to the remote vrep API
% wheelJoints is a collection of the YouBot's wheelJoints handles


% retrieve data from message
X = msg.Data;
nvals = length(X);

if (clientID > -1)
    joints = wheelJoints;
    njoints = length(joints);
    nitems = min(njoints,nvals);
    % Pause v-rep communication to actuate all joints at once
    vrep.simxPauseCommunication(clientID, true);
    % send desired target position for each joint
    for i = 1 : nitems
        vrep.simxSetJointTargetVelocity(clientID,joints(i),X(i),vrep.simx_opmode_oneshot);
    end
    %restart v-rep communication
    vrep.simxPauseCommunication(clientID,false);
end

end

