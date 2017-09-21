function RequestJointPosCallBackYB(~, ~, clientID, vrep, publisher, armJoints)
% Callback function to retrieve joints' positions from a remote YouBot
% clientID is the ID of the current v-rep remote session
% vrep is the handle to the v-rep remote API
% publisher is the publisher to the '/poseReturn' topic to send back data
% to the concerned node
% armJoints is a collection of the YouBot's arm joints handle

if (clientID > -1)
    % retrieve joints handle from v-rep remote API
    joints = armJoints;

    njoints = length(joints);
    data = zeros(size(joints));

    % retrieve individual joint position
    for i = 1 : njoints
      [~, data(i)] = vrep.simxGetJointPosition(clientID, joints(i), vrep.simx_opmode_blocking);
    end

    % create ros message to send back joints position
    msg = rosmessage(publisher);
    msg.Data = data;

    % publish ros message 
    publisher.send(msg);
end

end

