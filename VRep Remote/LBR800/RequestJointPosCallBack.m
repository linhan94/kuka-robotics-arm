function RequestJointPosCallBack(~, ~, clientID, vrep, publisher)
% Callback function to retrieve joints' positions
% clientID is the ID of the current v-rep remote session
% vrep is the handle to the v-rep remote API
% publisher is the publisher to the '/poseReturn' topic to send back data
% to the concerned node

if (clientID > -1)
    % retrieve joints handle from v-rep remote API
   [res, joints] = vrep.simxGetObjects(clientID, vrep.sim_object_joint_type, vrep.simx_opmode_blocking);
   
   if (res == vrep.simx_return_ok)
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

end

