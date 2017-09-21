function SetTargetVelocity(clientID, vrep, velocity)
% Function to send target joint velocity to v-rep remoteAPI

if(clientID > -1)
    % retrieve joints from v-rep remote API
   [res, joints] = vrep.simxGetObjects(clientID, vrep.sim_object_joint_type, vrep.simx_opmode_blocking);
   
   if(res == vrep.simx_return_ok)
      njoints = length(joints);
      % Pause communication to send information to all joints at once
      vrep.simxPauseCommunication(clientID, true);
      for i = 1 : njoints
          % Set joint target velocity
          vrep.simxSetJointTargetVelocity(clientID, joints(i), velocity, vrep.simx_opmode_oneshot);
      end
      % Restart v-rep communication
      vrep.simxPauseCommunication(clientID, false);
   end
end