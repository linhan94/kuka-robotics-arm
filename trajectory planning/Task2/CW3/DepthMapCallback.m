function GetDepthMap(~, ~, clientID, vrep, kinect)
% Callback method to get the depth map from a kinect sensor
% msg is the rosmessage received by the subscriber
% clientID is the ID of the current vrep remote session
% vrep is the handle to the remote vrep API
% kinect is the handle of the kinect's depth sensor in the vrep scene
global depth;

[~,~,depth] = vrep.simxGetVisionSensorDepthBuffer2(clientID, kinect, vrep.simx_opmode_buffer);

end

