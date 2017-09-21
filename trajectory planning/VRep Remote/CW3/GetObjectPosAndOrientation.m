function X = GetObjectPosAndOrientation(clientID, vrep, obj_name)
%Function returns position orientation and bounding box of object
%identified by obj_name
% obj_name: string representation of the desired object's name
% results are set into a structure containing 4 fields:
%   - position: vector (x,y,z) absolute position of object in scene
%   - orientation: vector (a,b,g) euler angles of absolute orientation of
%   object in scene
%   - bbox_min: vector (x,y,z) absolute position of bounding box minimum
%   point
%   - bbox_max: vector (x,y,z) absolute position of bounding box maximum
%   point

p = inputParser;
addRequired(p, 'clientID', @isnumeric);
addRequired(p, 'vrep');
addRequired(p, 'obj_name', @ischar);


bboxmin = zeros(1,3);
bboxmax = zeros(1,3);

[~, obj] = vrep.simxGetObjectHandle(clientID, obj_name, vrep.simx_opmode_blocking);
[~, pos] = vrep.simxGetObjectPosition(clientID, obj, -1, vrep.simx_opmode_blocking);
[~, or] = vrep.simxGetObjectOrientation(clientID, obj, -1, vrep.simx_opmode_blocking);
[~, bboxmin(1)] = vrep.simxGetObjectFloatParameter(clientID, obj, 15, vrep.simx_opmode_blocking);
[~, bboxmin(2)] = vrep.simxGetObjectFloatParameter(clientID, obj, 16, vrep.simx_opmode_blocking);
[~, bboxmin(3)] = vrep.simxGetObjectFloatParameter(clientID, obj, 17, vrep.simx_opmode_blocking);
[~, bboxmax(1)] = vrep.simxGetObjectFloatParameter(clientID, obj, 18, vrep.simx_opmode_blocking);
[~, bboxmax(2)] = vrep.simxGetObjectFloatParameter(clientID, obj, 19, vrep.simx_opmode_blocking);
[~, bboxmax(3)] = vrep.simxGetObjectFloatParameter(clientID, obj, 20, vrep.simx_opmode_blocking);

X = struct('position', pos, 'orientation', or, 'bbox_max', bboxmax, 'bbox_min', bboxmin);

end

