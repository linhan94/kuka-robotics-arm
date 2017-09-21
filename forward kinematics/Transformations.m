%% UCL MSc Robotics - Transformations
% Author: George Dwyer
% Date: 26/10/2016
% Summary:
% Script showing composing and plotting transformations using the helper 
% functions provided by the Corke robotics toolbox.
close all

%% Composing transformations
% An identity matrix representing no change in translation or orientation
% can be shown as:
%%
%
% T_base = [ 1, 0, 0, 0;
% 0, 1, 0, 0;
% 0, 0, 1, 0;
% 0, 0, 0, 1]
% Or 
T_base = eye(4);
%% Translation
% The translation component of the transform can be shown as: 
% 
% T_translation = [1, 0, 0, x;
% 0, 1, 0, y;
% 0, 0, 1, z;
% 0, 0, 0, 1];
% T_translation = transl(x, y, z)

T_trans = transl(3, 4, 5);

h = figure;
hold on
grid on

trplot(T_base, 'rviz', 'frame', 'Base');
trplot(T_trans, 'rviz', 'frame', 'Translation');

%% Rotations
% T_rotation_x = [ 1, 0, 0, 0;
% 0, cos(theta), -sin(theta), 0;
% 0, sin(theta), cos(theta), 0;
% 0, 0, 0, 1];
%
% T_rotation_x = trotx(theta)
T_rot = trotx(pi/2);
%%
% Offset the frame from the base so that you can see it
T_rot(1, 4) = 2;
T_rot(2, 4) = 0;
%trplot(T_rot, 'rviz', 'frame', 'RotX')
%%
% Apply the rotation to T_trans
T = T_trans * T_rot;
trplot(T, 'rviz', 'frame', 'Rotation')

T_back = inv(T_trans) * T;
trplot(T_back, 'rviz', 'frame', 'Return')
view(45, 45)

%% Alternate Representations:
% Converting the orientation from 3x3 matrix to another representation. The
% robotics toolbox (both corke and matlab) have a functions to convert to:
% 
% * Angle axis
% * Quaternion
% * Euler
% 
% Allowing the following transformation
trans = rpy2tr(-pi/2, pi/4, 2*pi/3);
display(trans);
%%
% to be shown as:
%%
% Angle axis
tr2angvec(trans)
%%
% Quaternion
Quaternion(trans(1:3,1:3))
%%
% Euler angles (ZYZ)
tr2eul(trans)
%%
% Roll pitch yaw angles
tr2rpy(trans)

%% Tasks
% * Create a transformation which rotates around X-axis by -90 degree and subsequently translates by [3, 4, 5]
% * Create a transformation which translates by [3,4,5] and subsequently rotates around X-axis by -90 degree. 
% * Plots the results of the two transformations and which transformation is corresponding to 
% [1, 0, 0, 3;
%  0, 0, 1, 4;
%  0,-1, 0, 5;
%  0, 0, 0, 1]
% * Build a rotation matrix which represents these sequential rotaitions:
%%
% # Rotation around X-axis by 45 degrees
% # Rotation around Z-axis by 90 degrees
% # Rotation around Y-axis by -60 degrees
% # Rotation around Z-axis by 120 degrees
% # Rotation around X-axis by -30 degrees
%%
% * Convert this rotation matrix to: 
%%
% # angle axis representation
% # quaternion
% # ZYZ euler angle

%%
% * Convert these angle axis representations to rotation matrices
%%
% # axis = [-1, 1, 1]/sqrt(3), theta = 30
% # axis = [1, -1, -1]/sqrt(3), theta =  -30
% * Represent the frames overlaying the global frames
