%% UCL MSc Robotics - Forward Kinematics
% Author: George Dwyer
% Date: 26/10/2016
% Summary:
% Script showing how to create a manipulator object from dh parameters

close all

%% DH Parameters
% Each row of the DH table, is represented by the link class.
% The link is constructed using key value pairs and flags.
% Key Value Pairs:
%   theta - Joint angle (not specified if revolute joint is used)(default)
%   d - link offset (not specified if prismatic joint is used)
%   a - link length
%   alpha - link twist
%   offset - joint variable offset
%   qlim - joint limits (radians if revolute)
%
% Flags:
%   revolute - indicates revolute joint
%   prismatic - indicates prismatic joint
%   standard - using standard dh notation
%   modified - using modified dh notation

L1 = Link('d', 0.15, 'a', 0.1, 'alpha', pi/2, 'qlim', [deg2rad(-90) deg2rad(90)], 'standard');
L2 = Link('d', 0, 'a', 0.2, 'alpha', -pi/2, 'offset', 0, 'qlim', [deg2rad(-135) deg2rad(135)], 'standard');
L3 = Link('d', 0.05, 'a', 0.15, 'alpha', 0, 'qlim', [deg2rad(-180) deg2rad(180)], 'standard');

%%
% These links may then be assembled into a manipulator using the SerialLink
% class. Each link must be defined using the same dh convention.

R = SerialLink([L1 L2 L3], 'name', 'Example Manipulator');

display(R)

%% Forward Kinematics
% The SerialLink object then provides the transformation chain needed for forward kinematics.
% This is output as a series of split transformations, the same way the
% transformation would be constructed by hand from the DH parameters.
display(R.trchain())
%%
% Alternatively the joint values can be explicitly set, returning a
% transformation from the base to the end-effector.
display(R.fkine([0,0,0]));

%%
% Joint trajectories can also be applied in a similar way with a matrix n
% by m. Where n is the number of points and m is the number of actuatable
% joints.
jpath = rand(5, 3);
cpath = R.fkine(jpath);

%% Plotting
% The Corke toolbox provides a simple interface to visualise the robot
% structure. The static position with joint values can be shown using:
R.plot([0,0,0]);
%%
% Or a joint trajectory can be passed in to animate the robot along the
% path.
jpath = [linspace(0, pi/2, 20)', linspace(0, pi/2, 20)', linspace(0, pi/2, 20)'];
R.plot(jpath)
%% Tasks
% 
% * Create a SerialLink object representing a youbot manipulator
% * Plot the youbot in the zero position
% * Animate the manipulator across the joint limits