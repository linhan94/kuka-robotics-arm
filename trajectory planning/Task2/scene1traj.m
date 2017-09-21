function [ output_args ] = scene1traj( input_args )
%SCENE1TRAJ Summary of this function goes here
%   Detailed explanation goes here
    L1 = Link('d', 0.147, 'a', 0.033, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);

    %Construct the robot
    planar5Rrobot = SerialLink([L1, L2, L3, L4, L5]);
    mask = [1,1,1,0,0,0];
    %start_position = [0.2; 0.5; 0.17];
    %start_rotation = [-90*pi/180, 0*pi/180, -90*pi/180];
    %start_position = [-0.1352-0.1954; 0.2181-0.4527; 0.4378-0.033];
    %start_rotation = [0.1284+1.58, -0.1663-0.2536, -2.9294+0.34];
    %start_position = [-0.1352; 0.2181; 0.4378];
    start_position = [-0.5528+0.1954; 0.6197+0.4527; 0.3569+0.033];
    start_rotation = [0.1284, -0.1663, -2.9294];
%     rotationMat = [-0.9754   -0.2076   -0.0748;
%     0.1978   -0.9728    0.1206;
%     -0.0978    0.1029    0.9899];
    rotationMat = rotationVectorToMatrix(start_rotation);
    startMatrix = zeros(4,4);
    startMatrix(1:3,4) = start_position;
    startMatrix(1:3,1:3) = rotationMat;
    startMatrix(4,4) = 1;
    pose = planar5Rrobot.ikine(startMatrix, [0.1,0.1,0.1,0.1,0.1], mask, 'varstep');    
    %start_pose = ikine_closedform(startMatrix);
    %start_pose = [0,0,0,0,0];
end

