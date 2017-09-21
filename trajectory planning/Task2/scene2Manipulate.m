%% Step 1. Initialize ros network nodes and remote vrep connection
CW3Setup

%% Step 2. Move from start point to top

%grab the right one
joint_profile = zeros(100,5);
velocity_profile = zeros(100,5);
zeroPoint = zeros(1,5);
x = quinticPoly([1.1504,0.5443,1.2191,-0.2032,0],zeroPoint,zeroPoint,[pi,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([pi,0,0,0,0],zeroPoint,zeroPoint,[pi,0.9,0.15,1,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(closeGripperPub);
pause(3);
x = quinticPoly([pi,0.9,0.15,1,0],zeroPoint,zeroPoint,[-0.17,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([-0.17,0,0,0,0],zeroPoint,zeroPoint,[-0.17,0.2,1.35,1.1,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(openGripperPub);
pause(2);
x = quinticPoly([-0.17,0.2,1.35,1.1,0],zeroPoint,zeroPoint,[-0.17,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);

%grab the left one
x = quinticPoly([-0.17,0,0,0,0],zeroPoint,zeroPoint,[-1.28,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([-1.28,0,0,0,0],zeroPoint,zeroPoint,[-1.28,1.3,0.16,0.85,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(closeGripperPub);
pause(3);
x = quinticPoly([-1.28,1.3,0.16,0.85,0],zeroPoint,zeroPoint,[-0.17,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([-0.14,0,0,0,0],zeroPoint,zeroPoint,[-0.14,0.1,1.4,1,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(openGripperPub);
pause(1.5);
x = quinticPoly([-0.14,0.1,1.4,1,0],zeroPoint,zeroPoint,[-0.14,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);

%grab the back and big one
x = quinticPoly([-0.14,0,0,0,0],zeroPoint,zeroPoint,[pi,0,1.8,-0.9,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([pi,0,1.8,-0.9,0],zeroPoint,zeroPoint,[pi,0.8,1.8,-0.9,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(closeGripperPub);
pause(1.5);
x = quinticPoly([pi,0.8,1.8,-0.9,0],zeroPoint,zeroPoint,[pi,1,1,-1,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([pi,1,1,-1,0],zeroPoint,zeroPoint,[0.23,1,1,-1,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([0.23,1,1,-1,0],zeroPoint,zeroPoint,[0.23,0.5,2,-0.8,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
GripperAction(openGripperPub);
pause(1.5);
x = quinticPoly([0.23,0.5,2,-0.8,0],zeroPoint,zeroPoint,[0.23,0,2,-0.8,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
x = quinticPoly([0.23,0,2,-0.8,0],zeroPoint,zeroPoint,[0.23,0,0,0,0],zeroPoint,zeroPoint,0,12);
k = 1;
for t = linspace(0,12,100)
	joint_profile(k,:) = x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 ...
                         + x(5,:)*t^4 + x(6,:)*t^5;
	velocity_profile(k,:) = x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + ...
                            4*x(5,:)*t^3 + 5*x(6,:)*t^4;
	k = k + 1;
end
Trajectory(joint_profile, setPoseArmPub);
%% Step 3. Get Joint Position from vrep node
% this function takes as argument the publisher to the '/poseRequest' topic 
% then the subscribing node will publish the returned positions to the 
% '/poseReturn' topic. the result will be visible in the global variable 
% jointPos.
GetJointPosition(getArmPosePub);
GetObjectPosAndOrientation(clientID,vrep,'Rectangle2',armJoints)
GetObjectPosAndOrientation(clientID,vrep,'Rectangle15',armJoints)

GetObjectPosAndOrientation(clientID,vrep,'Rectangle14',armJoints)


%% Step 4. Free all ros and vrep variables

CW3FreeResources

%% quinticPoly
function x = quinticPoly(qi,vi,ai,qf,vf,af,ti,tf)
    A = [1, ti, ti^2, ti^3, ti^4, ti^5;
         0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4;
         0, 0, 2, 6*ti, 12*ti^2, 20*ti^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    b = [qi; vi; ai; qf; vf; af];
    x = A\b;
end