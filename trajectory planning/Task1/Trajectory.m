function Trajectory(JPM, pub)
%Trajectory
%   Moves the joint of a remote device according to the joint position
%   matrix JPM, each row of JPM should represent a different joint
%   configuration to be reached by the device. pub is the ros publisher
%   to send the message.

[m, n] = size(JPM);

for i = 1 : m
    SendJointTargetPosition(pub, JPM(i,:));
    pause(0.05); 
    % IMPORTANT: give some time to the device to reach the position you set
    % any new position sent will be immediately considered even if the
    % robot is still moving toward another, if you want it to follow the
    % trajectory you must ensure it can reach each point of the trajectory.
end


end

