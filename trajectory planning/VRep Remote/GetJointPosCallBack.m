function GetJointPosCallBack(~, msg)

% global variable to hold the returned positions
global armJointPos;
global wheelJointPos;
% assign message data to array
jointPos = msg.Data;
n = length(jointPos);

if (n == 4)
    wheelJointPos = jointPos;
else
    armJointPos = jointPos;
end
    
        

end

