function GetJointPosCallBack(~, msg)

% global variable to hold the returned positions
global jointPos;
% assign message data to array
jointPos = msg.Data;

end

