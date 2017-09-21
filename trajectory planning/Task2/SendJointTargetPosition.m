function SendJointTargetPosition(publisher, jointPos)

%create ros message according to publisher
message = rosmessage(publisher);
%populate message data with desired joint position
message.Data = jointPos;
%publish message to topic listeners
publisher.send(message);
