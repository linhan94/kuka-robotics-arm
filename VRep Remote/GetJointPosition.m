function GetJointPosition(publisher)

%create ros message according to publisher
message = rosmessage(publisher);
%publish message to topic listeners
publisher.send(message);


end

