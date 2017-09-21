function GripperAction( publisher )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%create ros message according to publisher
message = rosmessage(publisher);

%publish message to topic listeners
publisher.send(message);


end

