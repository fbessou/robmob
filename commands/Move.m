function [] = Move(Linear_Velocity )
%MOVE Basic commmand to make the robot go forward
    %To go backward, input a negative velocity
velmsg = rosmessage(robot);
velmsg.Linear.X = Linear_Velocity;
send(robot,velmsg);
end

