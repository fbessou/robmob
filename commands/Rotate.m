function [] = Rotate(Angular_Velocity,robot)
%ROTATE Commande to make the robot rotate
velmsg = rosmessage(robot);
velmsg.Angular.Z = Angular_Velocity;
send(robot,velmsg);
end

