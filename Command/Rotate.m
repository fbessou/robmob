function [] = Rotate(Angular_Velocity)
%ROTATE Commande to make the robot rotate
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);
velmsg.Angular.Z = Angular_Velocity;
send(robot,velmsg);
end

