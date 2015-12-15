function [robot,imsub,laser,velnull] = Init(IP)
rosinit(IP);
robot = rospublisher('/mobile_base/commands/velocity');
imsub = rossubscriber('/camera/rgb/image_raw/compressed');
laser = rossubscriber('/scan');
velnull = rosmessage(robot);
velnull.Linear.X = 0.00000001;
velnull.Angular.Z = 0.00000001;
end

