ipaddress = '192.168.199.132';
rosinit(ipaddress);
%%
robot = rospublisher('/cmd_vel_mux/input/teleop');
imsub = rossubscriber('/camera/rgb/image_raw/compressed');
laser = rossubscriber('/scan');
velnull = rosmessage(robot);
velnull.Linear.X = 0.0000000;
velnull.Angular.Z = 0.0000000;
%%
rostopic list
%%
rostopic info /mobile_base/commands/motor_power
%%
Rotate(1,robot);
%%
img = Camera(imsub);
imshow(readImage(img));
%%
f = 0.05; 
u0 = 320;
p1 =[6 9]';
p2 = [9 9]';
p3 = [9 5]';

img = Camera(imsub);
imshow(readImage(img));
%%
scaning = Scan(laser,4);
plot(scaning)
B = scaning.Ranges;
%%
ligne = double(squeeze(A(240,:,:)))/255;
distance = transpose(B);
distance = fliplr(distance);
x = 1:1:640;
scatter(x,distance,36,ligne)
%%
rosshutdown
clear