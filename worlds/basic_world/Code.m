ipaddress = '192.168.199.132';
rosinit(ipaddress);
robot = rospublisher('/mobile_base/commands/velocity');
imsub = rossubscriber('/camera/rgb/image_raw/compressed');
laser = rossubscriber('/scan');
%%
rostopic list
%%
rostopic info /mobile_base/commands/motor_power
%%
Rotate(1,robot);
%%
velmsg = rosmessage(robot);
velmsg.Linear.X = 1;
velmsg.Angular.Z = 0;
send(robot,velmsg);
%%
img = Camera();
pause(3);
imshow(readImage(img));
%%
scaning = Scan(4);
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