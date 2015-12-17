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
velmsg = rosmessage(robot);
velmsg.Linear.X = 1;
velmsg.Angular.Z = 0;
send(robot,velmsg);
%%
send(robot,velnull);
%%
f = 0.05; 
u0 = 320;
p1 =[6 9]';
p2 = [9 9]';
p3 = [9 5]';

img = Camera(imsub);
imshow(readImage(img));

[Lx Ly] = ginput(3);

% Calcul des angles
theta = atan((Lx-u0)/f);
alpha = theta(2) - theta(1);
beta  = theta(3) - theta(2);

% Calcul du centre des cercles et rayons
c1 = [0 -1; 1 0]*(p1-p2)./(2*tan(alpha))+ 0.5*(p2+p1);
c2 = [0 -1; 1 0]*(p2-p3)./(2*tan(beta))+ 0.5*(p3+p2);

r1 = sqrt(sum((p1-p2).^2)) / (2*sin(alpha));
r2 = sqrt(sum((p2-p3).^2)) / (2*sin(beta));

clf
plot(p1(1),p1(2),'bo','MarkerFace','b'); text(p1(1)+0.12,p1(2)+0.12,'p1');
hold on;
plot(p2(1),p2(2),'ro','MarkerFace','r'); text(p2(1)+0.12,p2(2)+0.12,'p2');
plot(p3(1),p3(2),'go','MarkerFace','g'); text(p3(1)+0.12,p3(2)+0.12,'p3');

circle(c1,r1,1000,'r');
circle(c2,r2,1000,'g');

% Pour avoir un grid sur les coins de tuiles
xTick = get(gca,'XTick');
xTick = min(xTick):1:max(xTick);
set(gca,'XTick',xTick);
yTick = get(gca,'YTick');
yTick = min(yTick):1:max(yTick);
set(gca,'YTick',yTick);
xlabel('x','FontSize',13);
ylabel('y','FontSize',13);
axis equal;
hold off;
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