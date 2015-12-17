ipaddress = '192.168.199.132';
rosinit(ipaddress);

robot = rospublisher('/cmd_vel_mux/input/teleop');
imsub = rossubscriber('/camera/rgb/image_raw/compressed');
laser = rossubscriber('/scan');
velnull = rosmessage(robot);
velnull.Linear.X = 0.0000000;
velnull.Angular.Z = 0.0000000;
%%
Point = [5 11; 2 8; 5 8; 8 8; 2 6; 5 6; 8 4; 5 4; 8 6; 2 2; 5 2; 8 2]; 
Edge = [[1,3];[3,2];[3,4];[3,6];[2,5];[4,7];[6,5];[6,7];[6,8];[7,9];[8,9];[8,11];[9,12];[11,10];[11,12]];
position = [9 8 -50];
but = [1 1];

List = Dijkstra(Point,Edge,[position(1) position(2)],but)
while size(List) > 0
x = cosd(position(3));
y = sind(position(3));

X = [position(1)  position(1) + x];
Y = [position(2)  position(2) + y];

nextPoint = Point(List(end),:)

angleGoal = atan2d(nextPoint(2) - position(2),(nextPoint(1) - position(1)));
distanceGoal = sqrt((nextPoint(2) - position(2)).^2 + (nextPoint(1) - position(1)).^2);

NewPosition = [nextPoint(1) nextPoint(2)];

xA = cosd(angleGoal);
yA = sind(angleGoal);

XA = [position(1)  position(1) + xA];
YA = [position(2)  position(2) + yA];

position(1) = NewPosition(1);
position(2) = NewPosition(2);
position(3) = angleGoal;
List = List(1:end-1)
clf
plotMap(basic_world);
hold on 
line(X,Y,'Color','r');
line(XA,YA,'Color','g');
scatter(Point(:,1),Point(:,2));
scatter(NewPosition(1),NewPosition(2),40,'filled');
scatter(but(1),but(2),'filled');
hold off

pause(1)
end
angle = atan2d((but(2) - position(2)),(but(1) - position(1)));
NewPosition = [but(1) but(2)]; 

xA = cosd(angle);
yA = sind(angle);

XA = [position(1)  position(1) + xA];
YA = [position(2)  position(2) + yA];

x = cosd(position(3));
y = sind(position(3));

X = [position(1)  position(1) + x];
Y = [position(2)  position(2) + y];

clf
plotMap(basic_world);
hold on 
line(X,Y,'Color','r');
line(XA,YA,'Color','g');
scatter(Point(:,1),Point(:,2));
scatter(NewPosition(1),NewPosition(2),200,'filled');
scatter(but(1),but(2),'filled');
hold off
%%


%%
rosshutdown
clear