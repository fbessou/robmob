%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';

rosinit(ipaddress);
vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
positionReader = rossubscriber('/odom/');
%%
linearWalk(1,vel_mux_publisher)
receive(positionReader)
