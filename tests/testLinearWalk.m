%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';

rosinit(ipaddress);
%%
vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
%%
linearWalk(-1,vel_mux_publisher);
gazeboPosition('mobile_base')
%%
reachVelocity(createTwist(),createTwist(),vel_mux_publisher,1,0.01);
