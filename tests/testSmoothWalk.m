%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';

rosinit(ipaddress);
%%
vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
%%
extractPosition(gazeboPose('mobile_base'))
smoothWalk(2,vel_mux_publisher);
extractPosition(gazeboPose('mobile_base'))
%%
reachVelocity(createTwist(),createTwist(),vel_mux_publisher,1,0.01);
