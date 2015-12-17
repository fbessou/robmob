%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';

rosinit(ipaddress);
vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
%%
startTwist = createTwist();
endTwist = arrayToTwist([-0.2,0,0,0,0,-1]);
reachVelocity(startTwist,endTwist,vel_mux_publisher,5,0.01);
reachVelocity(endTwist,startTwist,vel_mux_publisher,5,0.01);