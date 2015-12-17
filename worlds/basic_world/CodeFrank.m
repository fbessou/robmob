%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';

rosinit(ipaddress);
vel_mux_publisher = rospublisher('/cmd_vel_mux/input/teleop');
%%
%%
for i=0:-0.001:-0.1
  velmsg = rosmessage(vel_mux_publisher);

  velmsg.Linear.X = i;
  velmsg.Angular.Z = 0;
  send(vel_mux_publisher,velmsg);
  pause(0.01)
end

for i=-0.1:0.001:0
  velmsg = rosmessage(vel_mux_publisher);

  velmsg.Linear.X = i;
  velmsg.Angular.Z = 0;
  send(vel_mux_publisher,velmsg);
  pause(0.01)
end
%%
  velmsg = rosmessage(vel_mux_publisher);
  velmsg.Linear.X = 1;
  send(vel_mux_publisher,velmsg);
  %%
rosshutdown