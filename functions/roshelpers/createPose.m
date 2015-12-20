function [ pose ] = createPose( position )

pose = rosmessage(rostype.geometry_msgs_Pose);
pose.Position.X = position(1);
pose.Position.Y = position(2);
pose.Position.Z = 0;
pose.Orientation.X = 0;
pose.Orientation.Y = 0;
pose.Orientation.Z = sin(position(3)/2);
pose.Orientation.W = cos(position(3)/2);

end

