function [ angle ] = quaternionToAngle( Q )
    quaternion = [Q.X Q.Y Q.Z Q.W];
    angles= quat2eul(quaternion);
    angle=angles(3);
end

