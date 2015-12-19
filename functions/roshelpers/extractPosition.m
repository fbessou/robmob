function [ position ] = extractPosition( pose )
position= pointToArray(pose.Position);
position = position(1:2);
end

