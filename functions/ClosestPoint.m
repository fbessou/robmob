function [Index,distance] = ClosestPoint(Position,Point)
    distance = 9999999999;
    for i = 1:size(Point,1);
        temp = sqrt((Point(i,2) - Position(2)).^2 + (Point(i,1) - Position(1)).^2);
        if temp < distance
            distance = temp;
            Index = i;
        end
    end
end

