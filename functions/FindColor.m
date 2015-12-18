function [ Color ] = FindColor( color )
    Color = 0;
    if color(2) > 0.6
        if color(1) * 360 < 30 || color(1) * 360 > 330
            Color = 1;
        end
        if color(1) * 360 < 65 && color(1) * 360 > 55
            Color = 4;
        elseif color(1) * 360 < 270 && color(1) * 360 > 210
            Color = 2;
        elseif color(1) * 360 < 150 && color(1) * 360 > 90
            Color = 3;
        end
    end
end

