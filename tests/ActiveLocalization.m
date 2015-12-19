function [ Pose Cov ] = ActiveLocalization( velsub,imsub )
%ACTIVELOCALIZATION Summary of this function goes here
%   Detailed explanation goes here
    run('worlds/basic_world/Marker.m')
    hfov = 1.0472;
    f = (640/2) / tan(hfov/2); 
    lowerColor = [];
    
    while length(lowerColor) < 3
        linearRotate(pi/4,velsub,pi/4)
        img = Camera(imsub);
        imshow(readImage(img));
        A = rgb2hsv(readImage(img));

        [lowerColor,index] = ColorImageHeight(A,250);
        if length(lowerColor) >= 3
            lowerColor = lowerColor(1:3);
            index = index(1:3);

            higherColor = [];
            for j = 1:3
                color = lowerColor(j);
                for i = 250:-1:1
                    colortemp(i) = FindColor(A(i,index(j),:));
                    if colortemp(i) ~= lowerColor(j) && colortemp(i) ~= 0
                        color = colortemp(i);
                        break;
                    end
                end
                higherColor = [higherColor [color]];
            end

            lowerColor = lowerColor(1:3);
            higherColor = higherColor(1:3);
            index = index(1:3);

            u0 = 320;
            p1 =[marker(1,lowerColor(1),higherColor(1)) marker(2,lowerColor(1),higherColor(1))]';
            p2 = [marker(1,lowerColor(2),higherColor(2)) marker(2,lowerColor(2),higherColor(2))]';
            p3 = [marker(1,lowerColor(3),higherColor(3)) marker(2,lowerColor(3),higherColor(3))]';

            theta = atan((index-u0)/f);
            alpha = theta(2) - theta(1);
            beta  = theta(3) - theta(2);

            c1 = [0 -1; 1 0]*(p1-p2)./(2*tan(alpha))+ 0.5*(p2+p1);
            c2 = [0 -1; 1 0]*(p2-p3)./(2*tan(beta))+ 0.5*(p3+p2);

            r1 = sqrt(sum((p1-p2).^2)) / (2*sin(alpha));
            r2 = sqrt(sum((p2-p3).^2)) / (2*sin(beta));

            [i1, i2] = circleintersect(c1, r1, c2, r2, 'lr');

            if(isequalv(i1,p1) || isequalv(i1,p2) || isequalv(i1,p3))
                positionR = [i2(1)-0.118 i2(2)-0.118];
            else
                positionR = [i1(1)-0.118 i1(2)-0.118];
            end

            orientation = atan2(p2(2) - positionR(2), p2(1) - positionR(1)) - theta(2);

            Pose = [positionR(1); positionR(2); orientation];
            Cov = zeros(3,3);
        end
    end
end

