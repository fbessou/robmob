function [  Pose Cov Calibration] = triangularLocalization(velsub,imsub, lastPose, lastCov,command,Calibration)
%TRIANGULARELOCALIZATION Summary of this function goes here
%   Detailed explanation goes here


Cov = zeros(3,3);
run('worlds/basic_world/Marker.m')

if max(abs(eig(lastCov(1:2,1:2)))) > 0.5  || lastCov(3,3) > pi/4
    hfov = 1.0472;
    f = (640/2) / tan(hfov/2); 
    img = Camera(imsub);
    imshow(readImage(img));
    A = rgb2hsv(readImage(img));
    [lowerColor,index] = ColorImageHeight(A,240);
    while length(lowerColor) < 3
        linearRotate(7*pi/24,velsub)
        img = Camera(imsub);
        imshow(readImage(img));
        A = rgb2hsv(readImage(img));
        [lowerColor,index] = ColorImageHeight(A,240);
    end
        lowerColor = lowerColor(1:3);
        index = index(1:3);
        hold on;
        scatter(index,ones(3,1)*240)
        hold off;
        higherColor = [];
        for j = 1:3
            color = lowerColor(j);
            for i = 240:-1:1
                colortemp(i) = FindColor(A(i,index(j),:));
                if colortemp(i) ~= lowerColor(j) && colortemp(i) ~= 0
                    color = colortemp(i);
                    break;
                end
            end
            higherColor = [higherColor [color]];
        end
        pause;
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
            positionCamera = [i2(1); i2(2)];
        else
            positionCamera = [i1(1); i1(2)];
        end
        
        orientation = atan2(p2(2) - positionCamera(2), p2(1) - positionCamera(1));% - theta(2);
        positionRobot=positionCamera+0.118*[cos(orientation); sin(orientation)];
        Pose = [positionRobot(1); positionRobot(2); orientation];
        distance = sqrt((Pose(1) + lastPose(1))^2 + (Pose(2) + lastPose(2))^2);
            if isequaln(lastCov,zeros(3,3)) && distance < 2
                [m k] = min(Calibration(:,1) - command(1));
                Calibration(k,2) =  (distance + Calibration(k,2) * Calibration(k,3))/2;
                Calibration(k,3) =  Calibration(k,3) + 1;
            end
            
        Cov = [0.3 0 0 ; 0 0.3 0; 0 0 pi/6]
else
    Ks = 0.1;
    s=command(1);
    a=command(2);
    theta=lastPose(3);
    x=lastPose(1);
    y=lastPose(2);
    % Position update
    Pose = [x + s*cos(theta + a); 
            y + s*sin(theta + a);
            theta + a];
    % Covariance update
    % Transform 
    RobotRot = [cos(theta+a), -sin(theta+a), 0;
                sin(theta+a), cos(theta+a) , 0;
                0               , 0     , 1];


    Cov = lastCov
    Cov = Cov + RobotRot'*[Ks*s,0,0;0,0,0;0,0,pi/10*a]
    
%     mRotation = [cos(a), -sin(a);
%                  sin(a), cos(a)];
% 
%     mRotationGlobal = mRotation * [cos(theta), -sin(theta);
%                                     sin(theta), cos(theta)];
%     CovP =lastCov(1:2,1:2) + mRotationGlobal * (abs(command(1)) * CovX);
% 
%     Cov(1:2,1:2) = CovP;
%     Cov(3,3) = pi/6 * abs(command(2)) + lastCov(3,3);
end
end

