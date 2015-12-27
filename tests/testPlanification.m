%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';
%%
try
    %%
    rosinit(ipaddress);
    load('Commands')
    RotationCommandFactor(13,:)=[];
    RotationCommandFactor = mean(RotationCommands(:,2)./RotationCommands(:,1))
    load('basic_world_graph')
    load('basic_world')
    vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
    imsub = rossubscriber('/camera/rgb/image_raw');
catch
end

Pos = [0 0 0]';
Cov = eye(3,3);
localizationFunc = 'triangularLocalization';
[Pos, Cov,TranslationCommands] = feval(localizationFunc,vel_mux_publisher,imsub,Pos,Cov,[0 0]',TranslationCommands);
targetPosition = extractPosition(gazeboPose('robot_target'));
targetPosition = targetPosition +[5 12]';
lastTargetPosition = targetPosition;
Waypoints = Dijkstra(Points,Edges,Pos(1:2),targetPosition);

finished=false;
while ~finished
    %% Seek target
    while norm(Pos(1:2)-targetPosition)>0.5
        %% Check target position
        if norm(lastTargetPosition-targetPosition)>0.5
            Waypoints = Dijkstra(Points,Edges,Pos(1:2),targetPosition)
        end
        %% Search next point
        if ~isempty(Waypoints)
            nextPoint = Points(Waypoints(end),:)';
        else
            nextPoint = targetPosition;
        end
        deltaPos = nextPoint-Pos(1:2);
        angle = atan2(deltaPos(2),deltaPos(1));
        deltaAngle = mod(angle-Pos(3)+ pi,2*pi) - pi;      
        %% Update figure
        clf;
        hold on;
        axis equal
        plotMap(basic_world);
        scatter(Points(:,1),Points(:,2),'c+')
        for edgeIndex  = 1:size(Edges)
            edgePointsId = Edges(edgeIndex,:);
            edge = Points(edgePointsId,:);
            plot(edge(:,1),edge(:,2),'c')
        end
        scatter(nextPoint(1),nextPoint(2))
        %% Estimate :
        if norm(Cov(1:2,1:2))>0
            Cov(1:2,1:2)
            [vecs, vals] =eig(Cov(1:2,1:2));
            ellipse(real(vals(1,1)),real(vals(2,2)),atan2(real(vecs(:,1)),real(vecs(:,2))),Pos(1),Pos(2),'m');
        end
        scatter(Pos(1),Pos(2),'m')
        sightLine = Pos(1:2)+[cos(Pos(3)) , -sin(Pos(3));sin(Pos(3)), cos(Pos(3))]*[ 0.5; 0];
        plot([Pos(1) sightLine(1)],[Pos(2) sightLine(2)],'m');
        %% Real :
        [ RealPos cov_ ]= gazeboLocalization();
        scatter(RealPos(1),RealPos(2),'b')
        realSightLine = RealPos(1:2)+[cos(RealPos(3)) , -sin(RealPos(3));sin(RealPos(3)), cos(RealPos(3))]*[ 0.5; 0];
        plot([RealPos(1) realSightLine(1)],[RealPos(2) realSightLine(2)],'g');
        scatter(targetPosition(1),targetPosition(2),40,'b')
        %% Move toward next waypoint
        [p ktrans] = min(abs(TranslationCommands(:,2) - min(norm(deltaPos),1)));
        [p krot] = min(abs(RotationCommands(:,2) - deltaAngle));
        command = [TranslationCommands(ktrans,1); RotationCommands(krot,1)];
        linearRotate(command(2),vel_mux_publisher);
        smoothWalk(command(1) ,vel_mux_publisher);
        %% Update position
        [Pos, Cov , TranslationCommands] = feval(localizationFunc,vel_mux_publisher,imsub,Pos,Cov,command,TranslationCommands);
        lastTargetPosition = targetPosition;
        targetPosition = extractPosition(gazeboPose('robot_target'));
        targetPosition = targetPosition +[5 12]';
        %% Check distance to waypoint and remove if near enough
        if ~isempty(Waypoints) && norm(Pos(1:2) - nextPoint) < 0.4
            Waypoints = Waypoints(1:end-1);
        end
    end
    %% Wait for target to move

    while norm(lastTargetPosition-targetPosition)<0.5
        pause(0.1)
       	lastTargetPosition = targetPosition;
        targetPosition = extractPosition(gazeboPose('robot_target'));
        targetPosition = targetPosition +[5 12]';
    end
end
