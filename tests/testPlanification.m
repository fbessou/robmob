%ipaddress = '192.168.199.132';
ipaddress = '127.0.0.1';
Commands = [];
%%
try
    %%
    rosinit(ipaddress);
    load('basic_world_graph')
    load('basic_world')
    vel_mux_publisher = rospublisher('/mobile_base/commands/velocity');
    imsub = rossubscriber('/camera/rgb/image_raw');
catch
end


Pos = [0 0 0]';
Cov = ones(3,3);
localizationFunc = 'triangularLocalization';
[Pos, Cov] = ActiveLocalization(vel_mux_publisher,imsub);
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
        scatter(Points(:,1),Points(:,2),'+')
        for edgeIndex  = 1:size(Edges)
            edgePointsId = Edges(edgeIndex,:);
            edge = Points(edgePointsId,:);
            plot(edge(:,1),edge(:,2),'g')
        end
        scatter(nextPoint(1),nextPoint(2))
        scatter(Pos(1),Pos(2))
        sightLine = Pos(1:2)+[cos(Pos(3)) , -sin(Pos(3));sin(Pos(3)), cos(Pos(3))]*[ 0.5; 0]
        plot([Pos(1) sightLine(1)],[Pos(2) sightLine(2)],'R');
        scatter(targetPosition(1),targetPosition(2),40,'g')
        %% Move toward next waypoint
        k = find(abs(Commands(:,2) - min(norm(deltaPos),1)) > 0.1);
        command = [Commands(k,1); deltaAngle];
        linearRotate(deltaAngle,vel_mux_publisher);
        smoothWalk(command ,vel_mux_publisher);
        %% Update position
        if max(abs(eig(Cov(1:2,1:2)))) > 0.3  && Cov(3,3) < pi/4
            [Pos, Cov] = feval(localizationFunc,Pos,Cov,[min(norm(deltaPos),1); deltaAngle],imsub,Commands);
        else
            [Pos, Cov] = ActiveLocalization(vel_mux_publisher,imsub,Cov,Pos,[min(norm(deltaPos),1); deltaAngle],Commands);
        end
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
