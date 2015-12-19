function [List] = Dijkstra(Points,Edge,start,Goal)
    [indexGoal,~] = ClosestPoint(Goal,Points);
    [indexStart,~] = ClosestPoint(start,Points);
    queue = [indexStart];
    source = zeros(size(Points));
    for i =1:size(Points,1);
        source(i) = 0;
    end
    
    while size(queue) > 0
        vertices = queue(1);
        for i = 1:size(Edge,1)
            if Edge(i,1) == vertices && Edge(i,2) ~= indexStart && source(Edge(i,2)) == 0  
                source(Edge(i,2)) = vertices;
                queue = [queue [Edge(i,2)]];
            else if Edge(i,2) == vertices && Edge(i,1) ~= indexStart && source(Edge(i,1)) == 0  
                source(Edge(i,1)) = vertices;
                queue = [queue [Edge(i,1)]];
                end
            end
        end
        queue = queue(2:end);
    end
    
    List = [indexGoal];
    vertices = indexGoal;
    while vertices ~= indexStart
        List = [List [source(vertices)]];
        vertices = source(vertices);
    end
end

