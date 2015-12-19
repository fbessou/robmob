function smoothWalk( distance,publisher,speed)
    if nargin < 3
        speed = 0.3;
    end
    
    startTwist = createTwist();
    endTwist = createTwist();
    dir = sign(distance);
    endTwist.Linear.X = speed*dir;
    
    reachVelocity(startTwist,endTwist,publisher,0.5,0.01);
    
    maintainVelocity(endTwist,abs(distance-0.3*dir)/speed,publisher)
    
    reachVelocity(endTwist,startTwist,publisher,0.5,0.01);
    ensureImmobility('mobile_base')
end

