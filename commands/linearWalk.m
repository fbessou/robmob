function linearWalk( distance,publisher,speed)
    if nargin < 3
        speed = 0.3;
    end
    
    startTwist = createTwist();
    endTwist = createTwist();
    endTwist.Linear.X = speed*sign(distance);
    
    reachVelocity(startTwist,endTwist,publisher,0.5,0.01);
    
    maintainVelocity(endTwist,abs(distance)/speed,publisher)
    
    reachVelocity(endTwist,startTwist,publisher,0.5,0.01);
    ensureImmobility('mobile_base')
end

