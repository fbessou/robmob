function linearWalk( distance,publisher )
    startTwist = createTwist();
    endTwist = createTwist();
    endTwist.Linear.X = distance*0.2;
    reachVelocity(startTwist,endTwist,publisher,5,0.01);
    reachVelocity(endTwist,startTwist,publisher,5,0.01);
end

