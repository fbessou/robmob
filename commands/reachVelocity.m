function reachVelocity( twistBegin, twistEnd, publisher, duration, maxVelJump)
    %TEST
    if nargin < 4
        if nargin<3
            duration = 1
        end
        maxVelJump = 0.01 %m/s
    end

    arrayBegin = twistToArray(twistBegin);
    arrayEnd = twistToArray(twistEnd);

    maxLin = max(abs(arrayBegin(1:3)-arrayEnd(1:3)));
    nbSteps = maxLin/maxVelJump;
    if nbSteps<=0
        nbSteps=1
    end
    stepDuration = duration/nbSteps;
    
    for i=0:stepDuration:duration;
        x=i/duration;
        current = arrayBegin*(1-x) + arrayEnd *x;
        send(publisher,arrayToTwist(current));
        pause(stepDuration)
    end

end