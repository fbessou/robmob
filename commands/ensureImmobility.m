function  ensureImmobility(publisher,threshold)
    if nargin < 2
        threshold = 0.01;
    end
    
    pS = pointToArray(gazeboPosition('mobile_base'));
    pause(0.1)
    pE = pointToArray(gazeboPosition('mobile_base'));
    
    immobile = norm(pS-pE)<threshold;
    twist = createTwist();
    twist.Linear.X = 0.3;
    while ~immobile
        send(publisher,twist);
        pS = pE
        pE = pointToArray(gazeboPosition('mobile_base'));
        twist.Linear.X = -twist.Linear.X/4;
        immobile = norm(pS-pE)<threshold;
        pause(0.03)
    end
    
end

