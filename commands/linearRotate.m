function linearRotate(angle,publisher,speed)
    if nargin < 3
        speed = pi/6;
    end
    dir = sign(angle);
    endTwist = createTwist();
    endTwist.Angular.Z = speed*dir;
    maintainVelocity(endTwist,abs(angle/speed),publisher)
    ensureImmobility('mobile_base')
end

