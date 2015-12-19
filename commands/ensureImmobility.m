function  ensureImmobility(objectName,threshold)
    if nargin < 2
        threshold = 0.01;
    end
    
    pose = gazeboPose(objectName);
    gazeboForceModelState(objectName,pose)
end

