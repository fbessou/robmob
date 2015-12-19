function gazeboForceModelState( model,pose,twist )
    persistent gazePub;
    if isempty(gazePub)
        gazePub = rospublisher('/gazebo/set_model_state');
    end
    if nargin < 3
        twist = createTwist();
    end;
    poseTwist = rosmessage('gazebo_msgs/ModelState');
    poseTwist.ModelName = model;
    poseTwist.Pose = pose;
    poseTwist.Twist= twist;
    poseTwist.ReferenceFrame= 'world';
    send(gazePub,poseTwist)
end

