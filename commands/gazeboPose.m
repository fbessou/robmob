function pose = gazeboPose(modelName)
    persistent gazeSub;
    if isempty(gazeSub)
        gazeSub = rossubscriber('/gazebo/model_states');
    end
    models = receive(gazeSub);
    pose = models.Pose(strcmp(models.Name,modelName));
end

