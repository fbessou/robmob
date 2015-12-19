function position = gazeboPosition(modelName)
    persistent gazeSub;
    if isEmpty(gazeSub)
        gazeSub = rossubscriber('/gazebo/model_states');
    end
    models = receive(gazeSub);
    position = models.Pose(strcmp(models.Name,modelName)).Position;
end

