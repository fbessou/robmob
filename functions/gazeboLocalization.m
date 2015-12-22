function [ Pose Cov] = gazeboLocalization(lastPose, lastCov, command)
    %%Mock function for localization
    %
    %return a pose and its covariance given a previous pose and
    %covariance and a command
    
    GAZ_MAP_TF = [5 12]';
    Pose = [0 0 0]';
    Cov = zeros(3,3);
    gPose = gazeboPose('mobile_base');
    Pose(1:2) = extractPosition(gPose)+GAZ_MAP_TF;
    theta = quaternionToAngle(gPose.Orientation);
    Pose(3) = theta;
end

