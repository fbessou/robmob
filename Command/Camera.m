function [img] = Camera()
%CAMERA Command to make the robot take a photo
imsub = rossubscriber('/camera/rgb/image_raw/compressed');
img = receive(imsub);
end

