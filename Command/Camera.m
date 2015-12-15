function [img] = Camera(imsub)
%CAMERA Command to make the robot take a photo
img = receive(imsub);
end

