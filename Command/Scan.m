function [scan] = Scan(distance)
%SCAN Command to take a scan with the laser
%   Laser has a range of 0.8 to 4 meters
laser = rossubscriber('/scan');
scan = receive(laser,distance);
end

