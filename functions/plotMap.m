function plotMap(map)
% map : a (n x 4) matrix with n the number of lines in the map.
% each row contains two points : [[x1 y1] [x2 y2]]
for i=1:size(map,1)
    line([map(i,1),map(i,3)],[map(i,2), map(i,4)]);
end
