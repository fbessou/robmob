function [colorFound index] = ColorImageHeight( Image,Height )
%COULEURPICTURE Summary of this function goes here
%   Detailed explanation goes here
ligne = double(squeeze(Image(Height,:,:)));
ajout = 1;
colorFound = [];
index = [];
for i = 1:size(ligne,1)
    color = FindColor(ligne(i,:));
    if (color ~= 0 && ajout == 1)
        colorFound = [colorFound [color]];
        index = [index [i]];
        ajout = 0;
    else if color == 0
        ajout = 1;
        end
    end
end

end

