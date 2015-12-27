function [colorsFound indices] = ColorImageHeight( Image,Height )
    line = double(squeeze(Image(Height,:,:)));
    lineLength=size(line,1);
    colorsFound =[];
    indices = [];
    
    i = 1;
    while i<=lineLength
        currentColorId = FindColor(line(i,:)); 
        if currentColorId ~= 0
            colorsFound = [colorsFound [currentColorId]];
            newColorId = currentColorId;
            startIndex = i;
            i=i+1;
            while currentColorId == newColorId && i<=lineLength
                newColorId = FindColor(line(i,:));
                if currentColorId == newColorId
                    i=i+1;
                end
            end

            indices = [ indices [idivide(int32(i-1 +startIndex),2)]];
        else
            i = i+1;
        end
    end
    indices = double(indices);
end

