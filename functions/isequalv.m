function [truth] = isequalv( A,B )
    truth = 1;
    if(size(A) == size(B))
        for i=1:size(A)
            if abs(A(i) - B(i)) > 0.0001
                truth = 0;
            end
        end
    else
        truth = 0;
    end
end

