function [output] = isLegal(q,map)
    %Returns true if q is legal
    tempq = reshape(q,2,[])';


    %Check for equal workspace position
    if (length(q)>2)
        for i = 1:length(tempq)-1
            for j = i+1:length(tempq)
                if(isequal(tempq(i,:),tempq(j,:)))
                    output = false;
                    return
                end
            end
        end
    end

    %Check if position is freespace in workspace
    if (length(q)==2)
        if(checkOccupancy(map,q))
                output = false;
                return
        end
    end

    if (length(q)>2)
        for i = 1:length(tempq)
            if(checkOccupancy(map,tempq(i,:)))
                output = false;
                return
            end
        end
    end
    %check if critical section is visible, if any of the configurations is inside
    output = true;
end
