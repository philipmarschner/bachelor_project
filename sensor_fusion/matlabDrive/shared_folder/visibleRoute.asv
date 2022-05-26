figure



show(trueMap)
hold on
title('Visible route')
pause(0.5)
figure
combined = zeros(582,727);
for i=1:size(zones,3)
    combined = combined | squeeze(zones(:,:,i));
end
imshow(combined);
title('Critical zones of the environment')
hold on


for i = 1:length(route)
    tempq = reshape(route(i,:),2,[])';
    total_poly = FP.totalVisibility(route(i,:));
    for j = 1:FP.numRobots    
        zoneIDs = FP.robots(j).inZone(tempq(j,:));
        plot(tempq(j,1),trueMap.YWorldLimits(2)-tempq(j,2),'.','MarkerSize',10) 
        if ~isempty(zoneIDs)
            for k = 1:length(zoneIDs)
                zoneIDs(k)
                disp(FP.robots(j).robotSectorVis(total_poly,zoneIDs(k)))
            end  
        end
    end
end