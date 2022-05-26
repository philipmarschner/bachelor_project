figure
combined = zeros(582,727);
for i=1:size(zones,3)
    combined = combined | squeeze(zones(:,:,i));
end
imshow(combined);
title('Critical zones of the environment')
hold on

plot(route(:,1),582-route(:,2),'-','MarkerSize',5)
plot(route(:,3),582-route(:,4),'-','MarkerSize',5)
%plot(route(:,5),582-route(:,6),'-','MarkerSize',5)

figure
show(trueMap);
title('Critical zones of the environment')
hold on

plot(route(:,1),route(:,2),'-','MarkerSize',5)
plot(route(:,3),route(:,4),'-','MarkerSize',5)
%plot(route(:,5),582-route(:,6),'-','MarkerSize',5)