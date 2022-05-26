%Setings
MarkerSize = 50;
LineWidth = 2;


%------------------------
figure(1000)
combined = zeros(size(zones,1:2));
for i=1:size(zones,3)
    combined = combined | squeeze(zones(:,:,i));
end
imshow(combined);
title('Critical zones of the environment')
hold on

plot([start(:,1) start(:,3)],size(zones,1)-[start(:,2) start(:,4)],'.','MarkerSize',MarkerSize,'Color','b')
plot([goal(:,1) goal(:,3)],size(zones,1)-[goal(:,2) goal(:,4)],'.','MarkerSize',MarkerSize,'Color','r')
plot(route(:,1),size(zones,1)-route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
plot(route(:,3),size(zones,1)-route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
%plot(route(:,5),size(zones,1)-route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
legend('Start','Goal','Robot1','Robot2')





%---------------------------
figure(1001)
show(trueMap);
title('Critical zones of the environment')
hold on

plot([start(:,1) start(:,3)],[start(:,2) start(:,4)],'.','MarkerSize',MarkerSize,'Color','b')
plot([goal(:,1) goal(:,3)],[goal(:,2) goal(:,4)],'.','MarkerSize',MarkerSize,'Color','r')
plot(route(:,1),route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
plot(route(:,3),route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
%plot(route(:,5),582-route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
legend('Start','Goal','Robot1','Robot2')

pause(0.5)


%----------------------
figure(1002)
[im ,map] = imread('zoneTemplate.png');
template = imcrop(im,map,[0 0 size(zones,2) size(zones,1)]);
template = ind2rgb(template,map);

%Create mask for extracting from template
mask = zeros(582,727);
for i=1:size(zones,3)
    mask = mask | squeeze(zones(:,:,i));
end
%Expand mask to RGB
mask(:,:,2) = mask;
mask(:,:,3) = mask(:,:,1);

%Remove all of template that is out of mask
template(mask==0) = 0;

%Load map
img = imread(mapPath);

%Crop mask from map
img(mask==1) = 0;

%Add template to map
template(img==255) = 255;

imshow(template);
title('Critical zones of the environment')
hold on

plot([start(:,1) start(:,3)],size(zones,1)-[start(:,2) start(:,4)],'.','MarkerSize',MarkerSize,'Color','b')
plot([goal(:,1) goal(:,3)],size(zones,1)-[goal(:,2) goal(:,4)],'.','MarkerSize',MarkerSize,'Color','r')
plot(route(:,1),size(zones,1)-route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
plot(route(:,3),size(zones,1)-route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
%plot(route(:,5),size(zones,1)-route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
legend('Start','Goal','Robot1','Robot2','Critical zone')