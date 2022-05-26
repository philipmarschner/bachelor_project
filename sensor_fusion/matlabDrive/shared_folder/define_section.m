
figure
user_input = input("How many robot zones do you want to define?");




img = imread(mapPath);
imshow(img)
[y,x,~]  = size(img);
zones = zeros(y,x,user_input);


title('draw zones')
for i = 1:user_input
    roi = drawpolygon;
    
    poly = polyshape(roi.Position(:,1),roi.Position(:,2));
    mask = poly2mask(poly.Vertices(:,1),poly.Vertices(:,2),y,x);
    
    zones(:,:,i)=mask;
end
title('draw starts')
[xi, yi] = ginput(user_input);

vec=[xi y-yi]';
C=vec(:);

start = round(C)';
title('draw goals')
[xi, yi] = ginput(user_input);


vec=[xi y-yi]';
C=vec(:);

goal = round(C)';






save('start','start')
save('goal','goal')
save('zones','zones')

close all
