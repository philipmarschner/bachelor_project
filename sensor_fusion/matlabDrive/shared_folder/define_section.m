

user_input = input("How many robot zones do you want to define?")


zones = [];
img = imread('45deg_noObstacles.png');
imshow(img)
[y,x,~]  = size(img);



for i = 1:user_input
    roi = drawpolygon;
    
    temp = polyshape(roi.Position(:,1),y-roi.Position(:,2));
    zones = [zones temp];
end


save('polygon','zones')
close all
