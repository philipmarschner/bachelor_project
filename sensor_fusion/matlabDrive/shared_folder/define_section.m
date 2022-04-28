

user_input = input("How many robot zones do you want to define?");




img = imread('45deg_1Obstacles.png');
imshow(img)
[x,y,~]  = size(img);
zones = zeros(user_input,x,y);



for i = 1:user_input
    roi = drawpolygon;
    
    poly = polyshape(roi.Position(:,1),roi.Position(:,2));
    mask = poly2mask(poly.Vertices(:,1),poly.Vertices(:,2),x,y);
    
    zones(i,:,:)=mask;
end




save('zones','zones')
close all
