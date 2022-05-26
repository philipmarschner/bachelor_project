%Parameters
%mapPath = '90_deg_kval.png';
dist = 10;
r = 100;
lineWidth = 3;



%Code
I = imread(mapPath);
I = rgb2gray(I);
se = strel('cube',2);
%I = imdilate(I,se);
BW = edge(I,'canny');
[H,theta,rho] = hough(BW);
figure
imshow(imadjust(rescale(H)),[],...
       'XData',theta,...
       'YData',rho,...
       'InitialMagnification','fit');
xlabel('\theta (degrees)')
ylabel('\rho')
axis on
axis normal 
hold on
colormap(gca,hot)

drawnow
pause(0.5);

P = houghpeaks(H,30,'threshold',ceil(0.2*max(H(:))));
lines = houghlines(BW,theta,rho,P,'FillGap',5,'MinLength',7);


figure, imshow(I), hold on
max_len = 0;

end_points = [];
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
   end_points = [end_points ; [xy(1,:) xy(2,:)]];

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
%%
figure, imshow(I), hold on
zones = [];
first = true;
for i = 1:length(lines)-1   
    for j = i+1:length(lines)
        if norm(lines(i).point1-lines(j).point1) < dist
            pointA = lines(i).point2;
            pointB = lines(j).point2;
            interPoint = (lines(i).point1 + lines(j).point1)/2;

        elseif norm(lines(i).point1-lines(j).point2) < dist
            pointA = lines(i).point2;
            pointB = lines(j).point1;
            interPoint = (lines(i).point1 + lines(j).point2)/2;

        elseif norm(lines(i).point2-lines(j).point1) < dist
            pointA = lines(i).point1;
            pointB = lines(j).point2;
            interPoint = (lines(i).point2 + lines(j).point1)/2;

        elseif norm(lines(i).point2-lines(j).point2) < dist
            pointA = lines(i).point1;
            pointB = lines(j).point1;
            interPoint = (lines(i).point2 + lines(j).point2)/2;
        else
            continue
        end

        %Calculate angles from [-pi pi]
        angleA = -atan2(pointA(2)-interPoint(2),pointA(1)-interPoint(1));
        angleB = -atan2(pointB(2)-interPoint(2),pointB(1)-interPoint(1));
            
        %Convert angles to [0 2pi]
        angleA = mod(angleA+2*pi,2*pi);
        angleB = mod(angleB+2*pi,2*pi);
        
        %Sort so AngleA < angleB
        if(angleA > angleB)
            temp = angleB;
            angleB = angleA;
            angleA = temp;
            clear temp
        end


        if( (angleB - angleA) < pi)
            th = linspace(angleB,angleA+2*pi,50);
            x = r*cos(th)+interPoint(1);
            y = r*-sin(th)+interPoint(2);
            if x(25) < 0 || y(25) < 0 || x(25) > (size(I,2) -2) || y(25) > (size(I,1) - 2)
                continue;
            elseif(I(round(y(25)),round(x(25))) == 0 )
                continue;
            end
            plot(x,y,'LineWidth',lineWidth,'color','r');
        else
            th = linspace(angleB,angleA,50);
            x = r*cos(th)+interPoint(1);
            y = r*-sin(th)+interPoint(2);
            if x(25) < 0 || y(25) < 0 || x(25) > (size(I,2) -2) || y(25) > (size(I,1) - 2)
                continue;
            elseif(I(round(y(25)),round(x(25))) == 0 )
                continue;
            end
            plot(x,y,'LineWidth',lineWidth,'color','r');
        end
        x = [x interPoint(1)];
        y = [y interPoint(2)];
        mask = poly2mask(y,x,size(I,1),size(I,2));
        
        zones = cat(3,zones,mask);
        %zones(size(zones,1)+1,:,:) =mask;


    end
end
title('Map with critical zones')

figure
combined = zeros(size(I));
for i=1:size(zones,3)
    combined = combined | squeeze(zones(:,:,i));
end
imshow(combined);
title('Critical zones of the environment')

save('dynamicZones','zones')