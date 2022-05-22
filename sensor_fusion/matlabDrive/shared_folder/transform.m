

mapPath = "45_deg_kval.png";
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


intersections = [];
intersect_point = [];
dist = 10;
r = 10;

for i = 1:length(lines)-1
    
    for j = i+1:length(lines)
    
        if norm(lines(i).point1-lines(j).point1) < dist
            
           intersections = [intersections; [lines(i) lines(j)]]; 
           
           %point_xy = [lines(i).point1(1) height(I)-lines(i).point1(2)];
           intersect_point = [intersect_point;lines(i).point1];
           
           angle1 = atan2(lines(i).point2(2)-lines(i).point1(2),lines(i).point2(1)-lines(i).point1(1));
           angle2 = atan2(lines(j).point2(2)-lines(j).point1(2),lines(j).point2(1)-lines(j).point1(1));
           
         diff_angle = angle1-angle2;
           
           if abs(diff_angle) < pi
               th = linspace(angle1,2*pi-abs(diff_angle),50);
               x = r*cos(th)+lines(i).point1(1);
               y = r*-sin(th)+lines(i).point1(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
               disp('if')
               
           else
               th = linspace(angle2,angle1,50);
               x = r*cos(th)+lines(i).point1(1);
               y = r*-sin(th)+lines(i).point1(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
           end
               
               
               

           disp('1')
        
        
        end
         
         if norm(lines(i).point1-lines(j).point2) < dist
            
           intersections = [intersections; [lines(i) lines(j)]]; 
           intersect_point_xy = [lines(i).point1(1) height(I)-lines(i).point1(2)];
           
           angle1 = atan2(lines(i).point2(2)-lines(i).point1(2),lines(i).point2(1)-lines(i).point1(1));
           angle2 = atan2(lines(j).point1(2)-lines(j).point2(2),lines(j).point1(1)-lines(j).point2(1));
           
           intersect_point = [intersect_point;lines(i).point1];
           
       
           diff_angle = angle1-angle2;
           
           if abs(diff_angle) < pi
               th = linspace(angle1,2*pi-abs(diff_angle),50);
               x = r*cos(th)+lines(i).point1(1);
               y = r*-sin(th)+lines(i).point1(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
               disp('if')
               
           else
               th = linspace(angle2,angle1,50);
               x = r*cos(th)+lines(i).point1(1);
               y = r*-sin(th)+lines(i).point1(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
           end
           
           
  
            disp('2')
         end
         
          if norm(lines(i).point2-lines(j).point1) < dist
            
           intersections = [intersections; [lines(i) lines(j)]]; 
            point_xy = [lines(i).point2(1) height(I)-lines(i).point2(2)];
            intersect_point = [intersect_point;lines(i).point2];
            
            
             
           angle1 = atan2(lines(i).point1(2)-lines(i).point2(2),lines(i).point1(1)-lines(i).point2(1));
           angle2 = atan2(lines(j).point2(2)-lines(j).point1(2),lines(j).point2(1)-lines(j).point1(1));
             
      
           
           
           
           diff_angle = angle1-angle2;
           
           if abs(diff_angle) < pi
               th = linspace(angle1,2*pi-abs(diff_angle),50);
               x = r*cos(th)+lines(i).point2(1);
               y = r*-sin(th)+lines(i).point2(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
               disp('if')
               
           else
               th = linspace(angle2,angle1,50);
               x = r*cos(th)+lines(i).point2(1);
               y = r*-sin(th)+lines(i).point2(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
           end
            
            disp('3')
          end
         
           if norm(lines(i).point2-lines(j).point2) < dist
            
           intersections = [intersections; [lines(i) lines(j)]]; 
            
            intersect_point = [intersect_point;lines(i).point2];
            
              
           angle1 = atan2(lines(i).point1(2)-lines(i).point2(2),lines(i).point1(1)-lines(i).point2(1));
           angle2 = atan2(lines(j).point1(2)-lines(j).point2(2),lines(j).point1(1)-lines(j).point2(1));
              
           diff_angle = angle1-angle2;
           
           if abs(diff_angle) < pi
               th = linspace(angle1,2*pi-abs(diff_angle),50);
               x = r*cos(th)+lines(i).point2(1);
               y = r*-sin(th)+lines(i).point2(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
               disp('if')
               
           else
               th = linspace(angle2,angle1,50);
               x = r*cos(th)+lines(i).point2(1);
               y = r*-sin(th)+lines(i).point2(2);
               plot(x,y,'MarkerSize',5,'color','r');
               
           end
            
            
            disp('4')
           end
        
    end
end




for i = 1:length(intersect_point)
    
    %temp1 = deg2rad(intersections(i,1).theta);
    %temp2 = deg2rad(intersections(i,2).theta);
    
    
    
    if temp1 < 0
        test1 = temp1;
        temp1 = temp1 + 2*pi;
    end
    
    if temp2 < 0
        test2 = temp2;
        temp2 = temp2+2*pi;
        
    end
    
    if temp1 < temp2
        
        th = linspace(temp1, temp2, 100);
        th = th+temp1;
    
    else
        th = linspace(temp2, temp1, 100);
        th = th + temp2;
        
        %x,y coordinate
        %should be x,height -y
        
    end
    
%     
%     
%     R = 10;  %or whatever radius you want
%     x = R*cos(th) + intersect_point(i,1);
%     y = R*sin(th) + intersect_point(i,2);
%     plot(x,y); axis equal;
end


    
    
    
% highlight the longest line segment
%plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');