function vis = visibility(q,worldMap)
%Returns nubmer from 0 to 1, where 0 is no visibility and 1 is 100%
    run('config.m') %run config file with properties
    lidar = rangeSensor;
    lidar.HorizontalAngle = [lidar_min_angle lidar_max_angle]; 
    lidar.Range = [lidar_min_range lidar_max_range];
    lidar.HorizontalAngleResolution = 0.00872665; %0.5 degrees

    polyCombinedVis = polyshape; 

    for i = 1:2:length(q)
        pose = [q(i) q(i+1) 0]; % x y theta
        [ranges, angles] = lidar(pose,worldMap);
        ranges(isnan(ranges)) = lidar_max_range;
        x = ranges.*cos(angles) + pose(1); 
        y = ranges.*sin(angles) + pose(2);

        polyVis = polyshape(x,y); %Polygon of area in vision

        polyCombinedVis = union(polyVis,polyCombinedVis);
        
        
    end
    
    
    
%     vis = polyCombinedVis;
%     figure(2)
%     show(worldMap)
%     hold on
%     plot(polyCombinedVis)
end


