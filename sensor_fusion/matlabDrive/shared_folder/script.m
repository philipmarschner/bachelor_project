%% Create global "true" occupancy map
clear;
clc;
close all;
map = imread("big_world.png"); map = map(:, :, 2); %Load map and make 2dim
M = width(map); N = height(map);


mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = occupancyMap(mapOccupancy);
figure(1)
subplot(2,3,2)
show(trueMap)
title('Global map')


%% Planner
R1_planner = plannerAStarGrid(trueMap);
R1_start = [2 3]; R1_goal = [70 100];
R1_plan = plan(R1_planner, R1_start, R1_goal);
subplot(2,3,1)
show(R1_planner)

R2_planner = plannerAStarGrid(trueMap);
R2_start = [75 5]; R2_goal = [10 110];
R2_plan = plan(R2_planner, R2_start, R2_goal);
subplot(2,3,3)
show(R2_planner)



%% Create occupancy map based on simulated lidar data

occ = occupancyMap(M, N, 5);
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi pi]; 
minRange = 0; maxRange = 20; lidar.Range = [minRange maxRange];



%Generate lidar scan from pose
pauseTime = 0.5; %pause for x aomut of seconds between iterations
for i = 1:max(length(R1_plan),length(R2_plan))
    %Robot 1
    if(i <= length(R1_plan))
        R1_pose = [R1_plan(i,2) 81-R1_plan(i,1) pi/2]; % x y theta
        [ranges, angles] = lidar(R1_pose,trueMap);
        scan = lidarScan(ranges,angles);
        insertRay(occ,R1_pose,scan,maxRange);
        
        %Plot robot view
        subplot(2,3,4)
        plot(scan)
        title('Robot 1')
        hold on
        plot(0,0,'.','MarkerSize',20)
        hold off
    end
    %Robot 2
    if(i <= length(R2_plan))
        R2_pose = [R2_plan(i,2) 81-R2_plan(i,1) pi/2]; % x y theta
        [ranges, angles] = lidar(R2_pose,trueMap);
        scan = lidarScan(ranges,angles);
        insertRay(occ,R2_pose,scan,maxRange);
        
        %Plot robot view
        subplot(2,3,6)
        plot(scan)
        title('Robot 2')
        hold on
        plot(0,0,'.','MarkerSize',20)
        hold off
    end

    %Plot occupancy grid
    subplot(2,3,5)
    show(occ)
    hold on
    %Robot 1
    if(i <= length(R1_plan))
        plot(R1_plan(i,2),80-R1_plan(i,1),'.','Color','g','MarkerSize',20)
    end
    if(i <= length(R2_plan))
        plot(R2_plan(i,2),80-R2_plan(i,1),'.','Color','r','MarkerSize',20)
    end
    hold off

    pause(0.05)
end

%plannerRRTStar

