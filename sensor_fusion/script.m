%% Create global "true" occupancy map
clear;
clc;
map = imread("big_world.png"); map = map(:, :, 2); %Load map and make 2dim
M = width(map); N = height(map);


mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = occupancyMap(mapOccupancy);
figure(1)
show(trueMap)


%% Planner
planner = plannerAStarGrid(trueMap);
start = [2 3]; goal = [70 100];
plan = plan(planner, start, goal);
show(planner)


%% Create occupancy map based on simulated lidar data

occ = occupancyMap(M, N);
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi pi]; 
minRange = 0; maxRange = 20; lidar.Range = [minRange maxRange];



%Generate lidar scan from pose
pauseTime = 0.5; %pause for x aomut of seconds between iterations
for i = 1:length(plan)
    pose = [plan(i,2) 80-plan(i,1) pi/2]; % x y theta
    currentPose = pose;
    [ranges, angles] = lidar(currentPose,trueMap);

    scan = lidarScan(ranges,angles);
    insertRay(occ,currentPose,scan,maxRange);
    figure(2)
    
    %Plot robot view
    plot(scan)
    title('Ego View')
    hold on
    plot(0,0,'.','MarkerSize',20)
    hold off

    %Plot occupancy grid
    figure(3)
    show(occ)
    hold on
    plot(plan(i,2),80-plan(i,1),'.','MarkerSize',20)
    hold off


    %Pause 
    %pause(0.05)
end


