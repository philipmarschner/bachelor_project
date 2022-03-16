%% Create global "true" occupancy map
clear clc
map = imread("big_world.png"); map = map(:, :, 2); %Load map and make 2dim
M = width(map); N = height(map);


mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = occupancyMap(mapOccupancy);
figure(1)
show(trueMap)


%% Create occupancy map based on simulated lidar data

occ = occupancyMap(M, N, 'FreeThreshold',0.2cl);
lidar = rangeSensor;
lidar.HorizontalAngle = [-pi pi]; 
minRange = 0; maxRange = 20; lidar.Range = [minRange maxRange];

%Generate lidar scan from pose
pose = [5 5 pi/2]; % x y theta
currentPose = pose;
[ranges, angles] = lidar(currentPose,trueMap);

scan = lidarScan(ranges,angles);
insertRay(occ,currentPose,scan,maxRange);
figure(2)
plot(scan)
title('Ego View')



figure(3)
show(occ)


