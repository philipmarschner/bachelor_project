
%step size for planner
deltaQ = 3*7;


load('start.mat');
load('goal.mat');
load('dynamicZones.mat'); %Dynamic zones

%load('zones.mat');  %User defined zones

%If dist to goal < deltaQ, connect to goal, if possible
deltaGoal = 5*deltaQ;

%radius of neighbourhood

neighbourhood_radius = 3*deltaQ;

%number of samples in rrt planner
maxIterations = 15000;

%number of steps to check between two nodes
obstacleCheckSteps = 5;

%map to plan through
mapPath = 'hestesko.png';

%Bias towards goal
epsilonGoal = 0;

%start and goal configurations
%start = [30*6 5*9 110*6 60*9 60*6 65*9];
%goal = [60*6 65*9 30*6 5*9 110*6 60*9];

% dim = amount of robots
dim = length(start)/2;

% Map
map = imread(mapPath); map = map(:, :, 2); %Load map and make 2dim
mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = binaryOccupancyMap(mapOccupancy);
x = trueMap.GridSize(2); y = trueMap.GridSize(1);
clear map mapNorm mapOccupancy;

%lidar specs type 1
lidar_min_range = 0;
lidar_max_range = 120;
lidar_min_angle = -pi;
lidar_max_angle = pi;
lidar_type_1 = rangeSensor;


lidar_type_1.HorizontalAngle = [lidar_min_angle lidar_max_angle]; 
lidar_type_1.Range = [lidar_min_range lidar_max_range];
lidar_type_1.HorizontalAngleResolution = 0.00872665; %0.5 degrees

% Simulator
simPauseTime = 0.01;
interpolationSteps = 5;
visThreshold = 0.9;