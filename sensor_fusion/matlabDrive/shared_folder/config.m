
%step size for planner
deltaQ = 1.5;

polygons = load('polygon.mat');

%If dist to goal < deltaQ, connect to goal, if possible
deltaGoal = 1*deltaQ;

%number of samples in rrt planner
maxIterations = 1000;

%number of steps to check between two nodes
obstacleCheckSteps = deltaQ*2;

%map to plan through
mapPath = "45deg_noObstacles.png";

%Bias towards goal
epsilonGoal = 0.10;

%start and goal configurations
start = [30 5 110 60];
goal = [60 65 23 23];

%Robot 1 parameters
robot1_start = [30 5];
robot1_goal = [60 65];

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
lidar_max_range = 20;
lidar_min_angle = -pi;
lidar_max_angle = pi;
lidar_type_1 = rangeSensor;


lidar_type_1.HorizontalAngle = [lidar_min_angle lidar_max_angle]; 
lidar_type_1.Range = [lidar_min_range lidar_max_range];
lidar_type_1.HorizontalAngleResolution = 0.00872665; %0.5 degrees

% Simulator
simPauseTime = 0.01;
