% Main file
clear;
clc;
close all;
    

%% Setup of robots
run('config.m')

robots(1) = robot(lidar_type_1,zones,visThreshold,trueMap);
robots(2) = robot(lidar_type_1,zones,visThreshold,trueMap);
robots(3) = robot(lidar_type_1,zones,visThreshold,trueMap);
%robots(3) = robot(lidar_type_1,squeeze(zones(:,:,3)),visThreshold,trueMap);

%% Plan a path 
FP = FleetPlanner(trueMap, robots, start, goal, deltaQ, deltaGoal, ...
    maxIterations, epsilonGoal,obstacleCheckSteps,neighbourhood_radius);

FP = FP.plan; % List of waypoints in form of configuration
route = FP.start2goal();

%% Simulate
SIM = simulator(route,mapPath,robots,lidar_type_1,lidar_max_range,simPauseTime,interpolationSteps,zones,visThreshold);
%SIM.twoRobotSimulator;
SIM.plotVisibility;
pause(10)
figure
SIM.showPathAndZones;


