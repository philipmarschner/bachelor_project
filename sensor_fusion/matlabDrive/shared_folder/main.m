% Main file
clear;
clc;
close all;
disp('Todo: Plot visibility over simulation, sammenlign med RRT')

%% Setup of robots
run('config.m')


robots(1) = robot(lidar_type_1,squeeze(zones(1,:,:)),0.9,trueMap);
robots(2) = robot(lidar_type_1,squeeze(zones(2,:,:)),0.9,trueMap);


%% Plan a path 
FP = FleetPlanner(trueMap, robots, start, goal, deltaQ, deltaGoal, ...
    maxIterations, epsilonGoal,2*deltaQ);

FP = FP.plan; % List of waypoints in form of configuration
route = FP.start2goal(height(FP.legalConfigurations.Nodes));

%% Simulate
SIM = simulator(route,trueMap,mapPath,zones,lidar_type_1,lidar_max_range,simPauseTime);
SIM.plotPath;
%SIM.twoRobotSimulator;
