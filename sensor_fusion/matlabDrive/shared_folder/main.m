% Main file
clear;
clc;
close all;
    

%% Setup of robots
run('config.m')
load('polygon.mat')
robots(1) = robot(lidar_type_1,zones(1),0.9);
robots(2) = robot(lidar_type_1,zones(2),0.9);


%% Plan a path 
FP = FleetPlanner(trueMap, robots, start, goal, deltaQ, deltaGoal, ...
    maxIterations, epsilonGoal,2*deltaQ);

FP = FP.plan; % List of waypoints in form of configuration
route = FP.start2goal(height(FP.legalConfigurations.Nodes));

%% Simulate
%SIM = simulator(route,trueMap,polygons,lidar_type_1,lidar_max_range,simPauseTime);
%SIM.twoRobotSimulator;
