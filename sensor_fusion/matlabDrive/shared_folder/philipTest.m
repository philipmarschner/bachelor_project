% Main file
clear;
clc;
%close all;
    

%% Setup of robots
run('config.m')
map = ones(80,120);
map(:,1) = 0; map(1,:) = 0; map(80,:) = 0; map(:,120) = 0;
mapOccupancy = 1 - map;
trueMap = binaryOccupancyMap(mapOccupancy);
clear map mapOccupancy;
zones = zeros(80,120);
robots(1) = robot(lidar_type_1,zones,0.9,trueMap);
maxIterations = 80;

%% Plan a path 
start = [40, 50]; goal = [80, 10];
epsilonGoal = 0.05;
FP = rrtTestScript(trueMap, robots, start, goal, deltaQ, deltaGoal, ...
    maxIterations, epsilonGoal,2*deltaQ,neighbourhood_radius);

FP = FP.plan; % List of waypoints in form of configuration
%route = FP.start2goal();

%% Simulate
figure;
show(trueMap)
hold on
plot(start(1),start(2),'.','Color','b','MarkerSize',40);
plot(goal(1),goal(2),'.','Color','r','MarkerSize',40);
for i = 1:height(FP.legalConfigurations.Edges)
    startNodeID = FP.legalConfigurations.Edges.EndNodes(i,1);
    goalNodeID = FP.legalConfigurations.Edges.EndNodes(i,2);
    points(1,:) = FP.legalConfigurations.Nodes.conf(startNodeID,:);
    points(2,:) = FP.legalConfigurations.Nodes.conf(goalNodeID,:);
    plot(points(:,1),points(:,2),'k');
end
title('test')
title('Non-greedy RRT')
legend('start','goal')

