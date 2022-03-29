%% Create global "true" occupancy map (to be replaced in fucntion input)
clear;
clc;
close all;

run('config.m') %run config file with properties
%%Test data
%Properties



util = utilities(deltaQ,obstacleCheckSteps);

%a.add(2,2)

%a.multiplyBy(4,5)



dim = length(start)/2;

map = imread(mapPath); map = map(:, :, 2); %Load map and make 2dim
mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = binaryOccupancyMap(mapOccupancy);
x = trueMap.GridSize(2); y = trueMap.GridSize(1);


%trueMap = binaryOccupancyMap(120,80);
figure(1)
show(trueMap)
title('Global map')
hold on
% plot(start(1),start(2),'.','MarkerSize',20,'Color','b')
% plot(start(3),start(4),'.','MarkerSize',20,'Color','r')
% plot(goal(1),goal(2),'x','MarkerSize',20,'Color','b')
% plot(goal(3),goal(4),'x','MarkerSize',20,'Color','r')
hold off
close all;

%% Graph creation with qinit
%NodeTable = table(start,'VariableNames',{'Configuration'});
%G = graph(1,NodeTable); 




G = graph();

NodeProps = table(1,start, ...
    'VariableNames', {'nodeID','conf'});

G = addnode(G,NodeProps);



%% RRT algoritme
% Algorithm BuildRRT
%    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
%    Output: RRT graph G
%
%    G.init(qinit)
%    for k = 1 to K do
%        qrand ← RAND_CONF()
%        qnear ← NEAREST_VERTEX(qrand, G)
%        qnew ← NEW_CONF(qnear, qrand, Δq)
%        G.add_vertex(qnew)
%        G.add_edge(qnear, qnew)
%    return G


for i = 1:iterations

    if(rand < 1 - epsilonGoal)  %Sample goal epsilon times
        qrand = util.randq(x,y,dim); %adds randomly sampled configuration
    else
        qrand = goal;
    end
    
    %if(~isLegal(qrand,trueMap)) %resamples if random configuration is illegal
    %    continue;
    %end
    
    qnear = nearest_node(G,qrand); %finds nodeID for node in graph close


    qnew = util.newq(qnear.conf(1,:),qrand); %Take step towards sampled node
    if(~isLegal(qnew,trueMap)) %resamples if random configuration is illegal
        continue;
    end
    
    if(~util.isLegalPath(qnear.conf(1,:),qnew,trueMap)) %resamples if random configuration is illegal
        continue;
    end

    G = util.addConfiguration(G,qnear,qnew);
    
    %If dist to goal < deltaQ, connect to goal, if possible
    dist = norm(goal-G.Nodes.conf(end,:));
    if(dist < deltaGoal)
        [G, connected]= util.connectToGoal(goal,G,trueMap);
        if(connected)
            disp("goal found");
            break;
        end
    end
end
%% END OF RRT ALGORITHM

show(trueMap)
hold on
plot(G.Nodes.conf(:,1),G.Nodes.conf(:,2),'.','Color','b')
plot(G.Nodes.conf(:,3),G.Nodes.conf(:,4),'.','Color','r')