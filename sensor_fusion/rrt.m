%% Create global "true" occupancy map (to be replaced in fucntion input)
clear;
clc;
close all;

run('config.m') %run config file with properties
%%Test data
%Properties
deltaQ = 2;
x = 120; y = 80;

util = utilities();

%a.add(2,2)

%a.multiplyBy(4,5)

start = [40 60];
%start = [5 5 95 95];
goal = [60 80 30 5];
MAX_ITERATIONS = 1000;
dim = length(start)/2;

map = imread("big_world.png"); map = map(:, :, 2); %Load map and make 2dim
mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = binaryOccupancyMap(mapOccupancy);

%trueMap = binaryOccupancyMap(M,N);
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


for i = 1:150
    qrand = util.randq(x,y,dim); %adds randomly sampled configuration
    if(~isLeagal(qrand,trueMap)) %resamples if random configuration is illegal
        hej = 'skipped'
        continue;
    end
    
    qnear = nearest_node(G,qrand); %finds nodeID for node in graph close
    qnew = util.newq(qnear.conf(1,:),qrand,deltaQ); %Take step towards sampled node
    if(~isLeagal(qnew,trueMap)) %resamples if random configuration is illegal
        hej = 'skipped'
        continue;
    end

    %Add new node to graph
    NodeProps = table(numnodes(G)+1,qnew, ...
    'VariableNames', {'nodeID','conf'});
    G = addnode(G,NodeProps);

    %Add edge between nearest node and new node
    G = addedge(G,qnear.nodeID,numnodes(G));
end

show(trueMap)
hold on
plot(G.Nodes.conf(:,1),G.Nodes.conf(:,2),'.')