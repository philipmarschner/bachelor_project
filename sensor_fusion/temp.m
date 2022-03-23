% Create global "true" occupancy map
clear;
clc;




map = imread("big_world.png"); map = map(:, :, 2); %Load map and make 2dim
M = width(map); N = height(map);


mapNorm = double(map)/255;
mapOccupancy = 1 - mapNorm;
trueMap = occupancyMap(mapOccupancy);
%figure(1)
%show(trueMap)


start_config = [0,0,10,10];

goal_config = [10,10,0,0];

G = graph();

NodeProps = table(1,[1 2 3 4], ...
    'VariableNames', {'name','conf'});

G = addnode(G,NodeProps);

NodeProps = table(2,[5 6 7 8], ...
    'VariableNames', {'name','conf'});

G = addnode(G,NodeProps);

%NodeProps = table({'ATL' 'ANC'}', [false true]', ...
 %   'VariableNames', {'Name' 'WIFI'});

%G = addnode(G,NodeProps);

%G = addedge(G,2,3,1,'a');