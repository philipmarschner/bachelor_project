ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = trueMap;
ss.WeightTheta = 0;
ss.StateBounds = [trueMap.XWorldLimits; trueMap.YWorldLimits; -pi,pi];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxConnectionDistance = deltaQ;
s = rng(100, 'twister'); % repeatable result

iterations = 250:250:5000;
iterations = [iterations 5500:500:10000];
iterations = [iterations 11000:1000:20000];
data = [];
for i = 1:length(iterations)
    %rng(s);
    close all
    figure
    planner.MaxIterations = iterations(i);
    [pthObj, solnInfo] = plan(planner,[start(1:2) 0],[goal(1:2) 0]);
    trueMap.show
    hold on
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
    plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
    title(sprintf('%i iteratons',iterations(i)))
    saveas(gcf,sprintf('TestResults/rrt%i.png',iterations(i)))
    data = [data; iterations(i), solnInfo.PathCosts(end)];
end
figure
plot(data(:,1),data(:,2),'-')
