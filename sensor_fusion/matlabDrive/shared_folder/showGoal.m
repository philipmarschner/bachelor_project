figure
show(trueMap)
hold on
if length(goal)/2==2
    plot(goal(1),goal(2),'.','MarkerSize',50)
    plot(goal(3),goal(4),'.','MarkerSize',50)
elseif length(goal)/2==3
    plot(goal(1),goal(2),'.','MarkerSize',50)
    plot(goal(3),goal(4),'.','MarkerSize',50)
    plot(goal(5),goal(6),'.','MarkerSize',50)
end
title('goal')