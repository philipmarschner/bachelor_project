figure
show(trueMap)
hold on
if length(start)/2==2
    plot(start(1),start(2),'.','MarkerSize',50)
    plot(start(3),start(4),'.','MarkerSize',50)
elseif length(start)/2==3
    plot(start(1),start(2),'.','MarkerSize',50)
    plot(start(3),start(4),'.','MarkerSize',50)
    plot(start(5),start(6),'.','MarkerSize',50)
end

title('Start')