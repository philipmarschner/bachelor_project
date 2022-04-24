function output = obstacleFreePath(q1,q2,map,obstacleCheckSteps)
    %works for 2d point
    x = linspace(q1(1), q2(1), round(obstacleCheckSteps));
    y = linspace(q1(2), q2(2), round(obstacleCheckSteps));
    points = [x(:), y(:)];
    for i = 1:round(obstacleCheckSteps)
        if(checkOccupancy(map,points(i,:)))
                output = false;
                return
        end
    end
    output = true;
end