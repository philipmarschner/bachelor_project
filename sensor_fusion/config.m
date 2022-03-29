
%step size for planner
deltaQ = 1;

%If dist to goal < deltaQ, connect to goal, if possible
deltaGoal = 1*deltaQ;

%number of samples in rrt planner
iterations = 1000;

%number of steps to check between two nodes
obstacleCheckSteps = deltaQ*2;

%map to plan through
mapPath = "45deg_noObstacles.png";

%Bias towards goal
epsilonGoal = 0.05;

%start and goal configurations
start = [30 5 110 60];
goal = [60 65 23 23];
