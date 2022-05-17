classdef FleetPlanner
    %   Multidimensional fleet planner

    properties
        map                 % Occupancy grid of world
        start               % Start of planner
        goal                % Goal of planner
        deltaQ              % Stepsize for RRT
        deltaGoal           % If dist to goal < deltaQ, connect to goal, 
                            %   if possible
        robots              % List of robots
        rrtMaxIterations    % Max iterations for RRT
        % Self generated properties
        legalConfigurations % Graph of legal configurations
        epsilonGoal         % Bias towards sampling goal in RRT
        numRobots
        obstacleCheckSteps
        randomPoints
    end

    methods
        
        function obj = FleetPlanner(map, robots, start, goal, deltaQ, deltaGoal, rrtMaxIterations, epsilonGoal,obstacleCheckSteps)
            % Construct an instance of this class
            obj.map = map;
            obj.robots = robots;
            obj.start = start;
            obj.goal = goal;
            obj.deltaQ = deltaQ;
            obj.deltaGoal = deltaGoal;
            obj.rrtMaxIterations = rrtMaxIterations;
            obj.epsilonGoal = epsilonGoal;
            obj.numRobots = length(start)/2;
            obj.obstacleCheckSteps = obstacleCheckSteps;
            obj.randomPoints = [];

            %Graph creation
                G = graph();
                NodeProps = table(1,start, ...
                    'VariableNames', {'nodeID','conf'});
                G = addnode(G,NodeProps);
            obj.legalConfigurations = G;
            
        end
        
        function conf = iConfiguration(obj,i,q)
            conf = [q((i-1)*2+1) q((i-1)*2+2)];
        end

        function output = plan(obj) % Returns path from start to goal
            dist2goal = [];
            for i = 1:obj.rrtMaxIterations
                if i == obj.rrtMaxIterations
                    msg = 'Max iterations during planning';
                    
                    if ~connected
                        
                        error (msg);
                        
                    end
                end
                
                if(rand < 1 - obj.epsilonGoal)  %Sample goal epsilon times
                    qrand = obj.randq(obj.map.GridSize(2),obj.map.GridSize(1),obj.numRobots); %adds randomly sampled configuration
                    for j = 1:obj.numRobots
                        if(rand < obj.epsilonGoal)  %Sample goal individual
                            qrand((j-1)*2+1) = obj.goal((j-1)*2+1);
                            qrand((j-1)*2+2) = obj.goal((j-1)*2+2);
                        end
                    end
                else
                    qrand = obj.goal;
                end
                obj.randomPoints = [obj.randomPoints; qrand];

                qnear = obj.nearest_node(obj.legalConfigurations,qrand); %finds nodeID for node in graph close
                
                qnew = obj.newq(qnear.conf(1,:),qrand); %Take step towards sampled node
            
                if(~obj.isLegalConfiguration(qnew,obj.map)) %resamples if random configuration is illegal
                    continue;
                end
            
                if(~obj.isLegalConnection(qnear.conf(1,:),qnew,obj.map)) %resamples if random configuration is illegal
                    continue;
                end
            
                obj.legalConfigurations = obj.addConfiguration(obj.legalConfigurations,qnear,qnew);
            
                %If dist to goal < deltaQ, connect to goal, if possible
                dist = norm(obj.goal-obj.legalConfigurations.Nodes.conf(end,:));
                dist2goal = [dist2goal;dist];
                if(dist < obj.deltaGoal)
                    [obj.legalConfigurations, connected] = obj.connectToGoal(obj.goal,obj.legalConfigurations,obj.map);
                    if(connected)
                        disp("goal found");
                        %break;
                    end
                end
            end

            output = obj;
        end
        
        function total = totalPoly(obj,q)
            
            temp_poly = polyshape;
            
            
            for i = 1:obj.numRobots
                
                temp_poly = union(obj.robots(i).robotVisibleSector(iConfiguration(obj,i,q),obj.map),temp_poly);
                
            end
               
            total = temp_poly;
        end

        function visibilArea = totalVisibility(obj,q)
            
            visibilArea = zeros(obj.map.GridSize(1),obj.map.GridSize(2));
            
            
            for i = 1:obj.numRobots
                temp_map = obj.robots(i).robotVisibleSector(iConfiguration(obj,i,q),obj.map);
                temp_matrix = ~occupancyMatrix(temp_map);
                visibilArea = visibilArea | temp_matrix;
                
            end
        end
                
        function node = nearest_node(obj,g,q) %find node closest to node a in graph g
            node = g.Nodes(knnsearch(g.Nodes.conf,q),:); 
        end

        function output = randq(obj, width, height, numRobots)
            % Returns random configuration
            w = randi(width,1,numRobots);
            h = randi(height,1,numRobots);
            output = reshape([w; h],[],1)';
        end

        function qNew = newq(obj, qnear, qrand)
            qNew = qnear + ((qrand-qnear)/(norm(qrand-qnear)))*obj.deltaQ;
            qNew = round(qNew);
        end

        function isLegal = isLegalConnection(obj, q1, q2, map)
            tempq1 = reshape(q1,2,[])';
            tempq2 = reshape(q2,2,[])';
            
            for i = 1:length(tempq1(:,1))
                if(~obstacleFreePath(tempq1(i,:),tempq2(i,:),map,obj.obstacleCheckSteps))
                    isLegal = false;
                    return
                end
            end
            isLegal = true;
        end
        
        function [output] = isLegalConfiguration(obj,q,map)
            %Returns true if q is legal
            tempq = reshape(q,2,[])';
        
        
            %Check for equal workspace position
            if (length(q)>2)
                for i = 1:length(tempq)-1
                    for j = i+1:length(tempq)
                        if(isequal(tempq(i,:),tempq(j,:)))
                            output = false;
                            return
                        end
                    end
                end
            end
        
            %Check if position is freespace in workspace
            if (length(q)==2)
                if(checkOccupancy(map,q))
                        output = false;
                        return
                end
            end
        
            if (length(q)>2)
                for i = 1:length(tempq)
                    if(checkOccupancy(map,tempq(i,:)))
                        output = false;
                        return
                    end
                end
            end
            
            %check if critical section is visible, if any of the configurations is inside
            for i = 1:obj.numRobots      
                if obj.robots(i).inZone(tempq(i,:))
                    
                   
                    
                    if ~(obj.robots(i).legalVisibility(obj.totalVisibility(q)))
                        output = false;
                        return
                    end
                end
            end
           
                
            output = true;
        end
        
        function newGraph = addConfiguration(obj, gOld, qnear, qnew)
            %Add new node to graph
            NodeProps = table(numnodes(gOld)+1,qnew, ...
                'VariableNames', {'nodeID','conf'});
            newGraph = addnode(gOld,NodeProps);

            %Add edge between nearest node and new node
            newGraph = addedge(newGraph,qnear.nodeID,numnodes(newGraph));
        end

        function [newGraph, connected]= connectToGoal(obj,goal,gOld,map)
            q1 = gOld.Nodes(end,:);
            q2 = goal; %goal
            
            %Check if legal path to goal exists
            if(~obj.isLegalPath(q1.conf(1,:), q2, map))
                newGraph = gOld;
                connected = false;
                return
            end
            
            newGraph = obj.addConfiguration(gOld,q1,q2);
            connected = true;
        end
        
        function isLegal = isLegalPath(obj, q1, q2, map)
            tempq1 = reshape(q1,2,[])';
            tempq2 = reshape(q2,2,[])';
            for i = 1:length(tempq1(:,1))
                if(~obstacleFreePath(tempq1(i,:),tempq2(i,:),map,obj.obstacleCheckSteps))
                    isLegal = false;
                    return
                end
            end
            isLegal = true;
        end

        function path = start2goal(obj,nodeID)
            childID = nodeID;
            path = obj.legalConfigurations.Nodes.conf(nodeID,:);

            %Sorting
            config_array = table2array(obj.legalConfigurations.Edges);

            config_array = sortrows(config_array,2);
            
            %Create list of configurations from goal to start

            while true
                edge = config_array(childID - 1,:);
                parrentID = edge(1);
                path = [path; obj.legalConfigurations.Nodes.conf(parrentID,:)];
                childID = parrentID;
                if(childID == 1)
                    return;
                end
            end
        end
        
        
        function [inzone,out] = plot_route_visibility(obj,route)
            
           out = {}; 
           inzone = {};
           
            
           
           
           
               

            for i = 1:obj.numRobots    
                
                tempin = {};
                tempout = {};
                
                for j = 1:length(route)

                    if obj.robots(i).inZone(iConfiguration(obj,i,route(j,:)))

                   %inzone(i,:) = [inzone(i,:) [obj.robots(i).robotSectorVis(obj.totalVisibility(route(j,:))),route(j,:)]];
                   
                    tempin(1,end+1) = [tempin [obj.robots(i).robotSectorVis(obj.totalVisibility(route(j,:))),route(j,:)]];

                    else

                    tempout(1,end+1) = [tempout [obj.robots(i).robotSectorVis(obj.totalVisibility(route(j,:))),route(j,:)]];

                    end
                end
                
                
            end  
        end
    end
end