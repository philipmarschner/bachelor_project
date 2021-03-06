classdef rrtTestScript
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
        neighbourhood_radius
        goalNodeID
    end

    methods
        
        function obj = rrtTestScript(map, robots, start, goal, deltaQ, deltaGoal, rrtMaxIterations, epsilonGoal,obstacleCheckSteps,radius)
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
            obj.neighbourhood_radius = radius;
            

            %Graph creation
                G = graph();
                NodeProps = table(1,start,0, ...
                    'VariableNames', {'nodeID','conf','dist'});
                G = addnode(G,NodeProps);
            obj.legalConfigurations = G;
            
        end
        
        function conf = iConfiguration(obj,i,q)
            conf = [q((i-1)*2+1) q((i-1)*2+2)];
        end

        function output = plan(obj) % Returns path from start to goal
            goalFound = false;
            for i = 1:obj.rrtMaxIterations                
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
                

                qnear = obj.nearest_node(obj.legalConfigurations,qrand); %finds nodeID for node in graph close
            
                qnew = obj.newq(qnear.conf(1,:),qrand); %Take step towards sampled node
            
                if(~obj.isLegalConfiguration(qnew,obj.map)) %resamples if random configuration is illegal
                    continue;
                end
            
                if(~obj.isLegalConnection(qnear.conf(1,:),qnew,obj.map)) %resamples if random configuration is illegal
                    continue;
                end
                
                
                
                neighbours = obj.find_neighbours(obj.legalConfigurations,qnew,obj.neighbourhood_radius);
                
                start_to_qnew = qnear.dist+norm(qnear.conf(1,:)-qnew);
                
                %find closest neighbour
                
                for j=1:height(neighbours)
                    
                    dist_from_start = neighbours(j,:).dist+norm(neighbours(j,:).conf(1,:)-qnew);
                    
                    if(dist_from_start<start_to_qnew)
                        qnear = neighbours(j,:);
                        start_to_qnew = dist_from_start;
                        
                    end
                    
                end
                
                
                
            
                obj.legalConfigurations = obj.addConfiguration(obj.legalConfigurations,qnear,qnew);
            
                %If dist to goal < deltaQ, connect to goal, if possible
                dist = norm(obj.goal-obj.legalConfigurations.Nodes.conf(end,:));
                if(dist < obj.deltaGoal && ~goalFound)
                    [obj.legalConfigurations, connected] = obj.connectToGoal(obj.goal,obj.legalConfigurations,obj.map);
                    if(connected)
                        disp("goal found");
                        obj.goalNodeID = numnodes(obj.legalConfigurations);
                        goalFound = true;
                    end
                end
                
                %optimize graph
                
                %obj.optimize_graph(obj.legalConfigurations.Nodes(end,:),neighbours);
                
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
                    if ~(obj.robots(i).legalVisibility(obj.totalVisibility(q)) > 0.9)
                        output = false;
                        return
                    end
                end
            end
           
                
            output = true;
        end
        
        function newGraph = addConfiguration(obj, gOld, qnear, qnew)
            %Add new node to graph
            NodeProps = table(numnodes(gOld)+1,qnew,qnear.dist+(norm(qnew-qnear.conf(1,:))), ...
                'VariableNames', {'nodeID','conf','dist'});
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

        function path = start2goal(obj)
            childID = obj.goalNodeID;
            path = obj.legalConfigurations.Nodes.conf(childID,:);

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
        
        function neighbours = find_neighbours(obj,g,q,r)
            configs = obj.legalConfigurations.Nodes.conf(:,:);
            [neighbourIDs D] = rangesearch(configs,q,r,'Distance',@distfun);
            
            neighbourIDs = cell2mat(neighbourIDs);
            neighbours = table;
            nodes = obj.legalConfigurations.Nodes;
            for i = 1:length(neighbourIDs)
                neighbours = [neighbours; nodes(neighbourIDs(i),:)];
            end
        end
       
        function optimize_graph(obj,new_node,neighbours)
            for i=1:height(neighbours)
                dist_to_new_node = norm(new_node.conf(1,:)-neighbours(i,:).conf(1,:));
                if(new_node.dist+dist_to_new_node < neighbours(i,:).dist)
                    %remove neighbours last edge and replace with edge to
                    %new node
                    
                    obj.replace_parent_edge(neighbours(i,:),new_node);
                    
                    %disp(dist_to_new_node)
                    neighbours(i,:).dist = new_node.dist+dist_to_new_node;

                    obj.updateDistanceFromStart(neighbours(i,:));
                end
                    
               
                
            end
            
            
        end
        
        
        function replace_parent_edge(obj,node,new_parent)
            edges = obj.legalConfigurations.Edges.EndNodes;
            for i=1:numedges(obj.legalConfigurations)
                
                if(edges(i,2)==node.nodeID)
                    edgeTemp = obj.legalConfigurations.Edges.EndNodes(i,:);
                    obj.legalConfigurations = rmedge(obj.legalConfigurations,edgeTemp(1),edgeTemp(2));
                    
                    obj.legalConfigurations = addedge(obj.legalConfigurations,new_parent.nodeID,node.nodeID);
                    
                    
                    %obj.legalConfigurations.Edges.EndNodes(i,1) =new_parent.nodeID;
                    
                end
                
            end
            
            
            
        end
        
        
        function vec = plot_visibility(obj)
            
            vec = [];
            
             for i = 1:obj.numRobots      
                if obj.robots(i).inZone(tempq(i,:))
                    
                    
                    
                    if ~(obj.robots(i).legalVisibility(obj.totalVisibility(q)))
                        output = false;
                        return
                    end
                end
             end
        end
        
        function updateDistanceFromStart(obj, node)
            childIDs = obj.getChildrens(node);
            if isempty(childIDs)
                return
            end

            for i = 1:length(childIDs)
                start2parrent = obj.legalConfigurations.Nodes(node.nodeID,:).dist;
                parrent2child = norm(node.conf(1,:) - obj.legalConfigurations.Nodes(childIDs(i),:).conf(1,:));
                obj.legalConfigurations.Nodes(childIDs(i),:).dist = start2parrent + parrent2child;
                obj.updateDistanceFromStart(obj.legalConfigurations.Nodes(childIDs(i),:));
            end
        end
        
        function childIDs = getChildrens(obj, node)
            childIDs = [];
            edges = obj.legalConfigurations.Edges.EndNodes;
            for i=1:numedges(obj.legalConfigurations)
                if(edges(i,1)==node.nodeID)
                    childIDs = [childIDs; obj.legalConfigurations.Edges.EndNodes(i,2)];
                end
            end
        end

    end
end