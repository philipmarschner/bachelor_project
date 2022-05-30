classdef simulator
    %Simulator class

    properties
        route
        map
        mapPath;
        numRobots
        robots
        mapHeight
        mapWidth
        simulatorPauseTime
        lidar
        lidarMaxRange
        trueMap
        interpolationSteps
        zones
        visThreshold
    end

    methods
        function obj = simulator(route,mapPath,robots,lidar,lidarMaxRange,simulatorPauseTime, interpolationSteps,zones,visThreshold)
            % Construct an instance of this class
            obj.map = imread(mapPath);
            obj.mapPath = mapPath;
            obj.numRobots = size(route,2)/2;
            obj.robots = robots;
            [obj.mapHeight,obj.mapWidth] = deal(size(obj.map,1),size(obj.map,2));
            obj.lidarMaxRange = lidarMaxRange;
            obj.lidar = lidar;
            obj.simulatorPauseTime = simulatorPauseTime;
            map = obj.map(:, :, 2); %Load map and make 2dim
            mapNorm = double(map)/255;
            mapOccupancy = 1 - mapNorm;
            obj.trueMap = binaryOccupancyMap(mapOccupancy);
            obj.interpolationSteps = interpolationSteps;
            obj.route = obj.expandRoute(flip(route),obj.interpolationSteps);
            obj.zones = zones;
            obj.visThreshold = visThreshold;
        end

        function simulate(obj)
            if obj.numRobots == 2
                obj.twoRobotSimulator(obj);
            else
                msg = 'Simulator only implemented for multiple robots';
                error (msg);
            end
        end

        function twoRobotSimulator(obj)
            %% Path follower simulation for 2 robots
            figure()
            H = gcf;
            H.WindowState = 'maximized';

            %Show map
            subplot(2,3,2)
            obj.showPathAndZones;
            title('Map of environment')

            %Show zone for robot 1
            subplot(2,3,1)
            obj.plotMapWithZones(1);

            %Show zone for robot 2
            subplot(2,3,3)
            obj.plotMapWithZones(2);

            occ = occupancyMap(obj.mapWidth, obj.mapHeight, 5); % Empty map to be generated
            for i = 1:length(obj.route)
                %Robot 1
                    R1_pose = [obj.route(i,1) obj.route(i,2) pi/2]; % x y theta
                    [ranges, angles] = obj.lidar(R1_pose,obj.trueMap);
                    scan = lidarScan(ranges,angles);
                    insertRay(occ,R1_pose,scan,obj.lidarMaxRange);

                    %Plot robot view
                        subplot(2,3,4)
                        plot(scan)
                        title('Robot 1')
                        hold on
                        plot(0,0,'.','MarkerSize',20)
                        hold off
            
                %Robot 2
                    R2_pose = [obj.route(i,3) obj.route(i,4) pi/2]; % x y theta
                    [ranges, angles] = obj.lidar(R2_pose,obj.trueMap);
                    scan = lidarScan(ranges,angles);
                    insertRay(occ,R2_pose,scan,obj.lidarMaxRange);

                    %Plot robot view
                        subplot(2,3,6)
                        plot(scan)
                        title('Robot 2')
                        hold on
                        plot(0,0,'.','MarkerSize',20)
                        hold off
            
                %Plot occupancy grid
                    subplot(2,3,5)
                    show(occ)
                    hold on
                    %Robot 1
                        plot(obj.route(i,1),obj.route(i,2),'.','Color','g','MarkerSize',20)
                    %Robot 2
                        plot(obj.route(i,3), obj.route(i,4),'.','Color','r','MarkerSize',20)
                    hold off
            
                pause(obj.simulatorPauseTime)
            end
        end

        function plotVisibility(obj)
            vis = [];
            for i = 1:obj.numRobots
                figure
                vis = obj.getVisibility(i);
                vis = sortrows(vis);
                vis = obj.fixVis(vis);
                area(vis(:,1),vis(:,4),'FaceColor',[0.7 0.7 0.7])
                hold on
                idx = vis(:,3) > 0;
                zoneVis = vis(idx,:);
                area(zoneVis(:,1),zoneVis(:,4),'FaceColor',[0 0.4470 0.7410])
                plot([0 length(vis)*2],[obj.visThreshold obj.visThreshold],'--','Color','r')
                xlim([0 length(obj.route)])
                ylim([0 1.1])
                title('Visibility of critical zone')
                xlabel('Time')
                ylabel('Visibile % of zone')
                fig = gcf;
                fig.Position(3:4) = [250, 200];
                %legend('Visiblity of zone','Location','southeast')
            end
        end

        function vis = fixVis(~,vis)
            idx = [];
            for i = 1:length(vis)-1
                if(vis(i,1) ~= vis(i+1,1))
                    %Skip if not same time
                    continue;
                end
                if (sign(vis(i,3)) ~= sign(vis(i+1,3)))
                    %Skip if one is in zone
                    continue;
                end
                if ( vis(i,4) <  vis(i+1,4) )
                    idx = [idx, i];
                elseif ( vis(i,4) > vis(i+1,4))
                    idx = [idx, i+1];
                end
            end
            for i = 1:length(idx)
                vis(idx(i),:,:,:) = [nan nan nan nan];
            end
            vis(any(isnan(vis), 2), :) = [];
        end

        function Vis = getVisibility(obj,robotID)
            Vis = [];
            for i=1:length(obj.route)
                tempq = reshape(obj.route(i,:),2,[])';
                zoneIDs = obj.robots(robotID).inZone(tempq(robotID,:));
                combinedRobotVisibility = obj.totalVisibility(obj.route(i,:));
                if ~isempty(zoneIDs)
                    for j = 1:length(zoneIDs)
                        visScalar = obj.robots(robotID).compareToZone(combinedRobotVisibility,zoneIDs(j));
                        Vis = [Vis; i, robotID, j, visScalar]; 
                    end  
                else
                    for j = 1:size(obj.robots(robotID).awarenessSection,3)
                        visScalar = obj.robots(robotID).compareToZone(combinedRobotVisibility,j);
                        Vis = [Vis; i, robotID, -j, visScalar];
                    end
                end
            end
        end

        function visibilArea = totalVisibility(obj,q)
            visibilArea = zeros(obj.trueMap.GridSize(1),obj.trueMap.GridSize(2));
            
            for i = 1:obj.numRobots
                temp_map = obj.robots(i).robotVisibleSector(obj.iConfiguration(i,q),obj.trueMap);
                temp_matrix = ~occupancyMatrix(temp_map);
                visibilArea = visibilArea | temp_matrix;
            end
        end

        function conf = iConfiguration(~,i,q)
            conf = [q((i-1)*2+1) q((i-1)*2+2)];
        end

        function plotRouteTime(obj)
            figure;
            imshow(imread(obj.mapPath));
            hold on
            for i=1:obj.numRobots
                x = obj.route(:,(i-1)*2+1)';
                y = obj.route(:,(i-1)*2+2)';
                y = obj.mapHeight - y;
                z = zeros(size(x));
                time = 1:length(x);
                surface([x;x],[y;y],[z;z],[time;time],...
                    'facecol','no',...
                    'edgecol','interp',...
                    'linew',2);
            end
        end

        function plotRoute(obj)
            title('Critical zones of the environment')
            hold on
            plot([start(:,1) start(:,3)],size(zones,1)-[start(:,2) start(:,4)],'.','MarkerSize',MarkerSize,'Color','b')
            plot([goal(:,1) goal(:,3)],size(zones,1)-[goal(:,2) goal(:,4)],'.','MarkerSize',MarkerSize,'Color','r')
            plot(route(:,1),size(zones,1)-route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
            plot(route(:,3),size(zones,1)-route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
            %plot(route(:,5),size(zones,1)-route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
            plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
            legend('Start','Goal','Robot1','Robot2','Critical zone')
        end



        function plotMapWithZones(obj,robotID)
            [im ,bitMap] = imread('zoneTemplate.png');
            template = imcrop(im,bitMap,[0 0 size(obj.robots(robotID).awarenessSection,2) size(obj.robots(robotID).awarenessSection,1)]);
            template = ind2rgb(template,bitMap);
            
            %Create mask for extracting from template
            mask = zeros(size(obj.robots(robotID).awarenessSection,1:2));
            for i=1:size(obj.robots(robotID).awarenessSection,3)
                mask = mask | squeeze(obj.robots(robotID).awarenessSection(:,:,i));
            end
            %Expand mask to RGB
            mask(:,:,2) = mask;
            mask(:,:,3) = mask(:,:,1);
            
            %Load map
            img = obj.map;
            
            %Crop mask from map
            img(mask==1) = 0;

            %Remove all of template that is out of mask
            template(mask==0) = 0;
            
          
            
            %Crop mask from map
            img(mask==1) = 0;
            
            %Add template to map
            template(img==255) = 255;
            
            imshow(template);
                msg = sprintf('Critical zones for robot %i', robotID);
                title(msg)
            hold on
            plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
            legend('Critical zone','Location','southeast')
        end

        function expandedRoute = expandRoute(~,route,interpolationSteps)
            expandedRoute = route(1,:);
            for i = 1:length(route)-1
                for j = 1:width(route)
                    if (route(i,j) == route(i+1,j))
                        temp(:,j) = ones(interpolationSteps+1,1)*route(i,j);
                    else
                        temp(:,j) = (route(i,j):(route(i+1,j)-route(i,j))/(interpolationSteps):route(i+1,j))';
                    end
                end
                expandedRoute = [expandedRoute; temp(2:interpolationSteps+1,:)];
            end
            expandedRoute = round(expandedRoute);
        end

        function showPathAndZones(obj)
            MarkerSize = 50;
            LineWidth = 2;
            [im ,bitmap] = imread('zoneTemplate.png');
            template = imcrop(im,bitmap,[0 0 size(obj.zones,2) size(obj.zones,1)]);
            template = ind2rgb(template,bitmap);
            
            %Create mask for extracting from template
            mask = zeros(size(obj.zones,1:2));
            for i=1:size(obj.zones,3)
                mask = mask | squeeze(obj.zones(:,:,i));
            end
            %Expand mask to RGB
            mask(:,:,2) = mask;
            mask(:,:,3) = mask(:,:,1);
            
            %Load map
            img = obj.map;

            %Remove all of template that is out of mask
            template(mask==0) = 0;
            template(img==0) = 0;
            
            
            %Crop mask from map
            img(mask==1) = 0;
            
            %Add template to map
            template(img==255) = 255;
            
            imshow(template);
            title('Path planned for the robots')
            hold on
            

            if obj.numRobots == 3
                plot([obj.route(1,1) obj.route(1,3) obj.route(1,5)],size(obj.zones,1)-[obj.route(1,2) obj.route(1,4) obj.route(1,6)],'.','MarkerSize',MarkerSize,'Color','b')
                plot([obj.route(length(obj.route),1) obj.route(length(obj.route),3) obj.route(length(obj.route),5)],size(obj.zones,1)-[obj.route(length(obj.route),2) obj.route(length(obj.route),4) obj.route(length(obj.route),6)],'.','MarkerSize',MarkerSize,'Color','r')
                plot(obj.route(:,1),size(obj.zones,1)-obj.route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
                plot(obj.route(:,3),size(obj.zones,1)-obj.route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
                plot(obj.route(:,5),size(obj.zones,1)-obj.route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
                plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
                legend('Start','Goal','Robot1','Robot2','Robot3','Critical zone','Location','south')
            elseif obj.numRobots == 2
                plot([obj.route(1,1) obj.route(1,3)],size(obj.zones,1)-[obj.route(1,2) obj.route(1,4)],'.','MarkerSize',MarkerSize,'Color','b')
                plot([obj.route(length(obj.route),1) obj.route(length(obj.route),3)],size(obj.zones,1)-[obj.route(length(obj.route),2) obj.route(length(obj.route),4)],'.','MarkerSize',MarkerSize,'Color','r')
                plot(obj.route(:,1),size(obj.zones,1)-obj.route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
                plot(obj.route(:,3),size(obj.zones,1)-obj.route(:,4),'-','LineWidth',LineWidth,'Color','#D95319')
                %plot(route(:,5),size(zones,1)-route(:,6),'-','LineWidth',LineWidth,'Color','#EDB120')
                plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
                legend('Start','Goal','Robot1','Robot2','Critical zone','Location','southeast')
            elseif obj.numRobots == 1
                plot([obj.route(1,1)],size(obj.zones,1)-[obj.route(1,2)],'.','MarkerSize',MarkerSize,'Color','b')
                plot([obj.route(length(obj.route),1)],size(obj.zones,1)-[obj.route(length(obj.route),2)],'.','MarkerSize',MarkerSize,'Color','r')
                plot(obj.route(:,1),size(obj.zones,1)-obj.route(:,2),'-','LineWidth',LineWidth,'Color','#0072BD')
                plot([0 0.1],[0 0.1],'--','LineWidth',2,'Color','r');
                legend('Start','Goal','Robot1','Critical zone','Location','southeast')
                title('Path planned for the robot')
            end
        end
    end
end