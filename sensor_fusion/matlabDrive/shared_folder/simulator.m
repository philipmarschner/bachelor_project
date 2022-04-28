classdef simulator
    %Simulator class

    properties
        route
        map
        mapPath;
        numRobots
        polygons
        mapHeight
        mapWidth
        simulatorPauseTime
        lidar
        lidarMaxRange
    end

    methods
        function obj = simulator(route,map,mapPath,polygons,lidar,lidarMaxRange,simulatorPauseTime)
            % Construct an instance of this class
            obj.route = route;
            obj.map = map;
            obj.mapPath = mapPath;
            obj.numRobots = size(route,2)/2;
            obj.polygons = polygons;
            [obj.mapHeight,obj.mapWidth] = deal(map.GridSize(1),map.GridSize(2));
            obj.lidarMaxRange = lidarMaxRange;
            obj.lidar = lidar;
            obj.simulatorPauseTime = simulatorPauseTime;
        end

        function simulate(obj)
            if obj.numRobots == 2
                obj.twoRobotSimulator(obj);
            end
        end

        function twoRobotSimulator(obj)
            %% Path follower simulation for 2 robots
            figure()
            H = gcf;
            H.WindowState = 'maximized';

            %Show map
            subplot(2,3,2)
            show(obj.map);

            %Show zone for robot 1
            subplot(2,3,1)
            show(obj.map);
            hold on
            plot(obj.polygons.zones(1));
            hold off

            %Show zone for robot 2
            subplot(2,3,3)
            show(obj.map);
            hold on
            plot(obj.polygons.zones(2));
            hold off

            occ = occupancyMap(obj.mapWidth, obj.mapHeight, 5); % Empty map to be generated

            for i = 1:length(obj.route)
                %Robot 1
                    R1_pose = [obj.route(i,1) obj.route(i,2) pi/2]; % x y theta
                    [ranges, angles] = obj.lidar(R1_pose,obj.map);
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
                    [ranges, angles] = obj.lidar(R2_pose,obj.map);
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

        function plotPath(obj)
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
    end
end