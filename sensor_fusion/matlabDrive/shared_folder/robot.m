classdef robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lidar
        awarenessSection
        map
        threshold
        
    end
    
    methods
        function obj = robot(lidar,awarenessSection,threshold,map)
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.lidar = lidar;
            obj.awarenessSection = awarenessSection & ~occupancyMatrix(map);
            obj.threshold = threshold;
            obj.map = map;
            
        end
        
        
        
        function vis = robotVisibleSector(obj,q,map)
            %Returns nubmer from 0 to 1, where 0 is no visibility and 1 is 100%
            
            
            
            %should update map from global before
            temp = ones(map.GridSize(1),map.GridSize(2));
            occ = binaryOccupancyMap(temp);
           
            
            
            
            pose = [q(1) q(2) 0]; % x y theta
            [ranges, angles] = obj.lidar(pose,map);
            ranges(isnan(ranges)) = obj.lidar.Range(2);
            
            scan = lidarScan(round(ranges),round(angles));
          
            
            x = ranges.*cos(angles) + pose(1);
            y = ranges.*sin(angles) + pose(2);
            
            
            insertRay(occ,[pose(1) pose(2)],[x y]);
            
           
            vis = occ;
           
            

           
            
            

          
        end
        
        
        
        function vis_scalar = robotSectorVis(obj,total_poly)
            
            %intersection = intersect(total_poly,obj.awarenessSection);
            
            intersection = total_poly & obj.awarenessSection;
            
            vis_scalar = sum(intersection,'all')/sum(obj.awarenessSection,'all');
            
            %vis_scalar = area(intersection) / area(obj.awarenessSection);
           
            
        end
        
        
        function legal = legalVisibility(obj,total_poly)
            
            
            if robotSectorVis(obj,total_poly) > obj.threshold
                legal = true;
                
            else
                legal = false;
            end
            
        end
                 
        function output = inZone(obj,pos)
            output = obj.awarenessSection(pos(2),pos(1));
        end
        
        
        %make callback to update map when new ros scan is available

    end
end

