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
            obj.threshold = threshold;
            obj.map = map;
            
            obj.awarenessSection = []; %image coordinate
            
            for i = 1 : size(awarenessSection,3)
                obj.awarenessSection=  cat(3,obj.awarenessSection, squeeze(awarenessSection(:,:,i)) & ~occupancyMatrix(map));
            end
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
        
        
        
        function vis_scalar = compareToZone(obj,total_poly,zoneID)
            %intersection = intersect(total_poly,obj.awarenessSection);
            intersection = total_poly & squeeze(obj.awarenessSection(:,:,zoneID));
            vis_scalar = sum(intersection,'all')/sum(squeeze(obj.awarenessSection(:,:,zoneID)),'all');
            %vis_scalar = area(intersection) / area(obj.awarenessSection);
        end
        
        
        function legal = isLegalVisibility(obj,total_poly,zoneIDS)
            
            for i = 1:length(zoneIDS)

                if obj.compareToZone(total_poly,zoneIDS(i)) < obj.threshold
                    legal = false;
                    return;
                end
            end  
            legal = true;  
        end
                 
        function zoneIDs = inZone(obj,pos)

            temp = [];
            for i = 1:size(obj.awarenessSection,3)
                if obj.awarenessSection(size(obj.awarenessSection,1)-pos(2),pos(1),i)
                    temp = [temp i];
                end
            end

            zoneIDs = temp;
        end
        
        %make callback to update map when new ros scan is available

    end
end

