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
        function obj = robot(lidar,awarenessSection,threshold)
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.lidar = lidar;
            obj.awarenessSection = awarenessSection;
            obj.threshold = threshold;
            
        end
        
        
        
        function vis = robotVisibleSector(obj,q,map)
            %Returns nubmer from 0 to 1, where 0 is no visibility and 1 is 100%
            
            
            
            %should update map from global before
           
            pose = [q(1) q(2) 0]; % x y theta
            [ranges, angles] = obj.lidar(pose,map);
            ranges(isnan(ranges)) = obj.lidar.Range(2);
            x = ranges.*cos(angles) + pose(1);
            y = ranges.*sin(angles) + pose(2);

            vis = polyshape(x,y); %Polygon of area in vision

          
        end
        
        
        
        function vis_scalar = robotSectorVis(obj,total_poly)
            
            intersection = intersect(total_poly,obj.awarenessSection);
            
            vis_scalar = area(intersection) / area(obj.awarenessSection);
           
            
        end
        
        
        function legal = legalVisibility(obj,total_poly)
            
            
            if robotSectorVis(obj,total_poly) > obj.threshold
                legal = true;
                
            else
                legal = false;
            end
            
        end
                 
        function output = inZone(obj,pos)
            output = isinterior(obj.awarenessSection,pos);
        end
        
        
        %make callback to update map when new ros scan is available

    end
end

