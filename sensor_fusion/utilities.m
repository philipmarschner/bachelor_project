classdef utilities
   properties
       deltaQ
       obstacleCheckSteps
   end
   methods
      function obj = utilities(deltaQ_, obstacleCheckSteps_)
         obj.deltaQ = deltaQ_;
         obj.obstacleCheckSteps = obstacleCheckSteps_;
      end
      function r = add(obj,a,b)
         r = a+b;
      end

      function output = randq(obj, width, height, dim)
         % Returns random configuration
         w = randi(width,1,dim);
         h = randi(height,1,dim);
         output = reshape([w; h],[],1)';
      end
      
      function qNew = newq(obj, qnear, qrand)
         qNew = qnear + ((qrand-qnear)/(norm(qrand-qnear)))*obj.deltaQ;
         qNew = round(qNew);
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

      function newGraph = addConfiguration(obj, gOld, qnear, qnew)
              %Add new node to graph
        NodeProps = table(numnodes(gOld)+1,qnew, ...
            'VariableNames', {'nodeID','conf'});
        newGraph = addnode(gOld,NodeProps);

        %Add edge between nearest node and new node
        newGraph = addedge(newGraph,qnear.nodeID,numnodes(newGraph));
      end
   end
end