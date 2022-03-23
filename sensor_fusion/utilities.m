classdef utilities
   methods
      function r = add(obj,a,b)
         r = a+b;
      end

      function output = randq(obj, width, height, dim)
         % Returns random configuration
         w = randi(width,1,dim);
         h = randi(height,1,dim);
         output = reshape([w; h],[],1)';
      end

      function qNew = newq(obj, qnear, qrand, detlaQ)
         qNew = qnear + ((qrand-qnear)/(norm(qrand-qnear)))*detlaQ;
         qNew = round(qNew);
      end
   end
end