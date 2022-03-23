function node = nearest_node(g,q) %find node closest to node a in graph g
    
    qNearest = Inf;

    for i = 1:numnodes(g)
        temp_dist = norm(q - g.Nodes.conf(i,:));

        if temp_dist < qNearest
            qNearest = temp_dist;
            node = g.Nodes(i,:);
        end
    end 
end