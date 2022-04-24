function node = nearest_node(g,q) %find node closest to node a in graph g
    
    node = g.Nodes(knnsearch(g.Nodes.conf,q),:); 
end