function [model, path] = myAStar(model)


% initialization
[closed, open, topNode] = initialization_Astar(model);

%%%% Start Algorithm

while topNode.node ~= model.targetNode
    
    % finding neighbors (successors)
    if strcmp(model.adj_type, '4adj')
        neighbors=neighbors4(topNode,  closed, model);
    elseif strcmp(model.adj_type, '8adj')
        neighbors=neighbors8(topNode,  closed, model);
    end
    
    % update or extend Open list with the successor nodes
    open = updateOpen(open, neighbors);
    
    % select new Top Node
    [topNode, open] = selectTopNode(open, model.targetNode, topNode.dir);
    closed.count = closed.count+1;
    closed.nodes(closed.count) = topNode.node;
end

% optimal paths coordinations, nodes, directions
path = optimalPath(model, open);

end
