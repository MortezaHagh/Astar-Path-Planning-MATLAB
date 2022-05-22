function path = optimalPath(model, open)


% starting from the last (target) node
path_nodes(1) = model.targetNode;
i=2;

% Traverse Open and determine the parent nodes
parent_ind = [open.list.node]==path_nodes(1);
parent_node = open.list(parent_ind).pnode;

% going back to start node
while parent_node ~= model.startNode
    path_nodes(i) = parent_node;
    parent_ind = [open.list.node]==parent_node;
    parent_node =open.list(parent_ind).pnode;
    i=i+1;
end

path.nodes = [model.startNode, flip(path_nodes)];
path.coords = nodes2coords(path.nodes, model);
path.dirs = nodes2dirs(path.nodes, model);

end
