function path_dirs = nodes2dirs(nodes, model)

n_nodes = numel(nodes);
path_dirs = zeros(1,n_nodes);

for i=2:n_nodes
    node1_xy = model.nodes.cord(:,nodes(i-1));
    node2_xy = model.nodes.cord(:,nodes(i));
    
    dd = node2_xy - node1_xy;
    dx = dd(1);
    dy = dd(2);
    
    if dx==0 && dy==0
        path_dirs(i-1) = 'o';
    elseif dx==1 && dy==0
        path_dirs(i-1) = 'r';
    elseif dx==-1 && dy==0
        path_dirs(i-1) = 'l';
    elseif dy==1 && dx==0
        path_dirs(i-1) = 'u';
    elseif dy==-1 && dx==0
        path_dirs(i-1) = 'd';
    else
        path_dirs(i-1) = 'e';
    end
end

path_dirs(end) = path_dirs(end-1);
path_dirs = char(path_dirs);

end