function pathDirs = nodes2dirs(nodeNumbers, Model)
% return direction array of path nodes

n_nodes = numel(nodeNumbers);
pathDirs = zeros(1,n_nodes);

for i=2:n_nodes
    node1_xy = Model.Nodes.cord(:,nodeNumbers(i-1));
    node2_xy = Model.Nodes.cord(:,nodeNumbers(i));
    
    dd = node2_xy - node1_xy;
    dx = dd(1);
    dy = dd(2);
    
    if dx==0 && dy==0
        pathDirs(i-1) = 'o';
    elseif dx==1 && dy==0
        pathDirs(i-1) = 'r';
    elseif dx==-1 && dy==0
        pathDirs(i-1) = 'l';
    elseif dy==1 && dx==0
        pathDirs(i-1) = 'u';
    elseif dy==-1 && dx==0
        pathDirs(i-1) = 'd';
    else
        pathDirs(i-1) = 'e';
    end
end

pathDirs(end) = pathDirs(end-1);
pathDirs = char(pathDirs);

end