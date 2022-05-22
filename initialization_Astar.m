function [closed, open, topnode] = initialization_Astar(model)


% Closed list
% Closed: put all obstacles on the Closed list
closed.count = model.numOfObs;
closed.nodes = model.obstNode;

% Open list
% open.count
% open.list.
% visited node pnode cost_g cost_f dir

% set the starting node as the first node in Open
topnode.visited = 1;
topnode.node=model.startNode;
topnode.pnode=model.startNode;
topnode.dir = model.dir;
topnode.cost_g = 0;
cost_h = Distance(model.xs, model.ys,  model.xt,  model.yt, model.dist_type);
topnode.cost_f = topnode.cost_g + cost_h;

% insert start node in open list
open.count = 1;
open.list(1) = topnode;

% add last node to Closed
closed.count = closed.count+1;
closed.nodes(closed.count) = topnode.node;

end