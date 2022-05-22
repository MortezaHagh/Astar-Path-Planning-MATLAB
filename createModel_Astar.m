function model=createModel_Astar(model)

disp('Create Model Astar');

%% robot
% dir: direction , r,l,u,d
dir = int32('d');

% start & goal
xs = 0;
ys = 0;
xt = 6;
yt = 7;

%% Area
limArea = 28;
xmin = -1;
xmax = limArea;
ymin = -1;
ymax = limArea;

x_node_num=xmax;
y_node_num=ymax;

% Obstacle
r = 0.25;
xc1=[3 3 3 5 5 5 7 7 7 9 9 9 11 11 11];
yc1=[3 4 5 3 4 5 3 4 5 3 4 5 3 4 5];

xc1 = xc1+1;
yc1 = yc1+1;

xc2=[xc1 xc1 xc1 xc1];
yc2=[yc1 yc1+6 yc1+11 yc1+16];

xc3=xc2+12;
yc3=yc2;

xc=[xc2 xc3];
yc=[yc2 yc3];

%% nodes & adj
k = 0;
adj = cell(1,1);
for j = ymin:ymax
    for i = xmin:xmax
        k = k+1;
        adj{k,1} = k;                   % node number
        adj{k,2} = [i, j];              % node coordinates
        nodes.number(1, k) = k;         % node number
        nodes.cord(1:2, k) = [i, j]';   % node coordinates
        
        if i == xs && j == ys
            start_node = k;             % start node number
        elseif i == xt && j == yt
            target_node = k;            % target (final) node number
        end
    end
end

% obstacle node numbers
obstNode = zeros(1,length(xc));
for i = 1:length(xc)
    for j = 1:size(nodes.number,2)
        if xc(i) == nodes.cord(1,j) && yc(i) == nodes.cord(2,j)
            obstNode(i) = nodes.number(j);
        end
    end
end

%% save model
model.x_node_num = x_node_num;
model.y_node_num = y_node_num;
model.targetNode = target_node;
model.startNode = start_node;
model.obstNode = obstNode;
model.numOfObs=numel(xc);
model.limArea = limArea;
model.nodes = nodes;
model.xmin = xmin;
model.xmax = xmax;
model.ymin = ymin;
model.ymax = ymax;
model.obst_r = r;
model.dir = dir;
model.adj = adj;
model.xs = xs;
model.ys = ys;
model.xt = xt;
model.yt = yt;
model.xc = xc;
model.yc = yc;

%% plot model
% plotModel(model);

end