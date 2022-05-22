% A* replanning: Single Robot Path Planning Algorithm - MATLAB
% with remapping in case of new obstacle detection
% By Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

% adding paths
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB')
addpath('D:\00-Robotics\02-Robot Path Planning\Methods\Astar-Single & Multi-MATLAB\SRPP')

%% setting
model.dist_type = 'manhattan';  % euclidean manhattan;
model.adj_type='4adj';          % '4adj'  '8adj'

%% create model standard

% % % create or load Map
% %[Map, Name] = CreateMap(path_, name_, extension_);
% [Map, Name] = CreateMap('D:\00-Robotics\02-Robot Path Planning\Methods\Maps', 'warehouse-10-20-10-2-1', '.map');
% % load(map_name, 'Map');
% 
% % create model
% model = CreateModelFromMap(Map, model);
% 
% % add robot data to model
% model = AddRobotToModel(model);

%% Create My Model
model = createModel_Astar(model);

model0 = model;

%% start timer
tic

%% pp
% initial pp t0 (each displacement btween nodes takes 1 sec)
[model, path] = myAStar(model);

% preallocation
newobstNode=0;
sol.nodes = [0];

t=0;
pt=0;
while sol.nodes(end)~=model.targetNode
    t=t+1;
    pt=pt+1;
    
    % update model (insert new obstacles)
    if t==5
        newobstNode(end+1) = 37;
        model.obstNode(end+1) = 37;
        model.numOfObs = model.numOfObs+1;
    end
    % check if path replanning is needed
    if any(path.nodes(pt) == model.obstNode)
        model.startNode = sol.nodes(end);
        xy = model.nodes.cord(:, model.startNode);
        model.xs = xy(1);
        model.ys = xy(2);
        dirs = nodes2dirs(sol.nodes, model);
        model.dir = dirs(end);
        [model, path] = myAStar(model);
        pt=2;
    end
    
    % update final sol
    sol.nodes(t) = path.nodes(pt);
end

% process time
sol.pTime = toc;

% final solution
sol.coords = nodes2coords(sol.nodes, model);
sol.dirs = nodes2dirs(sol.nodes, model);

%% update model and calculate cost
newobstNode = newobstNode(2:end);
if numel(newobstNode)>0
    model0.xc = [model.xc model.nodes.cord(1,newobstNode)];
    model0.yc = [model.yc model.nodes.cord(2,newobstNode)];
end
[sol.cost, sol.solChar] = costLinear(model0, sol.coords);

%% display data and plot solution
disp(sol)

plotModel(model0)
plotSolution(sol.coords,[])
% plotAnimation2(sol.coords)

%% clear temporal data
clear xy t pt dirs newobstNode adj_type dist_type
