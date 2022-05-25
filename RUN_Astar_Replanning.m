% A* replanning: Single Robot Path Planning Algorithm - MATLAB
% with remapping in case of new obstacle detection
% Main code for running the algorithm.
% Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

%% setting
Model.distType = 'manhattan';  % euclidean manhattan;
Model.adjType = '4adj';          % '4adj'  '8adj'

%% create Map and Model - Using a Map Matrix

% % % create or load Map
% %[Map, Name] = createMap(path_, name_, extension_);
% [Map, Name] = createMap('D:\00-Robotics\02-Robot Path Planning\Methods\Maps', 'warehouse-10-20-10-2-1', '.map');
% % load(map_name, 'Map');
% 
% % create model
% Model = createModelFromMap(Map, Model);
% 
% % add robot data to model
% Model = addRobotToModel(Model);

%% Create Map and Model by User
Model = createModelAstar(Model);

Model_init = Model;

%% start timer
tic

%% pp
% initial pp t0 (each displacement btween nodes takes 1 sec)
[Model, Path] = myAStar(Model);

% preallocation
newObstNode=0;
Sol.nodeNumbers = [0];

t=0;
pt=0;
while Sol.nodeNumbers(end)~=Model.Robot.targetNode
    t=t+1;
    pt=pt+1;
    
    % update model (insert new obstacles)
    if t==5
        newObstNode(end+1) = 37;
        Model.Obst.x(end+1) = 5;
        Model.Obst.y(end+1) = 0;
        Model.Obst.nodeNumber(end+1) = 37;
        Model.Obst.count = Model.Obst.count+1;
    end
    % check if path replanning is needed
    if any(Path.nodeNumbers(pt) == Model.Obst.nodeNumber)
        Model.Robot.startNode = Sol.nodeNumbers(end);
        xy = Model.Nodes.cord(:, Model.Robot.startNode);
        Model.xs = xy(1);
        Model.ys = xy(2);
        dirs = nodes2dirs(Sol.nodeNumbers, Model);
        Model.dir = dirs(end);
        [Model, Path] = myAStar(Model);
        pt=2;
    end
    
    % update final sol
    Sol.nodeNumbers(t) = Path.nodeNumbers(pt);
end

% process time
Sol.pTime = toc;

% final solution
Sol.coords = nodes2coords(Sol.nodeNumbers, Model);
Sol.dirs = nodes2dirs(Sol.nodeNumbers, Model);

%% update model and calculate cost
newObstNode = newObstNode(2:end);
if numel(newObstNode)>0
    Model_init.xc = [Model.Obst.x Model.Nodes.cord(1,newObstNode)];
    Model_init.yc = [Model.Obst.y Model.Nodes.cord(2,newObstNode)];
end
[Sol.cost, Sol.solChar] = costLinear(Model_init, Sol.coords);

%% display data and plot solution
disp(Sol)

plotModel(Model)
plotSolution(Sol.coords,[])
% plotAnimation2(sol.coords)

%% clear temporal data
clear xy t pt dirs newobstNode adj_type dist_type
