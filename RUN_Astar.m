% A*: Single Robot Path Planning Algorithm - MATLAB
% Main code for running the algorithm.
% Morteza Haghbeigi, m.haghbeigi@gmail.com


% Initialization
clc
clear
close

%% settings
Model.distType = 'manhattan';  % euclidean manhattan;
Model.adjType = '4adj';          % '4adj'  '8adj'

%% create Map and Model - Using a Map Matrix

% % % create map or load Map
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
Model = createModelBase(Model);
% model = createModelObstacles('Obstacle9', model);

%% optimal path by Astar
% Path: nodeNumbers, coords, dirs
tic
[Model, Path] = myAStar(Model);
Sol = Path;
Sol.pTime = toc;
Sol.smoothness = smoothness(Sol.coords);
[Sol.cost, Sol.solChar]= costLinear(Model, Sol.coords);

%% modify path
tic
Mpath = modifyPath (Model, Path);
Msol = Mpath;
Msol.pTime = Sol.pTime + toc;
Msol.smoothness = smoothness(Msol.coords);
[Msol.cost, Msol.solChar] = costLinear(Model, Msol.coords);

%% # display data and plot solution
disp(['process time for path= ' num2str(Sol.pTime)])
disp(['process time for mpath= ' num2str(Msol.pTime)])
disp(Sol)

plotModel(Model)
plotSolution(Sol.coords, Msol.coords)
% plotAnimation2(sol.coords)

%% clear temporal data
clear adj_type dist_type

