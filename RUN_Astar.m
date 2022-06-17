% A*: Single Robot Path Planning Algorithm - MATLAB
% Main code for running the algorithm.
% Morteza Haghbeigi, m.haghbeigi@gmail.com


% Initialization
clc
clear
close

%% settings
Model.expandMethod = 'heading';  % random heading, tiebreaking
Model.distType = 'manhattan';    % euclidean manhattan;
Model.adjType = '4adj';          % '4adj'  '8adj'

%% create Map and Model - loading a Map Matrix

% % load Map and create model - (1:free, o:obstacles)
%  load(map_name, 'Map');
% Model = createModelFromMap(Map, Model);

% % add robot data to model
% Model = addRobotToModel(Model);

%% Create Map and Model by User
Model = createModelBase(Model);

%% optimal path by Astar
% Path: nodeNumbers, coords, dirs
tic
[Model, Path] = myAStar(Model);
Sol = Path;
Sol.pTime = toc;
Sol.cost = costL(Sol.coords);
Sol.smoothness = smoothness(Sol.coords);
% [Sol.cost, Sol.solChar]= costLinear(Model, Sol.coords);

%% modify path
tic
Mpath = modifyPath (Model, Path);
Msol = Mpath;
Msol.pTime = Sol.pTime + toc;
Msol.cost = costL(Msol.coords);
Msol.smoothness = smoothness(Msol.coords);
% [Msol.cost, Msol.solChar] = costLinear(Model, Msol.coords);

%% # display data and plot solution
disp(['process time for path= ' num2str(Sol.pTime)])
disp(Sol)

plotModel(Model)
plotSolution(Sol.coords, Msol.coords)
% plotAnimation2(Sol.coords)

%% clear temporal data
clear adj_type dist_type

