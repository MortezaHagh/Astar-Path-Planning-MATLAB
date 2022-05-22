% A*: Single Robot Path Planning Algorithm - MATLAB
% By Morteza Haghbeigi, m.haghbeigi@gmail.com

% Initialization
clc
clear
close

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

%% optimal path by Astar
tic
[model, path] = myAStar(model);
sol = path;
sol.pTime = toc;
sol.smoothness = smoothness(sol.coords);
[sol.cost, sol.solChar]= costLinear(model, sol.coords);

%% modify path
tic
mpath = modifyPath (model, path);
msol = mpath;
msol.pTime = sol.pTime + toc;
msol.smoothness = smoothness(msol.coords);
[msol.cost, msol.solChar] = costLinear(model, msol.coords);

%% # display data and plot solution
disp(['process time for path= ' num2str(sol.pTime)])
disp(['process time for mpath= ' num2str(msol.pTime)])
disp(sol)

plotModel(model)
plotSolution(sol.coords, msol.coords)
% plotAnimation2(sol.coords)

%% clear temporal data
clear adj_type dist_type

