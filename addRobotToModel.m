function Model = addRobotToModel(Model)
% add Robot data to Model


%% robot data
% dir: direction , r,l,u,d
Robot.dir = double('d');

% start & goal - start & target coordinates
% Robot.xs = 0;
% Robot.ys = 0;
% Robot.xt = 6;
% Robot.yt = 7;
Robot.xs = 6;
Robot.ys = 2;
Robot.xt = 40;
Robot.yt = 20;

%  start & goal - node numbers
Robot.startNode = (Robot.ys-Model.Map.yMin)*(Model.Map.xMax-Model.Map.xMin+1)+Robot.xs+abs(Model.Map.xMin-1);
Robot.targetNode = (Robot.yt-Model.Map.yMin)*(Model.Map.xMax-Model.Map.xMin+1)+Robot.xt+abs(Model.Map.xMin-1);

%% save model
Model.Robot = Robot;

end
