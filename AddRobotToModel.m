function model = AddRobotToModel(model)


%% robot
% dir: direction , r,l,u,d
dir = int32('d');

% start & goal
% xs = 100;
% ys = 10;
% xt = 100;
% yt = 30;

xs = 6;
ys = 2;
xt = 40;
yt = 20;

start_node = (ys-model.ymin)*(model.xmax-model.xmin+1)+xs+abs(model.xmin-1);
target_node = (yt-model.ymin)*(model.xmax-model.xmin+1)+xt+abs(model.xmin-1);

%% save model
model.targetNode = target_node;
model.startNode = start_node;
model.dir = dir;
model.xs = xs;
model.ys = ys;
model.xt = xt;
model.yt = yt;

end
