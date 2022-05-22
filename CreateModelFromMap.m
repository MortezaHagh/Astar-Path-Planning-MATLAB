function model = CreateModelFromMap(Map, model)

[H,W] = size(Map);

xmin = 1;
ymin = 1;
xmax = W;
ymax = H;
limArea = max(W, H);
x_node_num=xmax-xmin+1;
y_node_num=ymax-ymin+1;


k=1;
xc=[];
yc=[];
obstNode = [];
nodes_count = H*W;
nodes.number = zeros(1,nodes_count);
nodes.cord = zeros(2, nodes_count);
for i=1:H
    for j=1:W
        nodes.number(1,k)=k;
        nodes.cord(:,k)=[j,i]';
        if Map(i,j)==0
            xc=[xc j];
            yc=[yc i];
            obstNode = [obstNode k];
        end
        k = k+1;
    end
end

%% save model
model.x_node_num = x_node_num;
model.y_node_num = y_node_num;
model.obstNode = obstNode;
model.numOfObs=numel(xc);
model.limArea = limArea;
model.nodes = nodes;
model.obst_r = 0.25;
model.xmin = xmin;
model.xmax = xmax;
model.ymin = ymin;
model.ymax = ymax;
model.xc = xc;
model.yc = yc;

% % save
% model_name = 'model_1';
% save(model_name, 'model');

end