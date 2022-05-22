function neighbors = neighbors4(topnode, closed, model)
% arrange neighbors based on robot direction
% node pnode cost_g cost_f dir

xy = model.nodes.cord(:,topnode.node);
dir = topnode.dir;
x = xy(1);
y = xy(2);
nc=0; % neighbors.count

% prioritize expanded node based on direction
q1=[1 0; 0 1; 0 -1; -1 0];
q2=[-1 0; 0 1; 0 -1; 1 0];
q3=[0 1; 1 0; -1 0; 0 -1];
q4=[0 -1; 1 0; -1 0; 0 1];
if dir==int32('r'); qs=q1; end
if dir==int32('l'); qs=q2; end
if dir==int32('u'); qs=q3; end
if dir==int32('d'); qs=q4; end

for k=1:4
    % nn: new node
    i=qs(k,1);  j=qs(k,2);
    
    % direction
    if all([i,j]==[1,0]);  nn_dir=int32('r'); end
    if all([i,j]==[-1,0]); nn_dir=int32('l'); end
    if all([i,j]==[0,1]);  nn_dir=int32('u'); end
    if all([i,j]==[0,-1]); nn_dir=int32('d'); end
    
    nn_x = x+i;
    nn_y = y+j;
    
    % check if the new node is within limits
    if((nn_x>=model.xmin && nn_x<=model.xmax) && (nn_y>=model.ymin && nn_y<=model.ymax))
        new_node = topnode.node+i+(j*(model.xmax-model.xmin+1));

        
        % check if it is in Closed list
        if ~any(new_node==closed.nodes)
            nc=nc+1;
            list(nc).visited = 0;
            list(nc).node = new_node;
            list(nc).pnode = topnode.node;
            list(nc).cost_g = topnode.cost_g + Distance(x, y, nn_x, nn_y, model.dist_type);
            cost_h = Distance(model.xt, model.yt, nn_x, nn_y, model.dist_type);
            list(nc).cost_f = list(nc).cost_g + cost_h;
            list(nc).dir = nn_dir;
        end
    end
end

neighbors.count=nc;
if nc~=0
    neighbors.list = list;
else
    neighbors.list= [];
end

end