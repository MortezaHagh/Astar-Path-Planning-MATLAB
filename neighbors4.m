function Neighbors = neighbors4(TopNode, Closed, Model)
% arrange neighbors based on cost robot and direction
% nodeNumber pNode gCost fCost dir

curentXY = Model.Nodes.cord(:,TopNode.nodeNumber);
currentDir = TopNode.dir;
currentX = curentXY(1);
currentY = curentXY(2);

% prioritize expanding based on direction
% q# = [dx1 dy1; ...]
q=[1 0 double('r'); 0 1 double('u'); 0 -1 double('d'); -1 0 double('l')];
q1=q([1,2,3,4],:); % r
q2=q([4,2,3,1],:); % l
q3=q([2,1,4,3],:); % u
q4=q([3,1,4,2],:); % d

switch currentDir
    case int32('r')
        qs=q1;
    case int32('l')
        qs=q2;
    case int32('u')
        qs=q3;
    case int32('d')
        qs=q4;
end

nNeighbors=0;
for iNeighbor=1:4
    % new node
    ix=qs(iNeighbor,1);
    iy=qs(iNeighbor,2);
    newX = currentX+ix;
    newY = currentY+iy;
    newDir = qs(iNeighbor,3);
    
    % check if the new node is within limits
    if((newX>=Model.Map.xMin && newX<=Model.Map.xMax) && ...
            (newY>=Model.Map.yMin && newY<=Model.Map.yMax))
        newNodeNumber = TopNode.nodeNumber+ix+(iy*(Model.Map.xMax-Model.Map.xMin+1));
        
        % add newNode to list if it is not in Closed list
        if ~any(newNodeNumber==Closed.nodeNumber)
            nNeighbors=nNeighbors+1;
            list(nNeighbors).visited = 0;
            list(nNeighbors).nodeNumber = newNodeNumber;
            list(nNeighbors).pNode = TopNode.nodeNumber;
            list(nNeighbors).gCost = TopNode.gCost + Distance(currentX, currentY, newX, newY, Model.distType);
            hCost = Distance(Model.Robot.xt, Model.Robot.yt, newX, newY, Model.distType);
            list(nNeighbors).fCost = list(nNeighbors).gCost + hCost;
            list(nNeighbors).dir = newDir;
        end
    end
end

% Neighbors
Neighbors.count=nNeighbors;
if nNeighbors~=0
    Neighbors.List = list;
else
    Neighbors.List= [];
end

end