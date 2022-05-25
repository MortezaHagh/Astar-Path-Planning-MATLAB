function neighbors = neighbors8(TopNode, Closed, Model)
% nodeNumber pNode gCost fCost dir


curentXY = Model.Nodes.cord(:,TopNode.nodeNumber);
currentX = curentXY(1);
currentY = curentXY(2);

newDir = double('n');

nNeighbors=0;
for ix= 1:-1:-1
    for iy= 1:-1:-1
        
        if (ix~=iy || ix~=0) % &&
            % The node itself is not its successor
            %(i==0 || j==0)  % eliminate corner nodes -> 4 node
            newX = currentX +ix;
            newY = currentY +iy;
            
            % check if the new node is within limits
            if((newX>=Model.Map.xMin && newX<=Model.Map.xMax) && ...
                    (newY>=Model.Map.yMin && newY<=Model.Map.yMax))
                newNodeNumber = TopNode.nodeNumber+ix+(iy*(Model.Map.xMax-Model.Map.xMin+1));
                
                % check if it is in Closed list
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
    end
end

neighbors.count=nNeighbors;
if nNeighbors~=0
    neighbors.List = list;
else
    neighbors.List= [];
end

end