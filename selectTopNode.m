function [topNode, Open] = selectTopNode(Open, ~, dir)
% select next node to expand

% no_visit: not_visited_id
no_visit = ~[Open.List.visited];

if sum(no_visit)>0
    costArray = [[Open.List(no_visit).fCost];
        abs([Open.List(no_visit).dir]-dir);
        find(no_visit)]';
    
    [~,sortInds]=sortrows(costArray(:,1:2));
    open_top_ind = costArray(sortInds(1),3);
else
    error('no path!')
end

Open.List(open_top_ind).visited = 1;
topNode = Open.List(open_top_ind);

end
