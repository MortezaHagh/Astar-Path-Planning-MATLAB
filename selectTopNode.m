function [topNode, open] = selectTopNode(open, targetNode, dir)


% no_visit: not_visited_id
no_visit = ~[open.list.visited];

if sum(no_visit)>0
    cost_array = [[open.list(no_visit).cost_f];
        abs([open.list(no_visit).dir]-dir);
        find(no_visit)]';
    
    [~,sort_inds]=sortrows(cost_array(:,1:2));
    open_top_ind = cost_array(sort_inds(1),3);
else
    error('no path!')
end

open.list(open_top_ind).visited = 1;
topNode = open.list(open_top_ind);

end
