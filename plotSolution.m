function plotSolution(path_coords, mpath_coords)

x = path_coords(:,1);
y = path_coords(:,2);

% path
plot(x, y, 'b', 'LineWidth', 2);
plot(x(2:end-1), y(2:end-1), 'bo', 'MarkerFaceColor', 'b');

% modified path
if ~isempty(mpath_coords)
    mx = mpath_coords(:,1);
    my = mpath_coords(:,2);
    plot(mx, my, 'g');
    plot(mx(2:end-1), my(2:end-1), 'go','MarkerFaceColor','g');
end

end
