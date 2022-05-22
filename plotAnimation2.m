function plotAnimation2(path_coords)
% animated points with tale! concurrent.

x = path_coords(:,1);
y = path_coords(:,2);

Color='g';

h1=plot(x(1), y(1), 'o', 'MarkerFaceColor', Color, 'MarkerEdgeColor', Color(1,:));
h2=plot(x(1:2), y(1:2), 'Color', Color,'LineWidth', 2);
pause(0.2)

for i=2:numel(x)
    
    set(h1, 'XData', x(i))
    set(h1, 'YData', y(i))
    
    set(h2, 'XData', x(i-1:i))
    set(h2, 'YData', y(i-1:i))
    
    drawnow
    pause(0.4)
end

end