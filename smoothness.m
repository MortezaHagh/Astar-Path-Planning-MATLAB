function smoothness = smoothness(path_coords)

sol_x = path_coords(:,1);
sol_y = path_coords(:,2);

dx = diff(sol_x);
dy = diff(sol_y);

stall = dx==0 & dy==0;
dx(stall) = [];
dy(stall) = [];
sol_x(stall) = [];
sol_y(stall) = [];

if numel(sol_x)>2
    alpha = zeros(1, length(dx)-1);
    for i = 1:length(dx)-1
        a = sqrt(dx(i+1)^2+dy(i+1)^2);
        b = sqrt(dx(i)^2+dy(i)^2);
        c = (sol_x(i)-sol_x(i+2))^2+(sol_y(i)-sol_y(i+2))^2;
        gamma = abs(acosd((a^2+b^2-c)/(2*a*b)));
        alpha(i) = 180 - gamma;
    end
    sm = sum(alpha);
    smoothness = round(sm, 3);
else
    smoothness = 0;
end

end