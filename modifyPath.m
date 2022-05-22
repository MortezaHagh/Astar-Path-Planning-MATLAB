function path = modifyPath (model, path)

path_coords = path.coords;
path_nodes = path.nodes;
path_dirs = path.dirs;

it = 1;
NumPoints = 100;
while it <= size(path_coords,1) - 2
    X = linspace(path_coords(it,1), path_coords(it+2,1), NumPoints);
    Y = linspace(path_coords(it,2), path_coords(it+2,2), NumPoints);
    
    violation = zeros(1,length(model.xc));
    for k = 1:length(model.xc)
        dd = sqrt((X - model.xc(k)).^2+(Y - model.yc(k)).^2);
        v = max(1-dd/(model.obst_r),0);
        violation(k) = mean(v);
    end
    Violation = any(violation);
    
    if ~Violation
        path_coords(it+1,:) = [];
        path_nodes(it+1) = [];
        path_dirs(it+1) = [];
    else
        it = it +1;
    end
    
end

path.coords = path_coords;
path.nodes = path_nodes;
path.dirs = path_dirs;

end
