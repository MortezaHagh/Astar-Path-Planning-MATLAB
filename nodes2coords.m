function path_coords = nodes2coords(path, model)

path_coords = model.nodes.cord(:,path);
path_coords = path_coords';

end