function [Map, map_name] = CreateMap(path_, name_, extension_)


map_name = name_;

% readmap data
deli = '\';
if strcmp(path, '')
   deli = ''; 
end

fid = fopen([path_ deli name_ extension_]);

fgetl(fid);

% H
tline = fgetl(fid);
H = tline;
H = regexp(H,'\d*','Match');
H = str2num(H{1});

%W
tline = fgetl(fid);
W = tline;
W = regexp(W,'\d*','Match');
W = str2num(W{1});

%
fgetl(fid);

% map_cell
i=1;
map_cell = cell(H,1);
tline = fgetl(fid);
while ischar(tline)
    map_cell{i,1} =  tline;
    tline = fgetl(fid);
    i = i+1;
end
fclose(fid);

map_cell = flip(map_cell);

% Map matrix
Map = zeros(H, W);
for i=1:H
    for j=1:W
        Map(i,j) =  strcmp(map_cell{i,1}(j), '.');
    end
end

% % save
% save(name_,'Map', 'map_name');

end