function [cost, solChar] = costLinear(model, coords)

xPath = coords(:,1);
yPath = coords(:,2);

dxPath = diff(xPath);
dyPath = diff(yPath);

% path length
L = sum(sqrt(dxPath.^2 + dyPath.^2));

%  violation check, deviding path to smaller parts for violation check
Temp = 100;
xx = zeros(length(xPath) - 1, Temp);
yy = zeros(length(xPath) - 1, Temp);

for i = 1:length(xPath) - 1
    xx(i,:) = linspace(xPath(i), xPath(i+1),Temp);
    yy(i,:) = linspace(yPath(i), yPath(i+1),Temp);
end
xx = xx(:,1:end-1)';
xx = xx(:); xx = xx';
yy = yy(:,1:end-1)';
yy = yy(:); yy = yy';

violation = zeros(1,model.numOfObs);
for i = 1:model.numOfObs
    d = sqrt((xx - model.xc(i)).^2 + (yy - model.yc(i)).^2);
    v = max(1 - d/(model.obst_r), 0);
    violation(i) = mean(v);
end
violation = mean(violation);

% solution characteristics
beta = 10e10;
solChar.path_length = L;
solChar.violation = violation;
solChar.isFeasible = (violation == 0);

cost = solChar.path_length  + beta*solChar.violation ;

end
