function plotModel(model)

xmin=model.xmin;
xmax=model.xmax;
ymin=model.ymin;
ymax=model.ymax;
r=model.obst_r;
xs=model.xs;
ys=model.ys;
xt=model.xt;
yt=model.yt;
xc=model.xc;
yc=model.yc;

figure(1)
axis([xmin-1,xmax+1,ymin-1,ymax+1])
axis equal
grid on
box on
hold on

% g=gca;
% g.XTick=0:xmax;
% g.YTick=0:ymax;

% start
plot(xs,ys,'bs','MarkerSize',14,'MarkerEdgeColor',[0.5,0,0.5],...
    'MarkerFaceColor',[0.5,0,0.5]);
% target
plot(xt,yt,'bp','MarkerSize',14,'MarkerEdgeColor',[0,1,1],...
    'MarkerFaceColor',[0,1,1]);

% % Obstacles
% theta=linspace(0,2*pi,100);
% for i=1:length(xc)
%     fill(xc(i)+r*cos(theta),yc(i)+r*sin(theta),'k');
% end

% Obstacles
for i=1:length(xc)
    plot(xc(i),yc(i),'ko', 'MarkerSize',5,'MarkerFaceColor','k');
end

% % Obstacles patch
% for i = 1:size(model.obstX,1)
%     patch(model.obstX{i,1}, model.obstY{i,1},'y','EdgeColor','yellow','FaceAlpha',0.5)
%     scatter(model.obstX{i,1}, model.obstY{i,1},'r','filled')
% end

% walls
rectangle('Position',[xmin-0.5 ymin-0.5 (xmax-xmin+1) (ymax-ymin+1)], 'LineWidth',5)

end
