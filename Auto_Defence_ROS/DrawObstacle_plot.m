%% 绘制�?有障碍物位置
% 输入参数：obstacle �?有障碍物的坐�?   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR;
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
    x = r * cos(theta) + obstacle(id,1);
    y = r  *sin(theta) + obstacle(id,2);
    plot(x,y,'-k');hold on;
end
end
% plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % 绘制�?有障碍物位置