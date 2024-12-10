%% 计算动态窗口
% 返回 最小速度 最大速度 最小角速度 最大角速度速度
function Vr = CalcDynamicWindow(x,model)

V_SPD       = 4;%机器人速度
W_ANGLE_SPD = 5;%机器人角速度

MD_MAX_V = 1;%
MD_MAX_W = 2;%
MD_ACC   = 3;%
MD_VW    = 4;%

global dt;
% 车子速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vs=[0 model(MD_MAX_V) -model(MD_MAX_W) model(MD_MAX_W)];

% 根据当前速度以及加速度限制计算的动态窗口  依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vd = [x(V_SPD)-model(MD_ACC)*dt x(V_SPD)+model(MD_ACC)*dt x(W_ANGLE_SPD)-model(MD_VW)*dt x(W_ANGLE_SPD)+model(MD_VW)*dt];

% 最终的Dynamic Window
Vtmp = [Vs;Vd];  %2 X 4  每一列依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
end