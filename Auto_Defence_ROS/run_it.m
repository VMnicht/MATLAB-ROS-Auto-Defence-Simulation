% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab

% -------------------------------------------------------------------------





function [] = run_it()

close all;
clear all;
%%ROS初始化
%初始化ROS节点
ROS_init;
global ROS_data;
ROS_data = struct('X', [0,0,0,0], 'Y', [0,0,0,0], ...
    'Vx', [0,0,0,0],'Vy',[0,0,0,0],'yaw',[0,0,0,0],'W',[0,0,0,0]);
r1_odom_sub = rossubscriber("/robot1/odom",@r1_odom_callback,'DataFormat','struct');
pause(0.2);%等待数据更新
r2_odom_sub = rossubscriber("/robot2/odom",@r2_odom_callback,'DataFormat','struct');
pause(0.2);
r3_odom_sub = rossubscriber("/robot3/odom",@r3_odom_callback,'DataFormat','struct');
pause(0.2);
r4_odom_sub = rossubscriber("/robot4/odom",@r4_odom_callback,'DataFormat','struct');
pause(0.2);
r1_ctrl = rospublisher("/robot1/cmd_vel","geometry_msgs/Twist");
r2_ctrl = rospublisher("/robot2/cmd_vel","geometry_msgs/Twist");
r3_ctrl = rospublisher("/robot3/cmd_vel","geometry_msgs/Twist");
r4_ctrl = rospublisher("/robot4/cmd_vel","geometry_msgs/Twist");

r3_acc = rospublisher("/MATLAB_acc_r3","geometry_msgs/Accel");
r4_acc = rospublisher("/MATLAB_acc_r4","geometry_msgs/Accel");

r1_cmd = rosmessage(r1_ctrl);
r2_cmd = rosmessage(r2_ctrl);   
r3_cmd = rosmessage(r3_ctrl);
r4_cmd = rosmessage(r4_ctrl);

r3_acc_cmd = rosmessage(r3_acc);
r4_acc_cmd = rosmessage(r4_acc);

disp('topic init done');

%ROS中地图与MATLAB的地图对齐
ROS_map_x = 7.5;
ROS_map_y = 4;

disp('Dynamic Window Approach sample program start!!')

%% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% x=[0 0 pi/2 0 0]'; % 5x1矩阵 列矩阵  位置 0，0 航向 pi/2 ,速度、角速度均为0
x = [2 0 pi/2 0 0]'; 
% 下标宏定义 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度 


x(POSE_X) = ROS_data.Y(2)+ROS_map_y;
x(POSE_Y) = ROS_data.X(2)+ROS_map_x;
x(V_SPD) = sqrt((ROS_data.Vx(2))^2+(ROS_data.Vy(2))^2);
x(W_ANGLE_SPD) = ROS_data.W(2);
x(YAW_ANGLE) = ROS_data.yaw(2);

goal = [4,15];   % 目标点位置 [x(m),y(m)]

% 障碍物位置列表 [x(m) y(m)]
obstacle=[2 15;
          5 15];
obstacle(1,1) = ROS_data.Y(3)+ROS_map_y;
obstacle(1,2) = ROS_data.X(3)+ROS_map_x;
obstacle(2,1) = ROS_data.Y(4)+ROS_map_y;
obstacle(2,2) = ROS_data.X(4)+ROS_map_x;

obstacleR = 0.4;% 冲突判定用的障碍物半径
global dt; 
dt = 0.09;% 时间[s]

% 机器人运动学模型参数
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic = [4.0,toRadian(150.0),2.0,toRadian(170.0),0.15,toRadian(4)];
%定义Kinematic的下标含义
MD_MAX_V    = 1;%   最高速度m/s]
MD_MAX_W    = 2;%   最高旋转速度[rad/s]
MD_ACC      = 3;%   加速度[m/ss]
MD_VW       = 4;%   旋转加速度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转速分辨率[rad/s]]


% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
evalParam = [0.01, 1.1 ,0.4, 1.3];

area      = [-1 9 -1 16];% 模拟区域范围 [xmin xmax ymin ymax]

previous_error1 = [0 0 0 0];
integral1 = [0 0 0 0];
target_v = 0;

obstacle_V1 = [0 0];
obstacle_a1 = [0 0];
targetx1 = 0;
targety1 = 0;

previous_error2 = [0 0 0 0];
integral2 = [0 0 0 0];

obstacle_V2 = [0 0];
obstacle_a2 = [0 0];
targetx2 = 0;
targety2 = 0;
Vmax = 1.5;

r = 0.8;
t = 5;
Vbu = [0,0];

% 模拟实验的结果
result.x=[];   %累积存储走过的轨迹点的状态值 
tic; % 估算程序运行时间开始
while ROS_data.Vx(2)<0.1
    pause(0.1);
end
% movcount=0;
%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
for i = 1:5000  
     



     %三车距离
     distance_AB = sqrt((obstacle(1,1) - obstacle(2,1))^2 + (obstacle(1,2) - obstacle(2,2))^2);  %己方两车距离
     distance_AC = sqrt((obstacle(1,1) - x(1))^2 + (obstacle(1,2) - x(2))^2);
     distance_BC = sqrt((x(1) - obstacle(2,1))^2 + (x(2) - obstacle(2,2))^2);
    
     %1车  目标点位y轴设定
     if x(2) <=7.3
         y_PID1 = 1;
         targety1 = 8.0;
     else
         if x(2) - obstacle(1,2) > -0.65 || x(2) - obstacle(1,2) < -1.2
            y_PID1 = 1;
            targety1 = x(2)+1.5;
         else 
            targety1 = obstacle(1,2);
            y_PID1 = 0;
         end
     end
     %2车  目标点位y轴设定
     if x(2) <=7.3
         y_PID2 = 1;
         targety2 = 8.0;
     else
         if x(2) - obstacle(2,2) > -0.8 || x(2) - obstacle(2,2) < -1.2
             y_PID2 = 1;
            targety2 = x(2)+1.0;
         else 
            targety2 = obstacle(2,2);
            y_PID2 = 0;
         end
     end
    
    %目标点位x轴设定
    D = sqrt((x(1) - goal(1))^2 + (x(2) - goal(2)-3)^2);
    targetx1 = x(1) + (1 / D) * (goal(1) - x(1));
    targetx2 = x(1) + (1 / D) * (goal(1) - x(1));

    %两车同时防守
    if(distance_AB < 1.7)     %如果两车距离过近，会在他们连线的方向上额外添加一对反方向的加速度，避免相撞 
        % a1_x = 0.01/(obstacle(1,1) - obstacle(2,1));
        a1_x = (1.7 - distance_AB) * abs(obstacle(1,1) - obstacle(2,1))/distance_AB*2;
        % a1_y = 0.1/(obstacle(1,2) - obstacle(2,2));
        a1_y = (1.7 - distance_AB) * abs(obstacle(1,2) - obstacle(2,2))/distance_AB*2;
        if (obstacle(1,1) < obstacle(2,1))
            a1_x = -a1_x;
        end
        if (obstacle(1,2) < obstacle(2,2))
            a1_y = -a1_y;
        end
        a2_x = -a1_x;
        a2_y = -a1_y;


        if(obstacle(1,1) > obstacle(2,1))
            % targetx1 = targetx1 + 0.3;
            % targetx2 = targetx2 - 0.3;
            if targetx1 > 7.3
                targetx1 = 7;
            end
            if targetx2 <0.7
                targetx2 = 1;
            end
        else
            % targetx1 = targetx1 - 0.3;
            % targetx2 = targetx2 + 0.3;
            if targetx2 > 7.3
                targetx2 = 7;
            end
            if targetx1 <0.7
                targetx1 = 1;
            end            
        end

        
    else 
        a1_x = 0;
        a1_y = 0;
        a2_x = 0;
        a2_y = 0;
    end

    if(distance_AC < 0.9 && obstacle(1,2) - x(2) < 0.6)
        a1_x = (1.5 - distance_AC) * abs(obstacle(1,1) - x(1))/distance_AC*4;
        % a1_y = 0.1/(obstacle(1,2) - obstacle(2,2));
        a1_y = (1.5 - distance_AC) * abs(obstacle(1,2) - x(2))/distance_AC*4;
        if (obstacle(1,1) < x(1))
            a1_x = -a1_x;
        end
        if (obstacle(1,2) < x(2))
            a1_y = -a1_y;
        end
        a2_x = -a1_x;
        a2_y = -a1_y;
    end


    if(obstacle(1,2) < 8.5)
        a1_y = a1_y + (8.5 - obstacle(1,2));
    end



    if(distance_AB < 2.0)
        xiangduizuobiao = obstacle(2,:)-obstacle(1,:);
        xiangduisudu = obstacle_V1 - obstacle_V2;
        cos1 = (2*(two_point_distance(obstacle(1,:),obstacle(2,:))*two_point_distance(obstacle(1,:),obstacle(2,:))) - 2*r*r)/(2*two_point_distance(obstacle(1,:),obstacle(2,:)) * sqrt(two_point_distance(obstacle(1,:),obstacle(2,:))*two_point_distance(obstacle(1,:),obstacle(2,:)) - r*r));
        cos2 = (two_point_distance(obstacle(1,:),obstacle(2,:))*two_point_distance(obstacle(1,:),obstacle(2,:)) + xiangliangchangdu(xiangduisudu)*xiangliangchangdu(xiangduisudu) - two_point_distance(xiangduisudu,xiangduizuobiao)*two_point_distance(xiangduisudu,xiangduizuobiao))/(2*two_point_distance(obstacle(1,:),obstacle(2,:))*xiangliangchangdu(xiangduisudu));
        sin1 = sin_cos(cos1);
        sin2 = sin_cos(cos2);       
        if cos2 > cos1
            if xiangliangchangdu(xiangduisudu)*4 > two_point_distance(obstacle(1,:),obstacle(2,:))
            sinA = sin1*cos2 - cos1*sin2;
            %disp(["sinA=",num2str(sinA)])
            else
            Vbu = (xiangduisudu - xiangduizuobiao)*(1-two_point_distance(xiangduisudu,xiangduizuobiao)/r);
            end
        else 
            Vbu = [0,0];
        end
        
        % point1 = [targetx1,targety1];
        % if is_point_within_line_distance(obstacle(1,:),point1,obstacle(2,:),0.6)
        %     disp('目标点在连线的0.6米范围内');
        %     a1_x = a1_x - obstacle(2,2) + obstacle(1,2);
        %     a1_y = a1_y + obstacle(2,1) - obstacle(1,1);
        % end
        % point1 = [targetx1,targety1];
        % point2 = [x(1),x(2)];
        % if is_point_within_line_distance(obstacle(1,:),point1,point2,0.6)
        %     disp('目标点在连线的0.6米范围内');
        %     a1_x = a1_x - x(2) + obstacle(1,2);
        %     a1_y = a1_y + x(1) - obstacle(1,1);
        % end
        
    end

    point1 = [targetx1,targety1];
    plot(targetx1,targety1,'*g');
    if is_point_within_line_distance(obstacle(1,:),point1,obstacle(2,:),0.4)
        % disp('目标点在连线的0.5米范围内');
        a1_x = a1_x - obstacle(2,2) + obstacle(1,2);
        a1_y = a1_y + obstacle(2,1) - obstacle(1,1);
        y_PID1 = 1;
    end

    %PID控制
    targetx1 = targetx1 + a1_x;
    [target_v,previous_error1(1),integral1(1)] = PID(targetx1,obstacle(1,1),2.5,0.2,15.0,0,previous_error1(1),integral1(1));
    % target_v = target_v + a1_x;
    [obstacle_a1(1),previous_error1(2),integral1(2)] = PID(target_v,obstacle_V1(1),3.5,0.1,5.0,1.5,previous_error1(2),integral1(2));
    targetx2 = targetx2 + a2_x;
    [target_v,previous_error2(1),integral2(1)] = PID(targetx2,obstacle(2,1),2.5,0.2,5.0,0,previous_error2(1),integral2(1));     
    % target_v = target_v + a2_x;
    [obstacle_a2(1),previous_error2(2),integral2(2)] = PID(target_v,obstacle_V2(1),2.5,0.1,5.0,1.5,previous_error2(2),integral2(2));

    if y_PID1
    targety1 = targety1 + a1_y;
    end
    [target_v,previous_error1(3),integral1(3)] = PID(targety1,obstacle(1,2),3.0,0.01,50.0,0,previous_error1(3),integral1(3));
    % target_v = target_v + a1_y;
    [obstacle_a1(2),previous_error1(4),integral1(4)] = PID(target_v,obstacle_V1(2),3.0,0.0,2.0,1.5,previous_error1(4),integral1(4));

    if y_PID2
    targety2 = targety1 + a2_y;
    end
    [target_v,previous_error2(3),integral2(3)] = PID(targety2,obstacle(2,2),3.5,0.01,50.0,0,previous_error2(3),integral2(3));
    % target_v = target_v + a2_y;
    [obstacle_a2(2),previous_error2(4),integral2(4)] = PID(target_v,obstacle_V2(2),3.5,0.01,2.0,1.5,previous_error2(4),integral2(4));
    
    

    %加速度补偿
     %obstacle_a1(1) = obstacle_a1(1) + a1_x;
     %obstacle_a2(1) = obstacle_a2(1) + a2_x;
     % if abs(obstacle_a1(1)) > 2
     %     if(obstacle_a1(1) > 0)
     %         obstacle_a1(1) = 2;
     %     else
     %         obstacle_a1(1) = -2;
     %     end
     % end
     % if abs(obstacle_a2(1)) > 2
     %     if(obstacle_a2(1) > 0)
     %         obstacle_a2(1) = 2;
     %     else
     %         obstacle_a2(1) = -2;
     %     end
     % end
      
    %obstacle_a1(2) = obstacle_a1(2) + a1_y;
    %obstacle_a2(2) = obstacle_a2(2) + a2_y;

    
    %下一单位时间的点位计算（限制最大速度）
    
    [~,obstacle_V1] = update_position(obstacle(1,:),obstacle_V1(1,:),obstacle_a1,Vmax,dt);
    %if i > 40
    %[~,obstacle_V2] = update_position(obstacle(2,:),obstacle_V2(1,:),obstacle_a2,Vmax,dt);
    %end

    if i > 10
       r3_cmd.Linear.X=obstacle_V1(1,2);
       r3_cmd.Linear.Y=obstacle_V1(1,1);
       r3_acc_cmd.Linear.X=abs(obstacle_a1(1));
       r3_acc_cmd.Linear.Y=abs(obstacle_a1(2));
       % r3_acc_cmd.Linear.X=1.5;
       % r3_acc_cmd.Linear.Y=1.5;
       % r4_cmd.Linear.X=obstacle_V2(1,2);
       % r4_cmd.Linear.Y=obstacle_V2(1,1);
       % r4_acc_cmd.Linear.X=abs(obstacle_a2(1));
       % r4_acc_cmd.Linear.Y=abs(obstacle_a2(2));
       r4_cmd.Linear.X=0;
       r4_cmd.Linear.Y=0;
       r4_acc_cmd.Linear.X=1.5;
       r4_acc_cmd.Linear.Y=1.5;
       send(r3_acc,r3_acc_cmd);
       send(r4_acc,r4_acc_cmd);
       send(r3_ctrl,r3_cmd);
       send(r4_ctrl,r4_cmd);
    end
    if mod(i,50)==0
        % fprintf("3车目标速度为x:%.2f,y：%.2f)\n",obstacle_V1(1,1),obstacle_V1(1,2));
        % fprintf("4车目标速度为x:%.2f,y：%.2f)\n",obstacle_V2(1,1),obstacle_V2(1,2));
    end
    pause(dt);

    obstacle(1,1) = ROS_data.Y(3)+ROS_map_y;
    obstacle(1,2) = ROS_data.X(3)+ROS_map_x;
    % obstacle_V1(1,1) = ROS_data.Vx(3);
    % obstacle_V1(1,2) = ROS_data.Vy(3);
    obstacle(2,1) = ROS_data.Y(4)+ROS_map_y;
    obstacle(2,2) = ROS_data.X(4)+ROS_map_x;
    % obstacle_V1(2,1) = ROS_data.Vx(4);
    % obstacle_V1(2,2) = ROS_data.Vy(4);
    if mod(i,2)==0
        % fprintf("3车坐标为：(%.2f,%.2f)\n",obstacle(1,1),obstacle(1,2));
        fprintf("3车速度为x:%.2f,y：%.2f)\n",obstacle_V1(1,1),obstacle_V1(1,2));
        % fprintf("3车加速度为x:%.2f,y：%.2f)\n",obstacle_a1(1),obstacle_a1(2));
        % fprintf("4车坐标为：(%.2f,%.2f)\n",obstacle(2,1),obstacle(2,2)); 
        % fprintf("4车速度为x:%.2f,y：%.2f)\n",obstacle_V2(1,1),obstacle_V2(1,2));
        % fprintf("4车加速度为x:%.2f,y：%.2f)\n",obstacle_a2(1),obstacle_a2(2));
        fprintf("2车速度为x:%.2f,y：%.2f)\n",ROS_data.Vx(2),ROS_data.Vy(2));
    end
    if mod(i,30)==0
        obstacle_V1(2,1) = ROS_data.Vx(4);
        obstacle_V1(2,2) = ROS_data.Vy(4);
        obstacle_V1(1,1) = ROS_data.Vx(3);
        obstacle_V1(1,2) = ROS_data.Vy(3);
    end

    

% DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
 %   [u,traj] = DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR,area);
%x = f(x,u);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度

x(POSE_X) = ROS_data.Y(2)+ROS_map_y;
x(POSE_Y) = ROS_data.X(2)+ROS_map_x;
x(V_SPD) = sqrt((ROS_data.Vx(2))^2+(ROS_data.Vy(2))^2);
x(W_ANGLE_SPD) = ROS_data.W(2);
x(YAW_ANGLE) = ROS_data.yaw(2);

% 历史轨迹的保存
result.x = [result.x; x'];  %最新结果 以列的形式 添加到result.x

% 是否到达目的地
if norm(x(POSE_X:POSE_Y)-goal')<0.5   % norm函数来求得坐标上的两个点之间的距离
disp('Arrive Goal!!');
fprintf("防守时间：%.2f\n",i*dt);
break;
end

%====Animation====
hold off;               % 关闭图形保持功能。 新图出现时，取消原图的显示。
ArrowLength = 0.5;      % 箭头长度

% 机器人
% quiver(x,y,u,v) 在 x 和 y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)), 'ok'); % 绘制机器人当前位置的航向箭头
hold on;                                                     %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基础上，并自动调整坐标轴的范围

plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
rectangle('Position', [0, 0, 8, 15], 'EdgeColor', 'k', 'LineWidth', 2);
plot([0, 8], [7.5, 7.5], 'k--', 'LineWidth', 2);
plot(targetx1,targety1,'*r');
plot(targetx2,targety2,'*r');

%plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制所有障碍物位置
DrawObstacle_plot(obstacle,obstacleR);
DrawObstacle_plot4(x,obstacleR);

% % 探索轨迹 画出待评价的轨迹
% if ~isempty(traj) %轨迹非空
% for it=1:length(traj(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
% ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标 
% plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
% end
% end

axis(area); %根据area设置当前图形的坐标范围，分别为x轴的最小、最大值，y轴的最小最大值
grid on;
drawnow;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，需要使用这个语句。
%movcount = movcount+1;
%mov(movcount) = getframe(gcf);%  记录动画帧
end
toc  %输出程序运行时间  形式：时间已过 ** 秒。
%movie2avi(mov,'movie.avi');  %录制过程动画 保存为 movie.avi 文件

%% 绘制所有障碍物位置
% 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
x = r * cos(theta) + obstacle(id,1); 
y = r  *sin(theta) + obstacle(id,2);
plot(x,y,'-m','LineWidth', 2);hold on; 
end
function [] = DrawObstacle_plot4(xt,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(xt(1,1))
 x = r * cos(theta) + xt(1); 
 y = r  *sin(theta) + xt(2);
 plot(x,y,'-g','LineWidth', 2);hold on; 
end

%plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % 绘制所有障碍物位置



%% DWA算法实现 
% model  机器人运动学模型  最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss], 速度分辨率[m/s],转速分辨率[rad/s]]
% 输入参数：当前状态、模型参数、目标点、评价函数的参数、障碍物位置、障碍物半径
% 返回参数：控制量 u = [v(m/s),w(rad/s)] 和 轨迹集合 N * 31  （N：可用的轨迹数）
% 选取最优参数的物理意义：在局部导航过程中，使得机器人避开障碍物，朝着目标以较快的速度行驶。
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,ob,R,area)
% Dynamic Window [vmin,vmax,wmin,wmax] 最小速度 最大速度 最小角速度 最大角速度速度
Vr = CalcDynamicWindow(x,model);  % 根据当前状态 和 运动模型 计算当前的参数允许范围

% 评价函数的计算 evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
%               trajDB      每5行一条轨迹 每条轨迹都有状态x点串组成
[evalDB,trajDB]= Evaluation(x,Vr,goal,ob,R,model,evalParam,area);  %evalParam 评价函数参数 [heading,dist,velocity,predictDT]

if isempty(evalDB)
disp('no path to goal!!');
u=[0;0];return;
end

% 各评价函数正则化
evalDB = NormalizeEval(evalDB);

% 最终评价函数的计算
feval=[];
for id=1:length(evalDB(:,1))
feval = [feval;evalParam(1:3)*evalDB(id,3:5)']; %根据评价函数参数 前三个参数分配的权重 计算每一组可用的路径参数信息的得分
end
evalDB = [evalDB feval]; % 最后一组

[maxv,ind] = max(feval);% 选取评分最高的参数 对应分数返回给 maxv  对应下标返回给 ind
u = evalDB(ind,1:2)';% 返回最优参数的速度、角速度  

%% 评价函数 内部负责产生可用轨迹
% 输入参数 ：当前状态、参数允许范围（窗口）、目标点、障碍物位置、障碍物半径、评价函数的参数
% 返回参数：
%           evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
%           trajDB      每5行一条轨迹 每条轨迹包含 前向预测时间/dt + 1 = 31 个轨迹点（见生成轨迹函数）
function [evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam,area)
evalDB = [];
trajDB = [];
for vt = Vr(1):model(5):Vr(2)       %根据速度分辨率遍历所有可用速度： 最小速度和最大速度 之间 速度分辨率 递增 
for ot=Vr(3):model(6):Vr(4)     %根据角度分辨率遍历所有可用角速度： 最小角速度和最大角速度 之间 角度分辨率 递增  
% 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹（由轨迹点组成）
        [xt,traj] = GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
% 各评价函数的计算
heading = CalcHeadingEval(xt,goal); % 前项预测终点的航向得分  偏差越小分数越高
dist    = CalcDistEval(xt,ob,R,area);    % 前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高
vel     = abs(vt);                  % 速度得分 速度越快分越高
stopDist = CalcBreakingDist(vel,model); % 制动距离的计算
if dist > stopDist+0.5 % 如果可能撞到最近的障碍物 则舍弃此路径 （到最近障碍物的距离 大于 刹车距离 才取用）
evalDB = [evalDB;[vt ot heading dist vel]];
trajDB = [trajDB;traj];   % 每5行 一条轨迹  
end



end
end
function [control_signal,new_previous_error,new_integral] = PID(target, current,Kp,Ki,Kd,outmax,previous_error,integral)
    % 计算当前误差
    error = target - current;

    % 计算积分项
    new_integral = integral + error;
    if abs(new_integral) > 3
    new_integral = 3 * sign(new_integral);
    end

    % 计算微分项
    derivative = error - previous_error;

    % 计算控制信号
    control_signal = Kp * error + Ki * integral + Kd * derivative;

    if outmax ~= 0
        if abs(control_signal) > outmax
            if control_signal > 0
                control_signal = outmax;
            else
                control_signal = -outmax;
            end
        end
    end

    % 更新前一个误差
    new_previous_error = error;
function [x_new, y_new,Vx_new,Vy_new] = UpdatePosition(x, y, Vx, Vy, ax, ay, max_speed, dt)
    current_speed = sqrt(Vx^2 + Vy^2);
    if current_speed >= max_speed
        cos = Vx / current_speed;
        sin = Vy / current_speed;
        a = ax*sin - ay*cos;
        ax = sin*a;
        ay = cos*a;
    end
    % 更新速度
    Vx_new = Vx + ax * dt;
    Vy_new = Vy + ay * dt;
    
    % 更新位置
    x_new = x + Vx * dt + 0.5 * ax * dt^2;
    y_new = y + Vy * dt + 0.5 * ay * dt^2;
    
    % 如果需要返回新的速度，可以添加以下内容：
    % Vx_new 和 Vy_new 可用于后续计算

function [new_position, new_velocity] = update_position(current_position, current_velocity, acceleration, max_speed, dt)
    % current_position: 当前坐标 (1x2 向量) [x, y]
    % current_velocity: 当前速度 (1x2 向量) [vx, vy]
    % acceleration: 当前加速度 (1x2 向量) [ax, ay]
    % max_speed: 最大速度限制 (标量)
    % dt: 时间步长 (标量)

    % 计算下一个时间单位的速度
    new_velocity = current_velocity + acceleration * dt;

    % 计算速度的模长
    speed_magnitude = norm(new_velocity);

    % 检查速度是否超过最大速度限制
    if speed_magnitude > max_speed
        % 缩放速度
        scaling_factor = max_speed / speed_magnitude;
        new_velocity = new_velocity * scaling_factor;
    end

    % 更新位置
    new_position = current_position + new_velocity * dt;
function distance = two_point_distance(A,B)
    C = A-B;
    distance = sqrt(C(1)*C(1)+C(2)*C(2));
function distance = xiangliangchangdu(x)
    distance = sqrt(x(1)*x(1)+x(2)*x(2)); 
function sin_cos = sin_cos(data)
    sin_cos = sqrt(1-data*data);
function isWithin = is_point_within_line_distance(p1, p2, target, threshold)
    % p1 和 p2 为两点坐标，target 为目标点坐标，threshold 为距离阈值
    % p1 = [x1, y1], p2 = [x2, y2], target = [xt, yt]
    % threshold 为0.4米

    % 计算线段 p1 到 p2 的向量
    lineVec = p2 - p1;
    
    % 计算目标点到 p1 的向量
    targetVec = target - p1;
    
    % 计算线段长度的平方
    lineLengthSquared = dot(lineVec, lineVec);
    
    % 计算投影系数 t（目标点在线段上的投影）
    t = dot(targetVec, lineVec) / lineLengthSquared;
    
    % 如果投影点 t 在 [0, 1] 范围内，则投影点在线段上
    if t < 0
        nearestPoint = p1; % 投影点在 p1 之前，最近点为 p1
    elseif t > 1
        nearestPoint = p2; % 投影点在 p2 之后，最近点为 p2
    else
        nearestPoint = p1 + t * lineVec; % 投影点在线段上，最近点为投影点
    end
    
    % 计算目标点到最近点的距离
    distanceToLine = norm(target - nearestPoint);
    
    % 判断目标点是否在阈值范围内
    isWithin = distanceToLine <= threshold;

    
%% END

