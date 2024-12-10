function ROS_init()
    rosshutdown;
    %设置环境变量
    %从roscore运行信息里面复制过来ROS_MASTER_URI
    %如果是虚拟机运行ros就在win的cmd中输入ipconfig，从信息中复制'以太网适配器 以太网:'下的IPv4 地址填到ROS_IP后
    setenv('ROS_MASTER_URI','http://192.168.50.124:11311/');
    setenv('ROS_IP','192.168.50.64');
    %初始化ros网络
    rosinit;
    disp('init done');
end

