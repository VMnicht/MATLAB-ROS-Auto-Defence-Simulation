function r2_odom_callback(~,msg)
    global ROS_data;
    ROS_data.Vx(2) = msg.Twist.Twist.Linear.X;
    ROS_data.Vy(2) = msg.Twist.Twist.Linear.Y;
    ROS_data.X(2) = msg.Pose.Pose.Position.X;
    ROS_data.Y(2) = msg.Pose.Pose.Position.Y;
    ROS_data.W(2) = msg.Twist.Twist.Angular.Z;
    theta = 2 * atan2(msg.Pose.Pose.Orientation.Z, msg.Pose.Pose.Orientation.W);
    ROS_data.yaw(2) = theta * (180/pi);
end