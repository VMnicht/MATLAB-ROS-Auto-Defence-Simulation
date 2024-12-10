function r1_odom_callback(~,msg)
    global ROS_data;
    ROS_data.Vx(1) = msg.Twist.Twist.Linear.X;
    ROS_data.Vy(1) = msg.Twist.Twist.Linear.Y;
    ROS_data.X(1) = msg.Pose.Pose.Position.X;
    ROS_data.Y(1) = msg.Pose.Pose.Position.Y;
    ROS_data.W(1) = msg.Twist.Twist.Angular.Z;
    theta = 2 * atan2(msg.Pose.Pose.Orientation.Z, msg.Pose.Pose.Orientation.W);
    ROS_data.yaw(1) = theta * (180/pi);

end