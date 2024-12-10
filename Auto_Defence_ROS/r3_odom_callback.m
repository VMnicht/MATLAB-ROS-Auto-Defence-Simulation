function r3_odom_callback(~,msg)
    global ROS_data;
    ROS_data.Vx(3) = msg.Twist.Twist.Linear.X;
    ROS_data.Vy(3) = msg.Twist.Twist.Linear.Y;
    ROS_data.X(3) = msg.Pose.Pose.Position.X;
    ROS_data.Y(3) = msg.Pose.Pose.Position.Y;
    ROS_data.W(3) = msg.Twist.Twist.Angular.Z;
    theta = 2 * atan2(msg.Pose.Pose.Orientation.Z, msg.Pose.Pose.Orientation.W);
    ROS_data.yaw(3) = theta * (180/pi);
end