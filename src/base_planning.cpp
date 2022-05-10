#include <reachability/base_planning.h>

namespace base_planner
{

    void BasePlanner::BasePlan(double x_goal, double y_goal, double th_goal, std::vector < double >& xyth, ros::Time& current_time, ros::Time& last_time, ros::Publisher& odom_pub)
    {

        tf::TransformBroadcaster odom_broadcaster;

        double x = xyth[0];
        double y = xyth[1];
        double th = xyth[2];

        double vx = 0.5;
        double vy = 0.0;
        double vth = 0.3;

        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        
        double K_lin = 0.5;
        double distance = abs(sqrt(pow((x-x_goal), 2) + pow((y-y_goal), 2)));
        double linear_speed = distance * K_lin;

        if(linear_speed > vx)
            linear_speed = vx;

        double K_ang = 2.0;
        double angle_goal = atan2(y_goal-y, x_goal-x);
        double angular_speed;

        double delta_x = linear_speed * cos(th) * dt;
        double delta_y = linear_speed * sin(th) * dt;
        double delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        if (distance < 0.01)
        {   
            if (abs(th_goal-th) < 0.01)
            {
                ROS_INFO_STREAM("Base Planning End.");
                return;
            }
            else
            {
                //only rotation 
                angular_speed = (th_goal - th) * K_ang;

                delta_th = angular_speed * dt;

                th += delta_th;
                ROS_INFO_STREAM("Rotation.");
            }
        }
        else
        {
            //Do
            angular_speed = (angle_goal - th) * K_ang;
            delta_th = angular_speed * dt;

            x += delta_x;
            y += delta_y;           
            th += delta_th;

            ROS_INFO_STREAM("Go to Waypoint.");
        }

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_combined";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom_combined";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = linear_speed;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = angular_speed;

        //publish the message
        odom_pub.publish(odom);

        ROS_INFO_STREAM("\n Position:    \n --- \n" << x << '\n' << y);
        ROS_INFO_STREAM("\n Angle(rad):    \n --- \n" << th);

        last_time = current_time;

        xyth[0] = x;
        xyth[1] = y;
        xyth[2] = th;

        return;
    }

}
