#ifndef BASE_PLANNING_H
#define BASE_PLANNING_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>

namespace base_planner
{
class BasePlanner
{
public:
    BasePlanner(){}
    ~BasePlanner(){}
    
    void BasePlan(double x_goal, double y_goal, double th_goal, std::vector < double >& xyth, ros::Time& current_time, ros::Time& last_time, ros::Publisher& odom_pub);

};
}


#endif