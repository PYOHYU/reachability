#include <ros/ros.h>
#include <reachability/InvReach.h>
#include <cstdlib>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_client");
    
    if(argc != 9)
    {
        ROS_INFO("x y z r p y Cr NumSol");
        return 1;    
    }
 
    ros::NodeHandle nh;
    ros::NodeHandle vis;

    ros::ServiceClient client = nh.serviceClient<reachability::InvReach>("inv_service");
 
    reachability::InvReach srv;
 
    srv.request.eef_x = atof(argv[1]);
    srv.request.eef_y = atof(argv[2]);
    srv.request.eef_z = atof(argv[3]);
    srv.request.eef_roll = atof(argv[4]);
    srv.request.eef_pitch = atof(argv[5]);
    srv.request.eef_yaw = atof(argv[6]);
    srv.request.Cr = atof(argv[7]);
    srv.request.num_sol = atoi(argv[8]);

    if(client.call(srv))
    {
        ROS_INFO("send srv : %f, %f, %f, %f, %f, %f, %f, %f",
        srv.request.eef_x,
        srv.request.eef_y,
        srv.request.eef_z,
        srv.request.eef_roll,
        srv.request.eef_pitch,
        srv.request.eef_yaw,
        srv.request.Cr,
        srv.request.num_sol);
        ROS_INFO_STREAM("receive srv : " << srv.response.base_pose);

    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
 
}