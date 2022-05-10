#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <reachability/workspace_spreader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>


// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp



int main(int argc, char** argv)
{
    ros::init(argc, argv, "reachability");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);

    std::string chain_start, chain_end, urdf_param;
    double timeout;

    nh.param("chain_start", chain_start, std::string("torso_lift_link"));
    nh.param("chain_end", chain_end, std::string("r_wrist_roll_link"));

    ROS_INFO_STREAM("Base link: " << chain_start);
    ROS_INFO_STREAM("End effector link: " << chain_end);

    if (chain_start == "" || chain_end == "")
    {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    ROS_INFO_STREAM("Planning frame: " << move_group.getPlanningFrame());

    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, "/robot_description", 0.005, 1e-5);

    Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(chain_end);
    geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);

    std::vector<double> joints_init;
    joints_init = move_group.getCurrentJointValues();

    
    eef_pose.position.x = 0.5;
    eef_pose.position.y = -0.19;
    eef_pose.position.z = 1.25;
    

    // Visualization
    std::string base_frame = "base_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    
    

    while (ros::ok())
    {
        visual_tools.publishAxisLabeled(eef_pose, "waypoint");

        visual_tools.trigger();
        // IK using Trac-IK
        KDL::Chain chain;
        bool valid = ik_solver.getKDLChain(chain);

        if (!valid)
        {
            ROS_ERROR("There was no valid KDL chain found");
            return -1;
        }

        int JntNum = chain.getNrOfJoints();
        KDL::JntArray nominal(JntNum);
        ROS_INFO("Using %d joints", JntNum);

        KDL::JntArray InitJnt(JntNum);

        for(int i=0; i<JntNum; i++)
        {
            InitJnt(i) = joints_init[i];
            ROS_INFO_STREAM("joints_init: " << InitJnt(i));
        }


        int rc;
        KDL::Frame end_effector_pose;
        KDL::JntArray result;

        tf::poseMsgToKDL(eef_pose, end_effector_pose);           
        rc = ik_solver.CartToJnt(InitJnt, end_effector_pose, result);
        //rc = ik_solver.CartToJnt(InitJnt, InitJnt, result);

        ROS_INFO_STREAM("eef_pose: \n" << eef_pose);

        if (rc >= 0)
        {
            std::vector<double> solution;

            for (int z = 0; z < JntNum; z++)
            {
                solution.push_back(result(z));
                ROS_INFO_STREAM(solution[z]);
            }
            
        }
        else
        {
            ROS_INFO_STREAM("No Solution");
        }

        ROS_INFO("-----------------\n");

    }
    ros::waitForShutdown();
    return 0;
}