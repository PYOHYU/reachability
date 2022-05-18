#include <ros/ros.h>
#include <fstream>
#include <algorithm>
#include <utility>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reachability");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_STREAM("Planning frame: " << move_group.getPlanningFrame());

    int JntNum = joint_model_group->getVariableCount();

    ROS_INFO("Using %d joints", JntNum);

    std::vector<double> joints_init;
    joints_init = move_group.getCurrentJointValues();

// Visualization
    std::string base_frame = "torso_lift_link";
    std::string eef_frame = "r_wrist_roll_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.loadRemoteControl();

    while (ros::ok())
    {
        
        std::vector<double> target_joints(joint_model_group->getVariableCount());

// IK using Trac-IK
        int rc;

        geometry_msgs::Pose pose_col;

        pose_col.position.x = -0.05;
        pose_col.position.y = 0.00;
        pose_col.position.z = 1.00;
        pose_col.orientation.w = 1;

        std::vector< std::pair< geometry_msgs::Pose, double > > solution;
        std::vector< std::vector<double> > joint_solution;
        const unsigned int attempts = 10;
        const double timeout = 0.0;

        bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_col, attempts, timeout);

        if (found_ik)
        {   
            ROS_INFO("IKFast Success");
            kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);
            joint_solution.push_back(target_joints);

            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;
            kinematic_state->getJacobian(joint_model_group,
                                        kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                        reference_point_position, jacobian);

            for (int i = 0; i<target_joints.size(); i++)
            {
                ROS_INFO_STREAM("sol: " << target_joints[i]);
            }

        }
        else
        {
            ROS_INFO("IKFast Failed");
        }

        move_group.setJointValueTarget(target_joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group.execute(my_plan);
            ROS_INFO("Movegroup successful");
        }
        else
        {
            ROS_WARN("Movegroup failed");
        }
        
    }
    ros::waitForShutdown();
    return 0;
}