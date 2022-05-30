#include <ros/ros.h>
#include <fstream>
#include <algorithm>
#include <utility>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <reachability/workspace_spreader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


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

    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_STREAM("Planning frame: " << move_group.getPlanningFrame());

// Start to initial joint value
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
        ROS_INFO_STREAM("Make Worspace Voxels");
        workspace_spreader::WorkspaceSpreader ws;
        double diameter = 1.5;
        double resolution = 0.05;

//center of voxels
        std::vector< std::vector< double > > center;
        ws.CreateWorkspace(diameter, resolution, center);
        
        ROS_INFO_STREAM("No of voxels: " << center.size());

 // Visualize
        ros::Publisher markerarr_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        visualization_msgs::MarkerArray Markerarr;
        
        uint32_t shape = visualization_msgs::Marker::ARROW;


// All poses        
        std::vector< geometry_msgs::Pose > pose_col;
        std::vector< tf2::Quaternion > quat;
        ws.SpherePoses(quat);

        for (int i = 0; i < center.size(); i++)
        {
            geometry_msgs::Pose p;
            p.position.x = center[i][0];
            p.position.y = center[i][1];
            p.position.z = center[i][2];

            for (int j = 0; j < quat.size(); j++)
            {
                p.orientation.x = quat[j].getX();
                p.orientation.y = quat[j].getY();
                p.orientation.z = quat[j].getZ();
                p.orientation.w = quat[j].getW();

                pose_col.push_back(p);
              
            }
            
        }

        ROS_INFO_STREAM("No of orientations: " << quat.size());
        ROS_INFO_STREAM("No of poses: " << pose_col.size());

        int num_arrow = pose_col.size();
        Markerarr.markers.resize(num_arrow);

        ROS_INFO_STREAM("No of arrow: " << num_arrow);

        std::vector<double> target_joints(joint_model_group->getVariableCount());

// IK using Trac-IK
        int rc;

        std::vector< geometry_msgs::Pose > pose_col_filter;
        std::vector< std::pair< geometry_msgs::Pose, double > > solution;
        std::vector< std::vector<double> > joint_solution;
        const unsigned int attempts = 0;
        const double timeout = 0.0;

        for (int i = 0; i < pose_col.size(); i++)
        {   
            double mu, mu2;
            
            //bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_col[i], eef_frame, attempts, timeout);
            bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_col[i], eef_frame, attempts, timeout);

            if (found_ik)
            {
                kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);

                move_group.setJointValueTarget(target_joints);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;

                if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
                {
                    joint_solution.push_back(target_joints);
                    pose_col_filter.push_back(pose_col[i]);

                    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
                    Eigen::MatrixXd jacobian;
                    kinematic_state->getJacobian(joint_model_group,
                                                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                                reference_point_position, jacobian);

                    Eigen::MatrixXd jjt = jacobian * (jacobian.transpose());

                    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(jjt);

                    Eigen::VectorXd eigen_values = eigen_solver.eigenvalues().real();

                    std::vector<double> e_val;
                    for (int i = 0; i < eigen_values.size(); i++)
                    {
                        e_val.push_back(eigen_values[i]);
                    }

                    double lambda_max = *max_element(e_val.begin(), e_val.end());
                    double lambda_min = *min_element(e_val.begin(), e_val.end());

                    mu = sqrt(jjt.determinant());
                    mu2 = sqrt(lambda_min/lambda_max);

                    solution.push_back(make_pair(pose_col[i], mu2));

                }
                
            }
            else
            {
                mu = 0;
                mu2 = 0;
            }
                 
            if (i % 20000 == 0)
            {
                ROS_INFO_STREAM("Calculated " << i + 1);
                ROS_INFO_STREAM("No of sol: " << solution.size());
            }
            
        }

        ROS_INFO_STREAM("No of sol: " << solution.size());
        ROS_INFO("-----------------\n");

/*
        ROS_INFO_STREAM("Visualization Start");

        for (int i = 0; i < solution.size(); i++)
        {
            geometry_msgs::Pose arrow_pose = solution[i].first;
            double arrow_mu = solution[i].second;

            arrow_mu *= 5;
            if (arrow_mu > 1.0){arrow_mu = 1.0;}

            // Make Visualization Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = i + 10;
            marker.type = shape;
            marker.lifetime.sec = 30;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = arrow_pose.position.x;
            marker.pose.position.y = arrow_pose.position.y;
            marker.pose.position.z = arrow_pose.position.z;
            marker.pose.orientation.x = arrow_pose.orientation.x;
            marker.pose.orientation.y = arrow_pose.orientation.y;
            marker.pose.orientation.z = arrow_pose.orientation.z;
            marker.pose.orientation.w = arrow_pose.orientation.w;
            
            

            marker.scale.x = 0.05;
            marker.scale.y = 0.007;
            marker.scale.z = 0.007;

            marker.color.a = (arrow_mu == 0) ? 0.1 : 0.7;
            marker.color.r = 1.0 - arrow_mu;
            marker.color.g = arrow_mu;
            marker.color.b = 0.00;



            marker_pub.publish(marker);

            Markerarr.markers.push_back(marker);

        }

        markerarr_pub.publish(Markerarr);
        ROS_INFO("-----------------\n");
*/
        ROS_INFO_STREAM("Make reachability datafile");

        std::ofstream writeFile(ros::package::getPath("reachability") + "/src/" + "right_arm_reachability6.csv");

        writeFile << base_frame << "," << eef_frame << "," << JntNum << std::endl;

        //double offset_x = -0.05;
        //double offset_y = 0;
        //double offset_z = 0.907925;
        double offset_x = 0;
        double offset_y = 0;
        double offset_z = 0;
        for (int i = 0; i < solution.size(); i++)
        {
            geometry_msgs::Pose arrow_pose = solution[i].first;
            double arrow_mu = solution[i].second;
            std::vector< double > jnt_solution = joint_solution[i];

            tf::Quaternion q(
                arrow_pose.orientation.x,
                arrow_pose.orientation.y,
                arrow_pose.orientation.z,
                arrow_pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

                //base_footprint position
            writeFile << offset_x << ","
            << offset_y << ","
            << offset_z << ",";

            writeFile << arrow_pose.position.x << "," 
            << arrow_pose.position.y << ","
            << arrow_pose.position.z << ","
            << roll << ","
            << pitch << ","
            << yaw << ","
            << arrow_mu << ",";

            for (int j = 0; j < JntNum; j++)
            {
                writeFile << jnt_solution[j] << ",";
            }
            writeFile << std::endl;
        }

        writeFile.close();

        ROS_INFO_STREAM("Make datafile successful");
        return 0;
        
    }
    ros::waitForShutdown();
    return 0;
}