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
#include <trac_ik/trac_ik.hpp>
#include <kdl_conversions/kdl_msg.h>
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


// Start to initial joint value
    int JntNum = joint_model_group->getVariableCount();
    KDL::JntArray nominal(JntNum);
    ROS_INFO("Using %d joints", JntNum);

    std::vector<double> joints_init;
    joints_init = move_group.getCurrentJointValues();

    for(int i = 0; i < JntNum; i++)
    {
        nominal(i) = joints_init[i];
        ROS_INFO_STREAM("joints_init: " << nominal(i));
    }



// Visualization
    std::string base_frame = "torso_lift_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.loadRemoteControl();

    while (ros::ok())
    {
        ROS_INFO_STREAM("Make Worspace Voxels");
        workspace_spreader::WorkspaceSpreader ws;
        double diameter = 1.2;
        double resolution = 0.3;

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


// IK using Trac-IK
        KDL::Chain chain;
        bool valid = ik_solver.getKDLChain(chain);
        int nr = chain.getNrOfSegments();

        if (!valid)
        {
            ROS_ERROR("There was no valid KDL chain found");
            return -1;
        }

        int rc;
        KDL::Frame end_effector_pose;
        KDL::JntArray result;

        std::vector< geometry_msgs::Pose > pose_col_filter;
        std::vector< std::pair< geometry_msgs::Pose, double > > solution;
        std::vector< Eigen::VectorXd > joint_solution;



        for (int i = 0; i < pose_col.size(); i++)
        {   
            result = nominal;
            double mu, mu2;

            tf::poseMsgToKDL(pose_col[i], end_effector_pose);           
            rc = ik_solver.CartToJnt(nominal, end_effector_pose, result);

            if (rc >= 0)
            {
                joint_solution.push_back(result.data);

                pose_col_filter.push_back(pose_col[i]);

                kinematic_state->setJointGroupPositions(joint_model_group, joint_solution.back());

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

                solution.push_back(make_pair(pose_col[i], mu));
      
            }
            else
            {
                mu = 0;
                mu2 = 0;
            }
                    
            if (i % 2000 == 0)
            {
                ROS_INFO_STREAM("Calculated " << i + 1);
                ROS_INFO_STREAM("No of sol: " << solution.size());
            }
        }

        ROS_INFO_STREAM("No of sol: " << solution.size());
        ROS_INFO("-----------------\n");


        ROS_INFO_STREAM("Visualization Start");

        for (int i = 0; i < solution.size(); i++)
        {
            geometry_msgs::Pose arrow_pose = solution[i].first;
            double arrow_mu = solution[i].second;

            arrow_mu *= 5;
            if (arrow_mu > 1.0){arrow_mu = 1.0;}

            // Make Visualization Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/torso_lift_link";
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
            
            

            marker.scale.x = 0.06;
            marker.scale.y = 0.007;
            marker.scale.z = 0.015;

            marker.color.a = (arrow_mu == 0) ? 0.0 : 1.0;
            marker.color.r = 1.0 - arrow_mu;
            marker.color.g = arrow_mu;
            marker.color.b = 0.00;



            marker_pub.publish(marker);

            Markerarr.markers.push_back(marker);

        }

        markerarr_pub.publish(Markerarr);
        ROS_INFO("-----------------\n");

        ROS_INFO_STREAM("Make reachability datafile");

        std::ofstream writeFile(ros::package::getPath("reachability") + "/src/" +
          "right_arm_reachability.csv");

        writeFile << chain_start << "," << chain_end << "," << JntNum << std::endl;

        double offset_x = -0.05;
        double offset_y = 0;
        double offset_z = 0.907925;

        for (int i = 0; i < solution.size(); i++)
        {
            geometry_msgs::Pose arrow_pose = solution[i].first;
            double arrow_mu = solution[i].second;
            Eigen::VectorXd jnt_solution = joint_solution[i];

            tf::Quaternion q(
                arrow_pose.orientation.x,
                arrow_pose.orientation.y,
                arrow_pose.orientation.z,
                arrow_pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

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
    //ros::waitForShutdown();
    return 0;
}