#include <ros/ros.h>
#include <ros/package.h>
#include <reachability/InvReach.h>
#include <reachability/base_planning.h>
#include <fstream>
#include <algorithm>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

struct Data
{
    double mu, theta, new_base_x, new_base_y;
    std::vector<double> jnt;
};

void MakeMarker(visualization_msgs::Marker& marker, geometry_msgs::Pose& arrow_pose, std::string frame_id, int id)
{
        // Make Visualization Marker
    
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime.sec = 100;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = arrow_pose.position.x;
    marker.pose.position.y = arrow_pose.position.y;
    marker.pose.position.z = arrow_pose.position.z;
    marker.pose.orientation.x = arrow_pose.orientation.x;
    marker.pose.orientation.y = arrow_pose.orientation.y;
    marker.pose.orientation.z = arrow_pose.orientation.z;
    marker.pose.orientation.w = arrow_pose.orientation.w;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.00;
    
    return;
}

bool BasePoseCalcuator(reachability::InvReach::Request& req, reachability::InvReach::Response& res)
{
    ros::Time startit = ros::Time::now();
    ROS_INFO_STREAM("Load inverse reachability datafile");

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_inverse_reachability3.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no inverse reachability datafile.");
        return false;
    }

    std::string line = "";
    std::string chain_start;
    std::string chain_end;
    double JntNum;
    std::vector< std::vector<double> > base_data;
    std::vector< std::pair< std::vector<double>, std::vector<double> > > eef_data;
    std::vector< double > mu_data;
    std::vector< std::vector<double> > jnt_data;

    int sol_size = 0;

    double target_z = req.eef_z;
    double target_p = req.eef_pitch; //deg
    double bound_z = 0.05 / 2;
    double bound_p = 30 / 2; //deg

    int k = 0;

    while(getline(loadFile, line))
    {
        std::string tempString;
        std::stringstream inputString(line);

        if (k == 0)
        {
            getline(inputString, chain_start, ',');
            getline(inputString, chain_end, ',');
            getline(inputString, tempString, ',');
            JntNum = atof(tempString.c_str());
            k++;
        }
        else
        {
            std::vector<double> b, p, o, j;
            double m;
            
            //base position
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                b.push_back(atof(tempString.c_str()));
            }
            
            //eef postion
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                p.push_back(atof(tempString.c_str()));
            }
            
            //eef RPY
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                o.push_back(atof(tempString.c_str()));
            }

            //manipulability
            getline(inputString, tempString, ',');
            m = atof(tempString.c_str());

            //jnt value
            for (int i = 0; i < JntNum; i++)
            {
                getline(inputString, tempString, ',');
                j.push_back(atof(tempString.c_str()));
            }

            if (std::fabs(target_z - p[2]) < bound_z && std::fabs(target_p - o[1]*180/M_PI) < bound_p)
            {
                base_data.push_back(b);
                eef_data.push_back({p, o});
                mu_data.push_back(m);
                jnt_data.push_back(j);
                sol_size++;
            }            
        }
        line = "";
    }

    ROS_INFO_STREAM("Available Solution : " << sol_size);

    if (sol_size == 0)
    {
        ROS_ERROR("No valid Base pose");
        return false;
    }

    // Transform to get base pose
    std::vector< Data > data;
    int datasize = 0;

    for(int i = 0; i < sol_size; i++)
    {   
        //Initial base angle yaw
        double theta = (eef_data[i].second[2]) * 180 / M_PI;  //deg

        double bx_ = base_data[i][0];
        double by_ = base_data[i][1];

        double interval = 5;   // angle, deg
       
        double max_angle = req.eef_yaw + req.Cr;    //deg
        double min_angle = req.eef_yaw - req.Cr;

        for(double j = min_angle; j <= max_angle; j += interval)
        {
            double Cb = j - theta;
            double c = std::cos(Cb * M_PI/180); //rad
            double s = std::sin(Cb * M_PI/180);
            double nbx_ = c * bx_ - s * by_;
            double nby_ = s * bx_ + c * by_;

            Data d = {mu_data[i], (j * M_PI/180), nbx_, nby_,  jnt_data[i]};

            data.push_back(d);
            datasize++;
        }

    }

    std::sort(std::begin(data), std::end(data), [](Data& a, Data& b)
    {
        return a.mu > b.mu;
    });

    int NumSol = req.num_sol;

    std::vector< std::vector<double> > Solxy;
    std::vector< double >SolYaw;
    std::vector< std::vector< double > > SolJnt;
    std::vector< geometry_msgs::Pose > SolPos;

    int idx = 0;
    while(Solxy.size() < std::min(NumSol, datasize))
    {
        // These are pose of base_footprint
        std::vector<double> solutions(2);
        solutions[0] = data[idx].new_base_x + req.eef_x;
        solutions[1] = data[idx].new_base_y + req.eef_y;

        SolYaw.push_back(data[idx].theta);

        geometry_msgs::Pose sp;
        sp.position.x = data[idx].new_base_x + req.eef_x;
        sp.position.y = data[idx].new_base_y + req.eef_y;
        sp.position.z = 0;
        tf::Quaternion q_rot;
        q_rot.setRPY(0, 0, data[idx].theta);
        sp.orientation.x = q_rot[0];
        sp.orientation.y = q_rot[1];
        sp.orientation.z = q_rot[2];
        sp.orientation.w = q_rot[3];

        Solxy.push_back(solutions);
        SolJnt.push_back(data[idx].jnt);
        SolPos.push_back(sp);
        idx++;
    }

    double dif = ros::Duration( ros::Time::now() - startit).toNSec() / 1000000;
    ROS_INFO("Elasped time is %.2lf ms.", dif);
    ROS_INFO_STREAM("mu : " << data[0].mu);
    res.base_pose = SolPos[0];
    res.joint_arr.resize(JntNum);

        //visualize
    ros::NodeHandle vis;
    ROS_INFO_STREAM("visualization start");
    std::string base_frame = "odom_combined";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.loadRemoteControl();

    //geometry_msgs::Pose arrow_base = res.base_pose;
    geometry_msgs::Pose arrow_eef;
    arrow_eef.position.x = req.eef_x;
    arrow_eef.position.y = req.eef_y;
    arrow_eef.position.z = req.eef_z;
    tf::Quaternion q_eef;
    q_eef.setRPY(req.eef_roll*M_PI/180, req.eef_pitch*M_PI/180, req.eef_yaw*M_PI/180);
    arrow_eef.orientation.x = q_eef[0];
    arrow_eef.orientation.y = q_eef[1];
    arrow_eef.orientation.z = q_eef[2];
    arrow_eef.orientation.w = q_eef[3];

    int SolNum = std::min(NumSol, datasize);

        //base plan
    ros::NodeHandle nh_base;
    ros::Publisher odom_pub = nh_base.advertise<nav_msgs::Odometry>("virtual_joint", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0;

    double x_goal = Solxy[0][0];
    double y_goal = Solxy[0][1];
    double th_goal = SolYaw[0];

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

                        //eef plan
    ros::NodeHandle nh_eef;
    //ros::AsyncSpinner spinner(4);
    //spinner.start();
    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //ros::Rate rate(100);

    bool base_check = false;
    bool eef_check = false;

    while(ros::ok() && !eef_check)
    {
        ros::Publisher markerarr_pub = vis.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        ros::Publisher marker_pub = vis.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        visualization_msgs::MarkerArray Markerarr;
        visualization_msgs::Marker eef_marker;

        for(int i = 0; i < SolPos.size(); i++)
        {
            visualization_msgs::Marker base_marker;            
            MakeMarker(base_marker, SolPos[i], "/odom_combined", i);
            Markerarr.markers.push_back(base_marker);
        }

        MakeMarker(eef_marker, arrow_eef, "/odom_combined", 200);
        //marker_pub.publish(marker);
        Markerarr.markers.push_back(eef_marker);
        markerarr_pub.publish(Markerarr);


            //base planning
        base_planner::BasePlanner bp;
        bp.BasePlan(x_goal, y_goal, th_goal, x, y, th, current_time, last_time, odom_pub, base_check);

            //eef planning
        if (base_check == true)
        {

            ROS_INFO("End effector planning start");
            robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
            const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);

            const unsigned int attempts = 10;
            const double timeout = 0.0;
            bool found_ik = kinematic_state->setFromIK(joint_model_group, arrow_eef, attempts, timeout);

            move_group.setJointValueTarget(SolJnt[0]);            
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
        
            //move_group.execute(my_plan);
            ROS_INFO("End effector planning end");

            for(int i=0; i<JntNum; i++)
            {
                ROS_INFO_STREAM("jnt:" << SolJnt[0][i]);
            }

            eef_check = true;
        }

        //rate.sleep();
    }
    //ros::waitForShutdown();
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::ServiceServer server = nh.advertiseService("inv_service", BasePoseCalcuator);

    ROS_INFO("ready to serve");

    //ros::spin();
    ros::waitForShutdown();
    return 0;
}