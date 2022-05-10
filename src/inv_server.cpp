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

struct Data
{
    double mu, theta, Ct, new_base_x, new_base_y, new_eef_x, new_eef_y;
    std::vector<double> jnt;
};

void BasePlan(double x_goal, double y_goal, double th_goal, std::vector < double >& xyth, ros::Time& current_time, ros::Time& last_time, ros::Publisher& odom_pub)
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
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

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
    "right_arm_inverse_reachability_ikfast.csv");

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

            base_data.push_back(b);
            eef_data.push_back({p, o});
            mu_data.push_back(m);
            jnt_data.push_back(j);
            sol_size++;
        }

        line = "";
    }

        //time start

    double target_z = req.eef_z;
    double base_z = base_data[0][2];

    //target_z check
    double bound = 0.05 / 2;
    std::vector< int > idx;
    std::vector< int > idx2;

    for(int i = 0; i < sol_size; i++)
    {
        if (target_z >= ((eef_data[i].first)[2] - bound) && target_z < ((eef_data[i].first)[2] + bound))
        {
            idx2.push_back(i);
        }

    }

    if (idx2.size() == 0)
    {
        ROS_ERROR("No valid Base pose1");
        return false;
    }




    // Robot heading angle(yaw) check
    double Cr_ = req.Cr * M_PI / 180;

/*
    for(int i = 0; i < idx.size(); i++)
    {
        if ((eef_data[idx[i]].second)[2] >= minCr_ && (eef_data[idx[i]].second)[2] <= maxCr_)
        {
            idx2.push_back(idx[i]);
        }    
    }
*/    
    std::vector<int>().swap(idx);

    if (idx2.size() == 0)
    {
        ROS_ERROR("No valid Base pose2");
        return false;
    }

    // Transform to get base pose
    std::vector< Data > data;
    int datasize = 0;

    for(int i = 0; i < idx2.size(); i++)
    {   
        //Initial angle yaw
        double theta = eef_data[idx2[i]].second[2];

        double bx_ = base_data[idx2[i]][0];
        double by_ = base_data[idx2[i]][1];

        double ex_ = eef_data[idx2[i]].first[0];
        double ey_ = eef_data[idx2[i]].first[1];

        double radius = sqrt(ex_ * ex_ + ey_ * ey_);
        double interval = 0.1;
        int num_points = (int)(M_PI * radius / interval);
        

        for(int j = -num_points; j < num_points; j++)
        {
            
            //Robot heading relative to global coordinate
            double Ct = interval * j / radius;

            if(std::abs(Ct - req.eef_yaw) <= Cr_)
            {
                double c = cos(Ct);
                double s = sin(Ct);

                double nbx_ = c * bx_ - s * by_;
                double nby_ = s * bx_ + c * by_;
                double nex_ = c * ex_ - s * ey_;
                double ney_ = s * ex_ + c * ey_;

                Data d = {mu_data[idx2[i]], theta, Ct, nbx_, nby_, nex_, ney_, jnt_data[idx2[i]]};

                data.push_back(d);
                datasize++;
            }
        }
    }

    std::sort(std::begin(data), std::end(data), [](Data& a, Data& b)
    {
        return a.mu > b.mu;
    });

    int NumSol = req.num_sol;

    // offset from base_link to base_footprint
    std::vector<double> offset_bf = {0.061, 0.0, -0.1465};
    std::vector< geometry_msgs::Pose > SolPos;
    std::vector< double >SolYaw;
    std::vector< std::vector< double > > SolJnt;

    for (int j = 0; j < std::min(NumSol, datasize); j++)
    {
        // These are pose of base_link, not torso_lift_link!

        geometry_msgs::Pose solutions;
        solutions.position.x = (req.eef_x - data[j].new_eef_x) + data[j].new_base_x + offset_bf[0];
        solutions.position.y = (req.eef_y - data[j].new_eef_y) + data[j].new_base_y + offset_bf[1];
        solutions.position.z = 0;

        SolYaw.push_back(data[j].Ct - data[j].theta);
        tf::Quaternion q_rot;
        q_rot.setRPY(0, 0, data[j].Ct - data[j].theta);

        solutions.orientation.x = q_rot[0];
        solutions.orientation.y = q_rot[1];
        solutions.orientation.z = q_rot[2];
        solutions.orientation.w = q_rot[3];

        ROS_INFO_STREAM("base_pose : " << solutions);
        ROS_INFO_STREAM("mu : " << data[j].mu);
        SolPos.push_back(solutions);
        SolJnt.push_back(data[j].jnt);

        for (int k = 0; k < JntNum; k++)
        {
            ROS_INFO_STREAM("joint : " << SolJnt[j][k]);
        }
    }

    double dif = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    
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
    q_eef.setRPY(req.eef_roll, req.eef_pitch, req.eef_yaw);
    arrow_eef.orientation.x = q_eef[0];
    arrow_eef.orientation.y = q_eef[1];
    arrow_eef.orientation.z = q_eef[2];
    arrow_eef.orientation.w = q_eef[3];

    int SolNum = std::min(NumSol, datasize);

    ros::NodeHandle nh_plan;
    ros::Publisher odom_pub = nh_plan.advertise<nav_msgs::Odometry>("virtual_joint", 50);
    tf::TransformBroadcaster odom_broadcaster;

    std::vector< double > xyth(3);
    xyth[0] = 0.0;
    xyth[1] = 0.0;
    xyth[2] = 0.0;

    double x_goal = SolPos[0].position.x;
    double y_goal = SolPos[0].position.y;
    double th_goal = SolYaw[0];

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(100);

    while(ros::ok())
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
        eef_marker.type = visualization_msgs::Marker::SPHERE;
        eef_marker.scale.x = 0.02;
        Markerarr.markers.push_back(eef_marker);
        markerarr_pub.publish(Markerarr);


            //base planning
        BasePlan(x_goal, y_goal, th_goal, xyth, current_time, last_time, odom_pub);

        rate.sleep();
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inv_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer server = nh.advertiseService("inv_service", BasePoseCalcuator);

    ROS_INFO("ready to serve");

    ros::spin();

    return 0;
}