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
#include <random>
#include <numeric>

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

    std::vector<double> TimeDuration;

    std::random_device rng;
    std::uniform_real_distribution<double> rand_xy(-10, 10);
    std::uniform_real_distribution<double> rand_z(0.6, 1.2);
    std::uniform_real_distribution<double> rand_ry(-180, 180);
    std::uniform_real_distribution<double> rand_p(-90, 90);

    for(int iter = 0; iter < 300; iter++)
    {
        double eef_rand_x = rand_xy(rng);
        double eef_rand_y = rand_xy(rng);
        double eef_rand_z = rand_z(rng);
        double eef_rand_r = rand_ry(rng);
        double eef_rand_p = rand_p(rng);
        double eef_rand_yaw = rand_ry(rng);

        double interval = 5;   // angle interval, deg
        int NumSol = req.num_sol;
        int solperpos = (int)(req.Cr * 2 / interval) + 1;   //sols per one pose.
        int getdata = (int)(NumSol / solperpos) + 1;

        ros::Time startit = ros::Time::now();

        std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
        "right_arm_inverse_reachability2.csv");

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

        double target_z = eef_rand_z;
        double target_p = eef_rand_p; //deg
        double bound_z = 0.05 / 2;
        double bound_p = 30 / 2; //deg

        int k = 0;

        while(getline(loadFile, line) && sol_size < getdata)
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

        if (sol_size == 0)
        {
            ROS_ERROR("No valid Base pose");
            continue;
        }

        // Transform to get base pose
        std::vector< Data > data;
        int datasize = 0;
        

        for (int i = 0; i < sol_size; i++)
        {   
            if (datasize >= NumSol) { break; }

            //Initial base angle yaw
            double theta = (eef_data[i].second[2]) * 180 / M_PI;  //deg

            double bx_ = base_data[i][0];
            double by_ = base_data[i][1];

            
        
            double max_angle = eef_rand_yaw + req.Cr;    //deg
            double min_angle = eef_rand_yaw - req.Cr;

            for(double j = min_angle; j <= max_angle; j += interval)
            {
                
                double Cb = j - theta;
                double c = std::cos(Cb * M_PI/180); //rad
                double s = std::sin(Cb * M_PI/180);
                double nbx_ = c * bx_ - s * by_;
                double nby_ = s * bx_ + c * by_;

                Data d = {mu_data[i], (j * M_PI/180), nbx_, nby_,  jnt_data[i]};

                if (datasize >= NumSol) { break; }
                else
                {
                    data.push_back(d);
                    datasize++;
                }
            }
        }

        NumSol = std::min(NumSol, datasize);

        std::vector< std::vector<double> > Solxy;
        std::vector< double >SolYaw;
        std::vector< std::vector< double > > SolJnt;
        std::vector< geometry_msgs::Pose > SolPos;

        int idx = 0;
        while(Solxy.size() < NumSol)
        {
            // These are pose of base_footprint
            std::vector<double> solutions(2);
            solutions[0] = data[idx].new_base_x + eef_rand_x;
            solutions[1] = data[idx].new_base_y + eef_rand_y;

            SolYaw.push_back(data[idx].theta);

            geometry_msgs::Pose sp;
            sp.position.x = data[idx].new_base_x + eef_rand_x;
            sp.position.y = data[idx].new_base_y + eef_rand_y;
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
        res.base_pose = SolPos[0];
        res.joint_arr.resize(JntNum);
        TimeDuration.push_back(dif);
    }

    //avg and std
    double sum = std::accumulate(TimeDuration.begin(), TimeDuration.end(), 0.0);
    double mean = sum / TimeDuration.size();

    std::vector<double> diff(TimeDuration.size());
    std::transform(TimeDuration.begin(), TimeDuration.end(), diff.begin(), [mean](double x) {return x - mean;});
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / TimeDuration.size());

    ROS_INFO_STREAM("mean : " << mean);
    ROS_INFO_STREAM("stdev : " << stdev);
    ROS_INFO_STREAM("num : " << TimeDuration.size());

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