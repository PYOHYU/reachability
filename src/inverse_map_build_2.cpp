#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <fstream>
#include <cmath>

#include <Eigen/Geometry> 

void Rot_z(double yaw, double& x, double& y)
{
    double s = sin(M_PI - yaw);
    double c = cos(M_PI - yaw);

    double x_ = c * x - s * y;
    double y_ = s * x + c * y;

    x = x_;
    y = y_;
    return;
}

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("Load reachability datafile");

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_reachability_ikfast.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no file such name");
        return -1;
    }

    std::ofstream writeFile(ros::package::getPath("reachability") + "/src/" +
        "right_arm_inverse_reachability_ikfast.csv");


    std::string line = "";
    std::string chain_start;
    std::string chain_end;
    double JntNum;

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
            writeFile << chain_start << "," << chain_end << "," << JntNum << std::endl;
            k++;
        }
        else
        {
            std::vector<double> base_pos, eef_pos, eef_rpy, jnt;
            double mu;
            
            //base position
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                base_pos.push_back(atof(tempString.c_str()));
            }
            
            //eef postion
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                eef_pos.push_back(atof(tempString.c_str()));
            }
            
            //eef RPY
            for (int i = 0; i < 3; i++)
            {
                getline(inputString, tempString, ',');
                eef_rpy.push_back(atof(tempString.c_str()));
            }

            //manipulability
            getline(inputString, tempString, ',');
            mu = atof(tempString.c_str());

            //jnt value
            for (int i = 0; i < JntNum; i++)
            {
                getline(inputString, tempString, ',');
                jnt.push_back(atof(tempString.c_str()));
            }

            double roll = eef_rpy[0];
            double pitch = eef_rpy[1];
            double yaw = eef_rpy[2];
        
            Rot_z(yaw, base_pos[0], base_pos[1]);

            writeFile << base_pos[0] << ","
            << base_pos[1] << ","
            << base_pos[2] << ","
            << eef_pos[0] << ","
            << eef_pos[1] << ","
            << eef_pos[2] << ","
            << eef_rpy[0] << ","
            << eef_rpy[1] << ","
            << eef_rpy[2] << ","
            << mu << ",";

            for (int k = 0; k < JntNum; k++)
            {
                writeFile << jnt[k] << ",";
            }
            writeFile << std::endl;
        }

        line = "";
    }

    writeFile.close();
    ROS_INFO_STREAM("Make datafile successful");
    return 0;
    
}

