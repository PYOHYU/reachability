#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <fstream>
#include <cmath>
#include <algorithm>

#include <Eigen/Geometry> 

struct Data
{
    double new_base_x, new_base_y, new_base_z, new_eef_x, new_eef_y, new_eef_z, new_eef_r, new_eef_p, new_base_yaw, mu;
    std::vector<double> jnt;
};

bool Comp(const Data& a, const Data& b)
{
    if (a.new_eef_z < b.new_eef_z) {return true;}
    else if (a.new_eef_z == b.new_eef_z)
    {
        if (a.new_eef_p < b.new_eef_p) {return true;}
        else if (a.new_eef_p == b.new_eef_p)
        {
            if (a.mu > b.mu) {return true;}
        }
    }
    return false;
}


int main(int argc, char** argv)
{
    ROS_INFO_STREAM("Load reachability datafile");

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_reachability4.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no file such name");
        return -1;
    }

    std::ofstream writeFile(ros::package::getPath("reachability") + "/src/" +
        "right_arm_inverse_reachability4_.csv");


    std::string line = "";
    std::string chain_start;
    std::string chain_end;
    double JntNum;
    std::vector< Data > data;

    int datasize = 0;

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

            double yaw = eef_rpy[2];
        
            double x_diff = -base_pos[0] + eef_pos[0];
            double y_diff = -base_pos[1] + eef_pos[1];

            double base_new_x = x_diff * cos(M_PI - yaw) - y_diff * sin(M_PI-yaw);
            double base_new_y = x_diff * sin(M_PI-yaw) + y_diff * cos(M_PI-yaw);

            Data d = {base_new_x, base_new_y, base_pos[2], 0, 0, eef_pos[2], eef_rpy[0], eef_rpy[1], -yaw, mu, jnt};

            data.push_back(d);
            datasize++;
        }

        line = "";
    }

    //sort data, by new_eef_z, new_eef_p, mu
    ROS_INFO_STREAM("Sorting data");
    std::sort(std::begin(data), std::end(data), Comp);

    writeFile << chain_start << "," << chain_end << "," << JntNum << std::endl;

    for (int i = 0; i < datasize; i++)
    {
        writeFile << data[i].new_base_x << ","
        << data[i].new_base_y << ","
        << data[i].new_base_z << ","
        << data[i].new_eef_x << ","
        << data[i].new_eef_y << ","
        << data[i].new_eef_z << ","
        << data[i].new_eef_r << ","
        << data[i].new_eef_p << ","
        << data[i].new_base_yaw << "," 
        << data[i].mu << ",";

        for (int k = 0; k < JntNum; k++)
        {
            writeFile << (data[i].jnt)[k] << ",";
        }
        writeFile << std::endl;
    }


    writeFile.close();
    ROS_INFO_STREAM("Make datafile successful");
    return 0;
    
}

