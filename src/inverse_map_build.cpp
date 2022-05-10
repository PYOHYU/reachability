#include <ros/ros.h>
#include <ros/package.h>

#include <sstream>
#include <fstream>
#include <cmath>

void Rot_z(double yaw, double& x, double& y);

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("Load reachability datafile");

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_reachability.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no file such name");
        return -1;
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

    ROS_INFO_STREAM("Load data file success");
    ROS_INFO_STREAM("----------------\n");
    ROS_INFO_STREAM("Make inverse reachability datafile");

    std::ofstream writeFile(ros::package::getPath("reachability") + "/src/" +
        "right_arm_inverse_reachability.csv");
    
    writeFile << chain_start << "," << chain_end << "," << JntNum << std::endl;

    for (int i = 0; i < sol_size; i++)
    {
        double yaw = eef_data[i].second[2];

        Rot_z(yaw, base_data[i][0], base_data[i][1]);

        writeFile << base_data[i][0] << ","
        << base_data[i][1] << ","
        << base_data[i][2] << ","
        << eef_data[i].first[0] << ","
        << eef_data[i].first[1] << ","
        << eef_data[i].first[2] << ","
        << eef_data[i].second[0] << ","
        << eef_data[i].second[1] << ","
        << eef_data[i].second[2] << ","
        << mu_data[i] << ",";

        for (int j = 0; j < JntNum; j++)
        {
            writeFile << jnt_data[i][j] << ",";
        }
        writeFile << std::endl;
    }
    writeFile.close();
    ROS_INFO_STREAM("Make datafile successful");
    return 0;
    
}

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