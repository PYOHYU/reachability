#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_time_check");
    ros::NodeHandle nh;

    while(ros::ok())
    {
        ROS_INFO_STREAM("ifstream/ofstream");

        ros::Time startit = ros::Time::now();

        std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
        "right_arm_reachability5.csv");

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

        while (getline(loadFile, line))
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

        double dif = ros::Duration( ros::Time::now() - startit).toNSec() / 1000000;


        ROS_INFO("Time duration : %lf ms", dif);
        ROS_INFO_STREAM(sol_size);

    }
    ros::waitForShutdown();
    return 0;

}