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
#include <stdio.h>
#include <stdlib.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// global data
std::vector< std::pair<double, double> > zp_data;
std::vector< std::vector< std::vector<double> > > sub_data_list;
double JntNum;
double x = 0.0;
double y = 0.0;
double th = 0;

//KD tree From https://github.com/leehy0321/algorithm_study/blob/master/Class/0917_kdtree_search.cpp

struct node
{
	double z;
	double p;
    int idx;
	struct node *next;
};

struct node *head = 0;

// root of kdtree
struct kdtree_node *kd_root = 0;

// kd-tree struct
struct kdtree_node
{
	struct node *data;
	struct kdtree_node *left;
	struct kdtree_node *right;
};

int howManyNodes(struct node *_head)
{
	int cnt = 0;
	struct node *temp = _head;

	while (1)
	{
		if (temp == 0)
		{
			return cnt;
		}
		else
		{
			cnt++;
			temp = temp->next;
		}
	}
}

struct node *rebuildSLL(struct node **_array, int from, int to)
{
	if (from > to)
	{
		return 0;
	}
	struct node *cur = _array[from];
	cur->next = 0;
	struct node *temp = cur;
	for (int i = from + 1; i <= to; i++)
	{
		temp->next = _array[i];
		temp->next->next = 0;
		temp = temp->next;
	}
	return cur;
}

// kdtree generation
// return: address of kdtree node
// input: linked list
//       depth
//       dimension
struct kdtree_node *build_kdtree(struct node *sll,
	int _depth,
	int _dimension)
{
	if (sll == 0)
	{
		return 0;
	}
	// 1. save the address of nodes in array

	int numSLLNodes = howManyNodes(sll);

	struct node **nodeAddrArray = (struct node **)malloc(sizeof(struct node *)*numSLLNodes);

	int i;
	struct node *temp = sll;
	for (i = 0; i < numSLLNodes; i++)
	{
		nodeAddrArray[i] = temp;
		temp = temp->next;
	}
	
    int axis = _depth % _dimension;
	int _median = numSLLNodes / 2;

	struct kdtree_node *cur = (struct kdtree_node *)malloc(sizeof(struct kdtree_node));
	cur->data = nodeAddrArray[_median];
	cur->left = build_kdtree(rebuildSLL(nodeAddrArray, 0, _median - 1), _depth + 1, 2);
	cur->right = build_kdtree(rebuildSLL(nodeAddrArray, _median + 1, numSLLNodes - 1), _depth + 1, 2);
	free(nodeAddrArray);

	return cur;

}

//return 1 if (_z, _p) is in kd-tree
//	     0 if not
int searchKdTree(struct kdtree_node *_root, double _z, double _p)
{
	struct kdtree_node *temp = _root;
	int axis = 0;

	while (1)
	{
		if (temp == 0) // cannot find (_z,_p)
			return 0;

		if (temp->data->z == _z && temp->data->p == _p)
			return 1;

		if (axis == 0) // compare z coordinates
		{
			if (temp->data->z > _z)
				temp = temp->left;
			else
				temp = temp->right;
		}
		else // axis =1 -> compare p coordinates
		{
			if (temp->data->p > _p)
				temp = temp->left;
			else
				temp = temp->right;
		}

		axis = (axis + 1) % 2;// change axis
	}
}

void addToSLL(double _z, double _p, int _idx)
{
	struct node *new_one = (struct node *)malloc(sizeof(struct node));
	new_one->z = _z;
	new_one->p = _p;
	new_one->idx = _idx;
	new_one->next = 0;

	// add new_one
	if (head == 0)
	{
		head = new_one;
		return;
	}
	else  
	{
		struct node *temp = head;
		while (1)
		{
			if (temp->next == 0)  // leaf
			{
				temp->next = new_one;
				return;
			}
			else // not a leaf
			{
				temp = temp->next;
			}
		}
	}
}

//left or right
struct kdtree_node *whichWayToGo(struct kdtree_node *_root, double _z, double  _p, int axis)
{
	if (axis == 0)// compare z
	{
		if (_root->data->z > _z)
			return _root->left;
		else
			return _root->right;
	}
	else // compare p
	{
		if (_root->data->p > _p)
			return _root->left;
		else
			return _root->right;
	}
}

struct kdtree_node *whichWayNotToGo(struct kdtree_node *_root, double _z, double _p, int axis)
{
	if (axis == 0)// compare z
	{
		if (_root->data->z > _z)
			return _root->right;
		else
			return _root->left;
	}
	else // compare p
	{
		if (_root->data->p > _p)
			return _root->right;
		else
			return _root->left;
	}
}

struct kdtree_node *RealNearestNeighbor(struct kdtree_node *_root,
	double _z,
	double _p,
	double _minDiff, 
	struct kdtree_node *_minDiffNode, 
	int axis, 
	int dim)
{
	if (_root == 0)
	{
		return _minDiffNode; 
	}

	if (_root->data->z == _z && _root->data->p == _p)
	{
		return _root;
	}
 
	double dist = (_root->data->z - _z)*(_root->data->z - _z) + (_root->data->p - _p)*(_root->data->p - _p);
	if (dist < _minDiff)
	{
		_minDiff = dist;
		_minDiffNode = _root;
	}

	struct kdtree_node *wayTo = whichWayToGo(_root, _z, _p, axis%dim);
	struct kdtree_node *NN =
		RealNearestNeighbor(wayTo, _z, _p, _minDiff, _minDiffNode, axis + 1, dim);
															
	double dist_to_NN = (NN->data->z - _z)*(NN->data->z - _z) + (NN->data->p - _p)*(NN->data->p - _p);
	
    if (axis%dim == 0)
	{
		if (dist_to_NN > (_z - _root->data->z)*(_z - _root->data->z))
		{
			
			return RealNearestNeighbor(whichWayNotToGo(_root, _z, _p, axis%dim), _z, _p, dist_to_NN, NN, axis + 1, dim);
		}
		else
		{
			return NN;
		}
	}
	else
    {
		if (dist_to_NN > (_p - _root->data->p)*(_p - _root->data->p))
		{
		
			return RealNearestNeighbor(whichWayNotToGo(_root, _z, _p, axis%dim), _z, _p, dist_to_NN, NN, axis + 1, dim);
		}
		else
		{
			return NN;
		}
	}

}

struct Data
{
    double mu, theta, new_base_x, new_base_y;
    std::vector<double> jnt;
};


//marker visualize

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
    std::uniform_real_distribution<double> rand_ry(-M_PI, M_PI);
    std::uniform_real_distribution<double> rand_p(-M_PI/2, M_PI/2);

    for(int iter = 0; iter < 300; iter++)
    {
        std::vector< std::vector<double> > base_data;
        std::vector< std::pair< std::vector<double>, std::vector<double> > > eef_data;
        std::vector< double > mu_data;
        std::vector< std::vector<double> > jnt_data;

        double eef_rand_x = rand_xy(rng);
        double eef_rand_y = rand_xy(rng);
        double eef_rand_z = rand_z(rng);
        double eef_rand_r = rand_ry(rng);
        double eef_rand_p = rand_p(rng);
        double eef_rand_yaw = rand_ry(rng);

        ros::Time searchstart = ros::Time::now();

        double interval = 5;   // angle interval, deg
        int NumSol = req.num_sol;
        int solperpos = (int)(req.Cr * 2 / interval) + 1;   //sols per one pose.
        int getdata = (int)(NumSol / solperpos) + 1;

        struct kdtree_node *realPans = RealNearestNeighbor(kd_root, eef_rand_z, eef_rand_p, 10, 0, 0, 2);
        int realPansidx = realPans->data->idx;

        int max_size = sub_data_list[realPansidx].size();
        getdata = std::min(getdata, max_size);

        for(int i = 0; i < getdata; i++)
        {
            std::vector<double> base_rec{sub_data_list[realPansidx][i][0], sub_data_list[realPansidx][i][1], sub_data_list[realPansidx][i][2]};

            base_data.push_back(base_rec);

            std::vector<double> eef_rec{sub_data_list[realPansidx][i][3], sub_data_list[realPansidx][i][4], zp_data[i].first};

            std::vector<double> eef_rpy_rec{sub_data_list[realPansidx][i][5], zp_data[i].second, sub_data_list[realPansidx][i][6]};

            eef_data.push_back(std::make_pair(eef_rec, eef_rpy_rec));

            mu_data.push_back(sub_data_list[realPansidx][i][7]);

            std::vector<double> jnt_rec;

            for (int j=0; j<sub_data_list[realPansidx][i].size() - 8; j++)
            {
                jnt_rec.push_back(sub_data_list[realPansidx][i][8+j]);
            }

            jnt_data.push_back(jnt_rec);

        }

        // Transform to get base pose
        std::vector< Data > data;
        int datasize = 0;
        
        for(int i=0; i < getdata; i++)
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

        double dif = ros::Duration( ros::Time::now() - searchstart).toNSec();
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

    ROS_INFO_STREAM("IRM data open");

    ros::Time openstart = ros::Time::now();

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_inverse_reachability2_.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no file such name");
        return -1;
    }

    std::string line = "";
    std::string chain_start;
    std::string chain_end;

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
            //base position
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
        
            std::pair<double, double> zp{p[2], o[1]};
            std::vector<double> etc{b[0], b[1], b[2], p[0], p[1], o[0], o[2], m};

            //jnt value
            for (int i = 0; i < JntNum; i++)
            {
                getline(inputString, tempString, ',');
                etc.push_back(atof(tempString.c_str()));
            }
          
            int node_num = zp_data.size();
            
            if (node_num == 0)
            {
                //first data
                zp_data.push_back(zp);
                addToSLL(zp.first, zp.second, node_num);

                std::vector< std::vector<double> > sub_data;
                sub_data.push_back(etc);
                sub_data_list.push_back(sub_data);
            }
            else if (zp_data[node_num - 1].first == zp.first && zp_data[node_num - 1].second == zp.second)
            {
                //same z and pitch
                sub_data_list[node_num - 1].push_back(etc);
            }
            else
            {
                //different z and pitch
                zp_data.push_back(zp);
                addToSLL(zp.first, zp.second, node_num);

                std::vector< std::vector<double> > sub_data;
                sub_data.push_back(etc);
                sub_data_list.push_back(sub_data);
            }
               
        }
        line = "";
    }

    kd_root = build_kdtree(head, 0, 2);

    double dif_kd = ros::Duration( ros::Time::now() - openstart).toNSec();
    ROS_INFO_STREAM("Make K-D tree : " << dif_kd);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::ServiceServer server = nh.advertiseService("inv_service", BasePoseCalcuator);

    ROS_INFO("ready to serve");

    //ros::spin();
    ros::waitForShutdown();
    return 0;
}