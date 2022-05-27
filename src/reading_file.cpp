#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>

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

void doBubbleSort(struct node **_array, int _byX, int _size)
{
	int i = 0;
	int j = 0;

	for (j = 0; j < _size; j++)
	{
		for (i = 0; i < _size - 1 - j; i++)
		{
			if (_byX == 1)  
			{
				if (_array[i]->z > _array[i + 1]->z)
				{
					// swap
					struct node *temp = _array[i];
					_array[i] = _array[i + 1];
					_array[i + 1] = temp;
				}
			}
			else        
			{
				if (_array[i]->p > _array[i + 1]->p)
				{
					// swap
					struct node *temp = _array[i];
					_array[i] = _array[i + 1];
					_array[i + 1] = temp;
				}
			}
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
	doBubbleSort(nodeAddrArray, !axis, numSLLNodes);
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

struct kdtree_node *NearestNeighbor(struct kdtree_node *_root, double _z, double _p)
{
	double minDiff = INT_MAX;
	struct kdtree_node *minDiffNode = 0;

	struct kdtree_node *temp = _root;
	int axis = 0; // start to z
	while (1)
	{
		if (temp == 0)
			return minDiffNode;

		// distance between (_z,_p) and  temp
		double dist = (temp->data->z - _z) *  (temp->data->z - _z) + (temp->data->p - _p)* (temp->data->p - _p);
        ROS_INFO("dist : %f", dist);
		//Find new Nearest Neighbor
		if (dist < minDiff)
		{
			minDiff = dist;
			minDiffNode = temp;
		}

		//struct kdtree_node *whichWayToGo(temp, _z, _p, axis)
        
		temp = whichWayToGo(temp, _z, _p, axis); // find next temp

		axis = (axis + 1) % 2;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_time_check");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("ifstream/ofstream");

    ros::Time startit = ros::Time::now();

    std::ifstream loadFile(ros::package::getPath("reachability") + "/src/" + 
    "right_arm_inverse_reachability5_.csv");

    if (loadFile.fail())
    {
        ROS_ERROR("There is no file such name");
        return -1;
    }

    std::string line = "";
    std::string chain_start;
    std::string chain_end;
    double JntNum;
    std::vector< std::pair<double, double> > zp_data;
    std::vector< std::vector< std::vector<double> > > sub_data_list;
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

    ROS_INFO_STREAM(sub_data_list[0].size());

    int num = howManyNodes(head);
    ROS_INFO("howManyNodes : %d", num);

    kd_root = build_kdtree(head, 0, 2);

    ros::Time searchstart = ros::Time::now();

    struct kdtree_node *ans = NearestNeighbor(kd_root, 1.05, -1.37);
    ROS_INFO("z : %f, p : %f, idx : %d", ans->data->z, ans->data->p, ans->data->idx);
    int ansidx = ans->data->idx;
    double ansmu = sub_data_list[ansidx][0][7];
    ROS_INFO("mu : %f", ansmu);

    struct kdtree_node *realPans = RealNearestNeighbor(kd_root, 1.05, -1.37, 10, 0, 0, 2);
    ROS_INFO("z : %f, p : %f, idx : %d", realPans->data->z, realPans->data->p, realPans->data->idx);
    int realPansidx = realPans->data->idx;
    double realPansmu = sub_data_list[realPansidx][0][7];
    ROS_INFO("mu : %f", realPansmu);

    double dif = ros::Duration( ros::Time::now() - searchstart).toNSec() / 1000000;
    ROS_INFO("Time duration : %lf ms", dif);

    return 0;

}