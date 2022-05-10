#ifndef WORKSPACE_SPREADER_H
#define WORKSPACE_SPREADER_H

#include <vector>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace workspace_spreader
{
class WorkspaceSpreader
{
public: 
    WorkspaceSpreader(){}
    ~WorkspaceSpreader(){}

    // Create Voxel
    void CreateWorkspace(double diameter, double resolution, std::vector< std::vector< double > >& center);

    // Make Sphere Poses
    void SpherePoses(std::vector< tf2::Quaternion >& quat);

};
}

#endif