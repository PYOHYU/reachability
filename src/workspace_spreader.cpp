#include <reachability/workspace_spreader.h>

namespace workspace_spreader
{
    // Create Voxel
    void WorkspaceSpreader::CreateWorkspace(double diameter, double resolution, std::vector< std::vector< double > >& center)
    {
        double dx = resolution;
        double dy = resolution;
        double dz = resolution;

        for (double x = 0; x <= diameter; x += dx)
        {
            for (double y = - diameter; y <= diameter; y += dy)
            {
                //for (double z = 0; z <= 2 * diameter; z += dz)
                for (double z = 0.6; z <= 1.2; z += dz)
                {
                    std::vector< double > vox{x, y, z};

                    center.push_back(vox);
                }
            }
        }
        return;
    }

    // Make Sphere Poses
    void WorkspaceSpreader::SpherePoses(std::vector< tf2::Quaternion >& quat)
    {
        const double DELTA = M_PI / 6;

        for (double phi = M_PI / 2; phi <= M_PI * 3 / 2; phi += DELTA) // elevation
        //for (double phi = M_PI; phi <= M_PI; phi += DELTA)
        {
            for (double theta = M_PI / 2; theta <= M_PI * 3 / 2; theta +=  DELTA)  // azimuth
            //for (double theta = M_PI; theta <= M_PI; theta +=  DELTA)
            {
                tf2::Quaternion q;
                q.setRPY(0, theta, phi);
                q.normalize();

                quat.push_back(q);
            }
        }
        return;
    }

}