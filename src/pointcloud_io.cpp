
#include <math.h>

#include <iostream>

#include "amap/point_cloud.h"

using namespace alive;

int main(int argc, char **argv)
{
    std::vector<float> src_points_vec{
        1.15495, 2.40671, 1.15061, 1.81481, 2.06281, 1.71927, 0.888322,
        2.05068, 2.04879, 3.78842, 1.70788, 1.30246, 1.8437, 2.22894,
        0.986237, 2.95706, 2.2018, 0.987878, 1.72644, 1.24356, 1.93486,
        0.922024, 1.14872, 2.34317, 3.70293, 1.85134, 1.15357, 3.06505,
        1.30386, 1.55279, 0.634826, 1.04995, 2.47046, 1.40107, 1.37469,
        1.09687, 2.93002, 1.96242, 1.48532, 3.74384, 1.30258, 1.30244};

    PointCloud source_l_down;
    for (int i = 0; i < 14; i++)
    {
        float x = src_points_vec[i * 3 + 0];
        float y = src_points_vec[i * 3 + 1];
        float z = src_points_vec[i * 3 + 2];
        PointCloud::PointType pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        source_l_down.push_back(pt);
    }

    PointCloud::Ptr pcd = source_l_down.make_shared();

    if(pcd)
    {
        printf("point size: %ld\n", pcd->size());
        for(int i = 0; i< pcd->size();i++)
        {
            printf("point info: %d,%f,%f,%f\n",i,(*pcd)[i].x,(*pcd)[i].y,(*pcd)[i].z);
        }
    }

    return 1;
}