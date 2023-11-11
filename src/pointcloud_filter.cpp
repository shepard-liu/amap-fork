
#include <math.h>

#include <iostream>

#include "amap/point_cloud.h"
#include "amap/voxel_filter.h"

using namespace alive;

int main(int argc, char **argv)
{
    VoxelFilterParam vgmparam;
    vgmparam.voxel_size = 0.2;
    VoxelGridFilter::Ptr downSizeFilterMap = VoxelGridFilter::create(vgmparam);

    return 1;
}