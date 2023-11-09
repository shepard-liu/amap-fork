
#include <iostream>
#include <math.h>
#include "point_cloud.h"
#include "voxel_filter.h"

using namespace alive;

int main(int argc, char **argv)
{
    VoxelFilterParam vgmparam;
    vgmparam.voxel_size = 0.2;
    VoxelGridFilter::Ptr downSizeFilterMap = VoxelGridFilter::create(vgmparam);

    return 1;
}