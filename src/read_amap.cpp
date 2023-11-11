
#include <math.h>

#include <iostream>

#include "amap/amap_factory.h"

using namespace alive;

int main(int argc, char **argv)
{
    std::string strMap = "/home/wlt027/dataset/test.amap";

    AliveMapFileLoaderPtr mLoader;

    mLoader = alive::AliveMapFileIOFactory::getLoader(strMap);

    if (mLoader)
    {
        mLoader->openFile(strMap);
        int nframeNum = mLoader->getNumOfFrames();
        printf("map nframeNum count : %d\n", nframeNum);

        int nSubmapNum = mLoader->getNumOfSubmaps();
        printf("map submap count : %d\n", nSubmapNum);

        // get every frame info
        for (unsigned int i = 0; i < nframeNum; i++)
        {
            alive::AliveMapFramePtr frame = mLoader->loadFrame(i);

            // global pose
            Eigen::Vector3d p;
            Eigen::Quaterniond q;
            double time_stamp = frame->getRealTimestamp();
            frame->getPose(p, q);

            alive::PointCloud::Ptr frame_pcd = frame->getPointCloud();

            for (auto &pt : frame_pcd->points())
            {
                // get point pose in world frame
                Eigen::Vector3d tmp_p(pt.x, pt.y, pt.z);
                Eigen::Vector3d global_p = q * tmp_p + p;
            }
        }
    }

    return 1;
}