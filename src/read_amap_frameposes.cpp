
#include <iostream>
#include <math.h>
#include "amap_factory.h"

using namespace alive;

int main(int argc, char **argv)
{
    std::string strMap = "/home/zhihui/dataset/timmo/2021-09-10-09-41-02.amap";
    
    AliveMapFileLoaderPtr mLoader;

    mLoader = alive::AliveMapFileIOFactory::getLoader(strMap);

    if (mLoader)
    {
        mLoader->openFile(strMap);
        int nframeNum = mLoader->getNumOfFrames();
        printf("map nframeNum count : %d\n", nframeNum);

        int nSubmapNum = mLoader->getNumOfSubmaps();
        printf("map submap count : %d\n", nSubmapNum);

        std::vector<alive::AliveMapFramePtr> frames;

        mLoader->loadFrames(frames);

        Eigen::aligned_vector<alive::AmapPoseData> poses;
        frames[0]->getFramePoses(poses);
        for(int i = 0; i < poses.size(); i++){
            std::cout << "---[" << i << "]---" << std::endl;
            std::cout << "time " << poses[i].timestamp << std::endl;
            std::cout << "pos " << poses[i].data_p.transpose() << std::endl;
            std::cout << "rot " << poses[i].data_q.coeffs().transpose() << std::endl;
            std::cout << "acc " << poses[i].data_acc.transpose() << std::endl;
            std::cout << "gyr " << poses[i].data_gyr.transpose() << std::endl;
            std::cout << "vel " << poses[i].data_vel.transpose() << std::endl;
            std::cout << "\n" << std::endl;
        }
    }
    
    return 1;
}