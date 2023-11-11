

#include <math.h>

#include <iostream>

#include "amap/amap_common.h"
#include "amap/amap_factory.h"
#include "amap/amap_io.h"

using namespace alive;

int main(int argc, char **argv)
{
    int headersize = sizeof(AliveMapHeader);

    printf("amap header size: %d\n",headersize);

    std::string strMap = "/home/zhihui/dataset/rs16/shenzhen_office_alpha.amap";
    
    AliveMapFileLoaderPtr mLoader;

    mLoader = alive::AliveMapFileIOFactory::getLoader(strMap);

    if (mLoader)
    {
        mLoader->openFile(strMap);
        int nframeNum = mLoader->getNumOfFrames();
        printf("map nframeNum count : %d\n", nframeNum);

        int nSubmapNum = mLoader->getNumOfSubmaps();
        printf("map submap count : %d\n", nSubmapNum);

        double min_x,max_x,min_y,max_y,min_z,max_z;

        mLoader->getBoundingBox(min_x,max_x,min_y,max_y,min_z,max_z);

        printf("bounding box : %lf,%lf,%lf,%lf,%lf,%lf\n", min_x,max_x,min_y,max_y,min_z,max_z);
    }
    return 1;
}
