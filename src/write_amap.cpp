
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>

#include "amap_factory.h"

using namespace alive;

inline bool file_exists(const std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv)
{
    std::string strMap = "/home/wlt027/dataset/2022-05-06-09-30-31.amap";
    std::string strMapOut = "/home/wlt027/dataset/test.amap";

    AliveMapFileLoaderPtr mLoader;
    AliveMapFileSaverPtr mAlphaSaver = nullptr;

    int type = 1;

    mAlphaSaver = AliveMapFileIOFactory::getSaver(type);

    if (file_exists(strMap))
    {
        mAlphaSaver->openFile(strMapOut);

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

                mAlphaSaver->saveFrame(frame);
            }
        }

        // do some operation
        // mAlphaSaver->saveFrame(kf);

        // mAlphaSaver->saveSubmap(smap);
    }

    if (mAlphaSaver)
    {
        mAlphaSaver->closeFile();
    }

    return 1;
}