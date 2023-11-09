

#include <iostream>
#include <math.h>
#include "amap_update.h"

using namespace alive;

inline bool file_exists(const std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv)
{
    std::string dataPath = "/home/wlt027/dataset/suzhou/all_map.amap";

    std::vector<alive::AmapPoseData> kf_poses;

    // add new key pose
    alive::AmapPoseData new_pose;
    new_pose.timestamp = 1;
    new_pose.data_p = Eigen::Vector3d(0,0,0);
    new_pose.data_q = Eigen::Quaterniond(1,0,0,0);
    kf_poses.push_back(new_pose);

    // update
    alive::AliveMapFileUpdater::Ptr updater = alive::AliveMapFileUpdater::create();

    if (updater && file_exists(dataPath))
    {
        updater->updatePoseToMapFile(kf_poses, dataPath);
    }

    return 1;
}
