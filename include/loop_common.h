#ifndef PROJECT_ALIVE_LOOP_COMMON_H
#define PROJECT_ALIVE_LOOP_COMMON_H

#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace alive
{
   struct LoopPose6D 
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

    class AliveLoopInfo
    {
    public:
        AliveLoopInfo()
        {
            current_id = -1;
            matched_id = -1;
            current_time = 0;
            matched_tm = 0;

            p = Eigen::Vector3d(0, 0, 0);
            q.setIdentity();
        }

    public:
        int current_id;
        int matched_id;

        double current_time;
        double matched_tm;

        Eigen::Vector3d p;    // position
        Eigen::Quaterniond q; // orientation

        Eigen::MatrixXd sc;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<AliveLoopInfo> LoopInfoPtr;

} // namespace alive

#endif