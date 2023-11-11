#ifndef PROJECT_AMAP_IO_H
#define PROJECT_AMAP_IO_H

#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

#include "geodetic_common.h"
#include "point_cloud.h"

namespace Eigen
{
    template <typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

} // namespace Eigen

namespace alive
{
    struct AmapPoseData
    {
        double timestamp;
        Eigen::Vector3d data_p;
        Eigen::Quaterniond data_q;
        Eigen::Vector3d data_acc;
        Eigen::Vector3d data_gyr;
        Eigen::Vector3d data_vel;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct GlobalData
    {
        double timestamp;
        Eigen::Vector3d lla;
        Eigen::Matrix3d covariance;
        double heading;
        double headingCov;
        GPS_COV_TYPE cov_type;
        GPS_FIX_TYPE fix_type;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct InertialData
    {
        double timestamp;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyr;
        Eigen::Quaterniond ori;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class AliveMapFrameBase
    {
    public:
        virtual int getType() = 0;
        virtual double getRealTimestamp() = 0;
        virtual void getPose(Eigen::Vector3d &p, Eigen::Quaterniond &q) = 0;
        virtual PointCloud::Ptr getPointCloud() = 0;
        virtual PointCloud::Ptr getCorners() = 0;
        virtual PointCloud::Ptr getSurfels() = 0;
        virtual unsigned int getFrameID() = 0;
        virtual int getKeyFrameID() = 0;
        virtual void setFrameID(int nID) = 0;
        virtual void setKeyFrameID(int nID) = 0;

        virtual Eigen::aligned_vector<GlobalData> getGlobalPose() = 0;
        virtual void getFramePoses(Eigen::aligned_vector<AmapPoseData> &frameposes) = 0;
        virtual void getFrameIMU(Eigen::aligned_vector<InertialData> &imus) = 0;
        virtual void getScanLoopInfo(Eigen::MatrixXd &info) = 0;
    };

    typedef std::shared_ptr<AliveMapFrameBase> AliveMapFramePtr;

    class AliveSubmapBase
    {
    public:
        virtual double getAnchorFrameTimestamp() = 0;

        virtual void getAnchorPose(Eigen::Vector3d &p, Eigen::Quaterniond &q) = 0;

        virtual PointCloud::Ptr getPointCloud() = 0;

        virtual double getRealTimestamp() = 0;

        virtual void getFramesID(std::vector<int> &frames) = 0;

        virtual void getContextInfoMatrix(Eigen::MatrixXd &info) = 0;
    };

    typedef std::shared_ptr<AliveSubmapBase> AliveSubmapPtr;

    class AliveLoopInfoBase
    {
    public:
        virtual void getRelativePose(Eigen::Vector3d &p, Eigen::Quaterniond &q) = 0;

        virtual int getCurrentKeyframeID() = 0;

        virtual int getMatchedKeyFrameID() = 0;

        virtual int getTimeStamp() = 0;
    };

    typedef std::shared_ptr<AliveLoopInfoBase> AliveLoopPtr;

    class AliveMapFileLoaderBase
    {
    public:
        virtual AliveMapFramePtr loadFrame(int i) = 0;

        virtual AliveMapFramePtr loadKeyFrame(int i) = 0;

        virtual AliveSubmapPtr loadSubmap(int i) = 0;

        virtual AliveLoopPtr loadLoop(int i) = 0;

        virtual bool loadFrames(std::vector<AliveMapFramePtr> &frames) = 0;

        virtual bool loadSubmaps(std::vector<AliveSubmapPtr> &submaps) = 0;

        virtual bool loadLoops(std::vector<AliveLoopPtr> &loops) = 0;

        virtual bool openFile(std::string path) = 0;

        virtual bool closeFile() = 0;

        virtual unsigned int getNumOfFrames() = 0;

        virtual unsigned int getNumOfKeyFrames() = 0;

        virtual unsigned int getNumOfSubmaps() = 0;

        virtual unsigned int getNumOfLoops() = 0;

        virtual unsigned int getNumOfPoints() = 0;

        virtual unsigned int getMapType() = 0;

        virtual void getGloablCenter(Eigen::Vector3d &c) = 0;

        virtual void getBodyGlobalOffset(Eigen::Vector3f &pbg) = 0;

        virtual int getType() = 0;

        virtual void getBoundingBox(double &min_x,double &max_x,double &min_y,double &max_y,double &min_z,double &max_z) = 0;
    };

    typedef std::shared_ptr<AliveMapFileLoaderBase> AliveMapFileLoaderPtr;

    class AliveMapFileSaverBase
    {
    public:
        virtual void saveFrame(AliveMapFramePtr &frame) = 0;

        virtual void saveSubmap(AliveSubmapPtr &submap) = 0;

        virtual void saveLoopInfo(AliveLoopPtr &loop) = 0;

        virtual bool openFile(std::string path) = 0;

        virtual void setBoundingBox(double &min_x,double &max_x,double &min_y,double &max_y,double &min_z,double &max_z) = 0;

        virtual bool closeFile() = 0;
    };

    typedef std::shared_ptr<AliveMapFileSaverBase> AliveMapFileSaverPtr;
} // namespace alive
#endif