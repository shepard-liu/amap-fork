﻿
#ifndef PROJECT_AMAP_IO_GAMMA_H
#define PROJECT_AMAP_IO_GAMMA_H

#include <memory>
#include <queue>
#include <map>
#include <fstream>

#include <Eigen/Dense>

#include "amap_common.h"
#include "amap_io.h"

namespace alive
{
    class AliveMapGammaFrame : public AliveMapFrameBase
    {
    public:
        AliveMapGammaFrame()
        {
            ori_corner = PointCloud::create();
            ori_surfel = PointCloud::create();
            und_surfel = PointCloud::create();
            tm = 0;
            P = Eigen::Vector3d(0, 0, 0);
            Q.setIdentity();
            frameID = -1;
            keyFrameID = -1;
        }

        virtual int getType()
        {
            return 3;
        }

        virtual double getRealTimestamp()
        {
            return tm;
        }

        virtual void getPose(Eigen::Vector3d &p, Eigen::Quaterniond &q)
        {
            p = P;
            q = Q;
        }

        virtual PointCloud::Ptr getPointCloud()
        {
            return und_surfel;
        }

        virtual PointCloud::Ptr getCorners()
        {
            return ori_corner;
        }

        virtual PointCloud::Ptr getSurfels()
        {
            return ori_surfel;
        }

        virtual unsigned int getFrameID()
        {
            return frameID;
        }

        virtual int getKeyFrameID()
        {
            return keyFrameID;
        }

        virtual void setFrameID(int nID)
        {
            frameID = nID;
        }

        virtual void setKeyFrameID(int nID)
        {
            keyFrameID = nID;
        }
        
        virtual Eigen::aligned_vector<GlobalData> getGlobalPose()
        {
            Eigen::aligned_vector<GlobalData> data;
            return data;
        }

        virtual void getFramePoses(Eigen::aligned_vector<AmapPoseData> &frameposes)
        {
            frameposes.clear();

            for (auto &p : mFramePoses)
            {
                frameposes.push_back(p);
            }
        }

        virtual void getFrameIMU(Eigen::aligned_vector<InertialData> &imus)
        {
            imus.clear();

            for (auto &p : mIMU)
            {
                imus.push_back(p);
            }
        }

        virtual void getScanLoopInfo(Eigen::MatrixXd &info)
        {
            info = scan_context;
        }

    public:
        PointCloud::Ptr und_surfel;

        PointCloud::Ptr ori_surfel;
        PointCloud::Ptr ori_corner;

        double tm;
        Eigen::Vector3d P;
        Eigen::Quaterniond Q;
        unsigned int frameID;
        int keyFrameID;

        std::vector<AmapPoseData> mFramePoses;
        std::vector<InertialData> mIMU;

        Eigen::MatrixXd scan_context;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<AliveMapGammaFrame> AliveMapGammaFramePtr;

    class AliveMapFileGammaLoader : public AliveMapFileLoaderBase
    {
    public:
        virtual AliveMapFramePtr loadFrame(int i)
        {
            if (i < 0 || i >= mHeader.num_of_frames)
                return nullptr;

            if (!fs.is_open())
                return nullptr;

            long long pos = fm_poses[i];

            fs.seekg(pos, std::ios::beg);

            AliveMapGammaFramePtr frame(new AliveMapGammaFrame);
            {
                // ID
                unsigned int frameID = 0;
                int keyFrameID = 0;
                double tm = 0;

                fs.read((char *)&frameID, sizeof(unsigned int));
                fs.read((char *)&keyFrameID, sizeof(int));
                fs.read((char *)&tm, sizeof(double));

                frame->frameID = frameID;
                frame->keyFrameID = keyFrameID;
                frame->tm = tm;

                // T_W_B
                double tx = 0;
                double ty = 0;
                double tz = 0;
                double qx = 0;
                double qy = 0;
                double qz = 0;
                double qw = 1;
                fs.read((char *)(&tx), sizeof(double));
                fs.read((char *)(&ty), sizeof(double));
                fs.read((char *)(&tz), sizeof(double));
                fs.read((char *)(&qx), sizeof(double));
                fs.read((char *)(&qy), sizeof(double));
                fs.read((char *)(&qz), sizeof(double));
                fs.read((char *)(&qw), sizeof(double));

                frame->P = Eigen::Vector3d(tx, ty, tz);
                frame->Q.x() = qx;
                frame->Q.y() = qy;
                frame->Q.z() = qz;
                frame->Q.w() = qw;
                frame->Q.normalize();

                // printf("load keyframe:%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",keyFrameID,tx,ty,tz,qx,qy,qz,qw);

                // laserScanRect
                int sizeOfRawScanRect = 0;
                fs.read((char *)&sizeOfRawScanRect, sizeof(int));

                for (int k = 0; k < sizeOfRawScanRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)&x, sizeof(float));
                    fs.read((char *)&y, sizeof(float));
                    fs.read((char *)&z, sizeof(float));
                    fs.read((char *)&intensity, sizeof(uint8_t));

                    // pcl::PointXYZI pt;
                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;

                    frame->und_surfel->push_back(pt);

                    if (mHeader.use_raw_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->und_surfel->push_back_time(pt_tm);
                    }
                    if (mHeader.use_raw_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->und_surfel->push_back_ring(pt_ring);
                    }
                }

                // corners
                int sizeOfScanCornerRect = 0;
                fs.read((char *)&sizeOfScanCornerRect, sizeof(int));

                for (int k = 0; k < sizeOfScanCornerRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)(&x), sizeof(float));
                    fs.read((char *)(&y), sizeof(float));
                    fs.read((char *)(&z), sizeof(float));
                    fs.read((char *)(&intensity), sizeof(uint8_t));

                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;
                    frame->ori_corner->push_back(pt);

                    if (mHeader.use_corner_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->ori_corner->push_back_time(pt_tm);
                    }
                    if (mHeader.use_corner_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->ori_corner->push_back_ring(pt_ring);
                    }
                }

                // surfels
                int sizeOfScanSurfRect = 0;
                fs.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                for (int k = 0; k < sizeOfScanSurfRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)(&x), sizeof(float));
                    fs.read((char *)(&y), sizeof(float));
                    fs.read((char *)(&z), sizeof(float));
                    fs.read((char *)(&intensity), sizeof(uint8_t));

                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;
                    frame->ori_surfel->push_back(pt);

                    if (mHeader.use_surfel_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->ori_surfel->push_back_time(pt_tm);
                    }
                    if (mHeader.use_surfel_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->ori_surfel->push_back_ring(pt_ring);
                    }
                }

                // GPSReadings
                int sizeOfGPS = 0;
                fs.read((char *)&sizeOfGPS, sizeof(int));

                for (int k = 0; k < sizeOfGPS; k++)
                {
                    double timestamp = 0;
                    Eigen::Vector3d lla(0, 0, 0);
                    Eigen::Matrix3d covariance;
                    covariance.setZero();

                    GPS_COV_TYPE cov_type;
                    GPS_FIX_TYPE fix_type;
                    double heading;
                    double headingCov;

                    fs.read((char *)(&timestamp), sizeof(double));
                    fs.read((char *)(&lla(0)), sizeof(double));
                    fs.read((char *)(&lla(1)), sizeof(double));
                    fs.read((char *)(&lla(2)), sizeof(double));

                    fs.read((char *)(&covariance(0, 0)), sizeof(double));
                    fs.read((char *)(&covariance(0, 1)), sizeof(double));
                    fs.read((char *)(&covariance(0, 2)), sizeof(double));
                    fs.read((char *)(&covariance(1, 0)), sizeof(double));
                    fs.read((char *)(&covariance(1, 1)), sizeof(double));
                    fs.read((char *)(&covariance(1, 2)), sizeof(double));
                    fs.read((char *)(&covariance(2, 0)), sizeof(double));
                    fs.read((char *)(&covariance(2, 1)), sizeof(double));
                    fs.read((char *)(&covariance(2, 2)), sizeof(double));

                    fs.read((char *)(&cov_type), sizeof(char));
                    fs.read((char *)(&fix_type), sizeof(char));
                    fs.read((char *)(&heading), sizeof(double));
                    fs.read((char *)(&headingCov), sizeof(double));

                    // printf("gps:%lf,%lf,%lf\n",lla(0),lla(1),lla(2));

                    GlobalData gps;
                    gps.timestamp = timestamp;
                    gps.lla = lla;
                    gps.covariance = covariance;
                    gps.cov_type = cov_type;
                    gps.fix_type = fix_type;
                    gps.heading = heading;
                    gps.headingCov = headingCov;
                }

                // frame poses
                if (mHeader.local_frame_count > 0)
                {
                    int sizeOfLocalFrame = 0;
                    fs.read((char *)&sizeOfLocalFrame, sizeof(int));

                    // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                    for (int k = 0; k < sizeOfLocalFrame; k++)
                    {
                        double timestamp = 0;
                        Eigen::Vector3d p(0, 0, 0);
                        Eigen::Quaterniond q;
                        q.setIdentity();

                        fs.read((char *)(&timestamp), sizeof(double));
                        fs.read((char *)(&p(0)), sizeof(double));
                        fs.read((char *)(&p(1)), sizeof(double));
                        fs.read((char *)(&p(2)), sizeof(double));

                        fs.read((char *)(&q.w()), sizeof(double));
                        fs.read((char *)(&q.x()), sizeof(double));
                        fs.read((char *)(&q.y()), sizeof(double));
                        fs.read((char *)(&q.z()), sizeof(double));

                        AmapPoseData apd;
                        apd.timestamp = timestamp;
                        apd.data_p = p;
                        apd.data_q = q;

                        frame->mFramePoses.push_back(apd);
                        // printf("pose time:%lf\n",apd.timestamp);
                    }
                }

                // scan context
                if (mHeader.use_sc > 0)
                {
                    int sizeOfrow = 0;
                    int sizeOfcol = 0;
                    fs.read((char *)&sizeOfrow, sizeof(int));
                    fs.read((char *)&sizeOfcol, sizeof(int));

                    frame->scan_context.resize(sizeOfrow, sizeOfcol);
                    for (int i = 0; i < sizeOfrow; i++)
                    {
                        for (int j = 0; j < sizeOfcol; j++)
                        {
                            double val = 0;
                            fs.read((char *)(&val), sizeof(double));

                            frame->scan_context(i, j) = val;
                        }
                    }
                }

                // local imu
                if (mHeader.use_imu > 0)
                {
                    int sizeOfIMU = 0;
                    fs.read((char *)&sizeOfIMU, sizeof(int));

                    // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                    for (int k = 0; k < sizeOfIMU; k++)
                    {
                        double timestamp = 0;
                        Eigen::Vector3d acc(0, 0, 0);
                        Eigen::Vector3d gyr(0, 0, 0);
                        Eigen::Quaterniond ori;
                        ori.setIdentity();

                        fs.read((char *)(&timestamp), sizeof(double));
                        fs.read((char *)(&acc(0)), sizeof(double));
                        fs.read((char *)(&acc(1)), sizeof(double));
                        fs.read((char *)(&acc(2)), sizeof(double));
                        fs.read((char *)(&gyr(0)), sizeof(double));
                        fs.read((char *)(&gyr(1)), sizeof(double));
                        fs.read((char *)(&gyr(2)), sizeof(double));
                        fs.read((char *)(&ori.w()), sizeof(double));
                        fs.read((char *)(&ori.x()), sizeof(double));
                        fs.read((char *)(&ori.y()), sizeof(double));
                        fs.read((char *)(&ori.z()), sizeof(double));

                        InertialData imu;
                        imu.timestamp = timestamp;
                        imu.acc = acc;
                        imu.gyr = gyr;
                        imu.ori = ori;

                        frame->mIMU.push_back(imu);
                    }
                }
            }

            return frame;
        }

        virtual AliveMapFramePtr loadKeyFrame(int i)
        {
            if (i < 0 || i >= mHeader.num_of_keyframes)
                return nullptr;

            if (!fs.is_open())
                return nullptr;

            long long pos = kfm_poses[i];

            fs.seekg(pos, std::ios::beg);

            AliveMapGammaFramePtr frame(new AliveMapGammaFrame);
            {
                // ID
                unsigned int frameID = 0;
                int keyFrameID = 0;
                double tm = 0;

                fs.read((char *)&frameID, sizeof(unsigned int));
                fs.read((char *)&keyFrameID, sizeof(int));
                fs.read((char *)&tm, sizeof(double));

                frame->frameID = frameID;
                frame->keyFrameID = keyFrameID;
                frame->tm = tm;

                // T_W_B
                double tx = 0;
                double ty = 0;
                double tz = 0;
                double qx = 0;
                double qy = 0;
                double qz = 0;
                double qw = 1;
                fs.read((char *)(&tx), sizeof(double));
                fs.read((char *)(&ty), sizeof(double));
                fs.read((char *)(&tz), sizeof(double));
                fs.read((char *)(&qx), sizeof(double));
                fs.read((char *)(&qy), sizeof(double));
                fs.read((char *)(&qz), sizeof(double));
                fs.read((char *)(&qw), sizeof(double));

                frame->P = Eigen::Vector3d(tx, ty, tz);
                frame->Q.x() = qx;
                frame->Q.y() = qy;
                frame->Q.z() = qz;
                frame->Q.w() = qw;
                frame->Q.normalize();

                // printf("load keyframe:%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",keyFrameID,tx,ty,tz,qx,qy,qz,qw);

                // laserScanRect
                int sizeOfRawScanRect = 0;
                fs.read((char *)&sizeOfRawScanRect, sizeof(int));

                for (int k = 0; k < sizeOfRawScanRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)&x, sizeof(float));
                    fs.read((char *)&y, sizeof(float));
                    fs.read((char *)&z, sizeof(float));
                    fs.read((char *)&intensity, sizeof(uint8_t));

                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;
                    frame->und_surfel->push_back(pt);

                    if (mHeader.use_raw_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->und_surfel->push_back_time(pt_tm);
                    }
                    if (mHeader.use_raw_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->und_surfel->push_back_ring(pt_ring);
                    }
                }

                // corners
                int sizeOfScanCornerRect = 0;
                fs.read((char *)&sizeOfScanCornerRect, sizeof(int));

                for (int k = 0; k < sizeOfScanCornerRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)(&x), sizeof(float));
                    fs.read((char *)(&y), sizeof(float));
                    fs.read((char *)(&z), sizeof(float));
                    fs.read((char *)(&intensity), sizeof(uint8_t));

                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;
                    frame->ori_corner->push_back(pt);

                    if (mHeader.use_corner_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->ori_corner->push_back_time(pt_tm);
                    }
                    if (mHeader.use_corner_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->ori_corner->push_back_ring(pt_ring);
                    }
                }

                // surfels
                int sizeOfScanSurfRect = 0;
                fs.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                for (int k = 0; k < sizeOfScanSurfRect; k++)
                {
                    float x = 0;
                    float y = 0;
                    float z = 0;
                    uint8_t intensity = 0;
                    fs.read((char *)(&x), sizeof(float));
                    fs.read((char *)(&y), sizeof(float));
                    fs.read((char *)(&z), sizeof(float));
                    fs.read((char *)(&intensity), sizeof(uint8_t));

                    PointCloud::PointType pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.intensity = intensity;
                    frame->ori_surfel->push_back(pt);

                    if (mHeader.use_surfel_time)
                    {
                        double pt_tm = 0;
                        fs.read((char *)&pt_tm, sizeof(double));
                        frame->ori_surfel->push_back_time(pt_tm);
                    }
                    if (mHeader.use_surfel_ring)
                    {
                        uint16_t pt_ring = 0;
                        fs.read((char *)&pt_ring, sizeof(uint16_t));
                        frame->ori_surfel->push_back_ring(pt_ring);
                    }
                }

                // GPSReadings
                int sizeOfGPS = 0;
                fs.read((char *)&sizeOfGPS, sizeof(int));

                for (int k = 0; k < sizeOfGPS; k++)
                {
                    double timestamp = 0;
                    Eigen::Vector3d lla(0, 0, 0);
                    Eigen::Matrix3d covariance;
                    covariance.setZero();

                    GPS_COV_TYPE cov_type;
                    GPS_FIX_TYPE fix_type;
                    double heading;
                    double headingCov;

                    fs.read((char *)(&timestamp), sizeof(double));
                    fs.read((char *)(&lla(0)), sizeof(double));
                    fs.read((char *)(&lla(1)), sizeof(double));
                    fs.read((char *)(&lla(2)), sizeof(double));

                    fs.read((char *)(&covariance(0, 0)), sizeof(double));
                    fs.read((char *)(&covariance(0, 1)), sizeof(double));
                    fs.read((char *)(&covariance(0, 2)), sizeof(double));
                    fs.read((char *)(&covariance(1, 0)), sizeof(double));
                    fs.read((char *)(&covariance(1, 1)), sizeof(double));
                    fs.read((char *)(&covariance(1, 2)), sizeof(double));
                    fs.read((char *)(&covariance(2, 0)), sizeof(double));
                    fs.read((char *)(&covariance(2, 1)), sizeof(double));
                    fs.read((char *)(&covariance(2, 2)), sizeof(double));

                    fs.read((char *)(&cov_type), sizeof(char));
                    fs.read((char *)(&fix_type), sizeof(char));
                    fs.read((char *)(&heading), sizeof(double));
                    fs.read((char *)(&headingCov), sizeof(double));

                    // printf("gps:%lf,%lf,%lf\n",lla(0),lla(1),lla(2));

                    GlobalData gps;
                    gps.timestamp = timestamp;
                    gps.lla = lla;
                    gps.covariance = covariance;
                    gps.cov_type = cov_type;
                    gps.fix_type = fix_type;
                    gps.heading = heading;
                    gps.headingCov = headingCov;
                }

                // frame poses
                if (mHeader.local_frame_count > 0)
                {
                    int sizeOfLocalFrame = 0;
                    fs.read((char *)&sizeOfLocalFrame, sizeof(int));

                    // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                    for (int k = 0; k < sizeOfLocalFrame; k++)
                    {

                        double timestamp = 0;
                        Eigen::Vector3d p(0, 0, 0);
                        Eigen::Quaterniond q;
                        q.setIdentity();

                        fs.read((char *)(&timestamp), sizeof(double));
                        fs.read((char *)(&p(0)), sizeof(double));
                        fs.read((char *)(&p(1)), sizeof(double));
                        fs.read((char *)(&p(2)), sizeof(double));

                        fs.read((char *)(&q.w()), sizeof(double));
                        fs.read((char *)(&q.x()), sizeof(double));
                        fs.read((char *)(&q.y()), sizeof(double));
                        fs.read((char *)(&q.z()), sizeof(double));

                        AmapPoseData apd;
                        apd.timestamp = timestamp;
                        apd.data_p = p;
                        apd.data_q = q;

                        frame->mFramePoses.push_back(apd);

                        // printf("pose time:%lf\n",apd.timestamp);
                    }
                }

                // scan context
                if (mHeader.use_sc > 0)
                {
                    int sizeOfrow = 0;
                    int sizeOfcol = 0;
                    fs.read((char *)&sizeOfrow, sizeof(int));
                    fs.read((char *)&sizeOfcol, sizeof(int));

                    frame->scan_context.resize(sizeOfrow, sizeOfcol);
                    for (int i = 0; i < sizeOfrow; i++)
                    {
                        for (int j = 0; j < sizeOfcol; j++)
                        {
                            double val = 0;
                            fs.read((char *)(&val), sizeof(double));

                            frame->scan_context(i, j) = val;
                        }
                    }
                }

                // local imu
                if (mHeader.use_imu > 0)
                {
                    int sizeOfIMU = 0;
                    fs.read((char *)&sizeOfIMU, sizeof(int));

                    // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                    for (int k = 0; k < sizeOfIMU; k++)
                    {
                        double timestamp = 0;
                        Eigen::Vector3d acc(0, 0, 0);
                        Eigen::Vector3d gyr(0, 0, 0);
                        Eigen::Quaterniond ori;
                        ori.setIdentity();

                        fs.read((char *)(&timestamp), sizeof(double));
                        fs.read((char *)(&acc(0)), sizeof(double));
                        fs.read((char *)(&acc(1)), sizeof(double));
                        fs.read((char *)(&acc(2)), sizeof(double));
                        fs.read((char *)(&gyr(0)), sizeof(double));
                        fs.read((char *)(&gyr(1)), sizeof(double));
                        fs.read((char *)(&gyr(2)), sizeof(double));
                        fs.read((char *)(&ori.w()), sizeof(double));
                        fs.read((char *)(&ori.x()), sizeof(double));
                        fs.read((char *)(&ori.y()), sizeof(double));
                        fs.read((char *)(&ori.z()), sizeof(double));

                        InertialData imu;
                        imu.timestamp = timestamp;
                        imu.acc = acc;
                        imu.gyr = gyr;
                        imu.ori = ori;

                        frame->mIMU.push_back(imu);
                    }
                }
            }

            return frame;
        }

        virtual AliveSubmapPtr loadSubmap(int i)
        {
            return nullptr;
        }

        virtual AliveLoopPtr loadLoop(int i)
        {
            return nullptr;
        }

        virtual bool loadFrames(std::vector<AliveMapFramePtr> &frames)
        {
            for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
            {
                AliveMapFramePtr frame = loadFrame(i);
                frames.push_back(frame);
            }
            return true;
        }

        virtual bool loadSubmaps(std::vector<AliveSubmapPtr> &submaps)
        {
            return false;
        }

        virtual bool loadLoops(std::vector<AliveLoopPtr> &loops)
        {
            return false;
        }

        virtual bool openFile(std::string path)
        {
            fs.open(path.data(), std::ios::in | std::ios::binary);

            if (!fs.is_open())
            {
                std::cout << "CANNOT OPEN" << std::endl;
                return false;
            }

            // read header first
            fs.read(reinterpret_cast<char *>(&mHeader), sizeof(AliveMapHeader));

            // get frames file address
            for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
            {
                long long nSize = fs.tellg();
                fm_poses[i] = nSize;

                {
                    // ID
                    unsigned int frameID = 0;
                    int keyFrameID = 0;
                    double tm = 0;

                    fs.read((char *)&frameID, sizeof(unsigned int));
                    fs.read((char *)&keyFrameID, sizeof(int));
                    fs.read((char *)&tm, sizeof(double));

                    if (keyFrameID >= 0)
                    {
                        kfm_poses[keyFrameID] = nSize;
                    }

                    // T_W_B
                    double tx = 0;
                    double ty = 0;
                    double tz = 0;
                    double qx = 0;
                    double qy = 0;
                    double qz = 0;
                    double qw = 1;
                    fs.read((char *)(&tx), sizeof(double));
                    fs.read((char *)(&ty), sizeof(double));
                    fs.read((char *)(&tz), sizeof(double));
                    fs.read((char *)(&qx), sizeof(double));
                    fs.read((char *)(&qy), sizeof(double));
                    fs.read((char *)(&qz), sizeof(double));
                    fs.read((char *)(&qw), sizeof(double));

                    // laserScanRect
                    int sizeOfRawScanRect = 0;
                    fs.read((char *)&sizeOfRawScanRect, sizeof(int));

                    for (int k = 0; k < sizeOfRawScanRect; k++)
                    {
                        float x = 0;
                        float y = 0;
                        float z = 0;
                        uint8_t intensity = 0;
                        fs.read((char *)&x, sizeof(float));
                        fs.read((char *)&y, sizeof(float));
                        fs.read((char *)&z, sizeof(float));
                        fs.read((char *)&intensity, sizeof(uint8_t));

                        if (mHeader.use_raw_time)
                        {
                            double pt_tm = 0;
                            fs.read((char *)&pt_tm, sizeof(double));
                        }
                        if (mHeader.use_raw_ring)
                        {
                            uint16_t pt_ring = 0;
                            fs.read((char *)&pt_ring, sizeof(uint16_t));
                        }
                    }

                    // corners
                    int sizeOfScanCornerRect = 0;
                    fs.read((char *)&sizeOfScanCornerRect, sizeof(int));

                    for (int k = 0; k < sizeOfScanCornerRect; k++)
                    {
                        float x = 0;
                        float y = 0;
                        float z = 0;
                        uint8_t intensity = 0;
                        fs.read((char *)(&x), sizeof(float));
                        fs.read((char *)(&y), sizeof(float));
                        fs.read((char *)(&z), sizeof(float));
                        fs.read((char *)(&intensity), sizeof(uint8_t));

                        if (mHeader.use_corner_time)
                        {
                            double pt_tm = 0;
                            fs.read((char *)&pt_tm, sizeof(double));
                        }
                        if (mHeader.use_corner_ring)
                        {
                            uint16_t pt_ring = 0;
                            fs.read((char *)&pt_ring, sizeof(uint16_t));
                        }
                    }

                    // surfels
                    int sizeOfScanSurfRect = 0;
                    fs.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                    for (int k = 0; k < sizeOfScanSurfRect; k++)
                    {
                        float x = 0;
                        float y = 0;
                        float z = 0;
                        uint8_t intensity = 0;
                        fs.read((char *)(&x), sizeof(float));
                        fs.read((char *)(&y), sizeof(float));
                        fs.read((char *)(&z), sizeof(float));
                        fs.read((char *)(&intensity), sizeof(uint8_t));

                        if (mHeader.use_surfel_time)
                        {
                            double pt_tm = 0;
                            fs.read((char *)&pt_tm, sizeof(double));
                        }
                        if (mHeader.use_surfel_ring)
                        {
                            uint16_t pt_ring = 0;
                            fs.read((char *)&pt_ring, sizeof(uint16_t));
                        }
                    }

                    // GlobalData
                    int sizeOfGPS = 0;
                    fs.read((char *)&sizeOfGPS, sizeof(int));

                    for (int k = 0; k < sizeOfGPS; k++)
                    {

                        double timestamp = 0;
                        Eigen::Vector3d lla(0, 0, 0);
                        Eigen::Matrix3d covariance;
                        covariance.setZero();

                        GPS_COV_TYPE cov_type;
                        GPS_FIX_TYPE fix_type;
                        double heading;
                        double headingCov;

                        fs.read((char *)(&timestamp), sizeof(double));
                        fs.read((char *)(&lla(0)), sizeof(double));
                        fs.read((char *)(&lla(1)), sizeof(double));
                        fs.read((char *)(&lla(2)), sizeof(double));

                        fs.read((char *)(&covariance(0, 0)), sizeof(double));
                        fs.read((char *)(&covariance(0, 1)), sizeof(double));
                        fs.read((char *)(&covariance(0, 2)), sizeof(double));
                        fs.read((char *)(&covariance(1, 0)), sizeof(double));
                        fs.read((char *)(&covariance(1, 1)), sizeof(double));
                        fs.read((char *)(&covariance(1, 2)), sizeof(double));
                        fs.read((char *)(&covariance(2, 0)), sizeof(double));
                        fs.read((char *)(&covariance(2, 1)), sizeof(double));
                        fs.read((char *)(&covariance(2, 2)), sizeof(double));

                        fs.read((char *)(&cov_type), sizeof(char));
                        fs.read((char *)(&fix_type), sizeof(char));
                        fs.read((char *)(&heading), sizeof(double));
                        fs.read((char *)(&headingCov), sizeof(double));
                    }

                    // local frame pose
                    if (mHeader.local_frame_count > 0)
                    {
                        int sizeOfLocalFrame = 0;
                        fs.read((char *)&sizeOfLocalFrame, sizeof(int));

                        for (int k = 0; k < sizeOfLocalFrame; k++)
                        {
                            double timestamp = 0;
                            Eigen::Vector3d p(0, 0, 0);
                            Eigen::Quaterniond q;
                            q.setIdentity();

                            fs.read((char *)(&timestamp), sizeof(double));
                            fs.read((char *)(&p(0)), sizeof(double));
                            fs.read((char *)(&p(1)), sizeof(double));
                            fs.read((char *)(&p(2)), sizeof(double));

                            fs.read((char *)(&q.w()), sizeof(double));
                            fs.read((char *)(&q.x()), sizeof(double));
                            fs.read((char *)(&q.y()), sizeof(double));
                            fs.read((char *)(&q.z()), sizeof(double));
                        }
                    }

                    // scan context
                    if (mHeader.use_sc > 0)
                    {
                        int sizeOfrow = 0;
                        int sizeOfcol = 0;
                        fs.read((char *)&sizeOfrow, sizeof(int));
                        fs.read((char *)&sizeOfcol, sizeof(int));

                        for (int i = 0; i < sizeOfrow; i++)
                        {
                            for (int j = 0; j < sizeOfcol; j++)
                            {
                                double val = 0;
                                fs.read((char *)(&val), sizeof(double));
                            }
                        }
                    }

                    // local imu
                    if (mHeader.use_imu > 0)
                    {
                        int sizeOfIMU = 0;
                        fs.read((char *)&sizeOfIMU, sizeof(int));

                        // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                        for (int k = 0; k < sizeOfIMU; k++)
                        {
                            double timestamp = 0;
                            Eigen::Vector3d acc(0, 0, 0);
                            Eigen::Vector3d gyr(0, 0, 0);
                            Eigen::Quaterniond ori;
                            ori.setIdentity();

                            fs.read((char *)(&timestamp), sizeof(double));
                            fs.read((char *)(&acc(0)), sizeof(double));
                            fs.read((char *)(&acc(1)), sizeof(double));
                            fs.read((char *)(&acc(2)), sizeof(double));
                            fs.read((char *)(&gyr(0)), sizeof(double));
                            fs.read((char *)(&gyr(1)), sizeof(double));
                            fs.read((char *)(&gyr(2)), sizeof(double));
                            fs.read((char *)(&ori.w()), sizeof(double));
                            fs.read((char *)(&ori.x()), sizeof(double));
                            fs.read((char *)(&ori.y()), sizeof(double));
                            fs.read((char *)(&ori.z()), sizeof(double));
                        }
                    }
                }
            }

            return true;
        }

        virtual bool closeFile()
        {
            if (fs.is_open())
                fs.close();

            return true;
        }

        virtual unsigned int getNumOfFrames()
        {
            return mHeader.num_of_frames;
        }

		virtual unsigned int getNumOfPoints()
        {
            return mHeader.points_of_map;
        }

        virtual unsigned int getNumOfKeyFrames()
        {
            return mHeader.num_of_keyframes;
        }

        virtual unsigned int getNumOfSubmaps()
        {
            return mHeader.num_of_submaps;
        }

        virtual unsigned int getNumOfLoops()
        {
            return mHeader.num_of_loops;
        }

        virtual unsigned int getMapType()
        {
            return mHeader.map_type;
        }

        virtual void getGloablCenter(Eigen::Vector3d &c)
        {
            c = Eigen::Vector3d(mHeader.enu_center_lat, mHeader.enu_center_lon, mHeader.enu_center_att);
        }

        virtual void getBodyGlobalOffset(Eigen::Vector3f &pbg)
        {
            pbg = Eigen::Vector3f(mHeader.pbg_x, mHeader.pbg_y, mHeader.pbg_z);
        }

        virtual int getType()
        {
            return 3;
        }

        virtual void getBoundingBox(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z, double &max_z)
        {
            min_x = mHeader.min_x;
            max_x = mHeader.max_x;
            min_y = mHeader.min_y;
            max_y = mHeader.max_y;
            min_z = mHeader.min_z;
            max_z = mHeader.max_z;
        }

        std::map<int, long long> fm_poses;

        std::map<int, long long> kfm_poses;

        std::fstream fs;

        AliveMapHeader mHeader;
    };

    typedef std::shared_ptr<AliveMapFileGammaLoader> AliveMapFileGammaLoaderPtr;

    class AliveMapFileGammaSaver : public AliveMapFileSaverBase
    {
    public:
        virtual void saveFrame(AliveMapFramePtr &frame)
        {
            AliveMapGammaFramePtr kf;
            if (frame && frame->getType() == 3)
            {
                kf = std::dynamic_pointer_cast<AliveMapGammaFrame>(frame);
            }

            if (kf)
            {
                // first frame
                if (mHeader.num_of_frames == 0)
                {
                    // mHeader.timestamp_base = kf->tm;
                    // mHeader.enu_center_lon = kf->enu_init_lon;
                    // mHeader.enu_center_lat = kf->enu_init_lat;
                    // mHeader.enu_center_att = kf->enu_init_att;

                    // mHeader.pbg_x = kf->b_g_offset_x;
                    // mHeader.pbg_y = kf->b_g_offset_y;
                    // mHeader.pbg_z = kf->b_g_offset_z;

                    // by default we save local frame poses
                    mHeader.local_frame_count = 1;
                    if (kf->mFramePoses.size() > 0)
                    {
                        mHeader.local_frame_count = 1;
                    }
                    else
                    {
                        mHeader.local_frame_count = 0;
                    }
                    mHeader.use_sc = 0;
                    if (kf->scan_context.cols() > 0 && kf->scan_context.rows() > 0)
                    {
                        mHeader.use_sc = 1;
                    }
                    else
                    {
                        mHeader.use_sc = 0;
                    }

                    mHeader.use_imu = 1;
                    if (kf->mIMU.size() > 0)
                    {
                        mHeader.use_imu = 1;
                    }
                    else
                    {
                        mHeader.use_imu = 0;
                    }

                    mHeader.use_raw_time = 0;
                    if (kf->und_surfel && kf->und_surfel->times().size() > 0)
                    {
                        mHeader.use_raw_time = 1;
                    }

                    mHeader.use_raw_ring = 0;
                    if (kf->und_surfel && kf->und_surfel->rings().size() > 0)
                    {
                        mHeader.use_raw_ring = 1;
                    }

                    mHeader.use_corner_time = 0;
                    if (kf->ori_corner && kf->ori_corner->times().size() > 0)
                    {
                        mHeader.use_corner_time = 1;
                    }

                    mHeader.use_corner_ring = 0;
                    if (kf->ori_corner && kf->ori_corner->rings().size() > 0)
                    {
                        mHeader.use_corner_ring = 1;
                    }

                    mHeader.use_surfel_time = 0;
                    if (kf->ori_surfel && kf->ori_surfel->times().size() > 0)
                    {
                        mHeader.use_surfel_time = 1;
                    }

                    mHeader.use_surfel_ring = 0;
                    if (kf->ori_surfel && kf->ori_surfel->rings().size() > 0)
                    {
                        mHeader.use_surfel_ring = 1;
                    }
                }

                // save frame id & timestamp
                fs.write(reinterpret_cast<const char *>(&(kf->frameID)), sizeof(unsigned int));
                fs.write(reinterpret_cast<const char *>(&(kf->keyFrameID)), sizeof(int));
                fs.write(reinterpret_cast<const char *>(&(kf->tm)), sizeof(double));

                // save pose
                double tx = kf->P[0];
                double ty = kf->P[1];
                double tz = kf->P[2];
                double qx = kf->Q.x();
                double qy = kf->Q.y();
                double qz = kf->Q.z();
                double qw = kf->Q.w();
                fs.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                fs.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                // save raw scan
                int sizeOfRawScanRect = 0;
                if (kf->und_surfel)
                    sizeOfRawScanRect = kf->und_surfel->points().size();
                fs.write(reinterpret_cast<const char *>(&sizeOfRawScanRect), sizeof(int));

                if (sizeOfRawScanRect > 0)
                {
                    for (int i = 0; i < sizeOfRawScanRect; i++)
                    {
                        float x = kf->und_surfel->points()[i].x;
                        float y = kf->und_surfel->points()[i].y;
                        float z = kf->und_surfel->points()[i].z;
                        uint8_t intensity = (uint8_t)kf->und_surfel->points()[i].intensity;

                        fs.write(reinterpret_cast<const char *>(&x), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&y), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&z), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&intensity), sizeof(uint8_t));

                        if (mHeader.use_raw_time)
                        {
                            double pt_time = kf->und_surfel->times()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_time), sizeof(double));
                        }
                        if (mHeader.use_raw_ring)
                        {
                            uint16_t pt_ring = kf->und_surfel->rings()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_ring), sizeof(uint16_t));
                        }
                    }
                }

                // save corners
                int sizeOfCorners = 0;
                if (kf->ori_corner)
                    sizeOfCorners = kf->ori_corner->points().size();
                fs.write(reinterpret_cast<const char *>(&sizeOfCorners), sizeof(int));

                if (sizeOfCorners > 0)
                {
                    for (int i = 0; i < sizeOfCorners; i++)
                    {
                        float x = kf->ori_corner->points()[i].x;
                        float y = kf->ori_corner->points()[i].y;
                        float z = kf->ori_corner->points()[i].z;
                        uint8_t intensity = (uint8_t)kf->ori_corner->points()[i].intensity;

                        fs.write(reinterpret_cast<const char *>(&x), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&y), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&z), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&intensity), sizeof(uint8_t));

                        if (mHeader.use_corner_time)
                        {
                            double pt_time = kf->ori_corner->times()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_time), sizeof(double));
                        }
                        if (mHeader.use_corner_ring)
                        {
                            uint16_t pt_ring = kf->ori_corner->rings()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_ring), sizeof(uint16_t));
                        }
                    }
                }

                // save surfels
                int sizeOfSurfels = 0;
                sizeOfSurfels = kf->ori_surfel->points().size();
                fs.write(reinterpret_cast<const char *>(&sizeOfSurfels), sizeof(int));

                if (sizeOfSurfels > 0)
                {
                    for (int i = 0; i < sizeOfSurfels; i++)
                    {
                        float x = kf->ori_surfel->points()[i].x;
                        float y = kf->ori_surfel->points()[i].y;
                        float z = kf->ori_surfel->points()[i].z;
                        uint8_t intensity = (uint8_t)kf->ori_surfel->points()[i].intensity;

                        fs.write(reinterpret_cast<const char *>(&x), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&y), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&z), sizeof(float));
                        fs.write(reinterpret_cast<const char *>(&intensity), sizeof(uint8_t));

                        if (mHeader.use_surfel_time)
                        {
                            double pt_time = kf->ori_surfel->times()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_time), sizeof(double));
                        }
                        if (mHeader.use_surfel_ring)
                        {
                            uint16_t pt_ring = kf->ori_surfel->rings()[i];
                            fs.write(reinterpret_cast<const char *>(&pt_ring), sizeof(uint16_t));
                        }
                    }
                }

                // save gps
                int sizeOfGPSs = 0;
                fs.write(reinterpret_cast<const char *>(&sizeOfGPSs), sizeof(int));

                // save local frame pose
                if (mHeader.local_frame_count > 0)
                {
                    int sizeOfLocalFrame = kf->mFramePoses.size();
                    fs.write(reinterpret_cast<const char *>(&sizeOfLocalFrame), sizeof(int));

                    for (auto p : kf->mFramePoses)
                    {
                        // printf("pose time:%lf,%lf,%lf,%lf\n",p.timestamp,p.data_p(0),p.data_p(1),p.data_p(2));

                        fs.write(reinterpret_cast<const char *>(&p.timestamp), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_p(0)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_p(1)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_p(2)), sizeof(double));

                        fs.write(reinterpret_cast<const char *>(&p.data_q.w()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_q.x()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_q.y()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.data_q.z()), sizeof(double));
                    }
                }

                // save scan context
                if (mHeader.use_sc > 0)
                {
                    int sizeOfrow = kf->scan_context.rows();
                    int sizeOfcol = kf->scan_context.cols();

                    fs.write(reinterpret_cast<const char *>(&sizeOfrow), sizeof(int));
                    fs.write(reinterpret_cast<const char *>(&sizeOfcol), sizeof(int));

                    for (int i = 0; i < sizeOfrow; i++)
                    {
                        for (int j = 0; j < sizeOfcol; j++)
                        {
                            double val = kf->scan_context(i, j);
                            fs.write(reinterpret_cast<const char *>(&val), sizeof(double));
                        }
                    }
                }

                // save frame imu
                if (mHeader.use_imu > 0)
                {
                    int sizeOfIMU = kf->mIMU.size();
                    fs.write(reinterpret_cast<const char *>(&sizeOfIMU), sizeof(int));

                    for (auto p : kf->mIMU)
                    {
                        // printf("pose time:%lf,%lf,%lf,%lf\n",p.timestamp,p.data_p(0),p.data_p(1),p.data_p(2));

                        fs.write(reinterpret_cast<const char *>(&p.timestamp), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.acc(0)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.acc(1)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.acc(2)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.gyr(0)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.gyr(1)), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.gyr(2)), sizeof(double));

                        fs.write(reinterpret_cast<const char *>(&p.ori.w()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.ori.x()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.ori.y()), sizeof(double));
                        fs.write(reinterpret_cast<const char *>(&p.ori.z()), sizeof(double));
                    }
                }

                mHeader.num_of_frames++;

                if (kf->keyFrameID >= 0)
                {
                    mHeader.num_of_keyframes++;
                }

                mHeader.points_of_map += sizeOfRawScanRect;

                // printf("current frame :%d,%d,%d,%d\n", mHeader.num_of_frames, mHeader.num_of_keyframes, kf->frameID, kf->keyFrameID);
            }
        }

        virtual void saveSubmap(AliveSubmapPtr &submap)
        {
        }

        virtual void saveLoopInfo(AliveLoopPtr &loop)
        {
        }

        virtual void setBoundingBox(double &min_x,double &max_x,double &min_y,double &max_y,double &min_z,double &max_z)
        {
            mHeader.min_x = min_x;
            mHeader.max_x = max_x;
            mHeader.min_y = min_y;
            mHeader.max_y = max_y;
            mHeader.min_z = min_z;
            mHeader.max_z = max_z;
        }
        
        virtual bool openFile(std::string path)
        {
            fs.open(path, std::ios::out | std::ios::binary);

            memset((void *)&mHeader, 0, sizeof(AliveMapHeader));

            mHeader.file_signature[0] = 'g';
            mHeader.file_signature[1] = 'a';
            mHeader.file_signature[2] = 'm';
            mHeader.file_signature[3] = 'a';

            mHeader.num_of_frames = 0;
            mHeader.num_of_keyframes = 0;
            mHeader.frame_type = 1;
            mHeader.points_of_map = 0;
            mHeader.map_type = 1;
            mHeader.timestamp_base = 0;

            mHeader.num_of_submaps = 0;
            mHeader.num_of_loops = 0;

            // save header
            fs.write(reinterpret_cast<const char *>(&mHeader), sizeof(AliveMapHeader));

            return true;
        }

        virtual bool closeFile()
        {
            // resave header
            fs.seekp(0, std::ios::beg);
            fs.write(reinterpret_cast<const char *>(&mHeader), sizeof(AliveMapHeader));

            fs.close();

            return true;
        }

        std::fstream fs;

        AliveMapHeader mHeader;
    };

    typedef std::shared_ptr<AliveMapFileGammaSaver> AliveMapFileGammaSaverPtr;

} // namespace alive

#endif