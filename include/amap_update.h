#ifndef PROJECT_AMAP_UPDATE_H
#define PROJECT_AMAP_UPDATE_H

#include <iostream>
#include <fstream>
#include "geodetic_conv.h"

#include "amap_common.h"
#include "amap_io.h"
#include "amap_io_alpha.h"
#include "amap_io_beta.h"
#include "amap_io_gamma.h"
#include "amap_factory.h"

namespace alive
{
    class AliveMapFileUpdater
    {
    public:
        typedef std::shared_ptr<AliveMapFileUpdater> Ptr;

        static AliveMapFileUpdater::Ptr create() { return AliveMapFileUpdater::Ptr(new AliveMapFileUpdater()); }

    public:
        bool updateAmx(std::string mappath)
        {
            AliveMapHeader header;
            std::fstream fs_tmp;
            fs_tmp.open(mappath.data(), std::ios::in | std::ios::binary);

            if (!fs_tmp.is_open())
            {
                std::cout << "cannot open map file" << std::endl;
                return false;
            }

            // read header first
            fs_tmp.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            // alpha-version map
            if (header.file_signature[0] == 'a')
            {
                std::fstream fs_index;

                std::string index_path = mappath.substr(0, mappath.find_last_of(".")) + ".amx";
                fs_index.open(index_path, std::ios::out | std::ios::binary);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                std::vector<long long> fm_poses;
                std::vector<long long> sm_poses;
                std::vector<long long> loop_poses;

                // read amap
                // get frames file address from amap
                for (unsigned int i = 0; i < header.num_of_frames; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    fm_poses.push_back(nSize);
                    {
                        // ID
                        unsigned int frameID = 0;
                        int keyFrameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&frameID, sizeof(unsigned int));
                        fs_tmp.read((char *)&keyFrameID, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(uint8_t));

                            if (header.use_raw_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_raw_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_tmp.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_corner_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_corner_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_tmp.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_surfel_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_surfel_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // GlobalData
                        int sizeOfGPS = 0;
                        fs_tmp.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_tmp.read((char *)(&timestamp), sizeof(double));
                            fs_tmp.read((char *)(&lla(0)), sizeof(double));
                            fs_tmp.read((char *)(&lla(1)), sizeof(double));
                            fs_tmp.read((char *)(&lla(2)), sizeof(double));

                            fs_tmp.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_tmp.read((char *)(&cov_type), sizeof(char));
                            fs_tmp.read((char *)(&fix_type), sizeof(char));
                            fs_tmp.read((char *)(&heading), sizeof(double));
                            fs_tmp.read((char *)(&headingCov), sizeof(double));
                        }

                        // local frame pose
                        if (header.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_tmp.read((char *)&sizeOfLocalFrame, sizeof(int));

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Vector3d vel(0, 0, 0);

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&p(0)), sizeof(double));
                                fs_tmp.read((char *)(&p(1)), sizeof(double));
                                fs_tmp.read((char *)(&p(2)), sizeof(double));

                                fs_tmp.read((char *)(&q.w()), sizeof(double));
                                fs_tmp.read((char *)(&q.x()), sizeof(double));
                                fs_tmp.read((char *)(&q.y()), sizeof(double));
                                fs_tmp.read((char *)(&q.z()), sizeof(double));

                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));

                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));

                                fs_tmp.read((char *)(&vel(0)), sizeof(double));
                                fs_tmp.read((char *)(&vel(1)), sizeof(double));
                                fs_tmp.read((char *)(&vel(2)), sizeof(double));
                            }
                        }

                        // scan context
                        if (header.use_sc > 0)
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_tmp.read((char *)&sizeOfrow, sizeof(int));
                            fs_tmp.read((char *)&sizeOfcol, sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_tmp.read((char *)(&val), sizeof(double));
                                }
                            }
                        }

                        // local imu
                        if (header.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_tmp.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));
                                fs_tmp.read((char *)(&ori.w()), sizeof(double));
                                fs_tmp.read((char *)(&ori.x()), sizeof(double));
                                fs_tmp.read((char *)(&ori.y()), sizeof(double));
                                fs_tmp.read((char *)(&ori.z()), sizeof(double));
                            }
                        }
                    }
                }

                // get submaps file address
                for (unsigned int i = 0; i < header.num_of_submaps; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    sm_poses.push_back(nSize);

                    {
                        // ID
                        int mapID = 0;
                        int anchorFrameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&mapID, sizeof(int));
                        fs_tmp.read((char *)&anchorFrameID, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // frame ids
                        int sizeOfFrameIDs = 0;
                        fs_tmp.read((char *)&sizeOfFrameIDs, sizeof(int));

                        for (int k = 0; k < sizeOfFrameIDs; k++)
                        {
                            int frame_id = 0;
                            fs_tmp.read((char *)(&frame_id), sizeof(int));
                        }

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(uint8_t));
                        }

                        // map context
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_tmp.read((char *)&sizeOfrow, sizeof(int));
                            fs_tmp.read((char *)&sizeOfcol, sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_tmp.read((char *)(&val), sizeof(double));
                                }
                            }
                        }
                    }
                }

                // get loop info address
                for (unsigned int i = 0; i < header.num_of_loops; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    loop_poses.push_back(nSize);

                    {
                        // ID
                        int current_id = 0;
                        int match_id = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&current_id, sizeof(int));
                        fs_tmp.read((char *)&match_id, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));
                    }
                }

                fs_index.seekp(0, std::ios::beg);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                for (auto &nSize : fm_poses)
                {
                    // printf("amx frame pose: %ld\n", nSize);
                    fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                }

                for (auto &nSize : sm_poses)
                {
                    // printf("amx submap pose: %ld\n", nSize);
                    fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                }

                for (auto &nSize : loop_poses)
                {
                    // printf("amx loop pose: %ld\n", nSize);
                    fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                }

                fs_index.close();
            }
            // beta-version map
            else if (header.file_signature[0] == 'b')
            {
                std::fstream fs_index;

                std::string index_path = mappath.substr(0, mappath.find_last_of(".")) + ".amx";
                fs_index.open(index_path, std::ios::out | std::ios::binary);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                std::vector<long long> fm_poses;

                // read amap
                // get frames file address
                for (unsigned int i = 0; i < header.num_of_frames; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    fm_poses.push_back(nSize);

                    {
                        // ID
                        unsigned int frameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&frameID, sizeof(unsigned int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(float));
                            fs_tmp.read((char *)&tm, sizeof(double));
                            fs_tmp.read((char *)&ring, sizeof(uint16_t));
                        }

                        int sizeOfCornerRect = 0;
                        fs_tmp.read((char *)&sizeOfCornerRect, sizeof(int));
                        for (int k = 0; k < sizeOfCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(float));
                            fs_tmp.read((char *)&tm, sizeof(double));
                            fs_tmp.read((char *)&ring, sizeof(uint16_t));
                        }

                        int sizeOfSurfelRect = 0;
                        fs_tmp.read((char *)&sizeOfSurfelRect, sizeof(int));

                        for (int k = 0; k < sizeOfSurfelRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(float));
                            fs_tmp.read((char *)&tm, sizeof(double));
                            fs_tmp.read((char *)&ring, sizeof(uint16_t));
                        }

                        // GlobalData
                        int sizeOfGPS = 0;
                        fs_tmp.read((char *)&sizeOfGPS, sizeof(int));

                        // printf("gps count: %d\n",sizeOfGPS);

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

                            fs_tmp.read((char *)(&timestamp), sizeof(double));
                            fs_tmp.read((char *)(&lla(0)), sizeof(double));
                            fs_tmp.read((char *)(&lla(1)), sizeof(double));
                            fs_tmp.read((char *)(&lla(2)), sizeof(double));

                            fs_tmp.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_tmp.read((char *)(&cov_type), sizeof(char));
                            fs_tmp.read((char *)(&fix_type), sizeof(char));
                            fs_tmp.read((char *)(&heading), sizeof(double));
                            fs_tmp.read((char *)(&headingCov), sizeof(double));
                        }

                        // local frame pose
                        if (header.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_tmp.read((char *)&sizeOfLocalFrame, sizeof(int));

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&p(0)), sizeof(double));
                                fs_tmp.read((char *)(&p(1)), sizeof(double));
                                fs_tmp.read((char *)(&p(2)), sizeof(double));

                                fs_tmp.read((char *)(&q.w()), sizeof(double));
                                fs_tmp.read((char *)(&q.x()), sizeof(double));
                                fs_tmp.read((char *)(&q.y()), sizeof(double));
                                fs_tmp.read((char *)(&q.z()), sizeof(double));
                            }
                        }

                        // local imu
                        if (header.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_tmp.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));
                                fs_tmp.read((char *)(&ori.w()), sizeof(double));
                                fs_tmp.read((char *)(&ori.x()), sizeof(double));
                                fs_tmp.read((char *)(&ori.y()), sizeof(double));
                                fs_tmp.read((char *)(&ori.z()), sizeof(double));
                            }
                        }

                        // printf("load map frame info:%d,%d,%d,%d\n",sizeOfScanCornerRect,sizeOfScanSurfRect,sizeOfGPS,sizeOfIMU);
                    }
                }

                fs_index.seekp(0, std::ios::beg);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                for (auto &nSize : fm_poses)
                {
                    // printf("amx frame pose: %ld\n", nSize);
                    fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                }

                fs_index.close();
            }
            // gamma-version map
            else if (header.file_signature[0] == 'g')
            {
                std::fstream fs_index;

                std::string index_path = mappath.substr(0, mappath.find_last_of(".")) + ".amx";
                fs_index.open(index_path, std::ios::out | std::ios::binary);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                std::vector<long long> fm_poses;

                // read amap
                // get frames file address
                for (unsigned int i = 0; i < header.num_of_frames; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    fm_poses.push_back(nSize);

                    {
                        // ID
                        unsigned int frameID = 0;
                        int keyFrameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&frameID, sizeof(unsigned int));
                        fs_tmp.read((char *)&keyFrameID, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(uint8_t));

                            if (header.use_raw_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_raw_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_tmp.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_corner_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_corner_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_tmp.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_surfel_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_surfel_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // GlobalData
                        int sizeOfGPS = 0;
                        fs_tmp.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_tmp.read((char *)(&timestamp), sizeof(double));
                            fs_tmp.read((char *)(&lla(0)), sizeof(double));
                            fs_tmp.read((char *)(&lla(1)), sizeof(double));
                            fs_tmp.read((char *)(&lla(2)), sizeof(double));

                            fs_tmp.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_tmp.read((char *)(&cov_type), sizeof(char));
                            fs_tmp.read((char *)(&fix_type), sizeof(char));
                            fs_tmp.read((char *)(&heading), sizeof(double));
                            fs_tmp.read((char *)(&headingCov), sizeof(double));
                        }

                        // local frame pose
                        if (header.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_tmp.read((char *)&sizeOfLocalFrame, sizeof(int));

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&p(0)), sizeof(double));
                                fs_tmp.read((char *)(&p(1)), sizeof(double));
                                fs_tmp.read((char *)(&p(2)), sizeof(double));

                                fs_tmp.read((char *)(&q.w()), sizeof(double));
                                fs_tmp.read((char *)(&q.x()), sizeof(double));
                                fs_tmp.read((char *)(&q.y()), sizeof(double));
                                fs_tmp.read((char *)(&q.z()), sizeof(double));
                            }
                        }

                        // scan context
                        if (header.use_sc > 0)
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_tmp.read((char *)&sizeOfrow, sizeof(int));
                            fs_tmp.read((char *)&sizeOfcol, sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_tmp.read((char *)(&val), sizeof(double));
                                }
                            }
                        }

                        // local imu
                        if (header.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_tmp.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));
                                fs_tmp.read((char *)(&ori.w()), sizeof(double));
                                fs_tmp.read((char *)(&ori.x()), sizeof(double));
                                fs_tmp.read((char *)(&ori.y()), sizeof(double));
                                fs_tmp.read((char *)(&ori.z()), sizeof(double));
                            }
                        }
                    }
                }

                fs_index.seekp(0, std::ios::beg);
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                for (auto &nSize : fm_poses)
                {
                    // printf("amx frame pose: %ld\n", nSize);
                    fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                }

                fs_index.close();
            }

            fs_tmp.close();

            return true;
        }

        // Notice: update all submap, partial submap update is NOT support
        bool updateSubmapToMapFile(std::vector<AliveSubmapPtr> &all_submap, std::string mappath)
        {
            AliveMapHeader header;
            std::fstream fs_tmp;
            fs_tmp.open(mappath.data(), std::ios::in | std::ios::binary);

            if (!fs_tmp.is_open())
            {
                std::cout << "cannot open map file" << std::endl;
                return false;
            }

            // read header first
            fs_tmp.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            // alpha-version map
            if (header.file_signature[0] == 'a')
            {
                std::vector<long long> fm_poses;
                std::vector<long long> sm_poses;
                std::vector<long long> loop_poses;
                long long sm_start_pose;

                // read amap
                // get frames file address from amap
                for (unsigned int i = 0; i < header.num_of_frames; i++)
                {
                    long long nSize = fs_tmp.tellg();
                    fm_poses.push_back(nSize);
                    {
                        // ID
                        unsigned int frameID = 0;
                        int keyFrameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&frameID, sizeof(unsigned int));
                        fs_tmp.read((char *)&keyFrameID, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(uint8_t));

                            if (header.use_raw_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_raw_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_tmp.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_corner_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_corner_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_tmp.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)(&x), sizeof(float));
                            fs_tmp.read((char *)(&y), sizeof(float));
                            fs_tmp.read((char *)(&z), sizeof(float));
                            fs_tmp.read((char *)(&intensity), sizeof(uint8_t));

                            if (header.use_surfel_time)
                            {
                                double pt_tm = 0;
                                fs_tmp.read((char *)&pt_tm, sizeof(double));
                            }
                            if (header.use_surfel_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_tmp.read((char *)&pt_ring, sizeof(uint16_t));
                            }
                        }

                        // GlobalData
                        int sizeOfGPS = 0;
                        fs_tmp.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_tmp.read((char *)(&timestamp), sizeof(double));
                            fs_tmp.read((char *)(&lla(0)), sizeof(double));
                            fs_tmp.read((char *)(&lla(1)), sizeof(double));
                            fs_tmp.read((char *)(&lla(2)), sizeof(double));

                            fs_tmp.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_tmp.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_tmp.read((char *)(&cov_type), sizeof(char));
                            fs_tmp.read((char *)(&fix_type), sizeof(char));
                            fs_tmp.read((char *)(&heading), sizeof(double));
                            fs_tmp.read((char *)(&headingCov), sizeof(double));
                        }

                        // local frame pose
                        if (header.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_tmp.read((char *)&sizeOfLocalFrame, sizeof(int));

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Vector3d vel(0, 0, 0);

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&p(0)), sizeof(double));
                                fs_tmp.read((char *)(&p(1)), sizeof(double));
                                fs_tmp.read((char *)(&p(2)), sizeof(double));

                                fs_tmp.read((char *)(&q.w()), sizeof(double));
                                fs_tmp.read((char *)(&q.x()), sizeof(double));
                                fs_tmp.read((char *)(&q.y()), sizeof(double));
                                fs_tmp.read((char *)(&q.z()), sizeof(double));

                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));

                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));

                                fs_tmp.read((char *)(&vel(0)), sizeof(double));
                                fs_tmp.read((char *)(&vel(1)), sizeof(double));
                                fs_tmp.read((char *)(&vel(2)), sizeof(double));
                            }
                        }

                        // scan context
                        if (header.use_sc > 0)
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_tmp.read((char *)&sizeOfrow, sizeof(int));
                            fs_tmp.read((char *)&sizeOfcol, sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_tmp.read((char *)(&val), sizeof(double));
                                }
                            }
                        }

                        // local imu
                        if (header.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_tmp.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_tmp.read((char *)(&timestamp), sizeof(double));
                                fs_tmp.read((char *)(&acc(0)), sizeof(double));
                                fs_tmp.read((char *)(&acc(1)), sizeof(double));
                                fs_tmp.read((char *)(&acc(2)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(0)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(1)), sizeof(double));
                                fs_tmp.read((char *)(&gyr(2)), sizeof(double));
                                fs_tmp.read((char *)(&ori.w()), sizeof(double));
                                fs_tmp.read((char *)(&ori.x()), sizeof(double));
                                fs_tmp.read((char *)(&ori.y()), sizeof(double));
                                fs_tmp.read((char *)(&ori.z()), sizeof(double));
                            }
                        }
                    }
                }

                sm_start_pose = fs_tmp.tellg();
                // get submaps file address
                for (unsigned int i = 0; i < header.num_of_submaps; i++)
                {
                    // long long nSize = fs_tmp.tellg();
                    // sm_poses.push_back(nSize);

                    {
                        // ID
                        int mapID = 0;
                        int anchorFrameID = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&mapID, sizeof(int));
                        fs_tmp.read((char *)&anchorFrameID, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        // frame ids
                        int sizeOfFrameIDs = 0;
                        fs_tmp.read((char *)&sizeOfFrameIDs, sizeof(int));

                        for (int k = 0; k < sizeOfFrameIDs; k++)
                        {
                            int frame_id = 0;
                            fs_tmp.read((char *)(&frame_id), sizeof(int));
                        }

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_tmp.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_tmp.read((char *)&x, sizeof(float));
                            fs_tmp.read((char *)&y, sizeof(float));
                            fs_tmp.read((char *)&z, sizeof(float));
                            fs_tmp.read((char *)&intensity, sizeof(uint8_t));
                        }

                        // map context
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_tmp.read((char *)&sizeOfrow, sizeof(int));
                            fs_tmp.read((char *)&sizeOfcol, sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_tmp.read((char *)(&val), sizeof(double));
                                }
                            }
                        }
                    }
                }

                std::vector<AliveLoopAlphaPtr> all_loops;

                // get loop info address
                for (unsigned int i = 0; i < header.num_of_loops; i++)
                {
                    // long long nSize = fs_tmp.tellg();
                    // loop_poses.push_back(nSize);

                    AliveLoopAlphaPtr loop(new AliveLoopInfoAlpha);
                    {
                        // ID
                        int current_id = 0;
                        int match_id = 0;
                        double tm = 0;

                        fs_tmp.read((char *)&current_id, sizeof(int));
                        fs_tmp.read((char *)&match_id, sizeof(int));
                        fs_tmp.read((char *)&tm, sizeof(double));

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_tmp.read((char *)(&tx), sizeof(double));
                        fs_tmp.read((char *)(&ty), sizeof(double));
                        fs_tmp.read((char *)(&tz), sizeof(double));
                        fs_tmp.read((char *)(&qx), sizeof(double));
                        fs_tmp.read((char *)(&qy), sizeof(double));
                        fs_tmp.read((char *)(&qz), sizeof(double));
                        fs_tmp.read((char *)(&qw), sizeof(double));

                        loop->P = Eigen::Vector3d(tx, ty, tz);
                        loop->Q.x() = qx;
                        loop->Q.y() = qy;
                        loop->Q.z() = qz;
                        loop->Q.w() = qw;
                        loop->Q.normalize();

                        all_loops.push_back(loop);
                    }
                }

                fs_tmp.close();

                // update to amap file
                {
                    std::fstream fs_update;
                    fs_update.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

                    fs_update.seekp(sm_start_pose, std::ios::beg);
                    header.num_of_submaps = 0;
                    header.num_of_loops = 0;

                    for (auto &submap : all_submap)
                    {
                        long long nSize = fs_update.tellg();

                        AliveSubmapAlphaPtr map;

                        map = std::dynamic_pointer_cast<AliveSubmapAlpha>(submap);

                        // save frame id & timestamp
                        fs_update.write(reinterpret_cast<const char *>(&(map->id)), sizeof(int));
                        fs_update.write(reinterpret_cast<const char *>(&(map->anchorFrameID)), sizeof(int));
                        fs_update.write(reinterpret_cast<const char *>(&(map->tm)), sizeof(double));

                        // save pose
                        double tx = map->P[0];
                        double ty = map->P[1];
                        double tz = map->P[2];
                        double qx = map->Q.x();
                        double qy = map->Q.y();
                        double qz = map->Q.z();
                        double qw = map->Q.w();
                        fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                        // save frame ids
                        int sizeOfFrameIDs = map->frameIDs.size();
                        fs_update.write(reinterpret_cast<const char *>(&sizeOfFrameIDs), sizeof(int));

                        if (sizeOfFrameIDs > 0)
                        {
                            for (auto p : map->frameIDs)
                            {
                                fs_update.write(reinterpret_cast<const char *>(&p), sizeof(int));
                            }
                        }

                        // save raw scan
                        int sizeOfRawScanRect = 0;
                        if (map->pcd)
                            sizeOfRawScanRect = map->pcd->points().size();
                        fs_update.write(reinterpret_cast<const char *>(&sizeOfRawScanRect), sizeof(int));

                        if (sizeOfRawScanRect > 0)
                        {
                            for (int i = 0; i < sizeOfRawScanRect; i++)
                            {
                                float x = map->pcd->points()[i].x;
                                float y = map->pcd->points()[i].y;
                                float z = map->pcd->points()[i].z;
                                uint8_t intensity = (uint8_t)map->pcd->points()[i].intensity;

                                fs_update.write(reinterpret_cast<const char *>(&x), sizeof(float));
                                fs_update.write(reinterpret_cast<const char *>(&y), sizeof(float));
                                fs_update.write(reinterpret_cast<const char *>(&z), sizeof(float));
                                fs_update.write(reinterpret_cast<const char *>(&intensity), sizeof(uint8_t));
                            }
                        }

                        {
                            int sizeOfrow = map->map_context.rows();
                            int sizeOfcol = map->map_context.cols();

                            fs_update.write(reinterpret_cast<const char *>(&sizeOfrow), sizeof(int));
                            fs_update.write(reinterpret_cast<const char *>(&sizeOfcol), sizeof(int));

                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = map->map_context(i, j);
                                    fs_update.write(reinterpret_cast<const char *>(&val), sizeof(double));
                                }
                            }
                        }

                        header.num_of_submaps++;
                        sm_poses.push_back(nSize);

                        // printf("current submap :%d,%d,%d\n", mHeader.num_of_frames, kf->frameID, kf->keyFrameID);
                    }

                    // save loop info
                    for (auto &loop : all_loops)
                    {
                        long long nSize = fs_update.tellg();

                        AliveLoopAlphaPtr info;
                        if (loop)
                        {
                            info = std::dynamic_pointer_cast<AliveLoopInfoAlpha>(loop);
                        }

                        if (info)
                        {
                            // save frame id & timestamp
                            fs_update.write(reinterpret_cast<const char *>(&(info->current_id)), sizeof(int));
                            fs_update.write(reinterpret_cast<const char *>(&(info->matched_id)), sizeof(int));
                            fs_update.write(reinterpret_cast<const char *>(&(info->current_time)), sizeof(double));

                            // save pose
                            double tx = info->P[0];
                            double ty = info->P[1];
                            double tz = info->P[2];
                            double qx = info->Q.x();
                            double qy = info->Q.y();
                            double qz = info->Q.z();
                            double qw = info->Q.w();
                            fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                            fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                            header.num_of_loops++;
                            loop_poses.push_back(nSize);
                            // printf("current loop :%d,%d,%d\n", mHeader.num_of_loops, info->current_id, info->matched_id);
                        }
                    }

                    // resave header
                    fs_update.seekp(0, std::ios::beg);
                    fs_update.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                    fs_update.close();

                    fs_update.close();
                }

                // update to index file
                {

                    std::fstream fs_index;

                    std::string index_path = mappath.substr(0, mappath.find_last_of(".")) + ".amx";
                    fs_index.open(index_path, std::ios::out | std::ios::binary);
                    fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                    fs_index.seekp(0, std::ios::beg);
                    fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                    for (auto &nSize : fm_poses)
                    {
                        // printf("amx frame pose: %ld\n", nSize);
                        fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                    }

                    for (auto &nSize : sm_poses)
                    {
                        // printf("amx submap pose: %ld\n", nSize);
                        fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                    }

                    for (auto &nSize : loop_poses)
                    {
                        // printf("amx loop pose: %ld\n", nSize);
                        fs_index.write(reinterpret_cast<const char *>(&(nSize)), sizeof(long long));
                    }

                    fs_index.close();
                }
            }

            return true;
        }

        bool updatePoseToMapFile(std::vector<AmapPoseData> &pose, std::string mappath)
        {
            AliveMapHeader header;
            std::fstream fs_tmp;
            fs_tmp.open(mappath.data(), std::ios::in | std::ios::binary);

            if (!fs_tmp.is_open())
            {
                std::cout << "cannot open map file" << std::endl;
                return false;
            }

            // read header first
            fs_tmp.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            fs_tmp.close();

            std::unordered_map<double, int> pose_time_idx_map;

            int ptim_index = 0;
            for (auto &p : pose)
            {
                pose_time_idx_map[p.timestamp] = ptim_index;
                ptim_index++;
            }

            // alpha-version map
            if (header.file_signature[0] == 'a')
            {
                std::fstream fs_update;

                AliveMapHeader mHeader;

                fs_update.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

                // read header first
                fs_update.read(reinterpret_cast<char *>(&mHeader), sizeof(AliveMapHeader));

                // get frames file address
                for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
                {
                    long long nFramePQBeginPose = 0;
                    AliveMapAlphaFramePtr frame(new AliveMapAlphaFrame);
                    {
                        // ID
                        unsigned int frameID = 0;
                        unsigned int keyFrameID = 0;
                        double tm = 0;

                        fs_update.read((char *)&frameID, sizeof(unsigned int));
                        fs_update.read((char *)&keyFrameID, sizeof(int));
                        fs_update.read((char *)&tm, sizeof(double));

                        frame->frameID = frameID;
                        frame->keyFrameID = keyFrameID;
                        frame->tm = tm;

                        nFramePQBeginPose = fs_update.tellg();

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_update.read((char *)(&tx), sizeof(double));
                        fs_update.read((char *)(&ty), sizeof(double));
                        fs_update.read((char *)(&tz), sizeof(double));
                        fs_update.read((char *)(&qx), sizeof(double));
                        fs_update.read((char *)(&qy), sizeof(double));
                        fs_update.read((char *)(&qz), sizeof(double));
                        fs_update.read((char *)(&qw), sizeof(double));

                        frame->P = Eigen::Vector3d(tx, ty, tz);
                        frame->Q.x() = qx;
                        frame->Q.y() = qy;
                        frame->Q.z() = qz;
                        frame->Q.w() = qw;
                        frame->Q.normalize();

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_update.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;

                            if (mHeader.use_raw_time)
                            {
                                double pt_tm = 0;
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->pcd->push_back_time(pt_tm);
                            }
                            if (mHeader.use_raw_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->pcd->push_back_ring(pt_ring);
                            }

                            frame->pcd->push_back(pt);
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_update.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->corners->push_back(pt);

                            if (mHeader.use_corner_time)
                            {
                                double pt_tm = 0;
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->corners->push_back_time(pt_tm);
                            }
                            if (mHeader.use_corner_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->corners->push_back_ring(pt_ring);
                            }
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_update.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->surfels->push_back(pt);

                            if (mHeader.use_surfel_time)
                            {
                                double pt_tm = 0;
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->surfels->push_back_time(pt_tm);
                            }
                            if (mHeader.use_surfel_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->surfels->push_back_ring(pt_ring);
                            }
                        }

                        // GPSReadings
                        int sizeOfGPS = 0;
                        fs_update.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_update.read((char *)(&timestamp), sizeof(double));
                            fs_update.read((char *)(&lla(0)), sizeof(double));
                            fs_update.read((char *)(&lla(1)), sizeof(double));
                            fs_update.read((char *)(&lla(2)), sizeof(double));

                            fs_update.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_update.read((char *)(&cov_type), sizeof(char));
                            fs_update.read((char *)(&fix_type), sizeof(char));
                            fs_update.read((char *)(&heading), sizeof(double));
                            fs_update.read((char *)(&headingCov), sizeof(double));

                            // printf("gps:%lf,%lf,%lf\n",lla(0),lla(1),lla(2));

                            GlobalData gps;
                            gps.timestamp = timestamp;
                            gps.lla = lla;
                            gps.covariance = covariance;
                            gps.cov_type = cov_type;
                            gps.fix_type = fix_type;
                            gps.heading = heading;
                            gps.headingCov = headingCov;

                            frame->mGPS.push_back(gps);
                        }

                        // frame poses
                        if (mHeader.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_update.read((char *)&sizeOfLocalFrame, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Vector3d vel(0, 0, 0);

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&p(0)), sizeof(double));
                                fs_update.read((char *)(&p(1)), sizeof(double));
                                fs_update.read((char *)(&p(2)), sizeof(double));

                                fs_update.read((char *)(&q.w()), sizeof(double));
                                fs_update.read((char *)(&q.x()), sizeof(double));
                                fs_update.read((char *)(&q.y()), sizeof(double));
                                fs_update.read((char *)(&q.z()), sizeof(double));

                                fs_update.read((char *)(&acc(0)), sizeof(double));
                                fs_update.read((char *)(&acc(1)), sizeof(double));
                                fs_update.read((char *)(&acc(2)), sizeof(double));

                                fs_update.read((char *)(&gyr(0)), sizeof(double));
                                fs_update.read((char *)(&gyr(1)), sizeof(double));
                                fs_update.read((char *)(&gyr(2)), sizeof(double));

                                fs_update.read((char *)(&vel(0)), sizeof(double));
                                fs_update.read((char *)(&vel(1)), sizeof(double));
                                fs_update.read((char *)(&vel(2)), sizeof(double));

                                AmapPoseData apd;
                                apd.timestamp = timestamp;
                                apd.data_p = p;
                                apd.data_q = q;
                                apd.data_acc = acc;
                                apd.data_gyr = gyr;
                                apd.data_vel = vel;

                                frame->mFramePoses.push_back(apd);
                                // printf("pose time:%lf\n",apd.timestamp);
                            }
                        }

                        // scan context
                        if (mHeader.use_sc > 0)
                        {
                            int sizeOfrow = 0;
                            int sizeOfcol = 0;
                            fs_update.read((char *)&sizeOfrow, sizeof(int));
                            fs_update.read((char *)&sizeOfcol, sizeof(int));

                            frame->scan_context.resize(sizeOfrow, sizeOfcol);
                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_update.read((char *)(&val), sizeof(double));

                                    frame->scan_context(i, j) = val;
                                }
                            }
                        }

                        // local imu
                        if (mHeader.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_update.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&acc(0)), sizeof(double));
                                fs_update.read((char *)(&acc(1)), sizeof(double));
                                fs_update.read((char *)(&acc(2)), sizeof(double));
                                fs_update.read((char *)(&gyr(0)), sizeof(double));
                                fs_update.read((char *)(&gyr(1)), sizeof(double));
                                fs_update.read((char *)(&gyr(2)), sizeof(double));
                                fs_update.read((char *)(&ori.w()), sizeof(double));
                                fs_update.read((char *)(&ori.x()), sizeof(double));
                                fs_update.read((char *)(&ori.y()), sizeof(double));
                                fs_update.read((char *)(&ori.z()), sizeof(double));

                                InertialData imu;
                                imu.timestamp = timestamp;
                                imu.acc = acc;
                                imu.gyr = gyr;
                                imu.ori = ori;

                                frame->mIMU.push_back(imu);
                            }
                        }
                    }

                    long long nFrameEndPose = fs_update.tellg();

                    // update frame pose
                    auto it = pose_time_idx_map.find(frame->tm);

                    if (it != pose_time_idx_map.end())
                    {
                        AmapPoseData apd = pose[it->second];

                        frame->P = apd.data_p;
                        frame->Q = apd.data_q;

                        fs_update.seekg(nFramePQBeginPose, std::ios::beg);

                        // save pose
                        double tx = frame->P[0];
                        double ty = frame->P[1];
                        double tz = frame->P[2];
                        double qx = frame->Q.x();
                        double qy = frame->Q.y();
                        double qz = frame->Q.z();
                        double qw = frame->Q.w();
                        fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                        fs_update.seekg(nFrameEndPose, std::ios::beg);
                    }

                    float percent = (float)i / (float)(mHeader.num_of_frames);
                    std::string msg = "process percent";

                    runCallBack(percent, msg);
                }

                fs_update.close();
            }
            // beta-version map
            else if (header.file_signature[0] == 'b')
            {
                std::fstream fs_update;

                AliveMapHeader mHeader;

                fs_update.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

                // read header first
                fs_update.read(reinterpret_cast<char *>(&mHeader), sizeof(AliveMapHeader));

                // get frames file address
                for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
                {
                    long long nFramePQBeginPose = 0;
                    AliveMapBetaFramePtr frame(new AliveMapBetaFrame);
                    {
                        // ID
                        unsigned int frameID = 0;
                        unsigned int keyFrameID = 0;
                        double tm = 0;

                        fs_update.read((char *)&frameID, sizeof(unsigned int));
                        fs_update.read((char *)&tm, sizeof(double));

                        frame->frameID = frameID;
                        frame->tm = tm;

                        nFramePQBeginPose = fs_update.tellg();

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_update.read((char *)(&tx), sizeof(double));
                        fs_update.read((char *)(&ty), sizeof(double));
                        fs_update.read((char *)(&tz), sizeof(double));
                        fs_update.read((char *)(&qx), sizeof(double));
                        fs_update.read((char *)(&qy), sizeof(double));
                        fs_update.read((char *)(&qz), sizeof(double));
                        fs_update.read((char *)(&qw), sizeof(double));

                        frame->P = Eigen::Vector3d(tx, ty, tz);
                        frame->Q.x() = qx;
                        frame->Q.y() = qy;
                        frame->Q.z() = qz;
                        frame->Q.w() = qw;
                        frame->Q.normalize();

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_update.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(float));
                            fs_update.read((char *)&tm, sizeof(double));
                            fs_update.read((char *)&ring, sizeof(uint16_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->ori_surfel->push_back(pt);
                            frame->ori_surfel->push_back_time(tm);
                            frame->ori_surfel->push_back_ring(ring);
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_update.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(float));
                            fs_update.read((char *)&tm, sizeof(double));
                            fs_update.read((char *)&ring, sizeof(uint16_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->ori_corners->push_back(pt);
                            frame->ori_corners->push_back_time(tm);
                            frame->ori_corners->push_back_ring(ring);
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_update.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            float intensity = 0;
                            double tm = 0;
                            uint16_t ring = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(float));
                            fs_update.read((char *)&tm, sizeof(double));
                            fs_update.read((char *)&ring, sizeof(uint16_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->undistort_surfel->push_back(pt);
                            frame->undistort_surfel->push_back_time(tm);
                            frame->undistort_surfel->push_back_ring(ring);
                        }

                        // GPSReadings
                        int sizeOfGPS = 0;
                        fs_update.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_update.read((char *)(&timestamp), sizeof(double));
                            fs_update.read((char *)(&lla(0)), sizeof(double));
                            fs_update.read((char *)(&lla(1)), sizeof(double));
                            fs_update.read((char *)(&lla(2)), sizeof(double));

                            fs_update.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_update.read((char *)(&cov_type), sizeof(char));
                            fs_update.read((char *)(&fix_type), sizeof(char));
                            fs_update.read((char *)(&heading), sizeof(double));
                            fs_update.read((char *)(&headingCov), sizeof(double));

                            // printf("gps:%lf,%lf,%lf\n",lla(0),lla(1),lla(2));

                            GlobalData gps;
                            gps.timestamp = timestamp;
                            gps.lla = lla;
                            gps.covariance = covariance;
                            gps.cov_type = cov_type;
                            gps.fix_type = fix_type;
                            gps.heading = heading;
                            gps.headingCov = headingCov;

                            frame->mGPS.push_back(gps);
                        }

                        // frame poses
                        if (mHeader.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_update.read((char *)&sizeOfLocalFrame, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&p(0)), sizeof(double));
                                fs_update.read((char *)(&p(1)), sizeof(double));
                                fs_update.read((char *)(&p(2)), sizeof(double));

                                fs_update.read((char *)(&q.w()), sizeof(double));
                                fs_update.read((char *)(&q.x()), sizeof(double));
                                fs_update.read((char *)(&q.y()), sizeof(double));
                                fs_update.read((char *)(&q.z()), sizeof(double));

                                AmapPoseData apd;
                                apd.timestamp = timestamp;
                                apd.data_p = p;
                                apd.data_q = q;

                                frame->mFramePoses.push_back(apd);
                                // printf("pose time:%lf\n",apd.timestamp);
                            }
                        }

                        // local imu
                        if (mHeader.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_update.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&acc(0)), sizeof(double));
                                fs_update.read((char *)(&acc(1)), sizeof(double));
                                fs_update.read((char *)(&acc(2)), sizeof(double));
                                fs_update.read((char *)(&gyr(0)), sizeof(double));
                                fs_update.read((char *)(&gyr(1)), sizeof(double));
                                fs_update.read((char *)(&gyr(2)), sizeof(double));
                                fs_update.read((char *)(&ori.w()), sizeof(double));
                                fs_update.read((char *)(&ori.x()), sizeof(double));
                                fs_update.read((char *)(&ori.y()), sizeof(double));
                                fs_update.read((char *)(&ori.z()), sizeof(double));

                                InertialData imu;
                                imu.timestamp = timestamp;
                                imu.acc = acc;
                                imu.gyr = gyr;
                                imu.ori = ori;

                                frame->mIMU.push_back(imu);
                            }
                        }
                    }

                    long long nFrameEndPose = fs_update.tellg();

                    // update frame pose
                    auto it = pose_time_idx_map.find(frame->tm);

                    if (it != pose_time_idx_map.end())
                    {
                        AmapPoseData apd = pose[it->second];

                        frame->P = apd.data_p;
                        frame->Q = apd.data_q;

                        fs_update.seekg(nFramePQBeginPose, std::ios::beg);

                        // save pose
                        double tx = frame->P[0];
                        double ty = frame->P[1];
                        double tz = frame->P[2];
                        double qx = frame->Q.x();
                        double qy = frame->Q.y();
                        double qz = frame->Q.z();
                        double qw = frame->Q.w();
                        fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                        fs_update.seekg(nFrameEndPose, std::ios::beg);
                    }

                    float percent = (float)i / (float)(mHeader.num_of_frames);
                    std::string msg = "process percent";

                    runCallBack(percent, msg);
                }

                fs_update.close();
            }
            // gamma-version map
            else if (header.file_signature[0] == 'g')
            {
                std::fstream fs_update;

                AliveMapHeader mHeader;

                fs_update.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

                // read header first
                fs_update.read(reinterpret_cast<char *>(&mHeader), sizeof(AliveMapHeader));

                // get frames file address
                for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
                {
                    long long nFramePQBeginPose = 0;
                    AliveMapGammaFramePtr frame(new AliveMapGammaFrame);
                    {
                        // ID
                        unsigned int frameID = 0;
                        unsigned int keyFrameID = 0;
                        double tm = 0;

                        fs_update.read((char *)&frameID, sizeof(unsigned int));
                        fs_update.read((char *)&keyFrameID, sizeof(int));
                        fs_update.read((char *)&tm, sizeof(double));

                        frame->frameID = frameID;
                        frame->keyFrameID = keyFrameID;
                        frame->tm = tm;

                        nFramePQBeginPose = fs_update.tellg();

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_update.read((char *)(&tx), sizeof(double));
                        fs_update.read((char *)(&ty), sizeof(double));
                        fs_update.read((char *)(&tz), sizeof(double));
                        fs_update.read((char *)(&qx), sizeof(double));
                        fs_update.read((char *)(&qy), sizeof(double));
                        fs_update.read((char *)(&qz), sizeof(double));
                        fs_update.read((char *)(&qw), sizeof(double));

                        frame->P = Eigen::Vector3d(tx, ty, tz);
                        frame->Q.x() = qx;
                        frame->Q.y() = qy;
                        frame->Q.z() = qz;
                        frame->Q.w() = qw;
                        frame->Q.normalize();

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_update.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(uint8_t));

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
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->und_surfel->push_back_time(pt_tm);
                            }
                            if (mHeader.use_raw_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->und_surfel->push_back_ring(pt_ring);
                            }
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_update.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->ori_corner->push_back(pt);

                            if (mHeader.use_corner_time)
                            {
                                double pt_tm = 0;
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->ori_corner->push_back_time(pt_tm);
                            }
                            if (mHeader.use_corner_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->ori_corner->push_back_ring(pt_ring);
                            }
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_update.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->ori_surfel->push_back(pt);

                            if (mHeader.use_surfel_time)
                            {
                                double pt_tm = 0;
                                fs_update.read((char *)&pt_tm, sizeof(double));
                                frame->ori_surfel->push_back_time(pt_tm);
                            }
                            if (mHeader.use_surfel_ring)
                            {
                                uint16_t pt_ring = 0;
                                fs_update.read((char *)&pt_ring, sizeof(uint16_t));
                                frame->ori_surfel->push_back_ring(pt_ring);
                            }
                        }

                        // GPSReadings
                        int sizeOfGPS = 0;
                        fs_update.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_update.read((char *)(&timestamp), sizeof(double));
                            fs_update.read((char *)(&lla(0)), sizeof(double));
                            fs_update.read((char *)(&lla(1)), sizeof(double));
                            fs_update.read((char *)(&lla(2)), sizeof(double));

                            fs_update.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_update.read((char *)(&cov_type), sizeof(char));
                            fs_update.read((char *)(&fix_type), sizeof(char));
                            fs_update.read((char *)(&heading), sizeof(double));
                            fs_update.read((char *)(&headingCov), sizeof(double));

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
                            fs_update.read((char *)&sizeOfLocalFrame, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&p(0)), sizeof(double));
                                fs_update.read((char *)(&p(1)), sizeof(double));
                                fs_update.read((char *)(&p(2)), sizeof(double));

                                fs_update.read((char *)(&q.w()), sizeof(double));
                                fs_update.read((char *)(&q.x()), sizeof(double));
                                fs_update.read((char *)(&q.y()), sizeof(double));
                                fs_update.read((char *)(&q.z()), sizeof(double));

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
                            fs_update.read((char *)&sizeOfrow, sizeof(int));
                            fs_update.read((char *)&sizeOfcol, sizeof(int));

                            frame->scan_context.resize(sizeOfrow, sizeOfcol);
                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_update.read((char *)(&val), sizeof(double));

                                    frame->scan_context(i, j) = val;
                                }
                            }
                        }

                        // local imu
                        if (mHeader.use_imu > 0)
                        {
                            int sizeOfIMU = 0;
                            fs_update.read((char *)&sizeOfIMU, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfIMU; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d acc(0, 0, 0);
                                Eigen::Vector3d gyr(0, 0, 0);
                                Eigen::Quaterniond ori;
                                ori.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&acc(0)), sizeof(double));
                                fs_update.read((char *)(&acc(1)), sizeof(double));
                                fs_update.read((char *)(&acc(2)), sizeof(double));
                                fs_update.read((char *)(&gyr(0)), sizeof(double));
                                fs_update.read((char *)(&gyr(1)), sizeof(double));
                                fs_update.read((char *)(&gyr(2)), sizeof(double));
                                fs_update.read((char *)(&ori.w()), sizeof(double));
                                fs_update.read((char *)(&ori.x()), sizeof(double));
                                fs_update.read((char *)(&ori.y()), sizeof(double));
                                fs_update.read((char *)(&ori.z()), sizeof(double));

                                InertialData imu;
                                imu.timestamp = timestamp;
                                imu.acc = acc;
                                imu.gyr = gyr;
                                imu.ori = ori;

                                frame->mIMU.push_back(imu);
                            }
                        }
                    }

                    long long nFrameEndPose = fs_update.tellg();

                    // update frame pose
                    auto it = pose_time_idx_map.find(frame->tm);

                    if (it != pose_time_idx_map.end())
                    {
                        AmapPoseData apd = pose[it->second];

                        frame->P = apd.data_p;
                        frame->Q = apd.data_q;

                        fs_update.seekg(nFramePQBeginPose, std::ios::beg);

                        // save pose
                        double tx = frame->P[0];
                        double ty = frame->P[1];
                        double tz = frame->P[2];
                        double qx = frame->Q.x();
                        double qy = frame->Q.y();
                        double qz = frame->Q.z();
                        double qw = frame->Q.w();
                        fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                        fs_update.seekg(nFrameEndPose, std::ios::beg);
                    }

                    float percent = (float)i / (float)(mHeader.num_of_frames);
                    std::string msg = "process percent";

                    runCallBack(percent, msg);
                }

                fs_update.close();
            }
            else
            {
                std::cerr << "no alive map type is not supported" << std::endl;
                // std::abort();
                return false;
            }

            std::string msg = "process done";
            runCallBack(1.0, msg);
            return true;
        }

        bool updatePoseToMapFile(AmapPoseData &pose, std::string mappath)
        {
            AliveMapHeader header;
            std::fstream fs_tmp;
            fs_tmp.open(mappath.data(), std::ios::in | std::ios::binary);

            if (!fs_tmp.is_open())
            {
                std::cout << "cannot open map file" << std::endl;
                return false;
            }

            // read header first
            fs_tmp.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            fs_tmp.close();

            // alpha-version map
            if (header.file_signature[0] == 'a')
            {
                std::fstream fs_update;

                AliveMapHeader mHeader;

                fs_update.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

                // read header first
                fs_update.read(reinterpret_cast<char *>(&mHeader), sizeof(AliveMapHeader));

                // get frames file address
                for (unsigned int i = 0; i < mHeader.num_of_frames; i++)
                {
                    long long nFramePQBeginPose = 0;
                    // long long nFrameBeginPose = fs_update.tellg();
                    AliveMapAlphaFramePtr frame(new AliveMapAlphaFrame);
                    {
                        // ID
                        unsigned int frameID = 0;
                        unsigned int keyFrameID = 0;
                        double tm = 0;

                        fs_update.read((char *)&frameID, sizeof(unsigned int));
                        fs_update.read((char *)&keyFrameID, sizeof(unsigned int));
                        fs_update.read((char *)&tm, sizeof(double));

                        frame->frameID = frameID;
                        frame->keyFrameID = keyFrameID;
                        frame->tm = tm;

                        nFramePQBeginPose = fs_update.tellg();

                        // T_W_B
                        double tx = 0;
                        double ty = 0;
                        double tz = 0;
                        double qx = 0;
                        double qy = 0;
                        double qz = 0;
                        double qw = 1;
                        fs_update.read((char *)(&tx), sizeof(double));
                        fs_update.read((char *)(&ty), sizeof(double));
                        fs_update.read((char *)(&tz), sizeof(double));
                        fs_update.read((char *)(&qx), sizeof(double));
                        fs_update.read((char *)(&qy), sizeof(double));
                        fs_update.read((char *)(&qz), sizeof(double));
                        fs_update.read((char *)(&qw), sizeof(double));

                        Eigen::Vector3d bef_p = Eigen::Vector3d(tx, ty, tz);
                        Eigen::Quaterniond bef_q = Eigen::Quaterniond(qw, qx, qy, qz);

                        Eigen::Quaterniond aft_q = pose.data_q * bef_q;
                        Eigen::Vector3d aft_p = pose.data_q * bef_p + pose.data_p;

                        frame->P = aft_p;
                        frame->Q = aft_q;
                        frame->Q.normalize();

                        // laserScanRect
                        int sizeOfRawScanRect = 0;
                        fs_update.read((char *)&sizeOfRawScanRect, sizeof(int));

                        for (int k = 0; k < sizeOfRawScanRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)&x, sizeof(float));
                            fs_update.read((char *)&y, sizeof(float));
                            fs_update.read((char *)&z, sizeof(float));
                            fs_update.read((char *)&intensity, sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->pcd->push_back(pt);
                        }

                        // corners
                        int sizeOfScanCornerRect = 0;
                        fs_update.read((char *)&sizeOfScanCornerRect, sizeof(int));

                        for (int k = 0; k < sizeOfScanCornerRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->corners->push_back(pt);
                        }

                        // surfels
                        int sizeOfScanSurfRect = 0;
                        fs_update.read((char *)(&sizeOfScanSurfRect), sizeof(int));

                        for (int k = 0; k < sizeOfScanSurfRect; k++)
                        {
                            float x = 0;
                            float y = 0;
                            float z = 0;
                            uint8_t intensity = 0;
                            fs_update.read((char *)(&x), sizeof(float));
                            fs_update.read((char *)(&y), sizeof(float));
                            fs_update.read((char *)(&z), sizeof(float));
                            fs_update.read((char *)(&intensity), sizeof(uint8_t));

                            PointCloud::PointType pt;
                            pt.x = x;
                            pt.y = y;
                            pt.z = z;
                            pt.intensity = intensity;
                            frame->surfels->push_back(pt);
                        }

                        // GPSReadings
                        int sizeOfGPS = 0;
                        fs_update.read((char *)&sizeOfGPS, sizeof(int));

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

                            fs_update.read((char *)(&timestamp), sizeof(double));
                            fs_update.read((char *)(&lla(0)), sizeof(double));
                            fs_update.read((char *)(&lla(1)), sizeof(double));
                            fs_update.read((char *)(&lla(2)), sizeof(double));

                            fs_update.read((char *)(&covariance(0, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(0, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(1, 2)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 0)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 1)), sizeof(double));
                            fs_update.read((char *)(&covariance(2, 2)), sizeof(double));

                            fs_update.read((char *)(&cov_type), sizeof(char));
                            fs_update.read((char *)(&fix_type), sizeof(char));
                            fs_update.read((char *)(&heading), sizeof(double));
                            fs_update.read((char *)(&headingCov), sizeof(double));

                            // printf("gps:%lf,%lf,%lf\n",lla(0),lla(1),lla(2));

                            GlobalData gps;
                            gps.timestamp = timestamp;
                            gps.lla = lla;
                            gps.covariance = covariance;
                            gps.cov_type = cov_type;
                            gps.fix_type = fix_type;
                            gps.heading = heading;
                            gps.headingCov = headingCov;

                            frame->mGPS.push_back(gps);
                        }

                        // frame poses
                        if (mHeader.local_frame_count > 0)
                        {
                            int sizeOfLocalFrame = 0;
                            fs_update.read((char *)&sizeOfLocalFrame, sizeof(int));

                            // printf("sizeOfLocalFrame:%ld\n",sizeOfLocalFrame);

                            for (int k = 0; k < sizeOfLocalFrame; k++)
                            {
                                double timestamp = 0;
                                Eigen::Vector3d p(0, 0, 0);
                                Eigen::Quaterniond q;
                                q.setIdentity();

                                fs_update.read((char *)(&timestamp), sizeof(double));
                                fs_update.read((char *)(&p(0)), sizeof(double));
                                fs_update.read((char *)(&p(1)), sizeof(double));
                                fs_update.read((char *)(&p(2)), sizeof(double));

                                fs_update.read((char *)(&q.w()), sizeof(double));
                                fs_update.read((char *)(&q.x()), sizeof(double));
                                fs_update.read((char *)(&q.y()), sizeof(double));
                                fs_update.read((char *)(&q.z()), sizeof(double));

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
                            fs_update.read((char *)&sizeOfrow, sizeof(int));
                            fs_update.read((char *)&sizeOfcol, sizeof(int));

                            frame->scan_context.resize(sizeOfrow, sizeOfcol);
                            for (int i = 0; i < sizeOfrow; i++)
                            {
                                for (int j = 0; j < sizeOfcol; j++)
                                {
                                    double val = 0;
                                    fs_update.read((char *)(&val), sizeof(double));

                                    frame->scan_context(i, j) = val;
                                }
                            }
                        }
                    }

                    long long nFrameEndPose = fs_update.tellg();

                    {
                        fs_update.seekg(nFramePQBeginPose, std::ios::beg);

                        // save pose
                        double tx = frame->P[0];
                        double ty = frame->P[1];
                        double tz = frame->P[2];
                        double qx = frame->Q.x();
                        double qy = frame->Q.y();
                        double qz = frame->Q.z();
                        double qw = frame->Q.w();
                        fs_update.write(reinterpret_cast<const char *>(&tx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&ty), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&tz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qx), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qy), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qz), sizeof(double));
                        fs_update.write(reinterpret_cast<const char *>(&qw), sizeof(double));

                        fs_update.seekg(nFrameEndPose, std::ios::beg);
                    }

                    float percent = (float)i / (float)(mHeader.num_of_frames);
                    std::string msg = "process percent";

                    runCallBack(percent, msg);
                }

                fs_update.close();
            }
            // beta-version map
            else if (header.file_signature[0] == 'b')
            {
                // todo
            }
            else
            {
                std::cerr << "no alive map type is not supported" << std::endl;
                // std::abort();
                return false;
            }

            std::string msg = "process done";
            runCallBack(1.0, msg);
            return true;
        }

        bool updateBoundingBoxToMapFile(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, std::string mappath)
        {
            AliveMapHeader header;
            std::fstream fs_tmp;
            fs_tmp.open(mappath.data(), std::ios::in | std::ios::out | std::ios::binary);

            if (!fs_tmp.is_open())
            {
                std::cout << "cannot open map file" << std::endl;
                return false;
            }

            // read header first
            fs_tmp.read(reinterpret_cast<char *>(&header), sizeof(AliveMapHeader));

            header.min_x = min_x;
            header.min_y = min_y;
            header.min_z = min_z;

            header.max_x = max_x;
            header.max_y = max_y;
            header.max_z = max_z;

            fs_tmp.seekg(0, std::ios::beg);

            // resave header
            fs_tmp.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

            fs_tmp.close();

            std::fstream fs_index;
            std::string index_path = mappath.substr(0, mappath.find_last_of(".")) + ".amx";
            fs_index.open(index_path.data(), std::ios::in | std::ios::binary);

            if (fs_index.is_open())
            {
                fs_index.seekg(0, std::ios::beg);

                // resave header
                fs_index.write(reinterpret_cast<const char *>(&header), sizeof(AliveMapHeader));

                fs_index.close();
            }

            return true;
        }

        void regProcessCallback(const std::function<void(float, std::string &)> &callback)
        {
            mProcessCbVec.emplace_back(callback);
        }

    protected:
        void runCallBack(float percent, std::string &msg)
        {
            for (auto &it : mProcessCbVec)
            {
                it(percent, msg);
            }
        }

        std::vector<std::function<void(float, std::string &)>> mProcessCbVec;
    };

} // namespace alive

#endif
