#ifndef PROJECT_VOXEL_FILTER_H
#define PROJECT_VOXEL_FILTER_H

#include <limits>
#include <memory>
#include <algorithm>
#include <math.h>
#include "point_cloud.h"

namespace alive
{
    struct cloud_point_index_idx_xyz
    {
        unsigned int idx;
        unsigned int cloud_point_index;
        float center_x;
        float center_y;
        float center_z;

        cloud_point_index_idx_xyz(unsigned int idx_, unsigned int cloud_point_index_, float x_, float y_, float z_) : idx(idx_), cloud_point_index(cloud_point_index_), center_x(x_), center_y(y_), center_z(z_) {}
        bool operator<(const cloud_point_index_idx_xyz &p) const { return (idx < p.idx); }
    };

    struct Array4f
    {
        float x;
        float y;
        float z;
        float C;
    };

    class VoxelFilterParam
    {
    public:
        VoxelFilterParam()
        {
            voxel_size = 1.f;
        }

        float voxel_size;
    };

    class VoxelGridFilter
    {
    public:
        typedef std::shared_ptr<VoxelGridFilter> Ptr;

        inline static VoxelGridFilter::Ptr create(VoxelFilterParam param) { return VoxelGridFilter::Ptr(new VoxelGridFilter(param)); }

        VoxelGridFilter(VoxelFilterParam param)
        {
            mParam = param;
        }

    public:
        void getMaxMin(PointCloud::Ptr &cloud, Array4f &min_p, Array4f &max_p)
        {
            //找x,y,z最小值
            if (cloud->size() == 0)
            {
                std::cout << "input point cloud is empty" << std::endl;
                return;
            }
            float min_x = std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float min_z = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::min();
            float max_y = std::numeric_limits<float>::min();
            float max_z = std::numeric_limits<float>::min();

            for (int i = 0; i < cloud->size(); i++)
            {
                PointCloud::PointType p;
                p.x = cloud->points()[i].x;
                p.y = cloud->points()[i].y;
                p.z = cloud->points()[i].z;

                if (p.x >= 3.40282e+38 || p.x <= -3.40282e+38 || p.y >= 3.40282e+38 || p.y <= -3.40282e+38 || p.z >= 3.40282e+38 || p.z <= -3.40282e+38)
                    continue;

                // pc.emplace_back(p);
                if (p.x < min_x)
                    min_x = p.x;
                if (p.y < min_y)
                    min_y = p.y;
                if (p.z < min_z)
                    min_z = p.z;

                if (p.x > max_x)
                    max_x = p.x;
                if (p.y > max_y)
                    max_y = p.y;
                if (p.z > max_z)
                    max_z = p.z;
            }

            // printf("pcd extent: %f,%f,%f,%f,%f,%f\n", min_x, min_y, min_z, max_x, max_y, max_z);

            max_p.x = max_x;
            max_p.y = max_y;
            max_p.z = max_z;
            max_p.C = 1;

            min_p.x = min_x;
            min_p.y = min_y;
            min_p.z = min_z;
            min_p.C = 1;
            return;
        }
        void filterPointCloud(PointCloud::Ptr inputCloudPoint, PointCloud::Ptr &outPointCloud)
        {
            float voxel_size = mParam.voxel_size;

            //先判断输入的点云是否为空
            if (inputCloudPoint->size() == 0)
            {
                std::cout << "input point cloud is empty!" << std::endl;
                return;
            }

            //存放输入点云的最大与最小坐标
            Array4f min_p, max_p;
            getMaxMin(inputCloudPoint, min_p, max_p);

            Array4f inverse_leaf_size_;
            inverse_leaf_size_.x = 1 / voxel_size;
            inverse_leaf_size_.y = 1 / voxel_size;
            inverse_leaf_size_.z = 1 / voxel_size;
            inverse_leaf_size_.C = 1;

            //计算最小和最大边界框值
            Array4f min_b_, max_b_, div_b_, divb_mul_;
            min_b_.x = static_cast<int>(std::floor(min_p.x * inverse_leaf_size_.x));
            max_b_.x = static_cast<int>(std::floor(max_p.x * inverse_leaf_size_.x));
            min_b_.y = static_cast<int>(std::floor(min_p.y * inverse_leaf_size_.y));
            max_b_.y = static_cast<int>(std::floor(max_p.y * inverse_leaf_size_.y));
            min_b_.z = static_cast<int>(std::floor(min_p.z * inverse_leaf_size_.z));
            max_b_.z = static_cast<int>(std::floor(max_p.z * inverse_leaf_size_.z));

            //计算沿所有轴所需的分割数
            div_b_.x = max_b_.x - min_b_.x + 1;
            div_b_.y = max_b_.y - min_b_.y + 1;
            div_b_.z = max_b_.z - min_b_.z + 1;
            div_b_.C = 0;

            //设置除法乘数
            divb_mul_.x = 1;
            divb_mul_.y = div_b_.x;
            divb_mul_.z = div_b_.x * div_b_.y;
            divb_mul_.C = 0;

            //用于计算idx和pointcloud索引的存储
            std::vector<cloud_point_index_idx_xyz> index_vector;
            index_vector.reserve(inputCloudPoint->size());

            //第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
            for (int i = 0; i < inputCloudPoint->size(); i++)
            {
                unsigned int ijk0 = static_cast<unsigned int>(std::floor(inputCloudPoint->points()[i].x * inverse_leaf_size_.x) - static_cast<float>(min_b_.x));
                unsigned int ijk1 = static_cast<unsigned int>(std::floor(inputCloudPoint->points()[i].y * inverse_leaf_size_.y) - static_cast<float>(min_b_.y));
                unsigned int ijk2 = static_cast<unsigned int>(std::floor(inputCloudPoint->points()[i].z * inverse_leaf_size_.z) - static_cast<float>(min_b_.z));

                float center_x = static_cast<float>(min_p.x) + (ijk0 + 0.5) * voxel_size;
                float center_y = static_cast<float>(min_p.y) + (ijk1 + 0.5) * voxel_size;
                float center_z = static_cast<float>(min_p.z) + (ijk2 + 0.5) * voxel_size;

                //计算质心索引
                unsigned int idx = ijk0 * divb_mul_.x + ijk1 * divb_mul_.y + ijk2 * divb_mul_.z;

                // if (i % 100 == 0)
                // {
                //     printf("voxel info : %f,%f,%f,%d,%d,%d,%f,%f,%f\n", inputCloudPoint->points()[i].x, inputCloudPoint->points()[i].y, inputCloudPoint->points()[i].z,
                //            ijk0, ijk1, ijk2, center_x, center_y, center_z);
                // }
                index_vector.push_back(cloud_point_index_idx_xyz(static_cast<unsigned int>(idx), i, center_x, center_y, center_z));
            }

            //第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
            std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx_xyz>());

            //第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
            unsigned int total = 0;
            unsigned int index = 0;
            unsigned int min_points_per_voxel_ = 0;
            //first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
            std::vector<std::pair<unsigned int, unsigned int>> first_and_last_indices_vector;
            first_and_last_indices_vector.reserve(index_vector.size()); //分配内存空间

            while (index < index_vector.size())
            {
                unsigned int i = index + 1;
                while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                    ++i;
                if (i - index >= min_points_per_voxel_)
                {
                    ++total;
                    first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
                }
                index = i;
            }

            // printf("total: %d\n", total);
            //第四步：计算到voxel中心最邻近的距离的点作为返回点
            outPointCloud->resize(total); //给输出点云分配内存空间
            float x_Center, y_Center, z_Center;

            PointCloud::PointType point;
            double pt_tm;
            int pt_ring;

            int with_time = inputCloudPoint->times().size() > 0 ? 1 : 0;
            int with_ring = inputCloudPoint->rings().size() > 0 ? 1 : 0;

            unsigned int first_index, last_index;

            int out_index = 0;

            for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
            {
                // 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
                first_index = first_and_last_indices_vector[cp].first;
                last_index = first_and_last_indices_vector[cp].second;

                x_Center = index_vector[first_index].center_x;
                y_Center = index_vector[first_index].center_y;
                z_Center = index_vector[first_index].center_z;

                float min_dis = std::numeric_limits<float>::max();

                for (unsigned int li = first_index; li < last_index; ++li)
                {
                    PointCloud::PointType src_pt = (*inputCloudPoint)[index_vector[li].cloud_point_index];

                    float dis = (src_pt.x - x_Center) * (src_pt.x - x_Center) + (src_pt.y - y_Center) * (src_pt.y - y_Center) + (src_pt.z - z_Center) * (src_pt.z - z_Center);

                    if (dis < min_dis)
                    {
                        point.x = src_pt.x;
                        point.y = src_pt.y;
                        point.z = src_pt.z;
                        point.intensity = src_pt.intensity;

                        if (with_time)
                        {
                            pt_tm = inputCloudPoint->times()[index_vector[li].cloud_point_index];
                        }
                        if (with_ring)
                        {
                            pt_ring = inputCloudPoint->rings()[index_vector[li].cloud_point_index];
                        }

                        min_dis = dis;
                    }
                }
                (*outPointCloud)[out_index] = point;

                if (with_ring)
                {
                    outPointCloud->push_back_ring(pt_ring);
                }
                if (with_time)
                {
                    outPointCloud->push_back_time(pt_tm);
                }
                out_index++;
            }

            return;
        }

    private:
        VoxelFilterParam mParam;
    };

} // namespace alive

#endif