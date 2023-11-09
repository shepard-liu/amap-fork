#ifndef PROJECT_POINT_CLOUD_H
#define PROJECT_POINT_CLOUD_H

#include <memory>
#include <vector>
#include <iostream>

namespace alive
{
    struct NormalXYZ
    {
        NormalXYZ()
        {
            x = 0;
            y = 0;
            z = 0;
        }
        float x;
        float y;
        float z;
    };

    struct PointXYZI
    {
        PointXYZI()
        {
            x = 0;
            y = 0;
            z = 0;
            intensity = 0;
        }
        float x;
        float y;
        float z;
        float intensity;
    };

    struct PointXYZINormal
    {
        PointXYZINormal()
        {
            x = 0;
            y = 0;
            z = 0;
            intensity = 0;
            normal_x = 0;
            normal_y = 0;
            normal_z = 0;
        }
        double x;
        double y;
        double z;
        float intensity;
        double normal_x;
        double normal_y;
        double normal_z;
    };

    // Stores 3D positions of points together with some additional data, e.g.
    // intensities times ring.
    class PointCloud
    {
    public:
        using PointType = PointXYZI;

        typedef std::shared_ptr<PointCloud> Ptr;

        static PointCloud::Ptr create() { return PointCloud::Ptr(new PointCloud()); }
        static PointCloud::Ptr create(std::vector<PointType> points) { return PointCloud::Ptr(new PointCloud(points)); }
        static PointCloud::Ptr create(std::vector<PointType> points, std::vector<double> times) { return PointCloud::Ptr(new PointCloud(points, times)); }

        PointCloud()
        {
        }

        PointCloud(std::vector<PointType> points)
        {
            points_.insert(points_.end(), points.begin(), points.end());
        }

        PointCloud(std::vector<PointType> points, std::vector<double> times)
        {
            if (!times_.empty())
            {
                if (points_.size() != times_.size())
                {
                    std::cerr << "times size is not equal to points size! " << std::endl;
                    std::abort();
                }
            }

            points_.insert(points_.end(), points.begin(), points.end());
            times_.insert(times_.end(), times.begin(), times.end());
        }

        PointCloud(std::vector<PointType> points, std::vector<double> times, std::vector<uint16_t> rings)
        {
            if (!times_.empty())
            {
                if (points_.size() != times_.size())
                {
                    std::cerr << "point time size is not equal to points size! " << std::endl;
                    std::abort();
                }
            }

            if (!rings_.empty())
            {
                if (points_.size() != rings_.size())
                {
                    std::cerr << "point ring size is not equal to points size! " << std::endl;
                    std::abort();
                }
            }

            points_.insert(points_.end(), points.begin(), points.end());
            times_.insert(times_.end(), times.begin(), times.end());
            rings_.insert(rings_.end(), rings.begin(), rings.end());
        }

        // Returns the number of points in the point cloud.
        size_t size() const
        {
            return points_.size();
        }

        // Checks whether there are any points in the point cloud.
        bool empty() const
        {
            return points_.empty();
        }

        const std::vector<PointType> &points() const
        {
            return points_;
        }

        const std::vector<float> &ambients() const
        {
            return ambients_;
        }

        const std::vector<double> &times() const
        {
            return times_;
        }
        const std::vector<uint16_t> &rings() const
        {
            return rings_;
        }
        const std::vector<NormalXYZ> &normals() const
        {
            return normals_;
        }

        PointType &operator[](const size_t index)
        {
            return points_[index];
        }

        // Iterator over the points in the point cloud.
        // using ConstIterator = std::vector<PointType>::const_iterator;
        // ConstIterator begin() const
        // {
        //     return points_.begin();
        // }
        // ConstIterator end() const
        // {
        //     return points_.end();
        // }
        using Iterator = std::vector<PointType>::iterator;
        Iterator begin()
        {
            return points_.begin();
        }
        Iterator end()
        {
            return points_.end();
        }

        void push_back(PointType value)
        {
            points_.push_back(std::move(value));
        }
        void push_back_point_normal(PointXYZINormal value)
        {
            PointType value1;
            value1.x = value.x;
            value1.y = value.y;
            value1.z = value.z;
            value1.intensity = value.intensity;
            NormalXYZ value2;
            value2.x = value.normal_x;
            value2.y = value.normal_y;
            value2.z = value.normal_z;
            points_.push_back(std::move(value1));
            normals_.push_back(std::move(value2));
        }
        void push_back_ambient(float ambient)
        {
            ambients_.push_back(std::move(ambient));
        }
        void push_back_time(double time)
        {
            times_.push_back(std::move(time));
        }
        void push_back_ring(uint16_t ring)
        {
            rings_.push_back(std::move(ring));
        }
        void push_back_normal(NormalXYZ normal)
        {
            normals_.push_back(std::move(normal));
        }
        void push_back(PointCloud::Ptr pcd)
        {
            points_.insert(points_.end(), pcd->points().begin(), pcd->points().end());
            ambients_.insert(ambients_.end(), pcd->ambients().begin(), pcd->ambients().end());
            times_.insert(times_.end(), pcd->times().begin(), pcd->times().end());
            rings_.insert(rings_.end(), pcd->rings().begin(), pcd->rings().end());
            normals_.insert(normals_.end(), pcd->normals().begin(), pcd->normals().end());
        }

        void copyPointCloud(PointCloud::Ptr &pc_in)
        {
            points_.clear();
            points_.insert(points_.end(), pc_in->points_.begin(), pc_in->points_.end());
        }
        void copyPointCloud(const PointCloud::Ptr &pc_in)
        {
            points_.clear();
            points_.insert(points_.end(), pc_in->points_.begin(), pc_in->points_.end());
        }

        void copyPointCloud(PointCloud::Ptr &pc1_in, PointCloud::Ptr &pc2_in)
        {
            points_.clear();
            points_.insert(points_.end(), pc1_in->points_.begin(), pc1_in->points_.end());
            points_.insert(points_.end(), pc2_in->points_.begin(), pc2_in->points_.end());
        }

        void addPointCloud(PointCloud::Ptr &pc_in)
        {
            points_.insert(points_.end(), pc_in->points_.begin(), pc_in->points_.end());
        }

        void resize(unsigned int num)
        {
            points_.resize(num);
        }
        // void erase(ConstIterator first, ConstIterator last)
        // {
        //     points_.erase(first, last);
        // }
        void erase(Iterator first, Iterator last)
        {
            points_.erase(first, last);
        }

        void clear()
        {
            points_.clear();
            ambients_.clear();
            times_.clear();
            rings_.clear();
            normals_.clear();
        }

        // Creates a PointCloud consisting of all the points for which `predicate`
        // returns true, together with the corresponding intensities.
        template <class UnaryPredicate>
        PointCloud copy_if(UnaryPredicate predicate) const
        {
            std::vector<PointType> points;

            // Note: benchmarks show that it is better to have this conditional outside
            // the loop.
            {
                for (size_t index = 0; index < size(); ++index)
                {
                    const PointType &point = points_[index];
                    if (predicate(point))
                    {
                        points.push_back(point);
                    }
                }
            }

            return PointCloud(points);
        }

        /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param[in] pc the cloud to copy into this
        */
        PointCloud(PointCloud &pc)
        {
            *this = pc;
        }

        /** \brief Copy constructor (needed by compilers such as Intel C++)
        * \param[in] pc the cloud to copy into this
        */
        PointCloud(const PointCloud &pc)
        {
            *this = pc;
        }

        /** \brief Copy the cloud to the heap and return a smart pointer
        * Note that deep copy is performed, so avoid using this function on non-empty clouds.
        * The changes of the returned cloud are not mirrored back to this one.
        * \return shared pointer to the copy of the cloud
        */
        inline Ptr
        make_shared() const { return Ptr(new PointCloud(*this)); }

    private:
        // For 2D points, the third entry is 0.f.
        std::vector<PointType> points_;
        // Ambients are optional. If non-empty, they must have the same size as points.
        std::vector<float> ambients_;
        // times are optional. If non-empty, they must have the same size as points.
        std::vector<double> times_;
        // rings are optional. If non-empty, they must have the same size as points.
        std::vector<uint16_t> rings_;
        // normals are optional. If non-empty, they must have the same size as points.
        std::vector<NormalXYZ> normals_;
    };
} // namespace alive

#endif