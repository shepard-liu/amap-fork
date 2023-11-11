#ifndef PROJECT_GEODETIC_COMMON_H
#define PROJECT_GEODETIC_COMMON_H

//! Pi
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

namespace alive
{
    enum GPS_COV_TYPE
    {
        COVARIANCE_TYPE_UNKNOWN = 0u,
        COVARIANCE_TYPE_APPROXIMATED = 1u,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
        COVARIANCE_TYPE_KNOWN = 3u,
    };

    enum GPS_FIX_TYPE
    {
        STATUS_NO_FIX = -1,
        STATUS_FIX = 0,
        STATUS_SBAS_FIX = 1,
        STATUS_GBAS_FIX = 2
    };

    struct GPSReading
    {
        double timestamp;
        double lla[3];
        double covariance[3];
        GPS_COV_TYPE cov_type;
        GPS_FIX_TYPE fix_type;
        double heading;
        double headingCov;
    };

    struct GPSHeading
    {
        double timestamp;
        double angle;
        double std;
    };
} // namespace alive

#endif //PROJECT_GEODETIC_COMMON_H
