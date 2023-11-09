#ifndef PROJECT_AMAP_COMMON_H
#define PROJECT_AMAP_COMMON_H

namespace alive
{
    class AliveMapHeader
    {
    public:
        char file_signature[4];         // 4 bytes
        unsigned int num_of_frames;     // 4 bytes
        unsigned int frame_type;        // 4 bytes
        unsigned int points_of_map;     // 4 bytes
        unsigned int map_type;          // 4 bytes
        double timestamp_base;          // 8 bytes
        double enu_center_lat;          // 8 bytes
        double enu_center_lon;          // 8 bytes
        double enu_center_att;          // 8 bytes
        float pbg_x;                    // 4 bytes
        float pbg_y;                    // 4 bytes
        float pbg_z;                    // 4 bytes
        unsigned int local_frame_count; // 4 bytes
        unsigned int num_of_keyframes;  // 4 bytes
        unsigned int use_sc;            // 4 bytes
        unsigned int use_imu;           // 4 bytes
        unsigned int num_of_submaps;    // 4 bytes
        unsigned int num_of_loops;      // 4 bytes
        unsigned int use_raw_time;      // 4 bytes
        unsigned int use_raw_ring;      // 4 bytes
        unsigned int use_corner_time;      // 4 bytes
        unsigned int use_corner_ring;      // 4 bytes
        unsigned int use_surfel_time;      // 4 bytes
        unsigned int use_surfel_ring;      // 4 bytes
        double min_x;                      // 8 bytes
        double min_y;                      // 8 bytes
        double min_z;                      // 8 bytes
        double max_x;                      // 8 bytes
        double max_y;                      // 8 bytes
        double max_z;                      // 8 bytes
        char reserve[852];
    };
} // namespace alive

#endif