#ifndef MAP_CLEAN_MAP_CONFIG_H
#define MAP_CLEAN_MAP_CONFIG_H

#include "config/config.h"

#define CONFIG_CLEAN_MAP    "cfg_clean_map"

DEFINE_CONFIG(CleanMap)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float resolution;               // 地图分辨率，单位 m
    int map_size_x;                 // 地图x方向栅格尺寸
    int map_size_y;                 // 地图y方向栅格尺寸
    float origin_pt_x;              // 地图原点x坐标
    float origin_pt_y;              // 地图原点y坐标
    float max_inflate_range;
    float smaller_inflate_radius_offset;

    float local_max_inflate_range;
    float local_smaller_inflate_radius_offset;
    int local_map_polling_frequency;
    int local_map_updating_frequency;
    int local_map_detecting_frequency;
    int print_local_map;
    int print_local_inflate_map;
END_CONFIG(CleanMap)

DEFINE_CONFIG_READ_FUNC(CleanMap)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(resolution);
    READ_CONFIG_MEMBER(map_size_x);
    READ_CONFIG_MEMBER(map_size_y);
    READ_CONFIG_MEMBER(origin_pt_x);
    READ_CONFIG_MEMBER(origin_pt_y);
    READ_CONFIG_MEMBER(max_inflate_range);
    READ_CONFIG_MEMBER(smaller_inflate_radius_offset);

    READ_CONFIG_MEMBER(local_max_inflate_range);
    READ_CONFIG_MEMBER(local_smaller_inflate_radius_offset);
    READ_CONFIG_MEMBER(local_map_polling_frequency);
    READ_CONFIG_MEMBER(local_map_updating_frequency);
    READ_CONFIG_MEMBER(local_map_detecting_frequency);
    READ_CONFIG_MEMBER(print_local_map);
    READ_CONFIG_MEMBER(print_local_inflate_map);
END_CONFIG_READ_FUNC(CleanMap)

#endif // MAP_CLEAN_MAP_CONFIG_H