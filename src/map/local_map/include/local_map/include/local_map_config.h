#ifndef MAP_LOCAL_MAP_CONFIG_H
#define MAP_LOCAL_MAP_CONFIG_H

#include "config/config.h"

#define CONFIG_LOCAL_MAP    "cfg_local_map"

DEFINE_CONFIG(LocalMap)
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
    int window_size_x;
    int window_size_y;
    float max_inflate_range;
    float path_inflate_radius_offset;
    float local_map_polling_frequency;
    float local_map_updating_frequency;
    float local_map_detecting_frequency;
    float virtual_bump_detect_frequency;
    std::string virtual_bump_log_name;
    bool virtual_bump_log_print_to_console;
    int virtual_bump_log_level;
    bool enable_rear_virtual_bump_detect;
    bool enable_rotate_virtual_bump_detect;
    bool enable_backward_virtual_bump_detect;
    bool save_virtual_bump_detect_data;
    float virtual_bump_trig_dist;
    float ob_contour_detect_frequency;
    bool save_ob_contour_img;
    std::string ob_contour_log_name;
    bool ob_contour_log_print_to_console;
    int ob_contour_log_level;
    bool print_local_map;
    bool print_local_inflate_map;
    bool rviz_display;
    std::string foot_str;
    bool use_esdf_map;
END_CONFIG(LocalMap)

DEFINE_CONFIG_READ_FUNC(LocalMap)
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
    READ_CONFIG_MEMBER(window_size_x);
    READ_CONFIG_MEMBER(window_size_y);
    READ_CONFIG_MEMBER(max_inflate_range);
    READ_CONFIG_MEMBER(path_inflate_radius_offset);
    READ_CONFIG_MEMBER(local_map_polling_frequency);
    READ_CONFIG_MEMBER(local_map_updating_frequency);
    READ_CONFIG_MEMBER(local_map_detecting_frequency);
    READ_CONFIG_MEMBER(virtual_bump_detect_frequency);
    READ_CONFIG_MEMBER(virtual_bump_log_name);
    READ_CONFIG_MEMBER(virtual_bump_log_print_to_console);
    READ_CONFIG_MEMBER(virtual_bump_log_level);
    READ_CONFIG_MEMBER(enable_rear_virtual_bump_detect);
    READ_CONFIG_MEMBER(enable_rotate_virtual_bump_detect);
    READ_CONFIG_MEMBER(enable_backward_virtual_bump_detect);
    READ_CONFIG_MEMBER(save_virtual_bump_detect_data);
    READ_CONFIG_MEMBER(virtual_bump_trig_dist);
    READ_CONFIG_MEMBER(ob_contour_detect_frequency);
    READ_CONFIG_MEMBER(save_ob_contour_img);
    READ_CONFIG_MEMBER(ob_contour_log_name);
    READ_CONFIG_MEMBER(ob_contour_log_print_to_console);
    READ_CONFIG_MEMBER(ob_contour_log_level);
    READ_CONFIG_MEMBER(print_local_map);
    READ_CONFIG_MEMBER(print_local_inflate_map);
    READ_CONFIG_MEMBER(foot_str);
    READ_CONFIG_MEMBER(rviz_display);
    READ_CONFIG_MEMBER(use_esdf_map);
END_CONFIG_READ_FUNC(LocalMap)

#endif // MAP_LOCAL_MAP_CONFIG_H