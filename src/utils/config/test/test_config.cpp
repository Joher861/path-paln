#include "config_manager.h"
#include <iostream>

using namespace planning_utils;

#define CONFIG_LOG_LEVEL	"cfg_log_level"
#define CONFIG_CPU	"cfg_cpu"

DEFINE_CONFIG(LogLevel)
	int log_level;
	int print_log;
	int save_navigation_log;
	int save_circle_log;
	int save_localmap_log;
	int save_chassis_speed_command_log;
	int save_path_log;
	int save_move_side_path_img;
	int save_clean_map;
	int save_localmap_img;
	float test_float;
	cv::Mat test_mat;
	std::string test_str;
	bool test_bool;
END_CONFIG(LogLevel)

DEFINE_CONFIG_READ_FUNC(LogLevel)
	READ_CONFIG_MEMBER(log_level)
	READ_CONFIG_MEMBER(print_log)
	READ_CONFIG_MEMBER(save_navigation_log)
	READ_CONFIG_MEMBER(save_circle_log)
	READ_CONFIG_MEMBER(save_localmap_log)
	READ_CONFIG_MEMBER(save_chassis_speed_command_log)
	READ_CONFIG_MEMBER(save_path_log)
	READ_CONFIG_MEMBER(save_move_side_path_img)
	READ_CONFIG_MEMBER(save_clean_map)
	READ_CONFIG_MEMBER(save_localmap_img)
	READ_CONFIG_MEMBER(test_float)
	READ_CONFIG_MEMBER(test_mat)
	READ_CONFIG_MEMBER(test_str)
	READ_CONFIG_MEMBER(test_bool)
END_CONFIG_READ_FUNC(LogLevel)

DEFINE_CONFIG_TYPE(CONFIG_LOG_LEVEL, LogLevel);

DEFINE_CONFIG(Cpu)
	int cpu_mask;       
    int all_cpu_mask;
END_CONFIG(Cpu)

DEFINE_CONFIG_READ_FUNC(Cpu)
	READ_CONFIG_MEMBER(cpu_mask)
	READ_CONFIG_MEMBER(all_cpu_mask)
END_CONFIG_READ_FUNC(Cpu)

DEFINE_CONFIG_TYPE(CONFIG_CPU, Cpu);

int main(int argc, char *argv[])
{
	ConfigManager cfg_mgr;
	
	cfg_mgr.LoadConfig("../test.yaml");
	ConfigLogLevel *cfg_log_level
		= dynamic_cast<ConfigLogLevel*>(cfg_mgr.GetSubConfig("cfg_log_level"));
	
	if (cfg_log_level) {
		std::cout << cfg_log_level->test_float << " " << cfg_log_level->save_localmap_img << std::endl;
		std::cout << cfg_log_level->test_mat << std::endl;
		std::cout << cfg_log_level->test_str << std::endl;
		std::cout << cfg_log_level->test_bool << std::endl;
	}
	
	const ConfigCpu *cfg_cpu
		= dynamic_cast<const ConfigCpu*>(cfg_mgr.GetSubConfig("cfg_cpu"));
	if (cfg_cpu) {
		std::cout << cfg_cpu->cpu_mask << " " << cfg_cpu->all_cpu_mask << std::endl;
	}
	
	const ConfigCpu *empty
		= dynamic_cast<const ConfigCpu*>(cfg_mgr.GetSubConfig("cfg_cpu2"));
	if (!empty) {
		std::cout << "cfg_cpu2 not exist" << std::endl;
	}
	
	return 0;
}



