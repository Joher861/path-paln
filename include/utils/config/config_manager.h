#ifndef CONFIG_MANAGER_H_
#define CONFIG_MANAGER_H_

#include <unordered_map>
#include <string>

#include "config.h"

namespace planning_utils
{
	using ConfigMapType = std::unordered_map<std::string, Config*>;

	class ConfigManager {
	public:
		ConfigManager();
		~ConfigManager();
		bool LoadConfig(std::string file_path);
		Config* GetSubConfig(std::string name);
		
	private:
		std::string file_path_;
		ConfigMapType cfg_map_;
	};
}

#endif


