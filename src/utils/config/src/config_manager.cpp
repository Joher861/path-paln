#include "config_manager.h"

#include <iostream>

using namespace planning_utils;

ConfigManager::ConfigManager()
{
}

ConfigManager::~ConfigManager()
{
	Config * ptr = nullptr;
	
	for(auto it = cfg_map_.begin(); it != cfg_map_.end(); it++) {
		ptr = it->second;
		delete ptr;
	}
}

bool ConfigManager::LoadConfig(std::string file_path)
{
	cv::FileStorage fs(file_path.c_str(), cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cout << "PMS Failed to open yaml file: " << file_path << std::endl;
		return false;
	}
	
	for (auto info : *ConfigType::info_list_)
	{
		cv::FileNode &&sub_node = fs[info->name];
		if (sub_node.empty())
			continue;
		Config *config = info->read(sub_node);
		cfg_map_.insert(std::make_pair(info->name, config));
	}
	
	fs.release();
	
	file_path_ = file_path;
	
	return true;
}


Config* ConfigManager::GetSubConfig(std::string name)
{
	ConfigMapType::const_iterator iter = cfg_map_.find(name);
	if (iter != cfg_map_.end()) {
		return iter->second;
	}
	
	return nullptr;
}

