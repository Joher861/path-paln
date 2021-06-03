#include "config.h"

using namespace planning_utils;

ConfigInfoListType *ConfigType::info_list_ = nullptr;

ConfigType::ConfigType(std::string name, ConfigReadFunc read)
{
	ConfigInfo *info = nullptr;
	
	if (nullptr == info_list_) {
		info_list_ = new ConfigInfoListType();
	}
	
	for (auto it = info_list_->cbegin(); it != info_list_->cend(); it++) {
		info = *it;
		if (0 == name.compare(info->name)) {
			std::cout << name << " config already existed" << std::endl;
			return;
		}
	}

	info = new ConfigInfo();
	info->name = name;
	info->read = read;
	info_list_->push_back(info);
	
	name_ = name;
}

ConfigType::~ConfigType()
{
	ConfigInfo *info = nullptr;
	
	for (auto it = info_list_->cbegin(); it != info_list_->cend(); it++) {
		info = *it;
		if (0 == name_.compare(info->name)) {
			info_list_->erase(it);
			break;
		}
	}
	
	if (0 == info_list_->size()) {
		delete info_list_;
		info_list_ = nullptr;
	}
}

