#ifndef UTILS_CONFIG_H
#define UTILS_CONFIG_H

#include <stdint.h>
#include <list>
#include <opencv2/opencv.hpp>

/**
     * @brief  定义配置类型，新配置类型继承自Config
     * @param  CfgType  新类型名，完整为Config##ConfigType
     * @retval None
     * @eg     定义RotatePlanner的配置类型
     *         DEFINE_CONFIG(RotatePlanner)
     *             ...
     *         END_CONFIG(RotatePlanner)
     */
#define DEFINE_CONFIG(CfgType)                                                 \
    struct Config##CfgType : planning_utils::Config                         \
    {                                                                          \
      public:                                                                  \
        Config##CfgType() : Config() {}
#define END_CONFIG(CfgType)                                                    \
    };

#define DECLARE_CONFIG_READ_FUNC(CfgType)                                      \
    planning_utils::Config* ReadConfig##CfgType##Func(cv::FileNode &node);

#define DEFINE_CONFIG_READ_FUNC(CfgType)                                       \
    inline planning_utils::Config* ReadConfig##CfgType##Func(cv::FileNode &node)   \
    {                                                                          \
	    Config##CfgType *cfg = new Config##CfgType();
#define READ_CONFIG_MEMBER(member)                                             \
    node[#member] >> cfg->member;
#define END_CONFIG_READ_FUNC(CfgType)                                          \
        return cfg;                                                            \
    }

#define DEFINE_CONFIG_TYPE(ConfigName, CfgType)                                \
    planning_utils::ConfigType                                              \
        _Config##CfgType##Type{ConfigName, ReadConfig##CfgType##Func};

namespace planning_utils
{
    struct Config
    {
        virtual ~Config() {}
    };

    using ConfigReadFunc = Config* (*)(cv::FileNode &node);

	struct ConfigInfo {
		std::string name;
		ConfigReadFunc read;
	};

	using ConfigInfoListType = std::list<ConfigInfo *>;

	class ConfigType {
	public:
		ConfigType(std::string name, ConfigReadFunc read);
		~ConfigType();
		
		static ConfigInfoListType *info_list_;
	private:
		std::string name_;
	};
}

#endif // UTILS_CONFIG_H