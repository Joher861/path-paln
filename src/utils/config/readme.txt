一.增加新的config步骤
1)在config_type.h中新增所需config的定义: 见ConfigCpu定义
2)在confit_sub.cpp中新增此config的解析函数typedef void *(*ReadFunc)(cv::FileStorage &fs, cv::FileNode &node):
见ReadCpuConfig
3)在confit_sub.cpp中新增此config的变量:见kCpuConfig
4)根椐名字获取此config void *ConfigManager::GetSubConfig(std::string name):
详见config_test.cpp中的调用.


二.注意事项
1)子类的格式请参见yaml语法,子类的空格需统一:目前子类采用四个空格定义,可以用其它n>=1个空格
2)ConfigType目前支持静态定义,暂未有动态定义需求<动态定义需增加互斥机制>.
3)config定义不支持析构调用.
4)config 不支持bool变量。类似下面的赋值不能通过编译：
    bool test_bool =  node["test_bool"];
5)如果yaml文件中一个参数，也程序中却引用了该名称，其将返回2147483647
int no_this =  node["no_this"];
no_this 将等于2147483647

三 编译命令
g++ -g -O0 config_manager.cpp config_sub.cpp config_test.cpp config_type.cpp -o config_test -std=gnu++11 -lopencv_core

