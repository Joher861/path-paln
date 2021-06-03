#include <iostream>
#include <string>

#include "thread_pool/ThreadPool.h"

#include "data.h"
#include "data_center.h"
#include "utils/log_manager.h"
#include "utils/config_manager.h"
#include "misc/planning_common_config.h"

using namespace planning_data;
using namespace planning_utils;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);

DEFINE_DATA(Test1)
    int x;
    int y;
END_DATA(Test1)

DEFINE_DATA(Test2)
    string str;
END_DATA(Test2)

int main(int argc, const char** argv)
{
    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("../../../../../config/planning.yaml");
    g_dc.init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    g_dc.registerUpdateCallback<DataTest1>(
        [] (const Data& data)
        {
            const DataTest1& test_data1 = dynamic_cast<const DataTest1&>(data);
            DATA_INFO_LOG("data 1 callback { %d, %d }", test_data1.x,
                test_data1.y);
        });
    g_dc.registerUpdateCallback<DataTest2>(
        [] (const Data& data)
        {
            const DataTest2& test_data2 = dynamic_cast<const DataTest2&>(data);
            DATA_INFO_LOG("data 2 callback str = %s", test_data2.str.c_str());
        });
    
    ThreadPool pool(3);

    pool.enqueue(
        [&] ()
        {
            DataTest1 data1;
            data1.x = 2;
            data1.y = 4;
            if (g_dc.updateData<DataTest1>(&data1) == 0)
                DATA_INFO_LOG("update data1 { %d, %d }", data1.x, data1.y);
            else
                DATA_INFO_LOG("failed to update data1");

            this_thread::sleep_for(chrono::seconds(3));

            DataTest2 data2;
            data2.str = "hahaha";
            if (g_dc.updateData<DataTest2>(&data2) == 0)
                DATA_INFO_LOG("update data2 str = %s",data2.str.c_str());
            else
                DATA_INFO_LOG("failed to update data2");
        }
    );

    pool.enqueue(
        [&] ()
        {
            this_thread::sleep_for(chrono::seconds(1));

            DataTest2 data2;
            data2.str = "wewewewe";
            if (g_dc.updateData<DataTest2>(&data2) == 0)
                DATA_INFO_LOG("update data2 str = %s",data2.str.c_str());
            else
                DATA_INFO_LOG("failed to update data2");

            this_thread::sleep_for(chrono::seconds(1));

            DataTest1 data1;
            data1.x = 33;
            data1.y = 6;
            if (g_dc.updateData<DataTest1>(&data1) == 0)
                DATA_INFO_LOG("update data1 { %d, %d }", data1.x, data1.y);
            else
                DATA_INFO_LOG("failed to update data1");

        }
    );

    pool.enqueue(
        [&] ()
        {
            DataTest1 data1;
            DataTest2 data2;
            while (true)
            {
                if (g_dc.getData<DataTest1>(data1) == 0)
                    DATA_INFO_LOG("get data1 { %d, %d }", data1.x, data1.y);
                else
                    DATA_INFO_LOG("failed to get data1");
                
                if (g_dc.getData<DataTest2>(data2) == 0)
                    DATA_INFO_LOG("get data2 str = %s\n",data2.str.c_str());
                else
                    DATA_INFO_LOG("failed to get data2");

                this_thread::sleep_for(chrono::milliseconds(300));
            }
        }
    );

    return 0;
}