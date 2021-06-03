#include <iostream>

#include "thread_pool/ThreadPool.h"
#include <opencv2/imgcodecs.hpp>

#include "log_manager.h"
#include "csv_log_data.h"

using namespace planning_utils;
using namespace std;


#define LOG_TEST1_FLAG test1
#define LOG_TEST1    "test1"
#define TEST1_DEBUG_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST1), __LINE__, __FILE__, LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define TEST1_INFO_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST1), __LINE__, __FILE__, LEVEL_INFO, fmt, ##__VA_ARGS__)
#define TEST1_WARN_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST1), __LINE__, __FILE__, LEVEL_WARN, fmt, ##__VA_ARGS__)
#define TEST1_ERROR_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST1), __LINE__, __FILE__, LEVEL_ERROR, fmt, ##__VA_ARGS__)

    
CREATE_LOG(PlainText, LOG_TEST1_FLAG, LOG_TEST1, "test1",
    "tmp/testlog/test", "log", READABLE_US, true, 100 KB, 2, LEVEL_DEBUG);

#define LOG_TEST2_FLAG test2
#define LOG_TEST2    "test2"
#define TEST2_DEBUG_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST2), __LINE__, __FILE__, LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define TEST2_INFO_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST2), __LINE__, __FILE__, LEVEL_INFO, fmt, ##__VA_ARGS__)
#define TEST2_WARN_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST2), __LINE__, __FILE__, LEVEL_WARN, fmt, ##__VA_ARGS__)
#define TEST2_ERROR_LOG(fmt, ...)        \
    LOG(LOG_NAMES(LOG_TEST2), __LINE__, __FILE__, LEVEL_ERROR, fmt, ##__VA_ARGS__)

CREATE_LOG(PlainText, LOG_TEST2_FLAG, LOG_TEST2, "test2",
    "tmp/testlog/test", "log", READABLE_US, true, 1 KB, 6, LEVEL_INFO);

struct CSVPoseData : CSVLogData
{
    float x;
    float y;
    float z;

    CSVPoseData(float _x, float _y, float _z)
        : x(_x), y(_y), z(_z)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(x) + delimiter
                            + std::to_string(y) + delimiter
                            + std::to_string(z);
        return log_str;
    }
};

#define LOG_POSE_FLAG pose
#define LOG_POSE    "pose"

#define POSE_LOG(x, y, z)   \
    LOG(LOG_NAMES(LOG_POSE), std::make_shared<CSVPoseData>(x, y, z).get())

CREATE_LOG(CSV, LOG_POSE_FLAG, LOG_POSE, "pose",
    "tmp/testlog/test", "csv", RAW_MS, true, 1 KB, 6, ", ");

#define LOG_POSE2_FLAG pose2
#define LOG_POSE2    "pose2"

CREATE_LOG(CSV, LOG_POSE2_FLAG, LOG_POSE2, "pose2",
    "tmp/testlog/test", "csv", RAW_MS, true, 1 KB, 6, ", ");

#define LOG_IMG_FLAG img
#define LOG_IMG      "img"

#define TEST_IMG_LOG(img)   \
    LOG(LOG_NAMES(LOG_IMG), img)

CREATE_LOG(Img, LOG_IMG_FLAG, LOG_IMG, "test_img",
    "tmp/testlog/test", "bmp", RAW_MS, true, 1000);

int main()
{
    ThreadPool pool(2);

    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("Work/Project/K905/k905/planning/config/planning.yaml");

    g_lm.init(cfg_mgr);

    CLEAR_LOG_EXCEPT();
    OPEN_LOG_EXCEPT();

    // pool.enqueue(
    //     [&] ()
    //     {
    //         uint64_t cnt = 0;
    //         while (cnt < 10)
    //         {
    //             // TEST1_DEBUG_LOG("test1 %lu", cnt++);
    //             PLAIN_TEXT_DEBUG_LOG(LOG_NAMES(LOG_TEST1, LOG_TEST2), "test multi %d", cnt++);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //         }
    //     }
    // );

    // pool.enqueue(
    //     [&] ()
    //     {
    //         uint64_t cnt = 0;
    //         while (cnt < 10)
    //         {
    //             TEST2_WARN_LOG("hahaha %s %lu", "s", cnt++);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //         }
    //     }
    // );

    // pool.enqueue(
    //     [&] ()
    //     {
    //         uint64_t cnt = 0;
    //         while (cnt < 10)
    //         {
    //             // CSV_LOG(LOG_NAMES(""), CSVPoseData, 1, 1, 1);
    //             // POSE_LOG(cnt, cnt / 10.0f, cnt / 100.0f);
    //             CSV_LOG(LOG_NAMES(LOG_POSE, LOG_POSE2), CSVPoseData,
    //                 cnt, cnt / 10.0f, cnt / 100.0f);
    //             cnt++;
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //         }
    //     }
    // );

    // TEST1_DEBUG_LOG("test %d\n", 1);
    // TEST2_INFO_LOG("test %d info\n", 2);
    // TEST2_DEBUG_LOG("test %d debug\n", 2);
    // for (size_t i = 0; i < 1000; ++i)
    //     TEST1_ERROR_LOG("test");
    // CLOSE_LOG(LOG_TEST1, LOG_TEST2);

    pool.enqueue(
        [&]()
        {
            cv::Mat &&img = cv::imread("/tmp/log/test/clean_map_17.57.09.810533_5954445439_138_226_507_465.bmp");
            
            int cnt = 0;
            while (cnt++ < 5)
            {
                printf("save img %d\n", cnt);
                TEST_IMG_LOG(img);
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
        }
    );

    // BACKUP_LOG("tmp/testlog/testbak");

    return 0;
}