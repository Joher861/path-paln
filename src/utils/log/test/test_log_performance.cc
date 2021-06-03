#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <sys/resource.h>
#include <mutex>
#include <ctime>

#include "thread_pool/ThreadPool.h"

#include "log_manager.h"

#include "timer/sleep_timer.h"

using namespace planning_utils;

// #define LOG_TEST1_FLAG test1
// #define LOG_TEST1    "test1"
    
// CREATE_LOG(PlainText, LOG_TEST1_FLAG, LOG_TEST1, "test1",
//     "tmp/testlog/test", "log", READABLE_US, true, 100 KB, 2, LEVEL_DEBUG);

uint64_t test_fixed(const std::unordered_set<std::string> &fixed_log_names,
    bool pc, size_t test_time)
{
    std::string fixed_log_path = "tmp/test_log_perf/fixed";
    std::set<std::string> names;
    std::unordered_map<std::string, bool> finished;
    std::mutex finished_mtx;
    for (auto &name : fixed_log_names)
    {
        if (test_time == 0)
        {
            int8_t flag = g_lm.createLog(name, PlainTextLogParam{
                name, fixed_log_path, "log", READABLE_US, pc, 0, 0, LEVEL_DEBUG });
        }
        names.insert(name);
        finished.insert(std::make_pair(name, false));
    }

    size_t log_size = fixed_log_names.size();

    g_lm.openLog(std::move(names));

    auto write_log_func = [&] (const std::string &name) {
        size_t line_cnt = 500;
        for (size_t i = 0; i < line_cnt; ++i)
        {
            LOG(LOG_NAME(name), __LINE__, __FILE__, LEVEL_DEBUG,
                "ajsidfoa faisjdofjaos%lu sdfjaisjfiaj faivje %lu", i, i);
        }

        std::unique_lock<std::mutex> lock(finished_mtx);
        finished.erase(name);
    };
    
    ThreadPool pool{log_size};

    uint64_t start_ts = Timer::getSystemTimestampUS();
    for (auto & name : fixed_log_names)
    {
        pool.enqueue(std::bind(write_log_func, name));
    }

    SleepTimer timer{1000};
    while (!finished.empty())
    {
        timer.sleep();
    }
    uint64_t end_ts = Timer::getSystemTimestampUS();
    uint64_t ts_diff = end_ts - start_ts;
    printf("test fixed finished, time used = %lu\n", ts_diff);

    g_lm.closeLog(std::move(names));

    return ts_diff;
}

uint64_t test_random(const std::vector<std::string> &random_log_names,
    bool pc)
{
    std::string random_log_path = "tmp/test_log_perf/random";
    std::string cmd = "rm -rf " + random_log_path + "/*";
    system(cmd.c_str());
    
    std::set<std::string> names;
    std::unordered_map<size_t, bool> finished;
    std::mutex finished_mtx;
    for (auto &name : random_log_names)
    {
        int8_t flag = g_lm.createLog(name, PlainTextLogParam{
            name, random_log_path, "log", READABLE_US, pc, 0, 0, LEVEL_DEBUG });
        names.insert(name);
    }

    size_t log_size = random_log_names.size();
    for (size_t i = 0; i < log_size; i++)
        finished.emplace(i, false);

    g_lm.openLog(std::move(names));

    auto write_log_func = [&] (size_t thread_idx) {
        size_t line_cnt = 500;
        for (size_t i = 0; i < line_cnt; ++i)
        {
            size_t idx = rand() % log_size;
            // printf("idx = %lu  ", idx);
            auto &name = random_log_names[idx];
            // printf("name = %s\n", name.c_str());
            LOG(LOG_NAME(name), __LINE__, __FILE__, LEVEL_DEBUG,
                "ajsidfoa faisjdofjaos%lu sdfjaisjfiaj faivje %lu", i, i);
        }

        std::unique_lock<std::mutex> lock(finished_mtx);
        finished.erase(thread_idx);
    };
    
    ThreadPool pool{log_size};

    uint64_t start_ts = Timer::getSystemTimestampUS();
    for (size_t i = 0; i < log_size; ++i)
    {
        pool.enqueue(std::bind(write_log_func, i));
    }

    SleepTimer timer{1000};
    while (!finished.empty())
    {
        timer.sleep();
    }
    uint64_t end_ts = Timer::getSystemTimestampUS();
    uint64_t ts_diff = end_ts - start_ts;
    printf("test random finished, time used = %lu\n", ts_diff);

    g_lm.closeLog(std::move(names));

    return ts_diff;
}

int main()
{
    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("Work/Project/K905/k905/planning/config/planning.yaml");

    g_lm.init(cfg_mgr);

    struct rlimit core_limits;
    core_limits.rlim_cur = core_limits.rlim_max = RLIM_INFINITY;
    setrlimit(RLIMIT_CORE, &core_limits);

    srand(time(nullptr));

    // OPEN_LOG_EXCEPT();

    size_t test_times = 50;
    uint64_t total_time = 0;

    std::unordered_set<std::string> fixed_log_names;
    fixed_log_names.insert("fas9dfsfj");
    fixed_log_names.insert("asdf90uuj");
    fixed_log_names.insert("sdfg3ggga");
    fixed_log_names.insert("sdfsjijse");
    fixed_log_names.insert("sdfsouj0i");
    fixed_log_names.insert("jiodf89w3");
    fixed_log_names.insert("99winsywh");
    fixed_log_names.insert("ahshngsuu");
    fixed_log_names.insert("hnsfjiwhf");
    fixed_log_names.insert("ahoazhffn");
    fixed_log_names.insert("0s09j2mfs");
    fixed_log_names.insert("zhofhnwhf");
    fixed_log_names.insert("pqmshduva");
    fixed_log_names.insert("xsdfjfnow");
    fixed_log_names.insert("089jfmasd");
    fixed_log_names.insert("cshaSfn2a");
    fixed_log_names.insert("afjs0dfkf");
    fixed_log_names.insert("fajsodfin");
    fixed_log_names.insert("jas0dfjwf");
    fixed_log_names.insert("asjdf0aj2");
    fixed_log_names.insert("809sjf23n");
    fixed_log_names.insert("vniozghwg");
    fixed_log_names.insert("fashofasd");
    fixed_log_names.insert("hfi2n3f9s");
    fixed_log_names.insert("f02kfilss");
    fixed_log_names.insert("fahsodif2");
    fixed_log_names.insert("eeijfsajf");
    fixed_log_names.insert("cvjoshf3e");
    fixed_log_names.insert("shnodighg");
    fixed_log_names.insert("lxchvwief");
    fixed_log_names.insert("oizjoeisu");
    fixed_log_names.insert("so8j3naad");
    fixed_log_names.insert("cjvcmnajb");
    fixed_log_names.insert("naoosenso");
    fixed_log_names.insert("psjfqpwms");

    for (size_t i = 0; i < test_times; i++)
    {
        total_time += test_fixed(fixed_log_names, false, i);
    }
    printf("test fixed average time = %lu\n", total_time / test_times);

    std::unordered_set<int> random_log_ints;
    total_time = 0;
    for (size_t i = 0; i < test_times; ++i)
    {
        std::vector<std::string> random_log_names;
        
        size_t size = 35;

        for (size_t cnt = 0; cnt < size; cnt++)
        {
            int ii;
            bool new_int = false;
            while (!new_int)
            {
                ii = rand();
                if (random_log_ints.find(ii) == random_log_ints.end())
                    new_int = true;
            }
            random_log_ints.insert(ii);
            random_log_names.emplace_back(std::to_string(ii));
        }

        // for (auto &random_int : random_log_ints)
        //     std::cout << random_int << std::endl; 

        total_time += test_random(random_log_names, false);
    }
    printf("test random average time = %lu\n", total_time / test_times);

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

    return 0;
}