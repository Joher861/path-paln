#ifndef DATA_DATA_CENTER_H
#define DATA_DATA_CENTER_H

#include <map>
#include <typeindex>
#include <shared_mutex>
#include <functional>

#include "data.h"
#include "include/data_center_log.h"
#include "config/config_manager.h"

using planning_utils::ConfigManager;

namespace planning_data
{
    /**
      * @desc   数据中心类，管理各模块间共享的数据
      * @eg     DataCenter dc = DataCenter::getInstance();
      */
    class DataCenter
    {
      public:

        /**
          * @desc   某一类数据相关信息的结构体，每一类存储在DataCenter中的Data子类数据都
          *         会有一个DataDetail与之对应。
          * @eg     None
          */
        struct DataDetail
        {
            /**
              * @brief  构造函数
              * @param  None
              * @retval None
              */
            DataDetail()
                : ready(false),     // 默认该类数据还没有准备好
                  addr(nullptr)     // 默认该类数据地址为空，还未分配
            {}

            bool ready;         // 表示该类数据是否h准备好，只有调用updateData更新过
                                // 一次该数据后才算准备好，可以被getData获取到
            Data* addr;         // DataCenter内为该类数据分配的地址
            std::shared_timed_mutex mtx;    // 每类数据对应的读写锁
            std::function<void(const Data&)> update_cb;     // 每次调用updateData
                                                            // 后将要执行的回调
        };

        /**
          * @brief  获取数据中心的单例
          * @param  None
          * @retval None
          */
        static DataCenter &getInstance();

        // 禁止拷贝和赋值构造函数
        DataCenter(const DataCenter &) = delete;
        void operator=(const DataCenter &) = delete;

        void init(ConfigManager &cfg_mgr);

        ~DataCenter();

        /**
          * @brief  更新数据的模板接口函数
          * @param  DataType    数据类型，需要是Data的子类
          *         data        该类数据的指针
          * @retval 是否成功更新数据  0   更新成功
          *                        -1   更新失败，输入的DataType非Data子类
          */
        template<typename DataType>
        int8_t updateData(const DataType* data);

        /**
          * @brief  获取数据的模板接口函数
          * @param  DataType    数据类型，需要是Data的子类
          *         data        该类数据的引用，保存获得的数据
          * @retval 是否成功获取数据  0   更新成功
          *                        -1   更新失败，输入的DataType非Data子类
          *                        -2   更新失败，DataCenter中还未存储过该类数据
          *                        -3   更新失败，DataCenter中该类数据还未ready
          */
        template<typename DataType>
        int8_t getData(DataType& data);

        /**
          * @brief  注册某类函数成功调用updataData接口后的回调函数。对同一类型的数据可多次
          *         调用该接口，新的回调将覆盖旧的。
          * @param  cb    回调函数，输入为Data数据的常量引用，返回为空
          * @retval None
          */
        template<typename DataType>
        void registerUpdateCallback(std::function<void(const Data&)> cb);

      private:

        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        DataCenter();

        std::map<std::type_index, DataDetail> m_data_details;
                                            // 每类数据和相关信息的映射关系
        std::shared_timed_mutex m_data_details_mtx;   // 映射关系表的互斥量
    };

    template<typename DataType>
    int8_t DataCenter::updateData(const DataType* data)
    {
        DATA_DEBUG_LOG("request update data %s", DataType::type_name.c_str());

        std::type_index type_idx = typeid(*data);

        std::shared_lock<std::shared_timed_mutex>
            data_details_slock(m_data_details_mtx);
        auto it = m_data_details.find(type_idx);
        if (it == m_data_details.end())
        {
            data_details_slock.unlock();
            std::unique_lock<std::shared_timed_mutex>
                data_details_ulock(m_data_details_mtx);
            if (m_data_details.find(type_idx) == m_data_details.end())
            {
                DATA_INFO_LOG("register data type = %s",
                    DataType::type_name.c_str());
                DataDetail& detail = m_data_details[type_idx];
                detail.addr = reinterpret_cast<Data*>(new DataType);
            }

            DataDetail& detail = m_data_details[type_idx];
            std::unique_lock<std::shared_timed_mutex> lock(detail.mtx);
            *reinterpret_cast<DataType*>(detail.addr) = *data;
            detail.ready = true;
            if (detail.update_cb)
                detail.update_cb(*detail.addr);
            DATA_INFO_LOG("data %s updated", DataType::type_name.c_str());
        }
        else
        {
            DataDetail& detail = it->second;
            std::unique_lock<std::shared_timed_mutex> lock(detail.mtx);
            *reinterpret_cast<DataType*>(detail.addr) = *data;
            detail.ready = true;
            if (detail.update_cb)
                detail.update_cb(*detail.addr);
            DATA_INFO_LOG("data %s updated", DataType::type_name.c_str());
        }

        // if (m_data_details.find(type_idx) == m_data_details.end())
        // {
        //     std::unique_lock<std::shared_timed_mutex>
        //         data_details_lock(m_data_details_mtx);
        //     DATA_INFO_LOG("register data type = %s",
        //         DataType::type_name.c_str());
        //     DataDetail& detail = m_data_details[type_idx];
        //     detail.addr = reinterpret_cast<Data*>(new DataType);
        // }

        // std::shared_lock<std::shared_timed_mutex>
        //     data_details_lock(m_data_details_mtx);
        // DataDetail& detail = m_data_details[type_idx];
        // std::unique_lock<std::shared_timed_mutex> lock(detail.mtx);
        // *reinterpret_cast<DataType*>(detail.addr) = *data;
        // detail.ready = true;
        // detail.update_cb(*detail.addr);
        // DATA_INFO_LOG("data %s updated", DataType::type_name.c_str());

        return 0;
    }

    template<typename DataType>
    int8_t DataCenter::getData(DataType& data)
    {
        std::type_index type_idx = typeid(data);
        std::shared_lock<std::shared_timed_mutex>
            data_details_lock(m_data_details_mtx);
        auto it = m_data_details.find(type_idx);
        if (it == m_data_details.end())
        {
            DATA_ERROR_LOG("required data is not registered");
            return -2;
        }
        auto &detail = it->second;
        if (!detail.ready)
        {
            DATA_ERROR_LOG("required data is not ready");
            return -3;
        }

        std::shared_lock<std::shared_timed_mutex> lock(detail.mtx);
        data = *reinterpret_cast<DataType*>(detail.addr);
        return 0;
    }

    template<typename DataType>
    void DataCenter::registerUpdateCallback(
        std::function<void(const Data&)> cb)
    {
        std::type_index type_idx = typeid(DataType);

        std::unique_lock<std::shared_timed_mutex>
            data_details_lock(m_data_details_mtx);
        auto it = m_data_details.find(type_idx);
        if (it == m_data_details.end())
        {
            DATA_INFO_LOG("register data type = %s",
                DataType::type_name.c_str());
            DataDetail& detail = m_data_details[type_idx];
            detail.addr = reinterpret_cast<Data*>(new DataType);
            DATA_INFO_LOG("register callback to data type = %s",
                DataType::type_name.c_str());
            detail.update_cb = cb;
        }
        else
        {
            it->second.update_cb = cb;
        }
    }
}

extern planning_data::DataCenter& g_dc;   // 事件中心的全局变量

#endif // DATA_DATA_CENTER_H
