#ifndef DETECTOR_DETECTOR_H
#define DETECTOR_DETECTOR_H

#include "detector_param.h"
#include "detector_context.h"
#include "hsm/hsm.h"
#include "config/config_manager.h"

using planning_utils::HSM;
using planning_utils::ConfigManager;

#define DEFINE_DETECTOR(DetectorType)                                     \
    class DetectorType##Detector : public planning_detector::Detector    \
    {                                                                          \
      protected:                                                               \
        DetectorType##Detector() : planning_detector::Detector() {}      \
      public:                                                                  \
        static DetectorType##Detector &getInstance()                             \
        {                                                                      \
            static DetectorType##Detector instance;                              \
            return instance;                                                   \
        }                                                                      \
        DetectorType##Detector(const DetectorType##Detector &) = delete;           \
        void operator=(const DetectorType##Detector &) = delete;
#define END_DETECTOR(DetectorType)                                        \
    };

namespace planning_detector
{

    class Detector : public HSM
    {
      public:

        // 抽象基类不能定义对象，所有子类将通过DEFINE_DETECTOR定义getInstance函数
        // /**
        //   * @brief  获取Detector的单例
        //   * @param  None
        //   * @retval None
        //   */
        // static Detector &getInstance()
        // {
        //     static Detector instance;
        //     return instance;
        // }

        // 禁止拷贝构造和赋值函数
        Detector(const Detector &) = delete;
        void operator=(const Detector &) = delete;

        virtual ~Detector() {};

        /**
          * @brief  通过配置管理器读取配置参数，初始化规划器
          * @param  cfg_mgr 配置管理器
          *         pool    线程池
          * @retval None
          */
        virtual void init(ConfigManager &cfg_mgr, ThreadPool *pool)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (m_initialized)
                return;

            float frequency = loadConfig(cfg_mgr);

            HSM::init(frequency, pool);
        }

        /**
          * @brief  开始运行检测器
          * @param  param   规划的输入
          * @retval bool    全局规划器是否成功开始执行，true成功，false失败
          */
        virtual bool startDetect(DetectorParamPtr param)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isRunning())
            {
                HSM_INFO_LOG(m_log_name, "start detector, type = %s...",
                    param->getTypeName().c_str());
                reset();
                if (!handleParam(param))
                {
                    HSM_INFO_LOG(m_log_name, "failed to handle param...");
                    return false;
                }
                start();
                return true;
            }
            return false;
        }

        /**
          * @brief  开始运行检测器
          * @param  ctx     上一次中断时执行的上下文
          * @retval bool    检测器是否成功开始执行，true成功，false失败
          */
        virtual bool startDetect(ContextDetectorPtr ctx)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isRunning())
            {
                HSM_INFO_LOG(m_log_name, "start detector from pause");
                start(ctx);
                return true;
            }
            return false;
        }

        /**
          * @brief  停止当前运行检测器
          * @param  None
          * @retval None
          */
        virtual void stopDetect()
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isIdle())
            {
                HSM_INFO_LOG(m_log_name, "stop planner");
                stop();
            }
        }

        /**
          * @brief  暂停当前运行检测器
          * @param  None
          * @retval ContextPtr  暂停时当前规划器运行的上下文
          */
        virtual ContextPtr pauseDetect()
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (isRunning())
            {
                HSM_INFO_LOG(m_log_name, "pause planner");
                return pause();
            }
            return nullptr;
        }

      protected:
        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        Detector()
            : HSM()
        {}

        /**
          * @brief  加载具体配置参数
          * @param  cfg_mgr 配置管理器
          * @retval 规划频率（frequency）
          */
        virtual float loadConfig(ConfigManager &cfg_mgr) = 0;

        using HSM::init;
        using HSM::start;
        using HSM::stop;
        using HSM::pause;

        /**
          * @brief  完成当前全局规划后调用的函数
          * @param  None
          * @retval None
          */
        virtual void finishDetect() = 0;

        /**
          * @brief  处理接收到的param
          * @param  param  收到的param
          * @retval None
          */
        virtual bool handleParam(DetectorParamPtr param) = 0;
    };
}

#endif // DETECTOR_DETECTOR_H
