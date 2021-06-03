#ifndef LOG_IMG_LOG_PARAM
#define LOG_IMG_LOG_PARAM

#include <memory>

#include "include/log_param.h"
#include "include/img_log_handler.h"
#include "include/img_log_func.h"

namespace planning_utils
{
    struct ImgLogParam : LogParam
    {
      public:

        ImgLogParam(std::string _name,
            std::string _path,
            std::string _extension,
            uint8_t _ts_mask,
            bool _print_to_console,
            uint64_t _max_img_cnt)
        {
            handler_param = CREATE_HANDLER_PARAM(Img,
                _name, _path, _extension, _max_img_cnt);
            func_param = CREATE_FUNC_PARAM(Img,
                _ts_mask, _print_to_console, _name, _extension);
        }

        std::string img_name;
        uint64_t max_img_cnt;

      private:

        virtual bool isValid() const
        {
            if (handler_param->name == "" || handler_param->path == "")
                return false;
            ImgHandlerParamPtr hp
                = std::dynamic_pointer_cast<ImgHandlerParam>(handler_param);
            if (hp->extension != "bmp" && hp->extension != "jpg"
                && hp->extension != "png")
                return false;

            ImgFuncParamPtr fp
                = std::dynamic_pointer_cast<ImgFuncParam>(func_param);
            if (fp->img_name == "")
                return false;
            
            return true;
        }

        virtual LogHandlerPtr generateNewHandler(HandlerParamPtr param) const
        {
            return CREATE_LOG_HANDLER(Img, param);
        }

        virtual LogFuncPtr findFunc(
            const std::unordered_map<std::type_index, LogFuncPtr> &funcs) const
        {
            std::type_index func_type = typeid(ImgLogFunc);
            auto it = funcs.find(func_type); 
            if (it == funcs.end())
                return nullptr;
            else
                return it->second;
        }
        
    };
}

#endif // LOG_IMG_LOG_PARAM