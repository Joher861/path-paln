#ifndef LOG_TEXT_LOG_PARAM
#define LOG_TEXT_LOG_PARAM

#include <memory>

#include "log_param.h"
// #include "text_log_handler.h"

namespace planning_utils
{
    #define KB    * (1024)
    #define MB    * (1024 * 1024)

    struct TextLogParam : LogParam
    {
      public:

        uint64_t max_file_size;
        uint8_t max_file_cnt;

      private:

        // TextLogParam(std::string _name,
        //     std::string _path,
        //     std::string _extension,
        //     uint8_t _ts_mask,
        //     bool _print_to_console,
        //     uint64_t _max_file_size,
        //     uint8_t _max_file_cnt)
        // {
        //     handler_param = CREATE_HANDLER_PARAM(Text,
        //         _name, _path, _extension, _max_file_size, _max_file_cnt);
        //     func_param = CREATE_FUNC_PARAM(PlainText, _ts_mask_);            
        // }


        // virtual bool isHandlerParamMatch(HandlerParamPtr param) const
        // {
        //     TextHandlerParamPtr text_param
        //         = std::dynamic_pointer_cast<TextHandlerParam>(param);
        //     if ()
        // }
        // virtual bool isFuncParamMatch(FuncParamPtr param) const
        // {}
        // virtual LogHandlerPtr generateNewHandler() const
        // {
          
        // }
    };
}

#endif // LOG_TEXT_LOG_PARAM