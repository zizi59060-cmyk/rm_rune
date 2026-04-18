/* tasks/auto_buff/trt_logger.hpp */
#pragma once

#include <NvInferRuntime.h>
#include <iostream>

// namespace sp::tasks::auto_buff // <-- 1. 删除这个错误的命名空间
namespace auto_buff // <-- 2. 使用这个正确的命名空间
{

// TensorRT 日志记录器
class TrtLogger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    // 仅打印 Error 和 Warning 信息
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TRT Buff] " << msg << std::endl;
    }
  }
};

}  // namespace auto_buff // <-- 3. 确保命名空间闭合正确