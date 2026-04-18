#include "daheng.hpp"

#include <opencv2/imgproc.hpp>
#include <chrono>
#include <vector>
#include <mutex>
#include <iostream>

#include "include/GxIAPILegacy.h"
#include "include/DxImageProc.h"

constexpr double MS_TO_US = 1000.0;
constexpr size_t THREAD_SAFE_QUEUE_MAX_SIZE = 5;

namespace io
{

static DaHeng* g_daheng_instance = nullptr;

// -------------------- SDK 状态转字符串 --------------------
static const char* gx_status_to_string(GX_STATUS status)
{
    switch (status)
    {
    case GX_STATUS_SUCCESS: return "SUCCESS";
    case GX_STATUS_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case GX_STATUS_INVALID_HANDLE: return "INVALID_HANDLE";
    case GX_STATUS_INVALID_CALL: return "INVALID_CALL";
    case GX_STATUS_INVALID_ACCESS: return "INVALID_ACCESS";
    case GX_STATUS_ERROR: return "GENERIC_ERROR";
    case GX_STATUS_OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "UNKNOWN";
    }
}

// -------------------- 回调函数 --------------------
void GX_STDC OnFrameCallback(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (g_daheng_instance && pFrame && pFrame->status == GX_FRAME_STATUS_SUCCESS)
        g_daheng_instance->onFrameCallback(pFrame);
}

// -------------------- 图像回调 --------------------
void DaHeng::onFrameCallback(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (!pFrame || pFrame->status != GX_FRAME_STATUS_SUCCESS)
        return;

    CameraData data;
    cv::Mat bgr(pFrame->nHeight, pFrame->nWidth, CV_8UC3);

    DX_BAYER_CONVERT_TYPE convert_type = RAW2RGB_NEIGHBOUR;
    DX_PIXEL_COLOR_FILTER color_filter = DX_PIXEL_COLOR_FILTER(pixel_color_filter_);
    DX_RGB_CHANNEL_ORDER order = DX_ORDER_BGR;

    if (DxRaw8toRGB24Ex((void*)pFrame->pImgBuf, (void*)bgr.data,
                        pFrame->nWidth, pFrame->nHeight,
                        convert_type, color_filter, false, order) != DX_OK)
        return;

    data.img = std::move(bgr);
    data.timestamp = std::chrono::steady_clock::now();

    // 直接推入队列（兼容 push 接口）
    queue_.push(std::move(data));
    frame_count_++;
}

// -------------------- 构造函数 --------------------
DaHeng::DaHeng(double exposure_ms, double gain, const std::string& vid_pid)
    : exposure_us_(exposure_ms * MS_TO_US),
      gain_(gain),
      handle_(nullptr),
      is_color_camera_(true),
      capturing_(false),
      capture_quit_(false),
      queue_(THREAD_SAFE_QUEUE_MAX_SIZE),
      pixel_color_filter_(0),
      frame_count_(0),
      last_fps_time_(std::chrono::steady_clock::now())
{
    GX_STATUS status;
    g_daheng_instance = this;

    if ((status = GXInitLib()) != GX_STATUS_SUCCESS)
        throw std::runtime_error("GXInitLib failed");

    uint32_t num = 0;
    GXUpdateDeviceList(&num, 1000);
    if (num == 0)
        throw std::runtime_error("No Daheng camera found");

    GX_OPEN_PARAM open;
    open.accessMode = GX_ACCESS_EXCLUSIVE;
    open.openMode = GX_OPEN_INDEX;
    open.pszContent = (char*)"1";

    if ((status = GXOpenDevice(&open, &handle_)) != GX_STATUS_SUCCESS)
        throw std::runtime_error("GXOpenDevice failed");

    GXGetEnum(handle_, GX_ENUM_PIXEL_COLOR_FILTER, &pixel_color_filter_);

    // -------- 相机参数设置 --------
    GXSetEnum(handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GXSetEnum(handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

    GXSetEnum(handle_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    GXSetEnum(handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(handle_, GX_FLOAT_EXPOSURE_TIME, exposure_us_);

    GXSetEnum(handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetFloat(handle_, GX_FLOAT_GAIN, gain_);

    GXSetEnum(handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetEnum(handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);

    GXSetEnum(handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    GXSetFloat(handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, 100.0);

    // 网络包优化
    bool implemented = false;
    GXIsImplemented(handle_, GX_INT_GEV_PACKETSIZE, &implemented);
    if (implemented)
        GXSetInt(handle_, GX_INT_GEV_PACKETSIZE, 1500);

    // 注册回调并启动采集
    GXRegisterCaptureCallback(handle_, nullptr, OnFrameCallback);
    GXSendCommand(handle_, GX_COMMAND_ACQUISITION_START);
    capturing_ = true;

    last_fps_time_ = std::chrono::steady_clock::now();
}

// -------------------- 析构函数 --------------------
DaHeng::~DaHeng()
{
    if (handle_)
    {
        if (capturing_)
        {
            GXSendCommand(handle_, GX_COMMAND_ACQUISITION_STOP);
            GXUnregisterCaptureCallback(handle_);
        }
        GXCloseDevice(handle_);
    }
    GXCloseLib();
    g_daheng_instance = nullptr;
}

// -------------------- 读帧函数（防止空帧中断） --------------------
void DaHeng::read(cv::Mat& img, std::chrono::steady_clock::time_point& ts)
{
    CameraData data;
    if (queue_.try_pop(data))
    {
        last_frame_ = std::move(data.img);
        last_timestamp_ = data.timestamp;
    }

    if (!last_frame_.empty())
    {
        img = last_frame_;
        ts = last_timestamp_;
    }
    else
    {
        img.release();
        ts = std::chrono::steady_clock::time_point();
    }
}

} // namespace io
