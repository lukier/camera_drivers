/**
 * ****************************************************************************
 * Copyright (c) 2018, Robert Lukierski.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * ****************************************************************************
 * Intel RealSense2 Driver.
 * ****************************************************************************
 */

#include <atomic>

#include <RealSense2Driver.hpp>

#include <librealsense2/rs.hpp>

drivers::camera::RealSense2::RealSense2Exception::RealSense2Exception(int status, const char* file, int line)
{
    std::stringstream ss;
    ss << "RS2 Error " << status << " , " << " at " << file << ":" << line;
    errormsg = ss.str();
}

#define RS2_CHECK_ERROR(err_code) { if(err_code != openni::STATUS_OK) { throw OpenNIException(err_code, __FILE__, __LINE__); } }

static inline bool getRSOption(drivers::camera::EFeature fidx, rs2_option& opt, bool isauto = false)
{
    switch(fidx)
    {
        case drivers::camera::EFeature::BRIGHTNESS: opt = RS2_OPTION_BRIGHTNESS; return true;
        case drivers::camera::EFeature::EXPOSURE: if(!isauto) { opt = RS2_OPTION_EXPOSURE; } else { opt = RS2_OPTION_ENABLE_AUTO_EXPOSURE; } return true;
        case drivers::camera::EFeature::SHARPNESS: opt = RS2_OPTION_SHARPNESS; return true;
        case drivers::camera::EFeature::WHITE_BALANCE: if(!isauto) { opt = RS2_OPTION_WHITE_BALANCE; } else { opt = RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE; } return true;
        case drivers::camera::EFeature::HUE: opt = RS2_OPTION_HUE; return true;
        case drivers::camera::EFeature::SATURATION: opt = RS2_OPTION_SATURATION; return true;
        case drivers::camera::EFeature::GAMMA: opt = RS2_OPTION_GAMMA; return true;
        case drivers::camera::EFeature::SHUTTER: opt = RS2_OPTION_LASER_POWER; return true;
        case drivers::camera::EFeature::GAIN: opt = RS2_OPTION_GAIN; return true;
        case drivers::camera::EFeature::TEMPERATURE: opt = RS2_OPTION_BACKLIGHT_COMPENSATION; return true;
        default: return false;
    }
}

struct drivers::camera::RealSense2::RealSense2FrameHolder
{
    rs2::frame frame;
};

struct drivers::camera::RealSense2::RealSense2APIPimpl
{
    RealSense2APIPimpl() : 
        color_valid(false), depth_valid(false), ir1_valid(false), ir2_valid(false), 
        pipe(ctx), aligner(RS2_STREAM_COLOR), rgb_aligned_to_depth(false), depth_aligned_to_color(false)
    {
        
    }
    
    ~RealSense2APIPimpl()
    {
        
    }
    
    void frame_release(void* img)
    {
        
    }
    
    bool color_valid, depth_valid, ir1_valid, ir2_valid;
    rs2::context ctx;
    rs2::device dev;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::align aligner;
    bool rgb_aligned_to_depth;
    bool depth_aligned_to_color; 
};

drivers::camera::RealSense2::RealSense2() : CameraDriverBase(), m_pimpl(new RealSense2APIPimpl()), is_running(false)
{
    //rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
}
   
drivers::camera::RealSense2::~RealSense2()
{
    close();
}

void drivers::camera::RealSense2::image_release(void* img)
{
    RealSense2FrameHolder* ft = static_cast<RealSense2FrameHolder*>(img);
    delete ft;
}

void drivers::camera::RealSense2::open(unsigned int idx)
{
    if(isOpened()) { return; }
    
    auto list = m_pimpl->ctx.query_devices(); // Get a snapshot of currently connected devices
    
    if(list.size() == 0) 
    {
        throw std::runtime_error("No RealSense2 devices");
    }
    
    if(idx >= list.size())
    {
        throw std::runtime_error("RealSense2 camera index out of range");
    }
      
    m_pimpl->dev = list[idx];
    
    auto get_info_string = [&](rs2_camera_info i)
    {
        if(m_pimpl->dev.supports(i))
        {
            return std::string(m_pimpl->dev.get_info(i));
        }
        else
        {
            return std::string("NA");
        }
    };
    
    m_cinfo.SerialNumber = get_info_string(RS2_CAMERA_INFO_SERIAL_NUMBER);
    m_cinfo.InterfaceType = CameraInfo::CameraInterface::USB;
    m_cinfo.IsColorCamera = true;
    m_cinfo.ModelName = get_info_string(RS2_CAMERA_INFO_NAME);
    m_cinfo.VendorName = std::string("Intel");
    m_cinfo.SensorInfo = get_info_string(RS2_CAMERA_INFO_PHYSICAL_PORT);
    m_cinfo.SensorResolution = std::string("NA");
    m_cinfo.DriverName = std::string("libRealSense2");
    m_cinfo.FirmwareVersion = get_info_string(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    m_cinfo.FirmwareBuildTime = std::string("NA");
}

bool drivers::camera::RealSense2::isOpenedImpl() const
{
    return m_pimpl->dev == true;
}

void drivers::camera::RealSense2::close()
{
    if(!isOpened()) { return; }
    
    if(is_running)
    {
        stop();
    }
    
    if(m_pimpl->depth_valid)
    {
        m_pimpl->cfg.disable_stream(RS2_STREAM_DEPTH);
        m_pimpl->depth_valid = false;
    }
    
    if(m_pimpl->color_valid)
    {
      m_pimpl->cfg.disable_stream(RS2_STREAM_COLOR);
        m_pimpl->color_valid = false;
    }
    
    if(m_pimpl->ir1_valid)
    {
        m_pimpl->cfg.disable_stream(RS2_STREAM_INFRARED, 1);
        m_pimpl->ir1_valid = false;
    }
    
    if(m_pimpl->ir2_valid)
    {
        m_pimpl->cfg.disable_stream(RS2_STREAM_INFRARED, 2);
        m_pimpl->ir2_valid = false;
    }
    
    m_pimpl->dev = rs2::device(); // invalidate
}

void drivers::camera::RealSense2::start()
{
    if(isStarted()) { return; }
    
    m_pimpl->profile = m_pimpl->pipe.start(m_pimpl->cfg);
    
    is_running = true;
}

void drivers::camera::RealSense2::stop()
{
    if(!isStarted()) { return; }
    
    is_running = false;
    
    m_pimpl->pipe.stop();
}

void drivers::camera::RealSense2::setDepthMode(std::size_t w, std::size_t h, unsigned int fps, 
                                               bool aligned_to_color)
{
    if(!isOpened()) { return; }
    
    m_pimpl->cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);

    m_pimpl->depth_aligned_to_color = aligned_to_color;
    
    if(m_pimpl->depth_aligned_to_color)
    {
        m_pimpl->aligner = rs2::align(RS2_STREAM_COLOR);
    }
        
    m_pimpl->depth_valid = true;
}

void drivers::camera::RealSense2::setRGBMode(std::size_t w, std::size_t h, unsigned int fps, 
                                             drivers::camera::EPixelFormat pixfmt, bool aligned_to_depth)
{
    if(!isOpened()) { return; }
    
    m_pimpl->rgb_aligned_to_depth = aligned_to_depth;
    
    if(m_pimpl->rgb_aligned_to_depth)
    {
        m_pimpl->aligner = rs2::align(RS2_STREAM_DEPTH);
    }
    
    rs2_format rsfmt = RS2_FORMAT_Y8;
    
    switch(pixfmt)
    {
        case EPixelFormat::PIXEL_FORMAT_MONO8:
            rsfmt = RS2_FORMAT_Y8;
            break;
        case EPixelFormat::PIXEL_FORMAT_MONO16:
            rsfmt = RS2_FORMAT_Y16;
            break;
        case EPixelFormat::PIXEL_FORMAT_RGB8:
            rsfmt = RS2_FORMAT_RGB8;
            break;
        case EPixelFormat::PIXEL_FORMAT_BGR8:
            rsfmt = RS2_FORMAT_BGR8;
            break;
        default:
            throw std::runtime_error("Unsupported pixel format");
    }
    
    m_pimpl->cfg.enable_stream(RS2_STREAM_COLOR, w, h, rsfmt, fps);
    
    m_pimpl->color_valid = true;
}

void drivers::camera::RealSense2::setIR1Mode(std::size_t w, std::size_t h, unsigned int fps)
{
    if(!isOpened()) { return; }
    
    m_pimpl->cfg.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y16, fps);
    
    m_pimpl->ir1_valid = true;
}

void drivers::camera::RealSense2::setIR2Mode(std::size_t w, std::size_t h, unsigned int fps)
{
    if(!isOpened()) { return; }
    
    m_pimpl->cfg.enable_stream(RS2_STREAM_INFRARED, 2, w, h, RS2_FORMAT_Y16, fps);
    
    m_pimpl->ir2_valid = true;
}

std::size_t drivers::camera::RealSense2::getRGBWidth() const
{
    if(m_pimpl->color_valid)
    {
        auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        return rgb_stream.width();
    }
    else
    {
        return 0;
    }
}

std::size_t drivers::camera::RealSense2::getRGBHeight() const
{
    if(m_pimpl->color_valid)
    {
        auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        return rgb_stream.height();
    }
    else
    {
        return 0;
    }
}

drivers::camera::EPixelFormat drivers::camera::RealSense2::getRGBPixelFormat() const
{
    return drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8;
}

std::size_t drivers::camera::RealSense2::getDepthWidth() const
{
    if(m_pimpl->depth_valid)
    {
        auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        return depth_stream.width();
    }
    else
    {
        return 0;
    }
}

std::size_t drivers::camera::RealSense2::getDepthHeight() const
{
    if(m_pimpl->depth_valid)
    {
        auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        return depth_stream.height();
    }
    else
    {
        return 0;
    }
}

drivers::camera::EPixelFormat drivers::camera::RealSense2::getDepthPixelFormat() const
{
    return drivers::camera::EPixelFormat::PIXEL_FORMAT_DEPTH_U16;
}

std::size_t drivers::camera::RealSense2::getIR1Width() const
{
    if(m_pimpl->ir1_valid)
    {
        auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
        return ir_stream.width();
    }
    else
    {
        return 0;
    }
}

std::size_t drivers::camera::RealSense2::getIR1Height() const
{
    if(m_pimpl->ir1_valid)
    {
        auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
        return ir_stream.height();
    }
    else
    {
        return 0;
    }
}

drivers::camera::EPixelFormat drivers::camera::RealSense2::getIR1PixelFormat() const
{
    return drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16;
}

std::size_t drivers::camera::RealSense2::getIR2Width() const
{
    if(m_pimpl->ir2_valid)
    {
        auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
        return ir_stream.width();
    }
    else
    {
        return 0;
    }
}

std::size_t drivers::camera::RealSense2::getIR2Height() const
{
    if(m_pimpl->ir2_valid)
    {
        auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
        return ir_stream.height();
    }
    else
    {
        return 0;
    }
}

drivers::camera::EPixelFormat drivers::camera::RealSense2::getIR2PixelFormat() const
{
    return drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16;
}

float drivers::camera::RealSense2::getDepthScale() const
{
    return m_pimpl->profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
}

void drivers::camera::RealSense2::getRGBIntrinsics(float& fx, float& fy, float& u0, float& v0, std::array<float,5>* dist) const
{
    auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto ints = rgb_stream.get_intrinsics();
    
    fx = ints.fx;
    fy = ints.fy;
    u0 = ints.ppx;
    v0 = ints.ppy;
    
    if(dist != nullptr)
    {
        for(std::size_t i = 0 ; i < 5 ; ++i) 
        {
            dist->at(i) = ints.coeffs[i];
        }
    }
}

void drivers::camera::RealSense2::getDepthIntrinsics(float& fx, float& fy, float& u0, float& v0, std::array<float,5>* dist) const
{
    auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const auto ints = depth_stream.get_intrinsics();
    
    fx = ints.fx;
    fy = ints.fy;
    u0 = ints.ppx;
    v0 = ints.ppy;
    
    if(dist != nullptr)
    {
        for(std::size_t i = 0 ; i < 5 ; ++i) 
        {
            dist->at(i) = ints.coeffs[i];
        }
    }
}

void drivers::camera::RealSense2::getIR1Intrinsics(float& fx, float& fy, float& u0, float& v0, std::array<float,5>* dist) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    fx = ints.fx;
    fy = ints.fy;
    u0 = ints.ppx;
    v0 = ints.ppy;
    
    if(dist != nullptr)
    {
        for(std::size_t i = 0 ; i < 5 ; ++i) 
        {
            dist->at(i) = ints.coeffs[i];
        }
    }
}

void drivers::camera::RealSense2::getIR2Intrinsics(float& fx, float& fy, float& u0, float& v0, std::array<float,5>* dist) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    fx = ints.fx;
    fy = ints.fy;
    u0 = ints.ppx;
    v0 = ints.ppy;
    
    if(dist != nullptr)
    {
        for(std::size_t i = 0 ; i < 5 ; ++i) 
        {
            dist->at(i) = ints.coeffs[i];
        }
    }
}

void drivers::camera::RealSense2::getExtrinsicsDepthToColor(float& tx, float& ty, float&tz, std::array<float,9>* rotMat) const
{
    auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto exts = depth_stream.get_extrinsics_to(rgb_stream);
    
    tx = exts.translation[0];
    ty = exts.translation[1];
    tz = exts.translation[2];
    if(rotMat != nullptr)
    {
        for(std::size_t i = 0 ; i < 9 ; ++i) { rotMat->at(i) = exts.rotation[i]; }
    }
}

void drivers::camera::RealSense2::getExtrinsicsColorToDepth(float& tx, float& ty, float&tz, std::array<float,9>* rotMat) const
{
    auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto exts = rgb_stream.get_extrinsics_to(depth_stream);
    
    tx = exts.translation[0];
    ty = exts.translation[1];
    tz = exts.translation[2];
    if(rotMat != nullptr)
    {
        for(std::size_t i = 0 ; i < 9 ; ++i) { rotMat->at(i) = exts.rotation[i]; }
    }
}

#ifdef CAMERA_DRIVERS_HAVE_CAMERA_MODELS
void drivers::camera::RealSense2::getRGBIntrinsics(cammod::PinholeDisparity<float>& cam) const
{
    auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto ints = rgb_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparity<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, (float)getRGBWidth(), (float)getRGBHeight());
}

void drivers::camera::RealSense2::getRGBIntrinsics(cammod::PinholeDisparityBrownConrady<float>& cam) const
{
    auto rgb_stream = m_pimpl->profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto ints = rgb_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparityBrownConrady<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, 
                                            ints.coeffs[0], ints.coeffs[1], ints.coeffs[2], ints.coeffs[3], ints.coeffs[4], 
                                            (float)getRGBWidth(), (float)getRGBHeight());
}

void drivers::camera::RealSense2::getDepthIntrinsics(cammod::PinholeDisparity<float>& cam) const
{
    auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const auto ints = depth_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparity<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, (float)getDepthWidth(), (float)getDepthHeight());
}

void drivers::camera::RealSense2::getDepthIntrinsics(cammod::PinholeDisparityBrownConrady<float>& cam) const
{
    auto depth_stream = m_pimpl->profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const auto ints = depth_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparityBrownConrady<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, 
                                            ints.coeffs[0], ints.coeffs[1], ints.coeffs[2], ints.coeffs[3], ints.coeffs[4], 
                                            (float)getDepthWidth(), (float)getDepthHeight());
}

void drivers::camera::RealSense2::getIR1Intrinsics(cammod::PinholeDisparity<float>& cam) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparity<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, (float)getIR1Width(), (float)getIR1Height());
}

void drivers::camera::RealSense2::getIR1Intrinsics(cammod::PinholeDisparityBrownConrady<float>& cam) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparityBrownConrady<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, 
                                            ints.coeffs[0], ints.coeffs[1], ints.coeffs[2], ints.coeffs[3], ints.coeffs[4], 
                                            (float)getIR1Width(), (float)getIR1Height());
}

void drivers::camera::RealSense2::getIR2Intrinsics(cammod::PinholeDisparity<float>& cam) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparity<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, (float)getIR2Width(), (float)getIR2Height());
}

void drivers::camera::RealSense2::getIR2Intrinsics(cammod::PinholeDisparityBrownConrady<float>& cam) const
{
    auto ir_stream = m_pimpl->profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
    const auto ints = ir_stream.get_intrinsics();
    
    cam = cammod::PinholeDisparityBrownConrady<float>(ints.fx, ints.fy, ints.ppx, ints.ppy, 
                                            ints.coeffs[0], ints.coeffs[1], ints.coeffs[2], ints.coeffs[3], ints.coeffs[4], 
                                            (float)getIR2Width(), (float)getIR2Height());
}
#endif // CAMERA_DRIVERS_HAVE_CAMERA_MODELS

bool drivers::camera::RealSense2::getFeatureAuto(EFeature fidx)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return false; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        return sensor.get_option(opt) > 0.0f;
    }
    
    return false;
}

void drivers::camera::RealSense2::setFeatureAuto(EFeature fidx, bool b)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        sensor.set_option(opt, b == true ? 1.0 : 0.0);
    }
}

float drivers::camera::RealSense2::getFeatureValueAbs(EFeature fidx)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return 0.0f; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        return sensor.get_option(opt);
    }
    
    return 0.0f;
}

uint32_t drivers::camera::RealSense2::getFeatureMin(EFeature fidx)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return 0; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        auto optr = sensor.get_option_range(opt);
        return optr.min;
    }
    
    return 0.0f;
}

uint32_t drivers::camera::RealSense2::getFeatureMax(EFeature fidx)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return 0; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        auto optr = sensor.get_option_range(opt);
        return optr.max;
    }
    
    return 0.0f;
}

void drivers::camera::RealSense2::setFeatureValueAbs(EFeature fidx, float val)
{
    rs2_option opt;
    if(!getRSOption(fidx,opt)) { return; }
    
    auto sensor = m_pimpl->dev.first<rs2::sensor>();
    
    if(sensor.supports(opt))
    {
        sensor.set_option(opt, val);
    }
}

bool drivers::camera::RealSense2::captureFrameImpl(FrameBuffer* cf1, FrameBuffer* cf2, FrameBuffer* cf3, 
                                                   FrameBuffer* cf4, int64_t timeout)
{
    rs2::frameset frames_pre;
    if(timeout > 0)
    {
        frames_pre = m_pimpl->pipe.wait_for_frames(timeout);
    }
    else
    {
        if(!m_pimpl->pipe.poll_for_frames(&frames_pre))
        {
            return false;
        }
    }
    
    rs2::frameset frames_post;
    
    const std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    
    if(frames_pre.size() == 0) 
    { return false; } // timeout
    
    const std::chrono::nanoseconds d = tp.time_since_epoch();
    
    if(m_pimpl->depth_aligned_to_color || m_pimpl->rgb_aligned_to_depth) // use aligner
    {
        frames_post = m_pimpl->aligner.process(frames_pre);
    }
    else
    {
        frames_post = frames_pre;
    }
    
    auto setupFrame = [&](FrameBuffer* cf, rs2::frame s, drivers::camera::EPixelFormat pf)
    {
        if(cf != nullptr)
        {
            if(s)
            {
                RealSense2FrameHolder* fh = new RealSense2FrameHolder();
                fh->frame = s;
                
                cf->create(fh, std::bind(&drivers::camera::RealSense2::image_release, this, std::placeholders::_1), 
                           static_cast<uint8_t*>(const_cast<void*>(s.get_data())), 
                           s.as<rs2::video_frame>().get_width(), s.as<rs2::video_frame>().get_height(), 
                           pf, s.as<rs2::video_frame>().get_stride_in_bytes());
                cf->setFrameCounter(s.get_frame_number());
                cf->setTimeStamp((uint64_t)(s.get_timestamp() * 1000000.0));
                cf->setPCTimeStamp(d.count());
            }
        }
    };

    if(m_pimpl->color_valid && m_pimpl->depth_valid && m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 4 R D I1 I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf3, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
        setupFrame(cf4, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(m_pimpl->color_valid && m_pimpl->depth_valid && m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 3 R D I1
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf3, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
    }
    else if(m_pimpl->color_valid && m_pimpl->depth_valid && !m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 3 R D I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf3, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(m_pimpl->color_valid && !m_pimpl->depth_valid && m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 3 R I1 I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
        setupFrame(cf3, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(!m_pimpl->color_valid && m_pimpl->depth_valid && m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 3 D I1 I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
        setupFrame(cf3, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(m_pimpl->color_valid && m_pimpl->depth_valid && !m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 2 R D
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
    }
    else if(m_pimpl->color_valid && !m_pimpl->depth_valid && m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 2 R I1
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
    }
    else if(m_pimpl->color_valid && !m_pimpl->depth_valid && !m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 2 R I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(!m_pimpl->color_valid && m_pimpl->depth_valid && m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 2 D I1
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
    }
    else if(!m_pimpl->color_valid && m_pimpl->depth_valid && !m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 2 D I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(!m_pimpl->color_valid && !m_pimpl->depth_valid && m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 2 I1 I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
        setupFrame(cf2, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
    else if(m_pimpl->color_valid && !m_pimpl->depth_valid && !m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 1 R
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_COLOR), getRGBPixelFormat());
    }
    else if(!m_pimpl->color_valid && m_pimpl->depth_valid && !m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 1 D
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_DEPTH), getDepthPixelFormat());
    }
    else if(!m_pimpl->color_valid && !m_pimpl->depth_valid && m_pimpl->ir1_valid && !m_pimpl->ir2_valid) // 1 I1
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_INFRARED), getIR1PixelFormat());
    }
    else if(!m_pimpl->color_valid && !m_pimpl->depth_valid && !m_pimpl->ir1_valid && m_pimpl->ir2_valid) // 1 I2
    {
        setupFrame(cf1, frames_post.first(RS2_STREAM_INFRARED), getIR2PixelFormat());
    }
        
    return true;
}
