/**
 * ****************************************************************************
 * Copyright (c) 2015, Robert Lukierski.
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
 * Camera Driver Abstraction.
 * ****************************************************************************
 */

#include <CameraDrivers.hpp>

static const char* PixelFormatToStringMap[] = 
{
    "PIXEL_FORMAT_MONO8",
    "PIXEL_FORMAT_MONO16",
    "PIXEL_FORMAT_RGB8",
    "PIXEL_FORMAT_RGBA8",
    "PIXEL_FORMAT_MONO32F",
    "PIXEL_FORMAT_RGB32F",
    "PIXEL_FORMAT_DEPTH_U16",
    "PIXEL_FORMAT_DEPTH_U16_1MM",
    "PIXEL_FORMAT_DEPTH_U16_100UM",
    "PIXEL_FORMAT_DEPTH_F32_M"
};

static unsigned int PixelFormatToBytesPerPixelMap[] = 
{
    1, 2, 3, 4, sizeof(float), sizeof(float) * 3, 2, 2, 2, sizeof(float)
};

static unsigned int PixelFormatToChannelCountMap[] = 
{
    1, 1, 3, 4, 1, 3, 1, 1, 1, 1
};

const char* drivers::camera::PixelFormatToString(drivers::camera::EPixelFormat v)
{
    if((int)v <= (int)drivers::camera::EPixelFormat::PIXEL_FORMAT_DEPTH_F32_M)
    {
        return PixelFormatToStringMap[(int)v];
    }
    else
    {
        return "Unsupported";
    }
}

unsigned int drivers::camera::PixelFormatToBytesPerPixel(drivers::camera::EPixelFormat v)
{
    if((int)v <= (int)drivers::camera::EPixelFormat::PIXEL_FORMAT_DEPTH_F32_M)
    {
        return PixelFormatToBytesPerPixelMap[(int)v];
    }
    else
    {
        return 0;
    }
}

unsigned int drivers::camera::PixelFormatToChannelCount(drivers::camera::EPixelFormat v)
{
    if((int)v <= (int)drivers::camera::EPixelFormat::PIXEL_FORMAT_DEPTH_F32_M)
    {
        return PixelFormatToChannelCountMap[(int)v];
    }
    else
    {
        return 0;
    }
}

static const char* VideoModeToStringMap[] = 
{
    "VIDEOMODE_640x480RGB",
    "VIDEOMODE_640x480Y8",
    "VIDEOMODE_640x480Y16",
    "VIDEOMODE_800x600RGB",
    "VIDEOMODE_800x600Y8",
    "VIDEOMODE_800x600Y16",
    "VIDEOMODE_1024x768RGB",
    "VIDEOMODE_1024x768Y8",
    "VIDEOMODE_1024x768Y16",
    "VIDEOMODE_1280x960RGB",
    "VIDEOMODE_1280x960Y8",
    "VIDEOMODE_1280x960Y16",
    "VIDEOMODE_1600x1200RGB",
    "VIDEOMODE_1600x1200Y8",
    "VIDEOMODE_1600x1200Y16",
    "VIDEOMODE_CUSTOM"
};

static constexpr std::size_t VideoModeToWidthMap[] = 
{
    640, 640, 640,
    800, 800, 800,
    1024, 1024, 1024,
    1280, 1280, 1280,
    1600, 1600, 1600
};

static constexpr std::size_t VideoModeToHeightMap[] = 
{
    480, 480, 480,
    600, 600, 600,
    768, 768, 768,
    960, 960, 960,
    1200, 1200, 1200
};

static constexpr unsigned int VideoModeToChannelsMap[] = 
{
    3, 1, 1,
    3, 1, 1,
    3, 1, 1,
    3, 1, 1,
    3, 1, 1
};

static constexpr drivers::camera::EPixelFormat VideoModeToPixelFormatMap[] = 
{
    drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16,
    drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16,
    drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16,
    drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16,
    drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8, drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO16
};

const char* drivers::camera::VideoModeToString(drivers::camera::EVideoMode v)
{
    if((int)v <= (int)drivers::camera::EVideoMode::VIDEOMODE_CUSTOM)
    {
        return VideoModeToStringMap[(int)v];
    }
    else
    {
        return "Unsupported";
    }
}

std::size_t drivers::camera::VideoModeToWidth(drivers::camera::EVideoMode v)
{
    if((int)v <= (int)drivers::camera::EVideoMode::VIDEOMODE_1600x1200Y16)
    {
        return VideoModeToWidthMap[(int)v];
    }
    else 
    { 
        return 0; 
    }
}

std::size_t drivers::camera::VideoModeToHeight(drivers::camera::EVideoMode v)
{
    if((int)v <= (int)drivers::camera::EVideoMode::VIDEOMODE_1600x1200Y16)
    {
        return VideoModeToHeightMap[(int)v];
    }
    else
    {
        return 0;
    }
}

unsigned int drivers::camera::VideoModeToChannels(drivers::camera::EVideoMode v)
{
    return drivers::camera::PixelFormatToChannelCount(drivers::camera::VideoModeToPixelFormat(v));
}

drivers::camera::EPixelFormat drivers::camera::VideoModeToPixelFormat(drivers::camera::EVideoMode v)
{
    if((int)v <= (int)drivers::camera::EVideoMode::VIDEOMODE_1600x1200Y16)
    {
        return VideoModeToPixelFormatMap[(int)v];
    }
    else 
    { 
        return drivers::camera::EPixelFormat::PIXEL_FORMAT_MONO8; 
    }
}

static const char* FrameRateToStringMap[] = 
{
    "FRAMERATE_15",
    "FRAMERATE_30",
    "FRAMERATE_60",
    "FRAMERATE_120",
    "FRAMERATE_240",
    "FRAMERATE_CUSTOM"
};

static unsigned int FrameRateToFPSMap[] = 
{
    15, 30, 60, 120, 240
};

const char* drivers::camera::FrameRateToString(drivers::camera::EFrameRate fr)
{
    if((int)fr <= (int)drivers::camera::EFrameRate::FRAMERATE_CUSTOM)
    {
        return FrameRateToStringMap[(int)fr];
    }
    else
    {
        return "Unsupported";
    }
}

unsigned int drivers::camera::FrameRateToFPS(drivers::camera::EFrameRate fr)
{
    if((int)fr <= (int)drivers::camera::EFrameRate::FRAMERATE_240)
    {
        return FrameRateToFPSMap[(int)fr];
    }
    else 
    { 
        return 0; 
    }
}

static const char* FeatureToStringMap[] = 
{
    "BRIGHTNESS",
    "AUTO_EXPOSURE",
    "SHARPNESS",
    "WHITE_BALANCE",
    "HUE",
    "SATURATION",
    "GAMMA",
    "IRIS",
    "FOCUS",
    "ZOOM",
    "PAN",
    "TILT",
    "SHUTTER",
    "GAIN",
    "TRIGGER_MODE",
    "TRIGGER_DELAY",
    "FRAME_RATE",
    "TEMPERATURE",
};

const char* drivers::camera::FeatureToString(drivers::camera::EFeature ef)
{
    if((int)ef <= (int)drivers::camera::EFeature::TEMPERATURE)
    {
        return FeatureToStringMap[(int)ef];
    }
    else
    {
        return "Unsupported";
    }
}

void drivers::camera::FrameBuffer::create(drivers::camera::EVideoMode vm)
{
    create(VideoModeToWidthMap[(int)vm], VideoModeToHeightMap[(int)vm], VideoModeToPixelFormatMap[(int)vm], VideoModeToChannelsMap[(int)vm]);
}

void drivers::camera::FrameBuffer::create(std::size_t awidth, std::size_t aheight, drivers::camera::EPixelFormat apixfmt, std::size_t astride)
{
    width = awidth;
    height = aheight;
    pixfmt = apixfmt;
    bpp = PixelFormatToBytesPerPixel(pixfmt);
    if(astride == 0) { stride = width * PixelFormatToBytesPerPixelMap[(int)pixfmt]; } else { stride = astride; }
    unsigned int new_data_size = stride * height;
    if(new_data_size != data_size) // only reallocate when necessary
    {
        // release what we might have now
        release();
        
        data_size = new_data_size;
        create_byte_array(data_size);
    }
}

void drivers::camera::FrameBuffer::create(uint8_t* abuffer, std::size_t awidth, std::size_t aheight, drivers::camera::EPixelFormat apixfmt, std::size_t astride)
{
    // release what we might have now, a must for external buffers
    release();
    
    deleter = std::bind(&FrameBuffer::delete_nothing, this, std::placeholders::_1);
    associated_buffer = abuffer;
    memaccess = abuffer;
    width = awidth;
    height = aheight;
    pixfmt = apixfmt;
    bpp = PixelFormatToBytesPerPixel(pixfmt);
    if(astride == 0) { stride = width * PixelFormatToBytesPerPixelMap[(int)pixfmt]; } else { stride = astride; }
    data_size = stride * height;
}

void drivers::camera::FrameBuffer::create(void* extobject, std::function<void (void*)> adeleter, uint8_t* abuffer, std::size_t awidth, std::size_t aheight, drivers::camera::EPixelFormat apixfmt, std::size_t astride)
{
    // release what we might have now, a must for external buffers
    release();
    
    associated_buffer = extobject;
    deleter = adeleter;
    memaccess = abuffer;
    width = awidth;
    height = aheight;
    pixfmt = apixfmt;
    bpp = PixelFormatToBytesPerPixel(pixfmt);
    if(astride == 0) { stride = width * PixelFormatToBytesPerPixelMap[(int)pixfmt]; } else { stride = astride; }
    data_size = stride * height;
}
