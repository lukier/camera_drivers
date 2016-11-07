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
 * KinectOne Driver.
 * ****************************************************************************
 */

#include <atomic>
#include <KinectOneDriver.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

struct drivers::camera::KinectOne::KOAPIPimpl
{
    KOAPIPimpl() : rgb_width(1920), rgb_height(1080), depth_width(512), depth_height(424), ir_width(512), ir_height(424), frame_types(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir)
    {

    }
    
    std::size_t rgb_width, rgb_height;
    std::size_t depth_width, depth_height;
    std::size_t ir_width, ir_height;
    
    libfreenect2::Freenect2 freenect2;
    std::unique_ptr<libfreenect2::Freenect2Device> dev;
    std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener;
    
    unsigned int frame_types;
};

struct FrameTuple
{
    FrameTuple() : count(3)  { }
    libfreenect2::FrameMap frames;
    std::atomic<unsigned int> count;
};

drivers::camera::KinectOne::KinectOne() : CameraDriverBase(), m_pimpl(new KOAPIPimpl()), is_running(false)
{
    
}

drivers::camera::KinectOne::~KinectOne()
{
    if(isOpened()) 
    { 
        close();
    }
}

void drivers::camera::KinectOne::image_release(void* img)
{
    FrameTuple* ft = static_cast<FrameTuple*>(img);
    ft->count--;
    if(ft->count == 0)
    {
        if(m_pimpl->listener.get() != nullptr)
        {
            m_pimpl->listener->release(ft->frames);
        }
        delete ft;
    }
}

void drivers::camera::KinectOne::open(unsigned int idx, bool depth, bool rgb, bool ir)
{
    m_pimpl->dev = std::unique_ptr<libfreenect2::Freenect2Device>(m_pimpl->freenect2.openDefaultDevice()); // TODO FIXME
    if(m_pimpl->dev.get() != nullptr)
    {
        if(depth)
        {
            m_pimpl->frame_types |= libfreenect2::Frame::Depth;
        }
        else
        {
            m_pimpl->frame_types &= ~libfreenect2::Frame::Depth;
        }
        
        if(rgb)
        {
            m_pimpl->frame_types |= libfreenect2::Frame::Color;
        }
        else
        {
            m_pimpl->frame_types &= ~libfreenect2::Frame::Color;
        }
        
        if(ir)
        {
            m_pimpl->frame_types |= libfreenect2::Frame::Ir;
        }
        else
        {
            m_pimpl->frame_types &= ~libfreenect2::Frame::Ir;
        }
        
        m_pimpl->listener = std::unique_ptr<libfreenect2::SyncMultiFrameListener>(new libfreenect2::SyncMultiFrameListener(m_pimpl->frame_types));
        m_pimpl->dev->setColorFrameListener(m_pimpl->listener.get());
        m_pimpl->dev->setIrAndDepthFrameListener(m_pimpl->listener.get());
    }
    else
    {
        throw std::runtime_error("Cannot open device");
    }
}

bool drivers::camera::KinectOne::isOpenedImpl() const
{
    return m_pimpl->dev.get() != nullptr;
}

void drivers::camera::KinectOne::close()
{
    if(m_pimpl->dev.get() != nullptr)
    {
        m_pimpl->dev->close();
        m_pimpl->dev.reset();
        m_pimpl->listener.reset();
    }
}

void drivers::camera::KinectOne::start()
{
    m_pimpl->dev->start();
}

void drivers::camera::KinectOne::stop()
{
    m_pimpl->dev->stop();
}

std::size_t drivers::camera::KinectOne::getRGBWidth() const { return m_pimpl->rgb_width; }
std::size_t drivers::camera::KinectOne::getRGBHeight() const { return m_pimpl->rgb_height; }
drivers::camera::EPixelFormat drivers::camera::KinectOne::getRGBPixelFormat() const { return EPixelFormat::PIXEL_FORMAT_RGB8; }

std::size_t drivers::camera::KinectOne::getDepthWidth() const { return m_pimpl->depth_width; }
std::size_t drivers::camera::KinectOne::getDepthHeight() const { return m_pimpl->depth_height; }
drivers::camera::EPixelFormat drivers::camera::KinectOne::getDepthPixelFormat() const { return EPixelFormat::PIXEL_FORMAT_DEPTH_F32_M; }

std::size_t drivers::camera::KinectOne::getIRWidth() const { return m_pimpl->ir_width; }
std::size_t drivers::camera::KinectOne::getIRHeight() const { return m_pimpl->ir_height; }
drivers::camera::EPixelFormat drivers::camera::KinectOne::getIRPixelFormat() const { return EPixelFormat::PIXEL_FORMAT_MONO32F; }

// DEPTH, RGB, Infrared
bool drivers::camera::KinectOne::captureFrameImpl(FrameBuffer* cf1, FrameBuffer* cf2, FrameBuffer* cf3, FrameBuffer* cf4, int64_t timeout)
{
    FrameTuple* ft = new FrameTuple();
    
    bool was_timeout = false;
    
    if(timeout == 0)
    {
        m_pimpl->listener->waitForNewFrame(ft->frames);
    }
    else
    {
        was_timeout = !m_pimpl->listener->waitForNewFrame(ft->frames, timeout / 1000000);
    }
    
    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds d = tp.time_since_epoch();
    
    if(was_timeout)
    {
        delete ft;
        return false;
    }
    
    libfreenect2::Frame *rgb = ft->frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = ft->frames[libfreenect2::Frame::Depth];
    libfreenect2::Frame *ir = ft->frames[libfreenect2::Frame::Ir];       
    
    cf1->create(ft, std::bind(&drivers::camera::KinectOne::image_release, this, std::placeholders::_1), depth->data, depth->width, depth->height, EPixelFormat::PIXEL_FORMAT_DEPTH_F32_M);
    cf1->setFrameCounter(depth->sequence);
    cf1->setTimeStamp(depth->timestamp * 10 * 1000000);
    cf1->setPCTimeStamp(d.count());
    
    cf2->create(ft, std::bind(&drivers::camera::KinectOne::image_release, this, std::placeholders::_1), rgb->data, rgb->width, rgb->height, EPixelFormat::PIXEL_FORMAT_RGB8);
    cf2->setFrameCounter(rgb->sequence);
    cf2->setTimeStamp(rgb->timestamp * 10 * 1000000);
    cf2->setPCTimeStamp(d.count());
    
    cf3->create(ft, std::bind(&drivers::camera::KinectOne::image_release, this, std::placeholders::_1), ir->data, ir->width, ir->height, EPixelFormat::PIXEL_FORMAT_MONO32F);
    cf3->setFrameCounter(ir->sequence);
    cf3->setTimeStamp(ir->timestamp * 10 * 1000000);
    cf3->setPCTimeStamp(d.count());
    
    return true;
}
