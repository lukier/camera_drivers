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
 * Quick Test.
 * ****************************************************************************
 */

#include <cstdint>
#include <cstddef>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <FireWireDriver.hpp>
#include <KinectOneDriver.hpp>
#include <OpenNIDriver.hpp>
#include <PointGreyDriver.hpp>
#include <RealSenseDriver.hpp>
#include <RealSense2Driver.hpp>
#include <V4LDriver.hpp>
#include <VRMagicDriver.hpp>

struct Intrinsics
{
    float fx, fy;
    float u0, v0;
    std::array<float,5> dist;
    
    friend std::ostream& operator<< (std::ostream& stream, const Intrinsics& matrix) 
    {
      stream << matrix.fx << " , " << matrix.fy << " | " << matrix.u0 << " , " << matrix.v0 << " [ " 
        << matrix.dist[0] << ","
        << matrix.dist[1] << ","
        << matrix.dist[2] << ","
        << matrix.dist[3] << ","
        << matrix.dist[4] << ","
        << " ]";
        return stream;
    }
};

template<typename T> struct __attribute__ ((__packed__)) Triplet { T x,y,z; };

using uchar3 = Triplet<uint8_t>;

template<typename T>
static inline T clamp(const T& v, const T clamp_min, const T clamp_max)
{
    return std::max(clamp_min, std::min(v, clamp_max));  
}

template<typename T>
static inline T clampRescale(const T& v, const T vmin, const T vmax, const T clamp_min, const T clamp_max)
{
    const T alpha = T(1.0f) / (vmax - vmin);
    const T beta = -vmin * (T(1.0)/(vmax - vmin));
    
    return clamp(v * alpha + beta, clamp_min, clamp_max); 
}


static constexpr std::size_t WidthRGB = 1280;
static constexpr std::size_t HeightRGB = 720;

static constexpr std::size_t WidthDepth = 1280;
static constexpr std::size_t HeightDepth = 720;

static void convertRGB(const drivers::camera::FrameBuffer& fb, cv::Mat& img_out)
{
    for(std::size_t y = 0 ; y < fb.getHeight() ; ++y)
    {
        for(std::size_t x = 0 ; x < fb.getWidth() ; ++x)
        {
            const uchar3& pix_in = fb.getPixel<uchar3>(x,y);
            uchar3& pix_out = img_out.at<uchar3>(y,x);
            pix_out.x = pix_in.z;
            pix_out.y = pix_in.y;
            pix_out.z = pix_in.x;
        }
    }
}

static void convertDepth(const drivers::camera::FrameBuffer& fb, cv::Mat& img_out, float depthScale = 1.0)
{
    float dsum = 0.0, dcount = 0.0;
    float dmin = std::numeric_limits<float>::max(), dmax = -std::numeric_limits<float>::max();
    
    for(std::size_t y = 0 ; y < fb.getHeight() ; ++y)
    {
        for(std::size_t x = 0 ; x < fb.getWidth() ; ++x)
        {
            const uint16_t& depth_in = fb.getPixel<uint16_t>(x,y);
            
            if(depth_in > 0)
            {
                const float dth = float(depth_in) * depthScale;
                
                dsum += dth;
                dcount += 1.0f;
                
                dmin = std::min(dth, dmin);
                dmax = std::max(dth, dmax);
            }
        }
    }
    
    dmin = 0.0f;
    dmax = 12.0f;
    
    for(std::size_t y = 0 ; y < fb.getHeight() ; ++y)
    {
        for(std::size_t x = 0 ; x < fb.getWidth() ; ++x)
        {
            const uint16_t& depth_in = fb.getPixel<uint16_t>(x,y);
            uint8_t& pix_out = img_out.at<uint8_t>(y,x);
            
            if(depth_in > 0)
            {   
                const float d = float(depth_in) * depthScale;
                
                pix_out = (uint8_t)(clampRescale(d,dmin,dmax,0.0f,1.0f) * 255.0f);
            }
            else
            {
                pix_out = 0;
            }
              
        }
    }
}

extern "C" int main(int argc, char** argv)
{
    typedef drivers::camera::RealSense2 CamDriverT;
    
    CamDriverT camera;
    
    cv::namedWindow("Image");
    cv::namedWindow("Depth");
    
    camera.open();
    
    camera.setDepthMode(WidthDepth,HeightDepth,30,true);
    camera.setRGBMode(WidthRGB,HeightRGB,30,drivers::camera::EPixelFormat::PIXEL_FORMAT_RGB8,false);
    
    camera.start();
    
    std::cerr << "IsO: " << camera.isOpened() << " and " << camera.isStarted() << std::endl;
    
    cv::Mat img_rgb(HeightRGB, WidthRGB, CV_8UC3);
    cv::Mat img_depth(HeightDepth, WidthDepth, CV_8UC1);
    
    const float ds = camera.getDepthScale();
    
    std::cerr << "Depth Scale: " << ds << " or " << std::endl;
    
    Intrinsics i_rgb, i_depth;
    
    camera.getRGBIntrinsics(i_rgb.fx,i_rgb.fy,i_rgb.u0,i_rgb.v0,&i_rgb.dist);
    camera.getDepthIntrinsics(i_depth.fx,i_depth.fy,i_depth.u0,i_depth.v0,&i_depth.dist);
    
    std::cerr << "RGB Intrinsics: " << i_rgb << std::endl;
    std::cerr << "Depth Intrinsics: " << i_depth << std::endl;
    
    while(1)
    {
        drivers::camera::FrameBuffer fb_rgb, fb_depth;
        
        if(camera.captureFrame(fb_depth, fb_rgb,std::chrono::seconds(1)))
        {
            convertRGB(fb_rgb, img_rgb);
            convertDepth(fb_depth, img_depth, ds);
            //std::cerr << "OK" << std::endl;
        }
        else
        {
            //std::cerr << "Fail" << std::endl;
        }
        
        cv::imshow("Image", img_rgb);
        cv::imshow("Depth", img_depth);
        
        int key = cv::waitKey(5) & 0xFF;
        
        if((key == 27) || (key == 1048603)) { break; }
    }
    
    camera.stop();
    
    cv::destroyAllWindows();
    
    return 0;
}
