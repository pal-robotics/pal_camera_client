/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PAL_CAMERA_CLIENT_H_
#define _PAL_CAMERA_CLIENT_H_

//ROS headers
#include <sensor_msgs/CameraInfo.h>

//Boost headers
#include <boost/shared_ptr.hpp>

//STL headers
#include <string>
#include <vector>

namespace cv {
class Mat;
}

namespace pal {

class CameraClientImpl;

/**
 * @class CameraClient cameraClient.h "pal_camera_client/CameraClient.h"
 * @author Jordi Pages
 *
 * @brief The CameraClient class provides a simple way to get on demand image and intrinsic parameters from a ROS camera publisher.
 *
 * The class uses a ROS camera subscriber with the required callbacks running in a separate thread. Then, functions are provided to
 * get the latest image and intrinsic parameters using mutual exclusion with the callback functions.
 */
class CameraClient {

public:        

    /**
     * @enum transport
     * @brief The image transports supported
     */
    enum transport { RAW = 0, JPEG = 1 };

    /**      
     * @brief CameraClient constructor
     * @param[in] imgTopic name of the ROS topic where the image is published
     * @param[in] trans type of image transport. See CameraClient::transport
     * @param[in] timeout timeout in seconds when calling CameraClient::updateImage and CameraClient::updateCameraInfo
     * @param[in] maxRate max rate at which this client must have to provide images in Hz.
     * @param[in] camInfoTopic name of the ROS topic where the sensor_msgs::CameraInfo, i.e. intrinsic parameters, of the camera
     */
    CameraClient(const std::string& imgTopic,
                 transport trans = RAW,
                 double timeout = 0.5,
                 float maxRate = 10,
                 const std::string& camInfoTopic = "");

    /**
     * @brief ~CameraClient destructor. ROS subscribers are shutdown.
     */
    virtual ~CameraClient();

    /**
     * @brief isThereAnyPublisher checks if the image topic is being published
     * @return true if the image topic is being published.
     */
    bool isThereAnyPublisher() const;

    /**
     * @brief updateCameraInfo commands to keep an internal copy of the latest sensors_msgs::CameraInfo obtained by the corresponding ROS
     *                         subscriber callback function
     * @throws std::runtime_error if the timeout specified in CameraClient::CameraClient occurs
     */
    void updateCameraInfo();

    /**
     * @brief getCameraInfo get the sensor_msgs::CameraInfo kept by the latest call to CameraClient::updateCameraInfo
     * @param[out] camInfo
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    void getCameraInfo(sensor_msgs::CameraInfo& camInfo) const;

    /**
     * @brief updateImage commands to keep an internal copy of the latest image obtained by the corresponding ROS subscriber callback function
     * @throws std::runtime_error if the timeout specified in CameraClient::CameraClient occurs
     */
    void updateImage();    

    /**
     * @brief getImage get the image kept by the last call to CameraClient::updateImage
     * @param[out] img OpenCV image
     * @throws std::runtime_error if CameraClient::updateImage has never been called
     */
    void getImage(cv::Mat& img) const;

    /**
     * @brief getImageWidth
     * @return the image width in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    int getImageWidth() const;

    /**
     * @brief getImageHeight
     * @return the image height in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    int getImageHeight() const;

    /**
     * @brief getFx
     * @return the horizontal focal distance of the camera in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    double getFx() const;

    /**
     * @brief getFy
     * @return the vertical focal distance of the camera in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    double getFy() const;

    /**
     * @brief getCx
     * @return the horizontal position of the optical center in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    double getCx() const;

    /**
     * @brief getCy
     * @return the vertical position of the optical center in pixels
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    double getCy() const;

    /**
     * @brief getDistortion get the distortion coefficients of the camera being \f$k1,k2,p1,p2,k3,[k4,k5,k6]\f$
     *                      where \f$k_i\f$ are radial distortion coefficients and \f$p_j\f$ are tangential distortion coefficients
     * @param[out] distortion the distortion coefficents.
     * @throws std::runtime_error if CameraClient::updateCameraInfo has never been called
     */
    void getDistortion(std::vector<double>& distortion) const;

private:

    /**
     * @brief _impl class implementation
     */
    boost::shared_ptr<CameraClientImpl> _impl;
};

} //pal

#endif
