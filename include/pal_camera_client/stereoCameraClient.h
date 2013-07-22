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

#ifndef _PAL_STEREO_CAMERA_CLIENT_H_
#define _PAL_STEREO_CAMERA_CLIENT_H_

//ROS headers
#include <ros/message_forward.h>

//Boost headers
#include <boost/shared_ptr.hpp>

//STL headers
#include <string>
#include <vector>

namespace sensor_msgs {
  ROS_DECLARE_MESSAGE(CameraInfo);
}

namespace cv {
  class Mat;
}

namespace pal {

  class StereoCameraClientImpl;

/**
 * @class StereoCameraClient stereoCameraClient.h "pal_camera_client/stereoCameraClient.h"
 * @author Jordi Pages
 *
 * @brief The StereoCameraClient class provides a simple way to get on demand images and intrinsic parameters from a stereo camera
 *        published in ROS topics
 *
 * The class uses ROS camera subscribers with the required callbacks running in a separate thread. Image topics from the two cameras composing
 * the stereo rig are synchronized according to the user selected strategy. Then, functions are provided to
 * get the latest images and intrinsic parameters using mutual exclusion with the callback functions.
 */
  class StereoCameraClient {

  public:

    /**
     * @enum stereoEye
     * @brief enum to identify each camera in the stereo rig with a logical name referring to its position within the stereo rig
     */
    enum stereoEye { LEFT = 0, RIGHT = 1 };

    /**
     * @enum transport
     * @brief The image transports supported                   \n
     *                                                         \n
     *   RAW: subscribe to uncompressed image topic            \n
     *   JPEG: subscribe to image topic using jpeg compression \n
     */
    enum transport { RAW = 0, JPEG = 1 };

    /**
     * @enum synchronization
     * @brief The synchronization enum
     */
    enum synchronization { EXACT_TIME = 0, APPROX_TIME = 1 };

    /**
      * @brief CameraClient constructor
      * @param[in] imgTopicL name of the ROS topic where the left image is published
      * @param[in] imgTopicR name of the ROS topic where the right image is published
      * @param[in] trans type of image transport. See StereoCameraClient::transport
      * @param[in] sync type of topic messages synchronization. See StereoCameraClient::synchronization
      * @param[in] timeout timeout in seconds when calling CameraClient::updateImage and CameraClient::updateCameraInfo
      * @param[in] maxRate max rate at which this client must have to provide images in Hz.
      * @param[in] camInfoTopicL name of the ROS topic with the sensor_msgs::CameraInfo, i.e. intrinsic parameters, of the left camera
      * @param[in] camInfoTopicR name of the ROS topic with the sensor_msgs::CameraInfo, i.e. intrinsic parameters, of the right camera
      */
    StereoCameraClient(const std::string& imgTopicL,
                       const std::string& imgTopicR,
                       transport trans = RAW,
                       synchronization sync = APPROX_TIME,
                       double timeout = 5,
                       float maxRate = 10,
                       const std::string& camInfoTopicL = "",
                       const std::string& camInfoTopicR = "");

    /**
     * @brief ~CameraClient destructor. ROS subscribers are shutdown.
     */
    virtual ~StereoCameraClient();

    /**
     * @brief isThereAnyPublisher checks if the image topics are being published
     * @return true if the image topics are being published.
     */
    bool isThereAnyPublisher() const;

    /**
     * @brief getCameraInfo get the sensor_msgs::CameraInfo kept by the latest call to CameraClient::updateCameraInfo
     * @param[out] camInfoL left camera info
     * @param[out] camInfoR right camera info
     * @throws std::runtime_error if StereoCameraClient::updateCameraInfo has never been called
     */
    void getCameraInfos(sensor_msgs::CameraInfo& camInfoL,
                        sensor_msgs::CameraInfo& camInfoR) const;

    /**
     * @brief getCameraInfo get the sensor_msgs::CameraInfo of the given eye kept by the latest call to CameraClient::updateCameraInfo
     * @param[in] eye specifies the camera
     * @param[out] camInfo camera info of the specified camera
     * @throws std::runtime_error if StereoCameraClient::updateCameraInfo has never been called
     */
    void getCameraInfo(stereoEye eye, sensor_msgs::CameraInfo& camInfo) const;

    /**
     * @brief getImage get the images kept by the last call to StereoCameraClient::updateImage
     * @param[out] imgL left image
     ** @param[out] imgR right image
     * @throws std::runtime_error if CameraClient::updateImage has never been called
     */
    void getImages(cv::Mat& imgL, cv::Mat& imgR) const;

    /**
     * @brief getImage get the image of the specified eye kept by the last call to StereoCameraClient::updateImage
     * @param eye specifies the camera
     * @param img image
     */
    void getImage(stereoEye eye, cv::Mat& img) const;

  private:

    /**
     * @brief _impl class implementation
     */
    boost::shared_ptr<StereoCameraClientImpl> _impl;
  };

} //pal

#endif //_PAL_STEREO_CAMERA_CLIENT_H_
