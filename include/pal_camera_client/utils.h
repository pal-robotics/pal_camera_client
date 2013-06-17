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

#ifndef _PAL_CAMERA_CLIENT_UTILS_H_
#define _PAL_CAMERA_CLIENT_UTILS_H_

//ROS headers
#include <ros/message_forward.h>

//STL headers
#include <vector>

namespace sensor_msgs {
  ROS_DECLARE_MESSAGE(CameraInfo);
}

namespace pal {

  /**
   * Helper functions to get intrinsic parameters out of sensor_msgs::CameraInfo
   */
  namespace cameraInfo {

    /**
     * @brief getImageWidth get the image width in pixels
     * @param[in] camInfo
     * @return
     */
    int getImageWidth(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getImageHeight get the image height in pixels
     * @param[in] camInfo
     * @return
     */
    int getImageHeight(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getFx get the horizontal focal distance in pixels
     * @param[in] camInfo
     * @return
     */
    double getFx(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getFy get the vertical focal distance in pixels
     * @param[in] camInfo
     * @return
     */
    double getFy(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getCx get the horizontal coordinate of the optical center in pixels
     * @param[in] camInfo
     * @return
     */
    double getCx(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getCy get the vertical coordinate of the optical center in pixels
     * @param[in] camInfo
     * @return
     */
    double getCy(const sensor_msgs::CameraInfo& camInfo);
    /**
     * @brief getDistortion get the distortion coefficients of the camera \f$k1,k2,p1,p2,k3,[k4,k5,k6]\f$
     *                      where \f$k_i\f$ are radial distortion coefficients and \f$p_j\f$ are tangential distortion coefficients
     * @param[in] camInfo
     * @return
     */
    std::vector<double> getDistortion(const sensor_msgs::CameraInfo& camInfo);

  } //cameraInfo
} //pal

#endif //_PAL_CAMERA_CLIENT_UTILS_H_
