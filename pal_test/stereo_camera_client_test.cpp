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

// PAL headers
#include <pal_camera_client/stereoCameraClient.h>
#include <pal_camera_client/utils.h>
#include <pal_camera_publisher/camera_dummy.h>
#include <pal_camera_publisher/cameraPublisher.h>
#include <pal_core/util/string.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>

// Boost headers
#include <boost/thread.hpp>

// Gtest headers
#include <gtest/gtest.h>

bool isInteractive;

TEST(stereo_camera_client, test_stereo_camera_client)
{    
  double testDurationInSec = 20;
  double maxRate = 10;

  pal::StereoCameraClient camClient("/camera_dummy_to_test_client/left/image",
                                    "/camera_dummy_to_test_client/right/image",
                                    pal::StereoCameraClient::JPEG,
                                    pal::StereoCameraClient::EXACT_TIME,
                                    0.5,
                                    static_cast<float>(maxRate*2), //the CameraClient will check callbacks at this rate
                                    "/camera_dummy_to_test_client/left/camera_info",
                                    "/camera_dummy_to_test_client/right/camera_info");

  cv::Mat imgL, imgR;
  sensor_msgs::CameraInfo camInfoL, camInfoR;

  ros::Rate rate(maxRate);
  ros::Time start = ros::Time::now();
  while ( ros::ok() && (ros::Time::now() - start).toSec() < testDurationInSec )
  {    
    camClient.getImages(imgL, imgR);
    camClient.getCameraInfos(camInfoL, camInfoR);

    EXPECT_EQ( pal::cameraInfo::getImageWidth(camInfoR), 640 );
    EXPECT_EQ( pal::cameraInfo::getImageHeight(camInfoR), 480 );
    EXPECT_EQ( pal::cameraInfo::getFx(camInfoR), 370 );
    EXPECT_EQ( pal::cameraInfo::getFy(camInfoR), 371 );
    EXPECT_EQ( pal::cameraInfo::getCx(camInfoR), 376 );
    EXPECT_EQ( pal::cameraInfo::getCy(camInfoR), 240 );

    EXPECT_FLOAT_EQ( pal::cameraInfo::getDistortion(camInfoR)[0],  0.30000); //k1
    EXPECT_FLOAT_EQ( pal::cameraInfo::getDistortion(camInfoR)[1], -0.12000); //k1
    EXPECT_FLOAT_EQ( pal::cameraInfo::getDistortion(camInfoR)[2],  0.00520); //p1
    EXPECT_FLOAT_EQ( pal::cameraInfo::getDistortion(camInfoR)[3], -0.01090); //p2
    EXPECT_FLOAT_EQ( pal::cameraInfo::getDistortion(camInfoR)[4],  0.00038); //k3

    if ( isInteractive )
    {
      cv::imshow("image left",  imgL);
      cv::imshow("image right", imgR);
      cv::waitKey(15);
    }

    rate.sleep();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest");

  isInteractive = argc > 1 && pal::util::string::equalIgnoreCase(argv[1], "interactive");

  std::cout<< std::endl << std::endl << "NUMBER OF ARGS: " << argc << std::endl << std::endl;

  ros::NodeHandle nh;

  ros::Time::waitForValid();

  pal::dummy::Camera camera[2];

  camera[0].setImageSize(640, 480);
  camera[0].setNameSpace("/camera_dummy_to_test_client");
  camera[0].setImageTopic("left/image");
  camera[0].setDummyImage( ros::package::getPath("pal_camera_publisher") + "/etc/pal_logo_left.jpg");

  camera[1].setImageSize(640, 480);
  camera[1].setNameSpace("/camera_dummy_to_test_client");
  camera[1].setImageTopic("right/image");
  camera[1].setDummyImage( ros::package::getPath("pal_camera_publisher") + "/etc/pal_logo_right.jpg");

  pal::CameraPublisher camPub;

  std::string path = ros::package::getPath("pal_camera_publisher") + "/pal_test/";
  camPub.addCamera(camera[0], path);
  camPub.addCamera(camera[1], path);

  camPub.setRate(25);
  camPub.start(false); //non-blocking publisher

  return RUN_ALL_TESTS();
}

