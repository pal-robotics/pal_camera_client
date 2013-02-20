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
#include <pal_camera_client/cameraClient.h>
#include <pal_camera_publisher/camera_dummy.h>
#include <pal_camera_publisher/publisher.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>

// Boost headers
#include <boost/thread.hpp>

// Gtest headers
#include <gtest/gtest.h>


void publisherThread()
{
  pal::dummy::Camera camera;

  camera.setImageSize(640, 480);
  camera.setNameSpace("/camera_dummy_to_test_client");

  pal::CameraPublisher camPub;

  std::string path = ros::package::getPath("pal_camera_publisher") + "/pal_test/";
  camPub.addCamera(camera, path);

  camPub.setRate(25);
  camPub.start();
}


TEST(camera_client, test_camera_client)
{    
    double testDurationInSec = 20;
    double maxRate = 5;

    pal::CameraClient camClient("/camera_dummy_to_test_client/image",
                                pal::CameraClient::RAW,
                                0.5,
                                static_cast<float>(maxRate*2), //the CameraClient will check callbacks at this rate
                                "/camera_dummy_to_test_client/camera_info");

    cv::Mat img;
    sensor_msgs::CameraInfo camInfo;

    cv::namedWindow("image");

    int period = static_cast<int>(1000.0/maxRate);

    ros::Rate rate(maxRate);
    ros::Time start = ros::Time::now();
    while ( ros::ok() && (ros::Time::now() - start).toSec() < testDurationInSec )
    {
        camClient.updateImage();
        camClient.getImage(img);

        camClient.updateCameraInfo();
        camClient.getCameraInfo(camInfo);

        EXPECT_EQ( camClient.getImageWidth(), 640 );
        EXPECT_EQ( camClient.getImageHeight(), 480 );
        EXPECT_EQ( camClient.getFx(), 370 );
        EXPECT_EQ( camClient.getFy(), 371 );
        EXPECT_EQ( camClient.getCx(), 376 );
        EXPECT_EQ( camClient.getCy(), 240 );

        std::vector<double> distortion;
        camClient.getDistortion(distortion);

        EXPECT_FLOAT_EQ( distortion[0],  0.30000); //k1
        EXPECT_FLOAT_EQ( distortion[1], -0.12000); //k1
        EXPECT_FLOAT_EQ( distortion[2],  0.00520); //p1
        EXPECT_FLOAT_EQ( distortion[3], -0.01090); //p2
        EXPECT_FLOAT_EQ( distortion[4],  0.00038); //k3

        cv::imshow("image", img);
        cv::waitKey(15);

        rate.sleep();
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "utest");

    ros::NodeHandle nh;

    ros::Time::waitForValid();

    boost::thread cameraPublisherThread(publisherThread);

    return RUN_ALL_TESTS();
}

