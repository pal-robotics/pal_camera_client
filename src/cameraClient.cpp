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


//PAL headers
#include <pal_camera_client/cameraClient.h>

//ROS headers
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//Boost headers
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//C++ headers
#include <exception>
#include <iostream>


namespace pal {

  /// @cond DOXYGEN_IGNORE
  class CameraClientImpl {

  public:

    CameraClientImpl(const std::string& imgTopic,
                     CameraClient::transport trans,
                     double timeout,
                     float maxRate,
                     const std::string& camInfoTopic);

    virtual ~CameraClientImpl();

    bool isThereAnyPublisher() const;

    void getCameraInfo(sensor_msgs::CameraInfo& camInfo);

    void getImage(cv::Mat& img);

    void pause();

    void unpause();

    bool isPaused() const;

  protected:

    void createSubscribers();
    void shutdownSubscribers();
    void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void spin();

    std::string _imgTopic;    
    CameraClient::transport _transport;
    double _timeoutSec;
    std::string _camInfoTopic;
    float _maxRate;

    cv::Mat _image;
    long _imageCounter, _lastGetImageId;

    sensor_msgs::CameraInfo _camInfo, _updatedCamInfo;
    long _cameraInfoCounter, _lastGetCameraInfoId;

    ros::NodeHandle _node;
    ros::CallbackQueue _cbQueue;

    boost::shared_ptr<image_transport::ImageTransport> _imageTransport;
    image_transport::Subscriber _imageSub;
    ros::Subscriber _camInfoSub;

    boost::shared_ptr< boost::thread > _spinThread;
    boost::mutex _guardImage, _guardCameraInfo;

    bool _spinRunning, _shutDown;
  };

  CameraClientImpl::CameraClientImpl(const std::string& imgTopic,
                                     CameraClient::transport trans,
                                     double timeout,
                                     float maxRate,
                                     const std::string& camInfoTopic):
    _imgTopic(imgTopic),
    _transport(trans),
    _timeoutSec(timeout),
    _camInfoTopic(camInfoTopic),
    _maxRate(maxRate),
    _imageCounter(0),
    _lastGetImageId(0),
    _cameraInfoCounter(0),
    _lastGetCameraInfoId(0),
    _node(ros::NodeHandle()),
    _spinRunning(false),
    _shutDown(false)
  {
    _node.setCallbackQueue(&_cbQueue);
    _imageTransport.reset( new image_transport::ImageTransport( _node ) );    

    createSubscribers();

    _spinThread.reset( new boost::thread( boost::bind(&CameraClientImpl::spin, this) ) );
  }

    static long imageCallbackCount = 0;
  CameraClientImpl::~CameraClientImpl()
  {
    _shutDown = true;

    ros::Time start = ros::Time::now();

    while ( _spinRunning && ros::ok() && (ros::Time::now() - start).toSec() < 5 )
      ros::Duration(0.1).sleep();

    if ( _spinRunning )
      ROS_ERROR("Error in CameraClientImpl::~CameraClientImpl: not possible to stop spin thread");

    shutdownSubscribers();
  }

  void CameraClientImpl::createSubscribers()
  {
    if ( _camInfoTopic != "" )
      _camInfoSub = _node.subscribe(_camInfoTopic, 1, &CameraClientImpl::cameraInfoCallback, this);

    std::string transportStr;

    if ( _transport == CameraClient::RAW )
      transportStr = "raw";
    else if ( _transport == CameraClient::JPEG )
      transportStr = "compressed";

    image_transport::TransportHints transportHint(transportStr);

    _imageSub = _imageTransport->subscribe(_imgTopic, 1, &CameraClientImpl::imageCallback, this, transportHint);
  }

  void CameraClientImpl::shutdownSubscribers()
  {
    if ( _camInfoTopic != "" )
      _camInfoSub.shutdown();

    _imageSub.shutdown();
  }

  void CameraClientImpl::pause()
  {
    if ( isPaused())
      return;

    _shutDown = true;

    ros::Time start = ros::Time::now();

    while ( _spinRunning && ros::ok() && (ros::Time::now() - start).toSec() < 5 )
      ros::Duration(0.1).sleep();

    if ( _spinRunning )
      throw std::runtime_error("Error in CameraClientImpl::pause: not possible to stop spin thread");

    _shutDown = false;

    shutdownSubscribers();

    _spinThread.reset();
  }

  void CameraClientImpl::unpause()
  {    
    if (isPaused())
    {
      createSubscribers();
      _spinThread.reset( new boost::thread( boost::bind(&CameraClientImpl::spin, this) ) );
    }
  }

  bool CameraClientImpl::isPaused() const
  {
    return _spinThread.get() == NULL;
  }

  static long count = 0;
  void CameraClientImpl::spin()
  {
    ros::WallDuration period( static_cast<double>(1.0/_maxRate) );
    ros::Rate rate(static_cast<double>(_maxRate));
    _spinRunning = true;

    //ROS_ERROR("***************** spin %fs", _maxRate);
    //ROS_ERROR("***************** loops %ld", count);
    //ROS_ERROR("***************** imageCallbackCount %ld", imageCallbackCount);
    while ( ros::ok() && !_shutDown )
    {
      count++;
      //we call it  twice, one for image and one for CameraInfoConstPtr
      _cbQueue.callOne( period );
      _cbQueue.callOne( period );
      rate.sleep();
    }
    //ROS_ERROR("***************** loops %ld", count);
    //ROS_ERROR("***************** imageCallbackCount %ld", imageCallbackCount);
    _spinRunning = false;
  }

  void CameraClientImpl::imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
  {
    imageCallbackCount++;
    cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvShare(imgMsg);
    //cv_bridge::CvImagePtr cvImgPtr;
    //cvImgPtr = cv_bridge::toCvCopy(imgMsg, imgMsg->encoding);

    {
      boost::mutex::scoped_lock lock(_guardImage);
      //@bugdo we ned to call clone?
      /*static bool firstTime = true;
      if (firstTime)
      {
        firstTime = false;
        _image = cvImgPtr->image.clone();
      }
      else*/
        cvImgPtr->image.copyTo(_image);
      ++_imageCounter;
    }
  }

  void CameraClientImpl::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(_guardCameraInfo);
    _camInfo = *msg;
    ++_cameraInfoCounter;
  }

  bool CameraClientImpl::isThereAnyPublisher() const
  {
    return _imageSub.getNumPublishers() > 0;
  }

  void CameraClientImpl::getCameraInfo(sensor_msgs::CameraInfo& camInfo)
  {
    if ( _spinThread.get() == NULL ) //camera client paused
    {
      camInfo = _camInfo;
      return;
    }

    ros::Time start = ros::Time::now();
    while ( _cameraInfoCounter == _lastGetCameraInfoId && ros::ok() )
    {
      if ( (ros::Time::now() - start).toSec() > _timeoutSec )
        throw std::runtime_error("Error in CameraClientImpl::getCameraInfo: timeout occurred");
    }

    {
      boost::mutex::scoped_lock lock(_guardCameraInfo);
      camInfo              = _camInfo;
      _lastGetCameraInfoId = _cameraInfoCounter;
    }
  }

  void CameraClientImpl::getImage(cv::Mat& img)
  {
    if ( _spinThread.get() == NULL ) //camera client paused
    {
      img = _image; //.clone();
      return;
    }

    ros::Time start = ros::Time::now();
    ros::Duration snooze(0.010);
    while ( _imageCounter == _lastGetImageId && ros::ok() )
    {
      if ( (ros::Time::now() - start).toSec() > _timeoutSec )
        throw std::runtime_error("Error in CameraClientImpl::getImage: timeout occurred");
      snooze.sleep();
    }

    {
      boost::mutex::scoped_lock lock(_guardImage);
      img             = _image; //.clone();
      _lastGetImageId = _imageCounter;
    }
  }
  /// @endcond


  //////////////////////////////////////////////////////////////

  CameraClient::CameraClient(const std::string& imgTopic,
                             CameraClient::transport trans,
                             double timeout,
                             float maxRate,
                             const std::string& camInfoTopic)
  {
    _impl.reset(new CameraClientImpl(imgTopic, trans, timeout, maxRate, camInfoTopic));
  }

  CameraClient::~CameraClient()
  {
    _impl.reset();
  }

  bool CameraClient::isThereAnyPublisher() const
  {
    return _impl->isThereAnyPublisher();
  }

  void CameraClient::getCameraInfo(sensor_msgs::CameraInfo& camInfo) const
  {
    _impl->getCameraInfo(camInfo);
  }

  void CameraClient::getImage(cv::Mat& img) const
  {
    _impl->getImage(img);
  }

  void CameraClient::pause()
  {
    _impl->pause();
  }

  void CameraClient::unpause()
  {
    _impl->unpause();
  }

  bool CameraClient::isPaused() const
  {
    return _impl->isPaused();
  }

} //pal
