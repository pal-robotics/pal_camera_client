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
#include <pal_camera_client/stereoCameraClient.h>

//ROS headers
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Boost headers
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//C++ headers
#include <exception>


namespace pal {

  /// @cond DOXYGEN_IGNORE
  class StereoCameraClientImpl {

  public:

    StereoCameraClientImpl(const std::string& imgTopicL,
                           const std::string& imgTopicR,
                           StereoCameraClient::transport trans,
                           StereoCameraClient::synchronization sync,
                           double timeout,
                           float maxRate,
                           const std::string& camInfoTopicL,
                           const std::string& camInfoTopicR);


    virtual ~StereoCameraClientImpl();

    bool isThereAnyPublisher() const;

    void getCameraInfos(sensor_msgs::CameraInfo& camInfoL, sensor_msgs::CameraInfo& camInfoR);
    void getCameraInfo(StereoCameraClient::stereoEye eye, sensor_msgs::CameraInfo& camInfo);

    void getImages(cv::Mat& imgL, cv::Mat& imgR);
    void getImage(StereoCameraClient::stereoEye eye, cv::Mat& img);

  protected:

    void stereoCallback(const sensor_msgs::ImageConstPtr& msgImgL,
                        const sensor_msgs::ImageConstPtr& msgImgR,
                        const sensor_msgs::CameraInfoConstPtr& camInfoL,
                        const sensor_msgs::CameraInfoConstPtr& camInfoR);

    void updateDataFromTopics();

    void spin();

    std::string _imgTopicL, _imgTopicR;
    StereoCameraClient::transport _transport;
    StereoCameraClient::synchronization _sync;
    double _timeoutSec;
    std::string _camInfoTopicL, _camInfoTopicR;
    float _maxRate;

    cv::Mat _imageL, _imageR;
    sensor_msgs::CameraInfo _camInfoL, _camInfoR;
    long _dataCounter, _lastGetData;

    ros::NodeHandle _node;
    ros::CallbackQueue _cbQueue;

    boost::shared_ptr<image_transport::ImageTransport> _imageTransport;

    boost::shared_ptr< image_transport::SubscriberFilter > _imgSubL, _imgSubR;
    boost::shared_ptr< message_filters::Subscriber<sensor_msgs::CameraInfo> > _camInfoSubL, _camInfoSubR;

    typedef message_filters::TimeSynchronizer<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo> ExactSync;

    boost::shared_ptr< ExactSync > _exactSynchronizer;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo> ApproxSync;

    boost::shared_ptr< message_filters::Synchronizer<ApproxSync> > _approxSynchronizer;

    boost::shared_ptr< boost::thread > _spinThread;
    boost::mutex _guard;

    bool _spinRunning, _shutDown;
  };

  StereoCameraClientImpl::StereoCameraClientImpl(const std::string& imgTopicL,
                                                 const std::string& imgTopicR,
                                                 StereoCameraClient::transport trans,
                                                 StereoCameraClient::synchronization sync,
                                                 double timeout,
                                                 float maxRate,
                                                 const std::string& camInfoTopicL,
                                                 const std::string& camInfoTopicR):
    _imgTopicL(imgTopicL),
    _imgTopicR(imgTopicR),
    _transport(trans),
    _sync(sync),
    _timeoutSec(timeout),
    _camInfoTopicL(camInfoTopicL),
    _camInfoTopicR(camInfoTopicR),
    _maxRate(maxRate),
    _dataCounter(0),
    _lastGetData(0),
    _node(ros::NodeHandle()),
    _spinRunning(false),
    _shutDown(false)
  {
    if ( _imgTopicL == "" || _imgTopicR == "" || _camInfoTopicL == "" || _camInfoTopicR == "" )
      throw std::runtime_error("Error in StereoCameraClientImpl::StereoCameraClientImpl: the topic names cannot be empty");

    _node.setCallbackQueue(&_cbQueue);
    _imageTransport.reset( new image_transport::ImageTransport( _node ) );
    std::string transportStr;

    if ( trans == StereoCameraClient::RAW )
      transportStr = "raw";
    else if ( trans == StereoCameraClient::JPEG )
      transportStr = "compressed";

    image_transport::TransportHints transportHint(transportStr);

    _imgSubL.reset(new image_transport::SubscriberFilter(*_imageTransport, _imgTopicL, 1, transportHint) );
    _imgSubR.reset(new image_transport::SubscriberFilter(*_imageTransport, _imgTopicR, 1, transportHint) );
    _camInfoSubL.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(_node, _camInfoTopicL, 1) );
    _camInfoSubR.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(_node, _camInfoTopicR, 1) );


    if ( sync == StereoCameraClient::EXACT_TIME )
    {
      _exactSynchronizer.reset( new ExactSync(*_imgSubL, *_imgSubR,
                                              *_camInfoSubL, *_camInfoSubR, 1) );

      _exactSynchronizer->registerCallback(boost::bind(&StereoCameraClientImpl::stereoCallback, this, _1, _2, _3, _4));

    }
    else if ( sync == StereoCameraClient::APPROX_TIME )
    {
      _approxSynchronizer.reset( new message_filters::Synchronizer< ApproxSync >(ApproxSync(4),
                                                                                 *_imgSubL, *_imgSubR,
                                                                                 *_camInfoSubL, *_camInfoSubR) );

      _approxSynchronizer->registerCallback(boost::bind(&StereoCameraClientImpl::stereoCallback, this, _1, _2, _3, _4));
    }

    _spinThread.reset( new boost::thread( boost::bind(&StereoCameraClientImpl::spin, this) ) );
  }

  StereoCameraClientImpl::~StereoCameraClientImpl()
  {
    _shutDown = true;

    ros::Time start = ros::Time::now();

    while ( _spinRunning && ros::ok() && (ros::Time::now() - start).toSec() < 5 )
      ros::Duration(0.1).sleep();

    if ( _spinRunning )
      ROS_ERROR("Error in StereoCameraClientImpl::~StereoCameraClientImpl: not possible to stop spin thread");

  }

  void StereoCameraClientImpl::spin()
  {
    ros::WallDuration period( static_cast<double>(1.0/_maxRate) );
    ros::Rate rate(static_cast<double>(_maxRate));
    _spinRunning = true;
    while ( ros::ok() && !_shutDown )
    {
      _cbQueue.callAvailable( period );
      rate.sleep();
    }
    _spinRunning = false;
  }

  void StereoCameraClientImpl::stereoCallback(const sensor_msgs::ImageConstPtr& msgImgL,
                                              const sensor_msgs::ImageConstPtr& msgImgR,
                                              const sensor_msgs::CameraInfoConstPtr& camInfoL,
                                              const sensor_msgs::CameraInfoConstPtr& camInfoR)
  {
    cv_bridge::CvImagePtr cvImgPtrL, cvImgPtrR;

    cvImgPtrL = cv_bridge::toCvCopy(msgImgL, msgImgL->encoding);
    cvImgPtrR = cv_bridge::toCvCopy(msgImgR, msgImgR->encoding);

    boost::mutex::scoped_lock lock(_guard);
    _imageL = cvImgPtrL->image.clone();
    _imageR = cvImgPtrR->image.clone();
    _camInfoL = *camInfoL;
    _camInfoR = *camInfoR;
    ++_dataCounter;
  }

  bool StereoCameraClientImpl::isThereAnyPublisher() const
  {
    return _imgSubL->getNumPublishers() > 0 || _imgSubR->getNumPublishers() > 0;
  }

  void StereoCameraClientImpl::updateDataFromTopics()
  {
    ros::Time start = ros::Time::now();
    while ( _dataCounter == _lastGetData && ros::ok() )
    {
      if ( (ros::Time::now() - start).toSec() > _timeoutSec )
        throw std::runtime_error("Error in StereoCameraClientImpl::updateDataFromTopics: timeout occurred");
    }
  }

  void StereoCameraClientImpl::getCameraInfos(sensor_msgs::CameraInfo& camInfoL, sensor_msgs::CameraInfo& camInfoR)
  {
    updateDataFromTopics();

    boost::mutex::scoped_lock lock(_guard);
    camInfoL  = _camInfoL;
    camInfoR  = _camInfoR;
    _lastGetData = _dataCounter;
  }

  void StereoCameraClientImpl::getCameraInfo(StereoCameraClient::stereoEye eye, sensor_msgs::CameraInfo& camInfo)
  {
    updateDataFromTopics();

    boost::mutex::scoped_lock lock(_guard);

    if ( eye == StereoCameraClient::LEFT )
      camInfo = _camInfoL;
    else if ( eye == StereoCameraClient::RIGHT )
      camInfo = _camInfoR;
    else
      throw std::runtime_error("Error in StereoCameraClientImpl::getCameraInfo: unsupported eye parameter");
  }

  void StereoCameraClientImpl::getImages(cv::Mat& imgL, cv::Mat& imgR)
  {
    updateDataFromTopics();

    boost::mutex::scoped_lock lock(_guard);

    imgL = _imageL.clone();
    imgR = _imageR.clone();
  }


  void StereoCameraClientImpl::getImage(StereoCameraClient::stereoEye eye, cv::Mat& img)
  {
    updateDataFromTopics();

    boost::mutex::scoped_lock lock(_guard);

    if ( eye == StereoCameraClient::LEFT )
      img = _imageL.clone();
    else if ( eye == StereoCameraClient::RIGHT )
      img = _imageR.clone();
    else
      throw std::runtime_error("Error in StereoCameraClientImpl::getImage: unsupported eye parameter");
  }

  /// @endcond


  //////////////////////////////////////////////////////////////

  StereoCameraClient::StereoCameraClient(const std::string& imgTopicL,
                                         const std::string& imgTopicR,
                                         StereoCameraClient::transport trans,
                                         StereoCameraClient::synchronization sync,
                                         double timeout,
                                         float maxRate,
                                         const std::string& camInfoTopicL,
                                         const std::string& camInfoTopicR)
  {
    _impl.reset( new StereoCameraClientImpl(imgTopicL, imgTopicR,
                                             trans, sync,
                                             timeout, maxRate,
                                             camInfoTopicL, camInfoTopicR) );
  }

  StereoCameraClient::~StereoCameraClient()
  {
    _impl.reset();
  }

  bool StereoCameraClient::isThereAnyPublisher() const
  {
    return _impl->isThereAnyPublisher();
  }

  void StereoCameraClient::getCameraInfos(sensor_msgs::CameraInfo& camInfoL, sensor_msgs::CameraInfo& camInfoR) const
  {
    _impl->getCameraInfos(camInfoL, camInfoR);
  }


  void StereoCameraClient::getCameraInfo(StereoCameraClient::stereoEye eye, sensor_msgs::CameraInfo& camInfo) const
  {
    _impl->getCameraInfo(eye, camInfo);
  }

  void StereoCameraClient::getImages(cv::Mat& imgL, cv::Mat& imgR) const
  {
    _impl->getImages(imgL, imgR);
  }

  void StereoCameraClient::getImage(StereoCameraClient::stereoEye eye, cv::Mat& img) const
  {
    _impl->getImage(eye, img);
  }

}

