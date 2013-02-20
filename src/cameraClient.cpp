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
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//Boost headers
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//C++ headers
#include <exception>


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

    void updateCameraInfo();

    void getCameraInfo(sensor_msgs::CameraInfo& camInfo) const;

    void updateImage();

    void getImage(cv::Mat& img) const;

    int getImageWidth() const;
    int getImageHeight() const;
    double getFx() const;
    double getFy() const;
    double getCx() const;
    double getCy() const;
    void getDistortion(std::vector<double>& distortion) const;

protected:

    void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void spin();

    std::string _imgTopic;
    CameraClient::transport _transport;
    double _timeoutSec;
    std::string _camInfoTopic;
    bool _camInfoAvailable;
    bool _imageAvailable;
    float _maxRate;

    cv::Mat _image, _updatedImage;
    long _imageCounter, _updatedImageId;

    sensor_msgs::CameraInfo _camInfo, _updatedCamInfo;
    long _cameraInfoCounter, _updatedCameraInfoId;

    ros::NodeHandle _node;
    ros::CallbackQueue _cbQueue;

    boost::shared_ptr<image_transport::ImageTransport> _imageTransport;
    image_transport::Subscriber _imageSub;
    ros::Subscriber _camInfoSub;

    boost::shared_ptr< boost::thread > _spinThread;
    boost::mutex _guardImage, _guardCameraInfo;
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
    _camInfoAvailable(false),
    _imageAvailable(false),
    _maxRate(maxRate),
    _imageCounter(0),
    _updatedImageId(0),
    _cameraInfoCounter(0),
    _updatedCameraInfoId(0),
    _node(ros::NodeHandle())
{
    _node.setCallbackQueue(&_cbQueue);
    _imageTransport.reset( new image_transport::ImageTransport( _node ) );
    std::string transportStr;

    if ( trans == CameraClient::RAW )
        transportStr = "raw";
    else if ( trans == CameraClient::JPEG )
        transportStr = "compressed";

    image_transport::TransportHints transportHint(transportStr);

    if ( _camInfoTopic != "" )
        _camInfoSub = _node.subscribe(_camInfoTopic, 1, &CameraClientImpl::cameraInfoCallback, this);

    _imageSub = _imageTransport->subscribe(_imgTopic, 1, &CameraClientImpl::imageCallback, this, transportHint);

    _spinThread.reset( new boost::thread( boost::bind(&CameraClientImpl::spin, this) ) );
}

CameraClientImpl::~CameraClientImpl()
{    
    if ( _camInfoTopic != "" )
        _camInfoSub.shutdown();

    _imageSub.shutdown();
}

void CameraClientImpl::spin()
{
    ros::WallDuration period( static_cast<double>(1.0/_maxRate) );
    ros::Rate rate(static_cast<double>(_maxRate));
    while ( ros::ok() )
    {
        _cbQueue.callAvailable( period );
        rate.sleep();
    }
}

void CameraClientImpl::imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, imgMsg->encoding);

  {
    boost::mutex::scoped_lock lock(_guardImage);
    _image = cvImgPtr->image.clone();
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

void CameraClientImpl::updateCameraInfo()
{
    ros::Time start = ros::Time::now();
    while ( _imageCounter == _updatedImageId && ros::ok() )
    {
        if ( (ros::Time::now() - start).toSec() > _timeoutSec )
            throw std::runtime_error("Error in CameraClientImpl::updateImage: timeout occurred");
    }

    {
        boost::mutex::scoped_lock lock(_guardCameraInfo);
        _updatedCamInfo      = _camInfo;
        _updatedCameraInfoId = _cameraInfoCounter;
    }

    _camInfoAvailable = true;
}

void CameraClientImpl::getCameraInfo(sensor_msgs::CameraInfo& camInfo) const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getCameraInfo: CameraInfo not available. Call first updateCameraInfo()");

    camInfo = _updatedCamInfo;
}

void CameraClientImpl::updateImage()
{
    ros::Time start = ros::Time::now();
    while ( _imageCounter == _updatedImageId && ros::ok() )
    {
        if ( (ros::Time::now() - start).toSec() > _timeoutSec )
            throw std::runtime_error("Error in CameraClientImpl::updateImage: timeout occurred");
    }

    {
        boost::mutex::scoped_lock lock(_guardImage);
        _updatedImage = _image.clone();
        _updatedImageId = _imageCounter;
    }

    _imageAvailable = true;
}

void CameraClientImpl::getImage(cv::Mat& img) const
{
    if ( !_imageAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getImage: no image available. First call CameraClient::updateImage()");

    img = _updatedImage.clone();
}

int CameraClientImpl::getImageWidth() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getImageWidth: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.width;
}

int CameraClientImpl::getImageHeight() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getImageHeight: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.height;
}

double CameraClientImpl::getFx() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getFx: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.K[0];
}

double CameraClientImpl::getFy() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getFy: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.K[4];
}

double CameraClientImpl::getCx() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getCx: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.K[2];
}

double CameraClientImpl::getCy() const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getCy: CameraInfo not available. Call first updateCameraInfo()");

    return _camInfo.K[5];
}

void CameraClientImpl::getDistortion(std::vector<double>& distortion) const
{
    if ( !_camInfoAvailable )
        throw std::runtime_error("Error in CameraClientImpl::getDistortion: CameraInfo not available. Call first updateCameraInfo()");

    distortion = _camInfo.D;
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


void CameraClient::updateCameraInfo()
{
    _impl->updateCameraInfo();
}

void CameraClient::getCameraInfo(sensor_msgs::CameraInfo& camInfo) const
{
    _impl->getCameraInfo(camInfo);
}


void CameraClient::updateImage()
{
    _impl->updateImage();
}


void CameraClient::getImage(cv::Mat& img) const
{
    _impl->getImage(img);
}


int CameraClient::getImageWidth() const
{
    return _impl->getImageWidth();
}

int CameraClient::getImageHeight() const
{
    return _impl->getImageHeight();
}

double CameraClient::getFx() const
{
    return _impl->getFx();
}

double CameraClient::getFy() const
{
    return _impl->getFy();
}

double CameraClient::getCx() const
{
    return _impl->getCx();
}

double CameraClient::getCy() const
{
    return _impl->getCy();
}

void CameraClient::getDistortion(std::vector<double>& distortion) const
{    
    _impl->getDistortion(distortion);
}

}

