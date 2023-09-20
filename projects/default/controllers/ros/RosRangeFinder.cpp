// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "RosRangeFinder.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"

RosRangeFinder::RosRangeFinder(RangeFinder *range_finder, Ros *ros) : RosSensor(range_finder->getName(), range_finder, ros) {
  mRangeFinder = range_finder;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_info", &RosRangeFinder::getInfoCallback);
  mImageServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/save_image", &RosRangeFinder::saveImageCallback);

  mUseImageTransport = false;
}

RosRangeFinder::~RosRangeFinder() {
  mInfoServer.shutdown();
  mImageServer.shutdown();
  cleanup();

  if (mUseImageTransport) {
    mImagePub.shutdown();
  }
  mCameraInfoPublisher.shutdown();
}

// creates a publisher for range_finder image with
// a [ImageWidth x ImageHeight] {float} array
ros::Publisher RosRangeFinder::createPublisher(std::map<std::string, std::string> *topics) {
  std::string rangeImageTopicName = RosDevice::fixedDeviceName() + "/range_image";
  std::string cameraInfoTopicName = RosDevice::fixedDeviceName() + "/camera_info";
  if (topics != nullptr) {
    if (topics->find("camera_info") != topics->end()) {
      createCameraInfoPublisher(topics->at("camera_info"), true);
    } else {
      createCameraInfoPublisher("camera_info");
    }

    if (topics->find("range_image") != topics->end()) {
      return createRangeImagePublisher(topics->at("range_image"), true);
    }
  }

  createCameraInfoPublisher("camera_info");
  return createRangeImagePublisher("range_image");
}

ros::Publisher RosRangeFinder::createRangeImagePublisher(const std::string &name, bool override) {
  sensor_msgs::Image type;
  type.height = mRangeFinder->getHeight();
  type.width = mRangeFinder->getWidth();
  type.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  type.step = sizeof(float) * mRangeFinder->getWidth();

  if (override) {
    mRangeTopic = name;
  }
  else {
    mRangeTopic = RosDevice::fixedDeviceName() + "/" + name;
  }  
  
  return RosDevice::rosAdvertiseTopic(mRangeTopic, type);
}

void RosRangeFinder::createCameraInfoPublisher(const std::string &name, bool override) {
  sensor_msgs::CameraInfo type;
  if (override) {
    mCameraInfoPublisher = RosDevice::rosAdvertiseTopic(name, type);
  }
  else {
    mCameraInfoPublisher = RosDevice::rosAdvertiseTopic(RosDevice::fixedDeviceName() + "/" + name, type);
  }
}

// get image from the RangeFinder and publish it
void RosRangeFinder::publishValue(ros::Publisher publisher) {
  if (mUseImageTransport) {
    return;
  }
  
  publisher.publish(createImageMsg());
}

sensor_msgs::Image RosRangeFinder::createImageMsg() {
  const char *rangeImageVector;
  rangeImageVector = static_cast<const char *>(static_cast<void *>(const_cast<float *>(mRangeFinder->getRangeImage())));
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  if (mFrameOverride != "") {
    image.header.frame_id = mFrameOverride;
  }
  else {
    image.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  }
  image.height = mRangeFinder->getHeight();
  image.width = mRangeFinder->getWidth();
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = sizeof(float) * mRangeFinder->getWidth();

  image.data.resize(sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());
  memcpy(&image.data[0], rangeImageVector, sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());
  return image;
}

bool RosRangeFinder::getInfoCallback(webots_ros::range_finder_get_info::Request &req,
                                     webots_ros::range_finder_get_info::Response &res) {
  assert(mRangeFinder);
  res.width = mRangeFinder->getWidth();
  res.height = mRangeFinder->getHeight();
  res.Fov = mRangeFinder->getFov();
  res.minRange = mRangeFinder->getMinRange();
  res.maxRange = mRangeFinder->getMaxRange();
  return true;
}

sensor_msgs::CameraInfo RosRangeFinder::createCameraInfoMessage() {
  sensor_msgs::CameraInfo info;
  info.header.stamp = ros::Time::now();
  if (mFrameOverride != "") {
    info.header.frame_id = mFrameOverride;
  }
  else {
    info.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  }
  info.width = mRangeFinder->getWidth();
  info.height = mRangeFinder->getHeight();
  info.distortion_model = "plumb_bob";
  double hfov = mRangeFinder->getFov();
  double width = mRangeFinder->getWidth();
  double height = mRangeFinder->getHeight();
  double focal_length = width / (2.0 * tan(hfov/ 2.0)); //reference old plugin https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_camera_utils.cpp#L513
  std::vector<double> D = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.D = D;
  double fx, fy, cx, cy;
  fx = focal_length;
  fy = focal_length;
  cx = (width + 1.0) / 2.0;
  cy = (height + 1.0) / 2.0;
  boost::array<double, 9> K = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.K = K;
  boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; //important for stereo though
  info.R = R;
  boost::array<double, 12> P = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  info.P = P;
  info.binning_x = 0.0;
  info.binning_y = 0.0;
  info.roi.x_offset = 0;
  info.roi.y_offset = 0;
  info.roi.height = 0;
  info.roi.width = 0;
  info.roi.do_rectify = false;

  return info;
}

void RosRangeFinder::publishAuxiliaryValue() {
  if (mCameraInfoPublisher.getNumSubscribers() > 0) {
    mCameraInfoPublisher.publish(createCameraInfoMessage());
  }

  if (mUseImageTransport && mImagePub.getNumSubscribers() > 0) {
    mImagePub.publish(createImageMsg());
  }
}

bool RosRangeFinder::saveImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res) {
  assert(mRangeFinder);
  res.success = 1 + mRangeFinder->saveImage(req.filename, req.quality);
  return true;
}

//this is not the cleanest way right now
void RosRangeFinder::enableImageTransport() {
  mUseImageTransport = true;

  image_transport::ImageTransport it(*mRos->nodeHandle());
  mImagePub = it.advertise(mRangeTopic, 1);
}