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

#ifndef ROS_RANGE_FINDER_HPP
#define ROS_RANGE_FINDER_HPP

#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/save_image.h>
#include <webots/RangeFinder.hpp>
#include "RosSensor.hpp"
#include "image_transport/image_transport.h"

#include <map>
#include <string>

#include "sensor_msgs/CameraInfo.h"

using namespace webots;

class RosRangeFinder : public RosSensor {
public:
  RosRangeFinder(RangeFinder *range_finder, Ros *ros);
  virtual ~RosRangeFinder();

  ros::Publisher createPublisher(std::map<std::string, std::string> *topics = nullptr) override;
  void publishValue(ros::Publisher publisher) override;
  void publishAuxiliaryValue() override;
  bool getInfoCallback(webots_ros::range_finder_get_info::Request &req, webots_ros::range_finder_get_info::Response &res);
  bool saveImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res);

  void rosEnable(int samplingPeriod) override { mRangeFinder->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mRangeFinder->getSamplingPeriod(); }

  void enableImageTransport();
  int getNumSubscribers() /*override*/;
  
private:
  void cleanup() { mRangeFinder->disable(); }
  ros::Publisher createRangeImagePublisher(const std::string &name, bool override=false);
  void createCameraInfoPublisher(const std::string &name, bool override=false);
  sensor_msgs::Image createImageMsg();
  sensor_msgs::CameraInfo createCameraInfoMessage();
  bool mUseImageTransport;
  image_transport::Publisher mImagePub;
  ros::Publisher mCameraInfoPublisher;
  RangeFinder *mRangeFinder;
  std::string mRangeTopic;
  ros::ServiceServer mInfoServer;
  ros::ServiceServer mImageServer;
};

#endif  // ROS_RANGE_FINDER_HPP
