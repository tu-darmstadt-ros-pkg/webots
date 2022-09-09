// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "RosConnector.hpp"
#include "webots_ros/Int8Stamped.h"

RosConnector::RosConnector(Connector *connector, Ros *ros) :
  RosSensor(connector->getName() + "/presence_sensor", connector, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mLockServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/lock", &RosConnector::lockCallback);
  mIsLockedServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/is_locked", &RosConnector::isLockedCallback);
  mConnector = connector;
}

RosConnector::~RosConnector() {
  mLockServer.shutdown();
  mIsLockedServer.shutdown();
  cleanup();
}

ros::Publisher RosConnector::createPublisher(std::map<std::string, std::string> *topics) {
  std::string topicName = RosDevice::fixedDeviceName() + "/presence";
  if (topics != nullptr) {
    if (topics->find("presence") != topics->end()) {
      topicName = topics->at("presence");
    }
  }

  webots_ros::Int8Stamped type;

  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the connector and publish it
void RosConnector::publishValue(ros::Publisher publisher) {
  webots_ros::Int8Stamped value;
  value.header.stamp = ros::Time::now();
  if (mFrameOverride != "") {
    value.header.frame_id = mFrameOverride;
  }
  else {
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  }
  value.data = mConnector->getPresence();
  publisher.publish(value);
}

// cppcheck-suppress constParameter
// cppcheck-suppress constParameterCallback
bool RosConnector::lockCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  if (req.value) {
    mConnector->lock();
    res.success = true;
  } else {
    mConnector->unlock();
    res.success = true;
  }
  return true;
}

bool RosConnector::isLockedCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = mConnector->isLocked();
  return true;
}
