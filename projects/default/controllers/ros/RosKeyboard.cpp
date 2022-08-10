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

#include "RosKeyboard.hpp"
#include "webots_ros/Int32Stamped.h"

RosKeyboard::RosKeyboard(Keyboard *keyboard, Ros *ros) : RosSensor("keyboard", NULL, ros) {
  mKeyboard = keyboard;
}

// creates a publisher for keyboard values with a webots_ros/Int8Stamped as message type
ros::Publisher RosKeyboard::createPublisher(std::vector<std::string> *topics) {
  
  webots_ros::Int32Stamped type;
  std::string topicName = "keyboard/key";
  
  if (topics != nullptr) {
    if (topics->size() == 1) {
      topicName = topics->at(0);
    }
    else {
      std::cerr << "Invalid amount of topics provided for Keyboard " << RosDevice::fixedDeviceName() << std::endl;
    }
  }

  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get value from the keyboard and publish it
void RosKeyboard::publishValue(ros::Publisher publisher) {
  webots_ros::Int32Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mFrameIdPrefix + "keyboard";
  int key = mKeyboard->getKey();
  while (key >= 0) {
    value.data = key;
    publisher.publish(value);
    key = mKeyboard->getKey();
  }
}
