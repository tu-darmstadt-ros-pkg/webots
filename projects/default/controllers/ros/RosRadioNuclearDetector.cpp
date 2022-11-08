// Copyright TODO(FB)

#include "RosRadioNuclearDetector.hpp"
#include <radiation_msgs/DoseRate.h>

RosRadioNuclearDetector::RosRadioNuclearDetector(RadioNuclearDetector *detector, Ros *ros) : RosSensor(detector->getName(), detector, ros) {
  mRadioNuclearDetector = detector;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mGetMeasurementServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_measurement", &RosRadioNuclearDetector::getMeasurementCallback);
}

RosRadioNuclearDetector::~RosRadioNuclearDetector() {
  mGetMeasurementServer.shutdown();
  cleanup();
}

ros::Publisher RosRadioNuclearDetector::createPublisher(std::map<std::string, std::string> *topics) {
  std::string topicName = RosDevice::fixedDeviceName() + "/data";
  if (topics != nullptr) {
    if (topics->find("data") != topics->end()) {
      topicName = topics->at("data");
    }
  }

  radiation_msgs::DoseRate type;

  return RosDevice::rosAdvertiseTopic(topicName, type);
}

void RosRadioNuclearDetector::publishValue(ros::Publisher publisher) {
  radiation_msgs::DoseRate value;
  value.header.stamp = ros::Time::now();
  if (mFrameOverride != "") {
    value.header.frame_id = mFrameOverride;
  } else {
    value.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  }
  value.rate = mRadioNuclearDetector->getMeasurement();

  value.radiation_type = 4; // TODO(FB) update to correct depending on type; needs passthrough
  value.integration_time = 1.0; // TODO(FB) set 
  value.units = 2; // TODO(FB)
  publisher.publish(value);
}

bool RosRadioNuclearDetector::getMeasurementCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mRadioNuclearDetector);
  res.value = mRadioNuclearDetector->getMeasurement();
  return true;
}