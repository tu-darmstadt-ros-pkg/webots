// Copyright TODO(FB)

#ifndef ROS_RADIO_NUCLEAR_DETECTOR_HPP
#define ROS_RADIO_NUCLEAR_DETECTOR_HPP

#include <webots/RadioNuclearDetector.hpp>
#include "RosSensor.hpp"

#include <webots_ros/get_float.h>

using namespace webots;

class RosRadioNuclearDetector: public RosSensor {
public:
  RosRadioNuclearDetector(RadioNuclearDetector *detector, Ros *ros);
  virtual ~RosRadioNuclearDetector();

  ros::Publisher createPublisher(std::map<std::string, std::string> *topics = nullptr) override;
  void publishValue(ros::Publisher publisher) override;
  bool getMeasurementCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);

  void rosEnable(int samplingPeriod) override { mRadioNuclearDetector->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mRadioNuclearDetector->getSamplingPeriod(); }

private:
  void cleanup() { mRadioNuclearDetector->disable(); }

  RadioNuclearDetector *mRadioNuclearDetector;
  ros::ServiceServer mSetServer;
  ros::ServiceServer mSamplingPeriodServer;
  ros::ServiceServer mGetMeasurementServer;
};

#endif  // ROS_RADIO_NUCLEAR_DETECTOR_HPP
