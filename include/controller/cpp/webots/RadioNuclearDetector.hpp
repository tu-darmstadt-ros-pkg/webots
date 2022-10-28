// Copyright TODO(FB)
#ifndef RADIO_NUCLEAR_DETECTOR_HPP
#define RADIO_NUCLEAR_DETECTOR_HPP

#include <webots/Device.hpp>

namespace webots {
  class RadioNuclearDetector : public Device {
  public:
    explicit RadioNuclearDetector(const std::string &name) : Device(name) {}  // Use Robot::getRadioNuclearDetector() instead
    virtual ~ RadioNuclearDetector() {}
    virtual void enable(int sampling_period);
    virtual void disable();
    int getSamplingPeriod() const;
    double getMeasurement() const;
  };
}  // namespace webots

#endif  // RADIO_NUCLEAR_DETECTOR_HPP