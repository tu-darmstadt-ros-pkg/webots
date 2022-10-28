// Copyright TODO(FB)
#ifndef RADIO_NUCLEAR_SOURCE_HPP
#define RADIO_NUCLEAR_SOURCE_HPP

#include <webots/Device.hpp>

namespace webots {
  class RadioNuclearSource : public Device {
  public:
    explicit RadioNuclearSource(const std::string &name) : Device(name) {}  // Use Robot::getRadioNuclearSource() instead
    virtual ~ RadioNuclearSource() {}
    void setRange(double range);
    double getRange() const;
  };
}  // namespace webots

#endif  // RADIO_NUCLEAR_Source_HPP