// Copyright TODO(FB)

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/radio_nuclear_detector.h>
#include <webots/RadioNuclearDetector.hpp>

using namespace webots;

void RadioNuclearDetector::enable(int sampling_period) {
  wb_radio_nuclear_detector_enable(getTag(), sampling_period);
}

void RadioNuclearDetector::disable() {
  wb_radio_nuclear_detector_disable(getTag());
}

int RadioNuclearDetector::getSamplingPeriod() const {
  return wb_radio_nuclear_detector_get_sampling_period(getTag());
}

double RadioNuclearDetector::getMeasurement() const {
  return wb_radio_nuclear_detector_get_measurement(getTag());
}
