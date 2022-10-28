// Copyright TODO(FB)

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/radio_nuclear_source.h>
#include <webots/RadioNuclearSource.hpp>

using namespace webots;

void RadioNuclearSource::setRange(double range) {
  wb_radio_nuclear_source_set_range(getTag(), range);
}

double RadioNuclearSource::getRange() const {
  return wb_radio_nuclear_source_get_range(getTag());
}
