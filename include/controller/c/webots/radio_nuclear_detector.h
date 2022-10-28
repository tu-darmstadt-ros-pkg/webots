// Copyright TODO(FB)

#ifndef WB_RADIO_NUCLEAR_DETECTOR_H
#define WB_RADIO_NUCLEAR_DETECTOR_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// device functions
void wb_radio_nuclear_detector_enable(WbDeviceTag tag, int sampling_period);
void wb_radio_nuclear_detector_disable(WbDeviceTag tag);
int wb_radio_nuclear_detector_get_sampling_period(WbDeviceTag tag);

double wb_radio_nuclear_detector_get_measurement(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif  // RADIO_NUCLEAR_DETECTOR_H