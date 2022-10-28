// Copyright TODO(FB)
#ifndef WB_RADIO_NUCLEAR_SOURCE_H
#define WB_RADIO_NUCLEAR_SOURCE_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// device functions
double wb_radio_nuclear_source_get_range(WbDeviceTag tag);
void wb_radio_nuclear_source_set_range(WbDeviceTag tag, double range);
//TODO type?

#ifdef __cplusplus
}
#endif

#endif  // RADIO_NUCLEAR_SOURCE_H