// Copyright TODO(FB)
#include <stdio.h>
#include <stdlib.h>  // malloc and free
#include <webots/radio_nuclear_detector.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"

typedef struct {
  int enable : 1;             // need to enable device ?
  int sampling_period;        // milliseconds
  double dosageReading;       // read dosage
} RadioNuclearDetector;

static RadioNuclearDetector *radio_nuclear_detector_create() {
  RadioNuclearDetector *dt = malloc(sizeof(RadioNuclearDetector));
  dt->enable = false;
  dt->sampling_period = 0;
  dt->dosageReading = 0;
  return dt;
}

static RadioNuclearDetector *radio_nuclear_detector_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_RADIO_NUCLEAR_DETECTOR, true);
  return d ? d->pdata : NULL;
}

static void radio_nuclear_detector_read_answer(WbDevice *d, WbRequest *r) {
  RadioNuclearDetector *dt = d->pdata;
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:  // read target list
      break;
    case C_RADIO_NUCLEAR_DETECTOR_GET_MEASUREMENT:
      dt->dosageReading = request_read_double(r);
      break;
    default:
      ROBOT_ASSERT(0);
  }
  //TODO
  // TODO get measurement
}

static void radio_nuclear_detector_write_request(WbDevice *d, WbRequest *r) {
  // TODO
  RadioNuclearDetector *dt = d->pdata;
  if (dt->enable) {
    request_write_uchar(r, C_SET_SAMPLING_PERIOD);
    request_write_uint16(r, dt->sampling_period);
    dt->enable = false;  // done
  }
}

static void radio_nuclear_detector_cleanup(WbDevice *d) {
  RadioNuclearDetector *dt = d->pdata;
  free(dt);
}

// Exported functions

void wb_radio_nuclear_detector_init(WbDevice *d) {
  d->read_answer = radio_nuclear_detector_read_answer;
  d->write_request = radio_nuclear_detector_write_request;
  d->cleanup = radio_nuclear_detector_cleanup;
  d->pdata = radio_nuclear_detector_create();
}

// Public functions available from API

void wb_radio_nuclear_detector_enable(WbDeviceTag tag, int sampling_period) {
  if (sampling_period < 0) {
    fprintf(stderr, "Error: %s() called with negative sampling period.\n", __FUNCTION__);
    return;
  }
  robot_mutex_lock();
  RadioNuclearDetector *dt = radio_nuclear_detector_get_struct(tag);
  if (!dt)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else {
    dt->enable = true;
    dt->sampling_period = sampling_period;
  }
  robot_mutex_unlock();
}

void wb_radio_nuclear_detector_disable(WbDeviceTag tag) {
  RadioNuclearDetector *dt = radio_nuclear_detector_get_struct(tag);
  if (!dt)
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  else
    wb_radio_nuclear_detector_enable(tag, 0);
}

int wb_radio_nuclear_detector_get_sampling_period(WbDeviceTag tag) {
  int sampling_period;
  robot_mutex_lock();
  RadioNuclearDetector *dt = radio_nuclear_detector_get_struct(tag);
  if (!dt) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    sampling_period = 0;
  } else
    sampling_period = dt->sampling_period;
  robot_mutex_unlock();
  return sampling_period;
}

double wb_radio_nuclear_detector_get_measurement(WbDeviceTag tag) {
  robot_mutex_lock();
  RadioNuclearDetector *dt = radio_nuclear_detector_get_struct(tag);
  double dosage;
  if (!dt) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
    dosage = -1.0;
  } else 
    dosage = dt->dosageReading;
  robot_mutex_unlock();
  return dosage;
}
