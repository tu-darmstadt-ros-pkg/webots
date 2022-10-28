// Copyright TODO(FB)
#include <stdio.h>
#include <stdlib.h>  // malloc and free
#include <webots/radio_nuclear_source.h>
#include <webots/nodes.h>
#include <webots/types.h>
#include "device_private.h"
#include "messages.h"
#include "robot_private.h"


typedef struct {
  double dosage;       // dosage
  double range;
} RadioNuclearSource;

static RadioNuclearSource *radio_nuclear_source_create() {
  RadioNuclearSource *dt = malloc(sizeof(RadioNuclearSource));
  dt->dosage = 1.0;
  dt->range = -1.0;
  return dt;
}

static RadioNuclearSource *radio_nuclear_source_get_struct(WbDeviceTag t) {
  WbDevice *d = robot_get_device_with_node(t, WB_NODE_RADIO_NUCLEAR_SOURCE, true);
  return d ? d->pdata : NULL;
}

static void radio_nuclear_source_read_answer(WbDevice *d, WbRequest *r) {
  switch (request_read_uchar(r)) {
    case C_CONFIGURE:  {
      RadioNuclearSource *s = d->pdata;
      s->range = request_read_double(r);
      s->dosage = request_read_double(r);
      break;
    }
    case C_RADIO_NUCLEAR_SOURCE_SET_RANGE: {
      RadioNuclearSource *s = d->pdata;
      s->range = request_read_double(r);
      break;
    }
    case C_RADIO_NUCLEAR_SOURCE_SET_DOSAGE: {
      RadioNuclearSource *s = d->pdata;
      s->dosage = request_read_double(r);
      break;
    }
    default:
      ROBOT_ASSERT(0);
  }
  
  //RadioNuclearSource *dt = d->pdata;
  //int i = 0;
  /*switch (request_read_uchar(r)) {
    case C_CONFIGURE:  // read target list
  }*/
  //TODO
  // TODO get measurement
}

static void radio_nuclear_source_write_request(WbDevice *d, WbRequest *r) {
  //TODO
  RadioNuclearSource *s = d->pdata;

}

static void radio_nuclear_source_cleanup(WbDevice *d) {
  RadioNuclearSource *dt = d->pdata;
  free(dt);
}

// Exported functions

void wb_radio_nuclear_source_init(WbDevice *d) {
  d->read_answer = radio_nuclear_source_read_answer;
  d->write_request = radio_nuclear_source_write_request;
  d->cleanup = radio_nuclear_source_cleanup;
  d->pdata = radio_nuclear_source_create();
}

// Public functions available from API

double wb_radio_nuclear_source_get_range(WbDeviceTag tag) {
  robot_mutex_lock();
  RadioNuclearSource *dt = radio_nuclear_source_get_struct(tag);
  double range = -1;
  if (!dt) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  } else 
    range = dt->range;
  robot_mutex_unlock();
  return range;
}

void wb_radio_nuclear_source_set_range(WbDeviceTag tag, double range) {
  if (range < 0.0 && range != -1.0) {
    fprintf(stderr, "Error: %s(): invalid range=%f argument.\n", __FUNCTION__, range);
    return;
  }
  robot_mutex_lock();
  RadioNuclearSource *dt = radio_nuclear_source_get_struct(tag);
  if (!dt) {
    fprintf(stderr, "Error: %s(): invalid device tag.\n", __FUNCTION__);
  } else 
    dt->range = range;
  robot_mutex_unlock();
}