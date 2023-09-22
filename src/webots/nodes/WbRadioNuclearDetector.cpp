// Copyright 2022 TODO(FB)

#include "WbRadioNuclearDetector.hpp"

#include "WbDataStream.hpp"
#include "WbRadioNuclearSource.hpp"
#include "WbFieldChecker.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbRandom.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

#include <iostream>

static QList<WbRadioNuclearDetector *> gDetectorList;
static QList<WbRadioNuclearSource *> gSourceList;

class WbRadioNuclearInfo {
public:
  enum { ALPHA = 0, BETA = 1, GAMMA = 2 };
  static int decodeRadiationType(const QString &string) {
    if (string == "alpha")
      return ALPHA;
    else if (string == "beta")
      return BETA;
    else if (string == "gamma")
      return GAMMA;
    else
      return ALPHA;
  }
  WbRadioNuclearInfo(WbRadioNuclearSource *source, double dosage, int type) :
    mSource(source),
    mDosage(dosage),
    mType(type),
    mSourceDir(NAN, NAN, NAN) {}

  WbRadioNuclearInfo(WbRadioNuclearInfo &other) :
    mSource(other.mSource),
    mDosage(other.mDosage),
    mType(other.mType),
    mSourceDir(other.mSourceDir),
    mSourcePos(other.mSourcePos) {}

  ~WbRadioNuclearInfo() {}

  // TODO check if source dir is needed
  WbRadioNuclearSource *source() const { return mSource; }
  const WbVector3 &sourceDir() const { return mSourceDir; }
  void setSourceDir(const WbVector3 &dir) { mSourceDir = dir; }
  double dosage() const { return mDosage; }
  void setDosage(double dosage) { mDosage = dosage; }
  int type() const { return mType; }
  void setType(int type) { mType = type; }
private:
  WbRadioNuclearInfo &operator=(const WbRadioNuclearInfo &);  // non copyable
  WbRadioNuclearSource *mSource;
  double mDosage;
  int mType;
  WbVector3 mSourceDir;
  WbVector3 mSourcePos;
};

class RadioNuclearMeasurement {
public:

  RadioNuclearMeasurement(WbRadioNuclearSource *source, WbRadioNuclearDetector *d, dSpaceID spaceId) :
    mCollided(false) {
    mMeasurement = new WbRadioNuclearInfo(source, source->dosage(), source->radiationType());
    assert(spaceId && d);
    assert(source);
    mSource = source;

    // TODO fix
    // compute Ray
    /* const WbVector3 &ts = source->matrix().translation();
    WbVector3 dir = d->matrix().translation() - ts;

    // setup ray geom for ODE collision detection
    mGeom = dCreateRay(spaceId, dir.length());
    dGeomSetDynamicFlag(mGeom);
    dGeomRaySet(mGeom, ts[0], ts[1], ts[2], dir[0], dir[1], dir[2]);

    // set detector as callback data in case there is a collision
    dGeomSetData(mGeom, new WbOdeGeomData(d));*/
  }

  ~RadioNuclearMeasurement() {
    /*if (mGeom) {
      WbOdeGeomData *odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mGeom));
      delete odeGeomData;
      dGeomDestroy(mGeom);
    }*/
    delete mMeasurement;
  };

  WbRadioNuclearInfo *info() { return mMeasurement; }
  bool hasCollided() const { return mCollided; }
  void setCollided() { mCollided = true; }
  const WbRadioNuclearSource *source() const { return mSource; }
  dGeomID geom() const { return mGeom; }

  void recomputeRayDirection(const WbVector3 &detectorTranslation) {
    // compute ray direction and length
    /*mSource->updateTransformForPhysicsStep();
    const WbVector3 &ts = mSource->matrix().translation();
    WbVector3 dir = detectorTranslation - ts;
    dGeomRaySetLength(mGeom, dir.length());
    dGeomRaySet(mGeom, ts[0], ts[1], ts[2], dir[0], dir[1], dir[2]);*/

    // TODO fix
  }

private:
  WbRadioNuclearInfo *mMeasurement;         // Measurement
  WbRadioNuclearSource *mSource;          // Emitting Source
  dGeomID mGeom;                          // geom to check collision
  bool mCollided;                         // if there are collisions
};

void WbRadioNuclearDetector::init() {
  mSensor = NULL;
  mRadiationType = WbRadioNuclearDetector::GAMMA;
  mType = findSFString("type");
  mSignalStrengthNoise = findSFDouble("signalStrengthNoise");
  mDirectionNoise = findSFDouble("directionNoise");
  mNeedToConfigure = false;
  mResultingDosage = 0.0;
}

WbRadioNuclearDetector::WbRadioNuclearDetector(WbTokenizer *tokenizer) : WbSolidDevice("RadioNuclearDetector", tokenizer) {
  init();
}

WbRadioNuclearDetector::WbRadioNuclearDetector(const WbRadioNuclearDetector &other) : WbSolidDevice(other) {
  init();
}

WbRadioNuclearDetector::WbRadioNuclearDetector(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbRadioNuclearDetector::~WbRadioNuclearDetector() {
  qDeleteAll(mMeasurementList);
  // Not deleting all sources as this is wrong, only list should be deleted
  //qDeleteAll(mSourceList);
  mSourceList.clear();
  // remove myself from list
  gDetectorList.removeAll(this);
  delete mSensor;
}

void WbRadioNuclearDetector::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
  updateRadiationSetup();

  gDetectorList.append(this);  // add myself

  // check SourceList for available sources
  for (int i = 0; i < gSourceList.size(); i++) {
    WbRadioNuclearSource *source = gSourceList.at(i);

    // radiation types must match
    if (source->radiationType() == radiationType() || radiationType() == ALL) {
      if (mSourceList.indexOf(source) == -1) {
        mSourceList.append(source);
      }
    }
  }
}

void WbRadioNuclearDetector::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mType, &WbSFString::changed, this, &WbRadioNuclearDetector::updateRadiationSetup);
  connect(mSignalStrengthNoise, &WbSFDouble::changed, this, &WbRadioNuclearDetector::updateRadiationSetup);
  connect(mDirectionNoise, &WbSFDouble::changed, this, &WbRadioNuclearDetector::updateRadiationSetup);
}


int WbRadioNuclearDetector::decodeRadiationType(const QString &string) {
  if (string == "alpha")
    return ALPHA;
  else if (string == "beta")
    return BETA;
  else if (string == "gamma")
    return GAMMA;
  else if (string == "all")
    return ALL;
  else
    return ALL;
}

void WbRadioNuclearDetector::updateRadiationSetup() {
  mRadiationType = decodeRadiationType(mType->value());

  WbFieldChecker::resetDoubleIfNegative(this, mSignalStrengthNoise, 0);
  WbFieldChecker::resetDoubleIfNegative(this, mDirectionNoise, 0);

  mNeedToConfigure = true;
}

void WbRadioNuclearDetector::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot(), false);  // needed?
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  mNeedToConfigure = false;
  // TODO
}

void WbRadioNuclearDetector::writeAnswer(WbDataStream &stream) {
  // todo
  stream << tag();
  stream << (unsigned char)C_RADIO_NUCLEAR_DETECTOR_GET_MEASUREMENT;
  stream << (double) mResultingDosage;

  // is frequently called
  if (mNeedToConfigure)
    writeConfigure(stream);
}

void WbRadioNuclearDetector::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD: {
      short rate;
      stream >> rate;
      mSensor->setRefreshRate(rate);
      return;
    }
    default:
      assert(0);
  }
}

void WbRadioNuclearDetector::prePhysicsStep(double ms) {
  WbSolidDevice::prePhysicsStep(ms);
  if (mSensor->isEnabled() && !mSourceList.isEmpty()) {
    // receiver or emitter could move during physics step
    //mMeasurementList.clear();
    foreach (WbRadioNuclearSource *source, mSourceList) {
      RadioNuclearMeasurement *m = new RadioNuclearMeasurement(source, this, WbOdeContext::instance()->space());
      mMeasurementList.append(m);
    }

    // TODO fix
    //subscribeToRaysUpdate(mMeasurementList[0]->geom());
  }
}

// TODO
void WbRadioNuclearDetector::updateRaysSetupIfNeeded() {
  // TODO fix
  /*updateTransformForPhysicsStep();
  const WbVector3 position = matrix().translation();
  // update receiver position in pending packets
  foreach (RadioNuclearMeasurement *m, mMeasurementList)
    m->recomputeRayDirection(position);*/
}

// TODO collision function from receiver, update for ray depth in case of beta or similar?
// This function searches and destroys the information packets that have collided with an obstacle.
// It is called when the collision detection is over, that means
// after the last call to the function: WbReceiver::rayCollisionCallback()
// All the packets that did not collide with an obstacle are appended to the Receiver's buffer.
void WbRadioNuclearDetector::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  // TODO rewrite to crawl nodes instead of measurements? (have pointers of sources and check for dosage and position)
  // (and use registering to list instead, maybe map instead of list, with some unique id/ptr as key)

  mResultingDosage = 0.0;
  // TODO background noise?
  // TODO handle different in case 

  foreach (RadioNuclearMeasurement *m, mMeasurementList) {
    auto info = m->info();
    int type = info->type();

    // check range, no aperture for now
    if (!checkApertureAndRange(info->source(), this, true)) {
      delete m;  // Measurement also deletes it's info
      continue;
    }

    bool collision_check = true;
    if (type != GAMMA) {
      // check collision
      if (m->hasCollided()) {
        delete m;
        continue;
      }
    }

    // calculate strength and send measurements
    const WbMatrix4 &mat = matrix();
    const WbVector3 &t = info->source()->matrix().translation();
    const WbVector3 &sourcePos = mat.pseudoInversed(t);

    double dist = sourcePos.length();  // compute distance

    // compute the emitter direction vector
    /*if (mDirectionNoise->value() == 0)  // No noise on direction
      info->setEmitterDir(sourcePos / dist);
    else {
      WbVector3 sourceNoisedPos(sourcePos[0], sourcePos[1], sourcePos[2]);  // Extraction of the real emitter position
      sourceNoisedPos[0] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();  // Add noise on each components
      sourceNoisedPos[1] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();
      sourceNoisedPos[2] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();
      // Recompute distance from new position in order to have a correct normalization of the vector
      double newDist = sourceNoisedPos.length();
      info->setEmitterDir(sourceNoisedPos / newDist);  // Send noisy position normalized
    }*/
    // simulate signal strength from known distance
    double signalStrength = 1.0 / (dist * dist);
    if (mSignalStrengthNoise->value() != 0) { // No noise on the signal strength
      signalStrength *= (1 + mSignalStrengthNoise->value() * WbRandom::nextGaussian());  // add noise
      if (signalStrength < 0)
        signalStrength = 0;
    }

    // add dosage with simulated decay + noise
    mResultingDosage += info->dosage() * signalStrength;

    delete m;
  }
  mMeasurementList.clear();
}

// if the packet is blocked by an obstacle we can discard it
void WbRadioNuclearDetector::rayCollisionCallback(dGeomID geom, WbSolid *obstacle) {
  foreach (RadioNuclearMeasurement *m, mMeasurementList) {
    if (m->geom() == geom) {
      // we want to ignore collision of ray with the emitting robot
      // (collision with receiver's robot is already filtered out by odeNearCallback())
      if (!m->hasCollided())
        m->setCollided();

      return;  // we have found the right packet: no need to look further in the list
    }
  }

  assert(0);  // should never be reached
}

bool WbRadioNuclearDetector::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  mSensor->updateTimer();
  return true;
}

void WbRadioNuclearDetector::reset(const QString &id) {
  WbSolidDevice::reset(id);
  qDeleteAll(mMeasurementList);
  qDeleteAll(mSourceList);
  mMeasurementList.clear();
  mSourceList.clear();
}


// TODO discuss if aperture is needed /should be used?
bool WbRadioNuclearDetector::checkApertureAndRange(const WbRadioNuclearSource *source, const WbRadioNuclearDetector *detector, bool checkRangeOnly) {
  WbVector3 eTranslation = source->matrix().translation();
  WbVector3 rTranslation = detector->matrix().translation();

  // out of range ?
  /*if (source->range() != -1.0) {
    double range2 = detector->range() * source->range();
    double distance = (rTranslation - eTranslation).length2();
    if (distance > range2)
      return false;
  }

  if (checkRangeOnly)
    return true;*/

  // emission: check that receiver is within emitter's cone
  /*if (source->aperture() > 0.0) {
    const WbVector3 e2r = rTranslation - eTranslation;
    const WbVector4 eAxisX4 = source->matrix().column(0);
    const WbVector3 eAxisX3(eAxisX4[0], eAxisX4[1], eAxisX4[2]);
    if (eAxisX3.angle(e2r) > source->aperture() / 2.0)
      return false;
  }

  // reception: check that emitter is within receiver's cone
  if (detector->aperture() > 0.0) {
    const WbVector3 r2e = eTranslation - rTranslation;
    const WbVector4 rAxisX4 = detector->matrix().column(0);
    const WbVector3 rAxisX3(rAxisX4[0], rAxisX4[1], rAxisX4[2]);
    if (rAxisX3.angle(r2e) > detector->aperture() / 2.0)
      return false;
  }*/

  return true;
}

void WbRadioNuclearDetector::registerSource(WbRadioNuclearSource *source) {
  // add to global list
  if (gSourceList.indexOf(source) == -1) {
    gSourceList.append(source);
  } else {
    return;
  }

  // loop through detectors
  for (int i = 0; i < gDetectorList.size(); i++) {
    WbRadioNuclearDetector *d = gDetectorList.at(i);

    // radiation types must match
    if (source->radiationType() == d->radiationType() || d->radiationType() == ALL) {
      // check for collision, if not gamma
      /*if (d->radiationType() != GAMMA) {
        d->mMeasurementList.append(new RadioNuclearMeasurement(new WbRadioNuclearInfo(*info), d, WbOdeContext::instance()->space()));
      }*/
      // append if not in list
      if (d->mSourceList.indexOf(source) == -1) {
        d->mSourceList.append(source);
      }
    }
  }
}

void WbRadioNuclearDetector::unregisterSource(WbRadioNuclearSource *source) {
  auto res = gSourceList.indexOf(source);
  if (res) {
    return;  // not in list
  }

  gSourceList.removeAll(source);

  for (int i = 0; i < gDetectorList.size(); i++) {
    WbRadioNuclearDetector *d = gDetectorList.at(i);
    d->mSourceList.removeAll(source);
  }
}
