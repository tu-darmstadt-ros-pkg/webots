// Copyright 2022 TODO(FB)

#include "WbRadioNuclearSource.hpp"

#include "WbFieldChecker.hpp"
#include "WbDataStream.hpp"
#include "WbRadioNuclearDetector.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

#include<iostream>
void WbRadioNuclearSource::init() {
  mType = findSFString("type");
  mRange = findSFDouble("range");
  mDosage = findSFDouble("dosage");
  mRadiationType = WbRadioNuclearSource::GAMMA;
  mNeedToSetRange = false;
  mNeedToSetDosage = false;
}

WbRadioNuclearSource::WbRadioNuclearSource(WbTokenizer *tokenizer) : WbSolidDevice("RadioNuclearSource", tokenizer) {
  init();
}

WbRadioNuclearSource::WbRadioNuclearSource(const WbRadioNuclearSource &other) : WbSolidDevice(other) {
  init();
}

WbRadioNuclearSource::WbRadioNuclearSource(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbRadioNuclearSource::~WbRadioNuclearSource() {
  WbRadioNuclearDetector::unregisterSource(this);
}

void WbRadioNuclearSource::preFinalize() {
  WbSolid::preFinalize();

  WbRadioNuclearDetector::registerSource(this);

  // updateRadiationetup();  // TODO(FB) needed?
}

void WbRadioNuclearSource::postFinalize() {
  WbSolid::postFinalize();

  connect(mType, &WbSFString::changed, this, &WbRadioNuclearSource::updateRadiationSetup);
  connect(mRange, &WbSFDouble::changed, this, &WbRadioNuclearSource::updateRange);
  connect(mDosage, &WbSFDouble::changed, this, &WbRadioNuclearSource::updateDosage);
}

int WbRadioNuclearSource::decodeRadiationType(const QString &string) {
  if (string == "alpha")
    return ALPHA;
  else if (string == "beta")
    return BETA;
  else if (string == "gamma")
    return GAMMA;
  else
    return ALPHA;
}

void WbRadioNuclearSource::updateRadiationSetup() {
  mRadiationType = decodeRadiationType(mType->value());
  // remove and add to source list, as type has changed and sensors need to resubscribe
  WbRadioNuclearDetector::unregisterSource(this);
  WbRadioNuclearDetector::registerSource(this);
}

void WbRadioNuclearSource::updateRange() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mRange, -1, -1);
  mNeedToSetRange = true;
}

void WbRadioNuclearSource::updateDosage() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mDosage, 1.0, -1);
  mNeedToSetDosage = true;
}

void WbRadioNuclearSource::writeConfigure(WbDataStream &stream) {
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)mRange->value();
  stream << (double)mDosage->value();

  mNeedToSetRange = false;
  mNeedToSetDosage = false;
  //TODO
}

void WbRadioNuclearSource::writeAnswer(WbDataStream &stream) {
  if (mNeedToSetRange) {
    stream << tag();
    stream << (unsigned char)C_RADIO_NUCLEAR_SOURCE_SET_RANGE;
    stream << (double)mRange->value();
    mNeedToSetRange = false;
  }
  if (mNeedToSetDosage) {
    stream << tag();
    stream << (unsigned char)C_RADIO_NUCLEAR_SOURCE_SET_DOSAGE;
    stream << (double)mDosage->value();
    mNeedToSetDosage = false;
  }
  //TODO
}

void WbRadioNuclearSource::handleMessage(QDataStream &stream) {
  unsigned char command;
  double newDosage;
  double newRange;

  stream >> command;
  switch (command) {
    case C_RADIO_NUCLEAR_SOURCE_SET_RANGE:
      stream >> newRange;
      mRange->setValue(newRange);
      return;
    case C_RADIO_NUCLEAR_SOURCE_SET_DOSAGE:
      stream >> newDosage;
      mDosage->setValue(newDosage);
      return;
    default:
      assert(0);
  }
}

void WbRadioNuclearSource::prePhysicsStep(double ms) {
  WbSolid::prePhysicsStep(ms);

  // forward queued packets to receivers
  // WbRadioNuclearDetector::receiveMeasurement();
}

void WbRadioNuclearSource::reset(const QString &id) {
  WbSolid::reset(id);
}