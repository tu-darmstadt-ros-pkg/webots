// Copyright 2022 TODO(FB)
// modeled after emitter

#ifndef WB_RADIO_NUCLEAR_SOURCE_HPP
#define WB_RADIO_NUCLEAR_SOURCE_HPP

#include "WbMFInt.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSolidDevice.hpp"

#include <QtCore/QQueue>

class WbRadioNuclearSource : public WbSolidDevice {
  Q_OBJECT

public:
  enum { ALPHA = 0, BETA = 1, GAMMA = 2};
  // constructors and destructor
  explicit WbRadioNuclearSource(WbTokenizer *tokenizer = NULL);
  WbRadioNuclearSource(const WbRadioNuclearSource &other);
  explicit WbRadioNuclearSource(const WbNode &other);
  virtual ~WbRadioNuclearSource();

  // reimplemented public functions
  int nodeType() const override {return WB_NODE_RADIO_NUCLEAR_SOURCE; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void prePhysicsStep(double ms) override;
  void reset(const QString &id) override;

  int decodeRadiationType(const QString &string);

  // field accessors
  double range() const { return mRange->value(); }
  double dosage() const { return mDosage->value(); }
  int radiationType() const { return mRadiationType; }

private:
  int mRadiationType;
  bool mNeedToSetRange;
  bool mNeedToSetDosage;

  // user accessible fields
  WbSFString *mType;
  WbSFDouble *mRange;
  WbSFDouble *mDosage;

  // private functions
  WbRadioNuclearSource &operator=(const WbRadioNuclearSource &);  // non copyable
  WbNode *clone() const override { return new WbRadioNuclearSource(*this); }
  void init();

private slots:
  void updateRadiationSetup();
  void updateRange();
  void updateDosage();
};

#endif  // WB_RADIO_NUCLEAR_SOURCE_HPP