// Coypright 2022 TODO(FB)
// modeled after Receiver

#ifndef WB_RADIO_NUCLEAR_DETECTOR_HPP
#define WB_RADIO_NUCLEAR_DETECTOR_HPP

#include "WbMFInt.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSolidDevice.hpp"

#include <QtCore/QQueue>

class WbRadioNuclearSource;
class WbSensor;
class RadioNuclearMeasurement;
class WbRadioNuclearInfo;

class WbRadioNuclearDetector : public WbSolidDevice {
  Q_OBJECT

public:
  enum { ALPHA = 0, BETA = 1, GAMMA = 2, ALL = 3};
  //static WbRadioNuclearDetector * createPhysicsReceiver();

  // constructors and destructor
  explicit WbRadioNuclearDetector(WbTokenizer *tokenizer = NULL);
  WbRadioNuclearDetector(const WbRadioNuclearDetector &other);
  explicit WbRadioNuclearDetector(const WbNode &other);
  virtual ~WbRadioNuclearDetector();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RADIO_NUCLEAR_DETECTOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

  static void registerSource(WbRadioNuclearSource *source);
  static void unregisterSource(WbRadioNuclearSource *source);

  int decodeRadiationType(const QString &string);
  void rayCollisionCallback(dGeomID geom, WbSolid *obstacle);  // TODO check if needed /rename

  void updateRaysSetupIfNeeded();

  // field accessors
  int radiationType() const { return mRadiationType; }

signals:
  void measurementReceived(double dosage) const;

private:
  int mRadiationType;   // ALPHA, BETA, GAMMA, ALL
  QList<RadioNuclearMeasurement *> mMeasurementList;  // RadioNuclearMeasurements
  QList<WbRadioNuclearSource *> mSourceList;  // TODO, check if measurements could be left out
  WbSensor *mSensor;
  double mResultingDosage;

  // user accessible fields
  WbSFString *mType;
  WbSFDouble *mSignalStrengthNoise;
  WbSFDouble *mDirectionNoise;
  bool mNeedToConfigure;

  // private functions
  WbRadioNuclearDetector &operator=(const WbRadioNuclearDetector &);  // non copyable
  WbNode *clone() const override { return new WbRadioNuclearDetector(*this); }
  void init();

  bool checkApertureAndRange(const WbRadioNuclearSource *source, const WbRadioNuclearDetector *detector, bool checkRangeOnly = false);

private slots:
  void updateRadiationSetup();

};

#endif  // WB_RADIO_NUCLEAR_DETECTOR_HPP