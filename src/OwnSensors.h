#ifndef _OWNSENSORS_h
#define _OWNSENSORS_h

#include "WSBase.h"
#include "BMP180.h"
#include "BME280.h"
#include "DHTxx.h"
#include "LM75.h"
#include "SHT75.h"

class OwnSensors : public WSBase {
public: 
  OwnSensors();
  bool TryInitialize(bool tryDHT, bool trySHT75);
  bool HasBMP180();
  bool HasBME280();
  bool HasDHT22();
  bool HasLM75();
  bool HasSHT75();
  WSBase::Frame GetDataFrame();
  void Measure();
  String GetFhemDataString();
  void SetAltitudeAboveSeaLevel(int altitude);
  void SetCorrections(String corrT, String corrH);
  void SetID(byte id);
  BME280 *GetBME280Instance();
  BMP180 *GetBMP180Instance();
  DHTxx *GetDHTxxInstance();
  LM75 *GetLM75Instance();
  SHT75 *GetSHT75Instance();

protected:
  enum CorrectionMode {
    offset,
    percent
  };

  byte m_ID;
  WSBase::Frame m_data;
  bool m_hasBMP180;
  bool m_hasBME280;
  bool m_hasDHT22;
  bool m_hasLM75;
  bool m_hasSHT75;
  float m_corrTV;
  float m_corrHV;
  CorrectionMode m_corrTM;
  CorrectionMode m_corrHM;
  
  BMP180 m_bmp;
  BME280 m_bme;
  DHTxx m_dht;
  LM75 m_lm75;
  SHT75 m_sht75;
  
  unsigned long m_lastMeasurement;
};


#endif

