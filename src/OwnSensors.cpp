#include "OwnSensors.h"

OwnSensors::OwnSensors() {
  m_hasBMP180 = false;
  m_hasBME280 = false;
  m_hasDHT22 = false;
  m_hasLM75 = false;
  m_hasSHT75 = false;
  m_lastMeasurement = 0;
  m_ID = 0;
}

bool OwnSensors::TryInitialize(bool tryDHT, bool trySHT75) {
  SetAltitudeAboveSeaLevel(0);
  
  m_hasBME280 = m_bme.TryInitialize(0x76);
  m_hasBMP180 = m_bmp.TryInitialize();
  if (tryDHT) {
    while (millis() < 2000) {
      delay(10);
    }
    m_hasDHT22 = m_dht.TryInitialize(0);
  }
  if (trySHT75 && !m_hasDHT22) {
    m_hasSHT75 = m_sht75.TryInitialize(D3, D4);
  }
  m_hasLM75 = m_lm75.TryInitialize(0x4F);

  return m_hasSHT75 || m_hasBMP180 || m_hasBME280 || m_hasDHT22 || m_hasLM75;
}

bool OwnSensors::HasBMP180() {
  return m_hasBMP180;
}

bool OwnSensors::HasBME280() {
  return m_hasBME280;
}

bool OwnSensors::HasDHT22() {
  return m_hasDHT22;
}

bool OwnSensors::HasLM75() {
  return m_hasLM75;
}

bool OwnSensors::HasSHT75() {
  return m_hasSHT75;
}

void OwnSensors::SetAltitudeAboveSeaLevel(int altitude) {
  m_bmp.SetAltitudeAboveSeaLevel(altitude);
  m_bme.SetAltitudeAboveSeaLevel(altitude);
}

void OwnSensors::SetCorrections(String corrT, String corrH) {
  if (corrT.indexOf('%') != -1) {
    m_corrTM = CorrectionMode::percent;
    corrT.replace("%", "");
    m_corrTV = corrT.toFloat() / 100.0;
  }
  else {
    m_corrTM = CorrectionMode::offset;
    m_corrTV = corrT.toFloat();
  }

  if (corrH.indexOf('%') != -1) {
    m_corrHM = CorrectionMode::percent;
    corrH.replace("%", "");
    m_corrHV = corrH.toFloat() / 100.0;
  }
  else {
    m_corrHM = CorrectionMode::offset;
    m_corrHV = corrH.toFloat();
  }

}

void OwnSensors::SetID(byte id) {
  m_ID = id;
}

BME280* OwnSensors::GetBME280Instance(){
  return &m_bme;
}

BMP180* OwnSensors::GetBMP180Instance() {
  return &m_bmp;
}

DHTxx* OwnSensors::GetDHTxxInstance() {
  return &m_dht;
}

LM75 * OwnSensors::GetLM75Instance(){
  return &m_lm75;
}

SHT75 * OwnSensors::GetSHT75Instance() {
  return &m_sht75;
}

WSBase::Frame OwnSensors::GetDataFrame() {
  return m_data;
}

void OwnSensors::Measure() {
  m_data.ID = m_ID;
  m_data.CRC = 0;
  m_data.LowBatteryFlag = false;
  m_data.NewBatteryFlag = false;
  m_data.ErrorFlag = false;
  m_data.IsValid = false;
  m_data.HasHumidity = false;
  m_data.HasPressure = false;
  m_data.HasRain = false;
  m_data.HasTemperature = false;
  m_data.HasWindDirection = false;
  m_data.HasWindGust = false;
  m_data.HasWindSpeed = false;

  float bmeTemperature = 0.0, bmpTemperature = 0.0, dhtTemperature = 0.0, lm75Temperature = 0.0, sht75Temperature = 0.0;
  int bmeHumidity = 0, dhtHumidity = 0, sht75Hunidity = 0;
  int bmePressure = 0, bmpPressure = 0;

  if (m_hasSHT75) {
    sht75Temperature = round(m_sht75.GetTemperature() * 10.0) / 10.0;
    sht75Hunidity = round(m_sht75.GetHumidity());
  }
  if (m_hasBME280) {
    bmeTemperature = m_bme.GetTemperature();
    bmePressure = m_bme.GetPressure();
    bmeHumidity = m_bme.GetHumidity();
  }
  if (m_hasBMP180) {
    bmpTemperature = m_bmp.GetTemperature();
    bmpPressure = m_bmp.GetPressure();
  }
  if (m_hasDHT22 && m_dht.TryMeasure()) {
    dhtTemperature = m_dht.GetTemperature();
    dhtHumidity = m_dht.GetHumidity();
  }
  if (m_hasLM75) {
    lm75Temperature = m_lm75.GetTemperature();
  }
  if (m_hasBME280) {
    m_data.HasPressure = true;
    m_data.HasTemperature = true;
    m_data.HasHumidity = true;
    m_data.Temperature = bmeTemperature;
    m_data.Pressure = bmePressure;
    m_data.Humidity = bmeHumidity;
    m_data.IsValid = true;
  }

  if (!m_hasBME280 && m_hasBMP180) {
    m_data.HasPressure = true;
    m_data.HasTemperature = true;
    m_data.Temperature = bmpTemperature;
    m_data.Pressure = bmpPressure;
    m_data.IsValid = true;
  }

  if (!m_hasBME280 && m_hasDHT22) {
    m_data.HasTemperature = true;
    m_data.HasHumidity = true;
    m_data.Humidity = dhtHumidity;
    m_data.Temperature = dhtTemperature;
    m_data.IsValid = true;
  }

  if (!m_hasBME280 && !m_hasBMP180 && !m_hasDHT22 && m_hasLM75) {
    m_data.HasTemperature = true;
    m_data.Temperature = lm75Temperature;
    m_data.IsValid = true;
  }

  if (m_hasSHT75) {
    m_data.HasTemperature = true;
    m_data.HasHumidity = true;
    m_data.Temperature = sht75Temperature;
    m_data.Humidity = sht75Hunidity;
    m_data.IsValid = true;
  }

  if (m_data.HasTemperature) {
    m_data.Temperature = m_corrTM == CorrectionMode::offset ? m_data.Temperature + m_corrTV : m_data.Temperature + m_data.Temperature * m_corrTV;
  }
  if (m_data.HasHumidity) {
    m_data.Humidity = m_corrHM == CorrectionMode::offset ? m_data.Humidity + m_corrHV : m_data.Humidity + m_data.Humidity * m_corrHV;
  }

}

String OwnSensors::GetFhemDataString(){
  String fhemString = "";
   
  if (m_data.IsValid) {
    fhemString = BuildFhemDataString(&m_data, 4);
  }

  return fhemString;
}



 