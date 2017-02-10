#ifndef _ANALOGPORT_h
#define _ANALOGPORT_h

#include "Arduino.h"

class AnalogPort {
public: 
  AnalogPort();
  bool TryInitialize();
  String GetFhemDataString();
  word GetLastValue();
  bool IsEnabled();


protected:
  unsigned long m_lastMeasurement;
  word m_lastValue;
  bool m_enabled;
};


#endif

