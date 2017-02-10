#include "AnalogPort.h"

AnalogPort::AnalogPort() {
  m_enabled = false;
}

bool AnalogPort::TryInitialize() {
  m_enabled = true;
  return true;
}

 String AnalogPort::GetFhemDataString(){
   String result = "";
   
   if (millis() < m_lastMeasurement ) {
     m_lastMeasurement = 0;
   }

   if (millis() >= m_lastMeasurement + 4000) {
     m_lastMeasurement = millis();
     result += "LGW ANALOG";

     m_lastValue = analogRead(A0);
     result += " ";
     result += (byte)(m_lastValue >> 8);
     result += " ";
     result += (byte)(m_lastValue);
   }

   return result;
}

 word AnalogPort::GetLastValue() {
   return m_lastValue;
 }

 bool AnalogPort::IsEnabled() {
   return m_enabled;
 }
