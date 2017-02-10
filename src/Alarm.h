#ifndef _ALARM_h
#define _ALARM_h
#include "Arduino.h"

class Alarm {
public:
  typedef byte TPinCallback(byte command, byte pin, byte value);
  Alarm(byte pin, Alarm::TPinCallback pinFunction = nullptr);
  void Begin();
  void Handle();
  void Set(byte sequence, word duration);

private:
  Alarm::TPinCallback *m_pinCallback;
  byte m_pin;
  byte m_sequence;
  word m_duration;
  bool m_active;
  unsigned long m_lastToggle;
  unsigned long m_startTime;

  void On();
  void Off();

};
#endif

