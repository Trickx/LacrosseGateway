#ifndef _STATEMANAGER_h
#define _STATEMANAGER_h

#include "Arduino.h"
#include "HashMap.h"

#define TIMES_SIZE 200

class StateManager {
private:
  unsigned long m_lastKVPUpdate = 0;
  unsigned long m_lastFullKVPUpdate = 0;
  unsigned long m_times[TIMES_SIZE];
  String m_version;
  String m_identity;
  String m_hostname;
  unsigned long m_receivedFrames = 0;
  unsigned int m_framesPerMinute = 0;
   
  unsigned long m_loopCount = 0;
  unsigned long m_loopTotalTime = 0;
  unsigned long m_loopMeasureStart = 0;
  unsigned long m_loopStartTime = 0;
  word m_loopMinTime = 64000;
  word m_loopMaxTime = 0;
  String m_displayStatus = "";
  float m_wifiConnnectTime = 0.0;
   
  word m_loopDurationMin, m_loopDurationAvg, m_loopDurationMax;
  HashMap<String, String, 20> m_values;
  bool m_roolOverIsPossible = false;
  unsigned int m_uptimeDays = 0;

  void Update();

   
public:
  StateManager();
  void SetLoopStart();
  void SetLoopEnd();
  void SetDisplayStatus(String status);
  void SetHostname(String hostname);
  String GetHostname();
  unsigned int GetFramesPerMinute();
  void Begin(String version, String identity);
  String GetHTML();
  String GetXML();
  String GetKVP(bool full=true);
  String GetKVP(uint interval);
  String GetVersion();
  void Handle(byte receivedPackets);
  void ResetLastFullKVPUpdate();
  void SetWiFiConnectTime(float connectTime);
  float GetWiFiConnectTime();
};

#endif

