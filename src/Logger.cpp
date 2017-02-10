#include "Logger.h"

Logger::Logger() {
  m_enabled = true;
  m_bufferSize = 40;
}

void Logger::Clear() {
  while (Available()) {
    Pop();
  }
}

void Logger::Disable() {
  m_enabled = false;
  Clear();
}

void Logger::Enable() {
  m_enabled = true;
}

bool Logger::IsEnabled() {
  return m_enabled;
}

void Logger::println(LogType type) {
  println("", type);
}

void Logger::print(uint32_t data, LogType type) {
  print(String(data), type);
}
void Logger::println(uint32_t data, LogType type) {
  println(String(data), type);
}






void Logger::logData(String data, LogType type) {
  if (m_enabled &&  m_queue.Count() <= m_bufferSize) {
    m_queue.Push("DATA:" + data);
  }
}

void Logger::print(String data, LogType type) {
  if (m_enabled) {
    Serial.print(data);
  }
  if (m_enabled &&  m_queue.Count() <= m_bufferSize) {
    m_currentLine += data;
  }
}

void Logger::println(String data, LogType type) {
  if (m_enabled) {
    Serial.println(data);
  }
  String line = m_currentLine + data;
  if (m_enabled &&  m_queue.Count() <= m_bufferSize) {
    switch (type) {
      case Logger::SYS:
        line = "SYS: " + line;
        break;
      case Logger::DATA:
        line = "DATA: " + line;
        break;
      case Logger::PCA301:
        line = "PCA301: " + line;
        break;
      default:
        break;
    }
    m_queue.Push(line);
  }
  m_currentLine = "";
}

int Logger::Available()  {
  return m_queue.Count();
}

String Logger::Pop() {
  return m_queue.Pop();
}