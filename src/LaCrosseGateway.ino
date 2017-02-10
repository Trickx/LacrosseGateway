#define PROGNAME         "LaCrosseITPlusReader.Gateway"
#define PROGVERS         "1.25"

#define RFM1_SS          15
#define RFM2_SS          2
#define RFM3_SS          0
#define LED_PIN          16

// Arduino / ESP8266 libs
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "EEPROM.h"
#include "IPAddress.h"
#include "ESP8266WiFiType.h"
#include "ESP8266WiFi.h"
#include "WiFiClient.h"
#include "ESP8266WebServer.h"
#include "pins_arduino.h"
#include "ESP8266HTTPClient.h"
#include "ESP8266httpUpdate.h"
#include "WiFiUdp.h"
#include "ESP8266mDNS.h"
#include "ArduinoOTA.h"
#include "Ticker.h"


// Other libs
#include "ArrayList.h"
#include "BMP180.h"
#include "RFMxx.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#include "EMT7110.h"
#include "TX38IT.h"
#include "WSBase.h"
#include "WS1080.h"
#include "TX22IT.h"
#include "WT440XH.h"
#include "CustomSensor.h"
#include "LevelSenderLib.h"
#include "ESPTools.h"
#include "TypedQueue.h"
#include "HashMap.h"
#include "HTML.h"
#include "DHTxx.h"
#include "I2CBase.h"
#include "BME280.h"
#include "LM75.h"
#include "EC3000.h"
#include "ESP8266OTA.h"
#include "SC16IS750.h"
#include "SubProcessor.h"
#include "OLED.h"
#include "OLEDFonts.h"
#include "MCP23008.h"
#include "DigitalPorts.h"
#include "SHT75.h"
#include "PCA301Plug.h"
#include "PCA301.h"
#include "PCA301PlugList.h"
#include "ESP8266SoftSerial.h"

// LGW libraries
#include "XBM.h"
#include "Alarm.h"
#include "Logger.h"
#include "SerialPortFlasher.h"

// In this sketch
#include "AccessPoint.h"
#include "WebFrontend.h"
#include "DataPort.h"
#include "Settings.h"
#include "StateManager.h"
#include "OTAUpdate.h"
#include "OwnSensors.h"
#include "SerialBridge.h"
#include "SoftSerialBridge.h"
#include "Nextion.h"
#include "Display.h"
#include "DisplayAreas.h"
#include "DisplayValues.h"
#include "HardwarePageBuilder.h"
#include "Watchdog.h"
#include "AnalogPort.h"


extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}

// The following settings can also be set from FHEM
#define ENABLE_ACTIVITY_LED    1         // <n>a       set to 0 if the blue LED bothers
                                         // <n,d>b     Alert n beeps for d seconds
unsigned long DATA_RATE_S1   = 17241ul;  // <n>c       use one of the possible data rates (for transmit on RFM #1)
bool DEBUG                   = 0;        // <n>d       set to 1 to see debug messages
                                         // <8266>e    Clear EEPROM
                                         // <8377>e    Reboot
unsigned long INITIAL_FREQ   = 868300;   // <n>f       initial frequency in kHz (5 kHz steps, 860480 ... 879515) 
                                         // <n>g       get Information: 1 = settings
                                         // <n>h       Altitude
                                         // <n,f,i>i   Init PCA for Radio #<n> to <m>MHz and <i>s Interval
byte TOGGLE_MODE_R1          = 3;        // <n>m       bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #1)
byte TOGGLE_MODE_R2          = 3;        // <n>M       bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #2)
byte TOGGLE_MODE_R3          = 3;        // <n>#<x>m   bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #3)
byte TOGGLE_MODE_R4          = 3;        // <n>#<x>m   bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #4)
byte TOGGLE_MODE_R5          = 3;        // <n>#<x>m   bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #5)

                                         // <n>o       set HF-parameter e.g. 50305o for RFM12 or 1,4o for RFM69
byte PASS_PAYLOAD            = 0;        // <n>p       transmitted the payload on the serial port 1: all, 2: only undecoded data
unsigned long DATA_RATE_R1   = 17241ul;  // <n>r       use one of the possible data rates (for RFM #1)
unsigned long DATA_RATE_R2   = 9579ul;   // <n>R       use one of the possible data rates (for RFM #2)
unsigned long DATA_RATE_R3   = 8842ul;   // <n>#<x>r   use one of the possible data rates (for RFM #3)
unsigned long DATA_RATE_R4   = 20000ul;  // <n>#<x>r   use one of the possible data rates (for RFM #4)
unsigned long DATA_RATE_R5   = 111ul;    // <n>#<x>r   use one of the possible data rates (for RFM #5)


                                         // <x,x,...>s Send to PCA301 (must be 10 byte)
                                         // <x,x,...>S Send to CustomSensor

uint16_t TOGGLE_INTERVAL_R1  = 0;        // <n>t       0=no toggle, else interval in seconds (for RFM #1)
uint16_t TOGGLE_INTERVAL_R2  = 0;        // <n>T       0=no toggle, else interval in seconds (for RFM #2)
uint16_t TOGGLE_INTERVAL_R3  = 0;        // <n>#<x>t   0=no toggle, else interval in seconds (for RFM #3)
uint16_t TOGGLE_INTERVAL_R4  = 0;        // <n>#<x>t   0=no toggle, else interval in seconds (for RFM #4)
uint16_t TOGGLE_INTERVAL_R5  = 0;        // <n>#<x>t   0=no toggle, else interval in seconds (for RFM #5)

                                         // <xxxx>u    Send xxxx to SubProcessor
                                         // v          show version
bool USE_WIFI                = 1;        // <n>w       0=no wifi
                                         // x          test command 
byte ANALYZE_FRAMES          = 0;        // <n>z       set to 1 to display analyzed frame data instead of the normal data
uint KVINTERVAL              = 10;       //            Interval for KV-transmision

bool USE_SERIAL              = 1;        //            0=do not send sensor data on the serial
bool DEBUG_PCA301            = 0;        //            debug PCA301

// wifi settings
#define FRONTEND_PORT          80        // Port for the Web frontend
#define DATA_PORT1             81        // Port for data
#define DATA_PORT2             82        // Port for data
#define DATA_PORT3             83        // Port for data
#define OTA_PORT               8266
IPAddress AP_IP              = IPAddress(192, 168, 222, 1);
IPAddress AP_SUBNET          = IPAddress (255, 255, 225, 0);

byte AddOnPinHandler(byte command, byte pin, byte value);

// --- Variables -------------------------------------------------------------------------------------------------------
unsigned long lastToggleR1 = 0;
unsigned long lastToggleR2 = 0;
unsigned long lastToggleR3 = 0;
unsigned long lastToggleR4 = 0;
unsigned long lastToggleR5 = 0;
unsigned long commandData[32];
byte commandDataPointer = 0;
ESPTools esp(LED_PIN);
SC16IS750 sc16is750 = SC16IS750(SC16IS750_MODE_I2C, 0x90);
SC16IS750 sc16is750_2 = SC16IS750(SC16IS750_MODE_I2C, 0x92);

RFMxx rfm1(13, 12, 14, RFM1_SS);
RFMxx rfm2(13, 12, 14, RFM2_SS);
RFMxx rfm3(13, 12, 14, RFM3_SS);
RFMxx rfm4(13, 12, 14, 0, -1, AddOnPinHandler);
RFMxx rfm5(13, 12, 14, 1, -1, AddOnPinHandler);
RFMxx* rfms[5];

Alarm alarm(7, AddOnPinHandler);

Logger logger;
OwnSensors ownSensors;
WebFrontend frontend(FRONTEND_PORT);
DataPort dataPort1;
DataPort dataPort2;
DataPort dataPort3;
AccessPoint accessPoint (AP_IP, AP_IP, AP_SUBNET, "LaCrosseGateway");
StateManager stateManager;
PCA301 pca301;
ESP8266OTA ota;
unsigned int lastOtaProgress;
SubProcessor subProcessor(&sc16is750, 5, "addon.hex");
SerialBridge serialBridge(&sc16is750, 5, "addon.hex");
SubProcessor subProcessor2(&sc16is750_2, 5, "addon2.hex");
SerialBridge serialBridge2(&sc16is750_2, 5, "addon2.hex");
bool useSerialBridge = false;
bool useSerialBridge2 = false;
SoftSerialBridge softSerialBridge;
Nextion nextion;
DHTxx dht;
Ticker ticker;
bool isSC16IS750clone = false;
Display display;
DigitalPorts digitalPorts;
unsigned long lastSensorMeasurement = 0;
bool dataPort1Connected = false;
bool dataPort2Connected = false;
bool dataPort3Connected = false;
bool bridge1Connected = false;
bool bridge2Connected = false;
bool bridge3Connected = false;
Watchdog watchdog;
SerialPortFlasher serialPortFlasher;
AnalogPort analogPort;

byte AddOnPinHandler(byte command, byte pin, byte value) {
  byte result = 1;
  if (sc16is750.IsConnected()) {
    switch (command) {
    case 1:
      sc16is750.PinMode(pin, value);
      break;
    case 2:
      sc16is750.DigitalWrite(pin, value);
      break;
    case 3:
      result = sc16is750.DigitalRead(pin);
      break;
    default:
      break;
    }
  }

  return result;
}

void SetDebugMode(boolean mode) {
  DEBUG = mode;
  SensorBase::SetDebugMode(mode);
  rfm1.SetDebugMode(mode);
  rfm2.SetDebugMode(mode);
  rfm3.SetDebugMode(mode);
  rfm4.SetDebugMode(mode);
  rfm5.SetDebugMode(mode);
}

void Dispatch(String data, String raw="") {
  if(USE_SERIAL) {
    Serial.println(data);
  }
  
  if (USE_WIFI) {
    dataPort1.AddPayload(data);
    dataPort2.AddPayload(data);
    dataPort3.AddPayload(data);
  }

  if (raw.length() > 0) {
    raw = " [" + raw + "]";
  }

  if (data.startsWith("\n")) {
    data = data.substring(1);
  }
  logger.logData(data + raw);

  esp.Blink(1);
}

void SetDataRate(RFMxx *rfm, unsigned long dataRate) {
  if(rfm->GetDataRate() != 20000 && dataRate == 20000) {
    rfm->InitializeEC3000();
    rfm->EnableReceiver(true);
  }
  else if(rfm->GetDataRate() == 20000 && dataRate != 20000) {
    rfm->InitializeLaCrosse();
    rfm->EnableReceiver(true);
  }
  

  rfm->SetDataRate(dataRate);
}

static void HandleSerialPort(char c) {
  static unsigned long value;
  static bool rfmNumberFlag;
  static byte rfmNumber = 0;
  unsigned long dataRate = 0;
  static String commandString = "";
  static bool commandStringFlag = false;

  if(c != '"' && commandStringFlag) {
    commandString += c;
  }
  else if (c == '"') {
    if(!commandStringFlag) {
      commandStringFlag = true;
    }
    else {
      commandStringFlag = false;
      HandleCommandString(commandString);
      commandString = "";
    }
  }
  else if (c == ',') {
    commandData[commandDataPointer++] = value;
    value = 0;
  }
  else if(rfmNumberFlag) {
    rfmNumber = c - '0';
    rfmNumberFlag = false;
  }
  else if(c == '#') {
    rfmNumberFlag = true;
  }
  else if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  }
  else if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
    if (rfmNumber == 0) {
      rfmNumber = ('A' <= c && c <= 'Z') ? 2 : 1;
    }
  
    switch (c) {
    case 'a':
      // Activity LED    
      esp.EnableLED(value);
      break;

    case 'b':
      // Alarm 
      if (sc16is750.IsConnected()) {
        commandData[commandDataPointer] = value;
        if (++commandDataPointer == 2) {
          alarm.Set(commandData[0], commandData[1]);
        }
        else {
          alarm.Set(0, value);
        }
      }
      commandDataPointer = 0;
      break;

    case 'd':
      // DEBUG
      SetDebugMode(value);
      break;

    case 'e':
      if (value == 8266) {
        EEPROM.begin(EEPROM_SIZE);
        for (int i = 0; i < EEPROM_SIZE; i++) {
          EEPROM.write(i, 0);
        }
        EEPROM.commit();
        EEPROM.end();
        delay(500);
        ESP.restart();
        break;
      }
      else if (value == 8377) {
        ESP.restart();
      }
    
    case 'g':
      if (value == 1) {
        Settings settings;
        settings.Read();
        Dispatch(settings.ToString());
      }
      break;
      
    case 'h':
      // height
      ownSensors.SetAltitudeAboveSeaLevel(value);
      break;
    case 'x':
    case 'X':
      // Tests
      HandleCommandX(value);
      break;
    case 'w':
      // wifi    
      if (!value) {
        StopWifi();
      }
      break;
    case 'r':
    case 'R':
      // Data rate
      switch (value) {
      case 0:
        dataRate = 17241ul;
        break;
      case 1:
        dataRate = 9579ul;
        break;
      case 2:
        dataRate = 8842ul;
        break;
      case 3:
        dataRate = 20000ul;
        break;
      default:
        dataRate = value;
        break;
      }
      if (rfmNumber == 1 && rfm1.IsConnected()) {
        DATA_RATE_R1 = dataRate;
        SetDataRate(&rfm1, DATA_RATE_R1);
      }
      else if (rfmNumber == 2 && rfm2.IsConnected()) {
        DATA_RATE_R2 = dataRate;
        SetDataRate(&rfm2, DATA_RATE_R2);
      }
      else if (rfmNumber == 3 && rfm3.IsConnected()) {
        DATA_RATE_R3 = dataRate;
        SetDataRate(&rfm3, DATA_RATE_R3);
      }
      else if (rfmNumber == 4 && rfm4.IsConnected()) {
        DATA_RATE_R4 = dataRate;
        SetDataRate(&rfm4, DATA_RATE_R4);
      }
      else if (rfmNumber == 5 && rfm5.IsConnected()) {
        DATA_RATE_R5 = dataRate;
        SetDataRate(&rfm5, DATA_RATE_R5);
      }

      break;
    case 'm':
    case 'M':
      if (rfmNumber == 1 && rfm1.IsConnected()) {
        rfm1.ToggleMode = value;
      }
      else if (rfmNumber == 2 && rfm2.IsConnected()) {
        rfm2.ToggleMode = value;
      }
      else if (rfmNumber == 3 && rfm3.IsConnected()) {
        rfm3.ToggleMode = value;
      }
      else if (rfmNumber == 4 && rfm4.IsConnected()) {
        rfm4.ToggleMode = value;
      }
      else if (rfmNumber == 5 && rfm5.IsConnected()) {
        rfm5.ToggleMode = value;
      }
      break;
    case 'o':
    case 'O':
      // Set HF parameter
      commandData[commandDataPointer] = value;
      HandleCommandO(rfmNumber, value, commandData, ++commandDataPointer);
      break;
    case 'p':
      PASS_PAYLOAD = value;
      break;
    case 's':
      // Send
      commandData[commandDataPointer] = value;
      HandlePCA301Send(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;
    case 'S':
      // Send
      commandData[commandDataPointer] = value;
      HandleCommandS(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;
    case 'i':
      // Init PCA301
      commandData[commandDataPointer] = value;
      HandleCommandI(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;
    case 't':
    case 'T':
      // Toggle data rate
      if (rfmNumber == 1 && rfm1.IsConnected()) {
        rfm1.ToggleInterval = value;
      }
      else if (rfmNumber == 2 && rfm2.IsConnected()) {
        rfm2.ToggleInterval = value;
      }
      if (rfmNumber == 3 && rfm3.IsConnected()) {
        rfm3.ToggleInterval = value;
      }
      if (rfmNumber == 4 && rfm4.IsConnected()) {
        rfm4.ToggleInterval = value;
      }
      if (rfmNumber == 5 && rfm5.IsConnected()) {
        rfm5.ToggleInterval = value;
      }
      break;
    case 'v':
      // Version info
      HandleCommandV();
      break;

    case 'f':
    case 'F':
      if (rfmNumber == 1 && rfm1.IsConnected()) {
        rfm1.SetFrequency(value); 
      }
      else if (rfmNumber == 2 && rfm2.IsConnected()) {
        rfm2.SetFrequency(value);
      }
      else if (rfmNumber == 3 && rfm3.IsConnected()) {
        rfm3.SetFrequency(value);
      }
      else if (rfmNumber == 4 && rfm4.IsConnected()) {
        rfm4.SetFrequency(value);
      }
      else if (rfmNumber == 5 && rfm5.IsConnected()) {
        rfm5.SetFrequency(value);
      }
      break;

    case 'z':
      ANALYZE_FRAMES = (byte)value;
      break;

    default:
      HandleCommandV();
      break;
    }
    value = 0;
    rfmNumberFlag = false;
    rfmNumber = 0;
  }
  else if (' ' < c && c < 'A') {
    HandleCommandV();
  }
  
}

void HandleCommandString(String command) {
  String upperCommand = command;
  upperCommand.toUpperCase();
  if (upperCommand.startsWith("OLED ") && display.IsConnected()){
    display.Command(command.substring(5));
  }
  else if (upperCommand.startsWith("MCP ") && digitalPorts.IsConnected()){
    digitalPorts.Command(command.substring(4));
  }
  else if (upperCommand.startsWith("SETUP ")){
    Settings settings;
    settings.Read();
    settings.FromString(command.substring(6));
    settings.Write();
  }
  else if (upperCommand.startsWith("WATCHDOG ")) {
    watchdog.Command(command.substring(9));
  }
  else if (upperCommand.startsWith("FIRMWARE")) {
    serialPortFlasher.Begin();
  }
}

void HandleCommandI(unsigned long *commandData, byte length){
  word interval = 60;
  if (length == 3) {
    interval = commandData[2];
  }

  if (length >= 2) {
    RFMxx *rfm;
    switch (commandData[0]) {
    case 1:
      rfm = &rfm1;
      break;
    case 2:
      rfm = &rfm2;
      break;
    case 3:
      rfm = &rfm3;
      break;
    case 4:
      rfm = &rfm4;
      break;
    case 5:
      rfm = &rfm5;
      break;
    default:
      rfm = NULL;
      break;
    }
    if (rfm != NULL){
      pca301.Begin(rfm, commandData[1], interval, [](String key, String value, bool write) {
        String result = "";
        Settings settings;
        if (write) {
          settings.Read();
          settings.Add(key, value);
          settings.Write();
        }
        else {
          settings.Read();
          result = settings.Get(key, value);
        }

        return result;
      });
    }
  }
  
}


void onTick() {
  static bool isOnDummy;
  if (sc16is750.IsConnected()) {
    sc16is750.PinMode(6, OUTPUT);
    sc16is750.DigitalWrite(6, isOnDummy);

    sc16is750.PinMode(3, OUTPUT);
    sc16is750.DigitalWrite(3, !isOnDummy);

    isOnDummy = !isOnDummy;
  }
}

// This function is for testing 
void HandleCommandX(byte value) {
  if(value == 1) {
    if (sc16is750.Begin(57600, false)) {
      logger.println("SC16IS750 detected");
    }
    else {
      logger.println("no SC16IS750");
    }
  }
  else if (value == 2) {
    ticker.attach(0.5, onTick);    
  }
  else if (value == 3) {
    if (sc16is750.IsConnected()) {
      rfm4.Begin();
      if (rfm4.IsConnected()) {
        logger.println("RFM #4 found");
      }
      else {
        logger.println("no RFM #4");
      }
    }
  }
  else if (value == 4) {
    
  }
  else if (value == 22) {
    String dhtTest = dht.TryInitialize(0) ? "OK :-)" : "not found :-(";
    logger.println("DHT22: " + dhtTest);
  }
  else {

  }

}

void HandleCommandS(unsigned long *data, byte size) {
  rfm1.EnableReceiver(false);

  struct CustomSensor::Frame frame;
  frame.ID = data[0];
  frame.NbrOfDataBytes = size - 1;

  for (int i = 0; i < frame.NbrOfDataBytes; i++) {
    frame.Data[i] = data[i + 1];
  }

  CustomSensor::SendFrame(&frame, &rfm1, DATA_RATE_S1);

  rfm1.EnableReceiver(true);
}

void HandlePCA301Send(unsigned long *data, byte size) {
  if (size == 10 && pca301.IsInitialized()) {
    pca301.GetUsedRadio()->EnableReceiver(false);

    byte payload[10];
    for (int i = 0; i < 10; i++) {
      payload[i] = data[i];
    }
    pca301.SendPayload(payload, false);

    pca301.GetUsedRadio()->EnableReceiver(true);

  }
}

RFMxx *GetRfmForNumber(byte number) {
RFMxx *result = NULL;
  if (number == 1) {
    result = &rfm1;
  }
  else if (number == 2) {
    result = &rfm2;
  }
  else if (number == 3) {
    result = &rfm3;
  }
  else if (number == 4) {
    result = &rfm4;
  }
  else if (number == 5) {
    result = &rfm5;
  }

  return result;
}

void HandleCommandO(byte rfmNbr, unsigned long value, unsigned long *data, byte size) {
  // 50305o (is 0xC481) for RFM12 or 1,4o for RFM69
  RFMxx *rfm = GetRfmForNumber(rfmNbr);
  if (size == 1 && rfm->GetRadioType() == RFMxx::RFM12B) {
    rfm->SetHFParameter(value);
  }
  else if (size == 2 && rfm->GetRadioType() == RFMxx::RFM69CW) {
    rfm->SetHFParameter(data[0], data[1]);
  }
}

void HandleCommandV() {
  String result = "\n";

  result += "[";
  result += PROGNAME;
  result += ".";
  result += PROGVERS;

  if (rfm1.IsConnected()) {
    result += " (1=";
    result += rfm1.GetRadioName();
    result += " f:";
    result += rfm1.GetFrequency();

    if (rfm1.ToggleInterval) {
      result += " t:";
      result += rfm1.ToggleInterval;
      result += "~";
      result += rfm1.ToggleMode;
    }
    else {
      result += " r:";
      result += rfm1.GetDataRate();
    }
    result += ")";
  }

  if (rfm2.IsConnected()) {
    result += " + (2=";
    result += rfm2.GetRadioName();
    result += " f:";
    result += rfm2.GetFrequency();
    if (rfm2.ToggleInterval) {
      result += " t:";
      result += rfm2.ToggleInterval;
      result += "~";
      result += rfm2.ToggleMode;
    }
    else {
      result += " r:";
      result += rfm2.GetDataRate();

    }
    result += ")";
  }

  if (rfm3.IsConnected()) {
    result += " + (3=";
    result += rfm3.GetRadioName();
    result += " f:";
    result += rfm3.GetFrequency();
    if (rfm3.ToggleInterval) {
      result += " t:";
      result += rfm3.ToggleInterval;
      result += "~";
      result += rfm3.ToggleMode;
    }
    else {
      result += " r:";
      result += rfm3.GetDataRate();

    }
    result += ")";
  }

  if (rfm4.IsConnected()) {
    result += " + (4=";
    result += rfm4.GetRadioName();
    result += " f:";
    result += rfm4.GetFrequency();
    if (rfm4.ToggleInterval) {
      result += " t:";
      result += rfm4.ToggleInterval;
      result += "~";
      result += rfm4.ToggleMode;
    }
    else {
      result += " r:";
      result += rfm4.GetDataRate();

    }
    result += ")";
  }

  if (rfm5.IsConnected()) {
    result += " + (5=";
    result += rfm5.GetRadioName();
    result += " f:";
    result += rfm5.GetFrequency();
    if (rfm5.ToggleInterval) {
      result += " t:";
      result += rfm5.ToggleInterval;
      result += "~";
      result += rfm5.ToggleMode;
    }
    else {
      result += " r:";
      result += rfm5.GetDataRate();

    }
    result += ")";
  }

  if (ownSensors.HasSHT75()) {
    result += " + SHT75";
  }
  if (ownSensors.HasBMP180()) {
    result += " + BMP180";
  }
  if (ownSensors.HasBME280()) {
    result += " + BME280";
  }
  if (ownSensors.HasDHT22()) {
    result += " + DHT22";
  }
  if (ownSensors.HasLM75()) {
    result += " + LM75";
  }
  if (sc16is750.IsConnected() || sc16is750_2.IsConnected()) {
    result += " + SC16IS750";
    if (sc16is750.IsClone()) {
      result += "-Clone";
    }
    result += " (";
    if (sc16is750.IsConnected()) {
      result += "0x90";
    }
    if (sc16is750.IsConnected() && sc16is750_2.IsConnected()) {
      result += ", ";
    }
    if (sc16is750_2.IsConnected()) {
      result += "0x92";
    }
    result += ")";
  }
  if (display.IsConnected()) {
    result += " + OLED";
  }
  if (digitalPorts.IsConnected()) {
    result += " + MCP23008";
  }

  result += " {IP=";
  if (WiFi.status() == WL_CONNECTED) {
    result += WiFi.localIP().toString();
  }
  else if (!USE_WIFI) {
    result += "Disabled";
  }
  else {
    result += WiFi.softAPIP().toString();
  }
  result += "}";

  result += "]";
  
  Dispatch(result);

}

bool HandleReceivedData(RFMxx *rfm) {
  bool result = false;

  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);

  rfm->EnableReceiver(true);

  if (ANALYZE_FRAMES > 0) {
    unsigned long dataRate = rfm->GetDataRate();
    if (bitRead(ANALYZE_FRAMES, 0) && LaCrosse::IsValidDataRate(dataRate)) {
      logger.println(LaCrosse::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 1) && TX22IT::IsValidDataRate(dataRate)) {
      logger.println(TX22IT::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 2) && WS1080::IsValidDataRate(dataRate)) {
      logger.println(WS1080::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 3) && EMT7110::IsValidDataRate(dataRate)) {
      logger.println(EMT7110::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 4) && LevelSenderLib::IsValidDataRate(dataRate)) {
      logger.println(LevelSenderLib::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 5) && CustomSensor::IsValidDataRate(dataRate)) {
      logger.println(CustomSensor::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 6) && EC3000::IsValidDataRate(dataRate)) {
      logger.println(EC3000::AnalyzeFrame(payload));
    }
    if (bitRead(ANALYZE_FRAMES, 7) && PCA301::IsValidDataRate(dataRate) && pca301.IsInitialized()) {
      logger.println(pca301.AnalyzeFrame(payload));
    }
  }
  
  if (PASS_PAYLOAD == 1) {
    for (int i = 0; i < PAYLOADSIZE; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else {
    if (DEBUG) {
      Serial.print("\r\nEnd receiving, HEX raw data: ");
      for (int i = 0; i < 16; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      String raw = rfm->GetRadioName() + ": ";
      for (int i = 0; i < 5; i++) {
        String bt = String(payload[i], HEX);
        bt = bt.length() == 1 ? ("0" + bt) : bt;
        raw += bt;
        raw += " ";
      }
      raw.toUpperCase();
      logger.println(raw);
    }

    String data = "";
    byte frameLength = 16;

    // Try LaCrosse like TX29DTH
    if (data.length() == 0 && LaCrosse::IsValidDataRate(rfm->GetDataRate())) {
      data = LaCrosse::GetFhemDataString(payload);
      frameLength = LaCrosse::FRAME_LENGTH;
    }
    
    // Try EC3000
    if (data.length() == 0 && EC3000::IsValidDataRate(rfm->GetDataRate())) {
      data = EC3000::GetFhemDataString(payload);
      frameLength = EC3000::FRAME_LENGTH;
    }

    // Try TX22IT (WS 1600)
    if (data.length() == 0 && TX22IT::IsValidDataRate(rfm->GetDataRate())) {
      data = TX22IT::GetFhemDataString(payload);
      frameLength = TX22IT::GetFrameLength(payload);
    }

    // Try WS 1080
    if (data.length() == 0 && WS1080::IsValidDataRate(rfm->GetDataRate())) {
      data = WS1080::GetFhemDataString(payload);
      frameLength = WS1080::FRAME_LENGTH;
    }

    // Try LevelSender
    if (data.length() == 0 && LevelSenderLib::IsValidDataRate(rfm->GetDataRate())) {
      data = LevelSenderLib::GetFhemDataString(payload);
      frameLength = LevelSenderLib::FRAME_LENGTH;
    }

    // Try EMT7110
    if (data.length() == 0 && EMT7110::IsValidDataRate(rfm->GetDataRate())) {
      data = EMT7110::GetFhemDataString(payload);
      frameLength = EMT7110::FRAME_LENGTH;
    }

    // Try TX38IT
    if (data.length() == 0 && TX38IT::IsValidDataRate(rfm->GetDataRate())) {
      data = TX38IT::GetFhemDataString(payload);
      frameLength = TX38IT::FRAME_LENGTH;
    }

    // Try WT440XH
    if (data.length() == 0 && WT440XH::IsValidDataRate(rfm->GetDataRate())) {
      data = WT440XH::GetFhemDataString(payload);
      frameLength = WT440XH::FRAME_LENGTH;
    }

    // Try CustomSensor
    if (data.length() == 0 && CustomSensor::IsValidDataRate(rfm->GetDataRate())) {
      data = CustomSensor::GetFhemDataString(payload);
      frameLength = CustomSensor::GetFrameLength(payload);
    }
    
    // Try PCA301
    if (pca301.IsInitialized() && data.length() == 0 && PCA301::IsValidDataRate(rfm->GetDataRate())) {
      data = pca301.GetFhemDataString(payload);
      frameLength = 12;
    }

    if (data.length() > 0) {
      result = true;

      String raw = "";
      for (int i = 0; i < frameLength; i++) {
        String bt = String(payload[i], HEX);
        bt = bt.length() == 1 ? ("0" + bt) : bt;
        raw += bt;
        raw += i+1 < frameLength ? " " : "";
      }
      raw.toUpperCase();

      Dispatch(data, raw);
    }
  }

  
  
  return result;
}

void HandleDataRateToggle(RFMxx *rfm, unsigned long *lastToggle, unsigned long *dataRate) {
  if (rfm->ToggleInterval > 0) {
    // After about 50 days millis() will overflow to zero 
    if (millis() < *lastToggle) {
      *lastToggle = 0;
    }

    if (millis() > *lastToggle + rfm->ToggleInterval * 1000 && rfm->ToggleMode > 0) {
      // Bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps

      HashMap<unsigned long, unsigned long, 4> dataRates;
      if (rfm->ToggleMode & 1) {
        dataRates.Put(17241, 17241);
      }
      if (rfm->ToggleMode & 2) {
        dataRates.Put(9579, 9579);
        if (dataRates.Size() > 0) {
          *dataRates.GetValuePointerAt(dataRates.Size() - 2) = 9579;
        }
      }
      if (rfm->ToggleMode & 4) {
        dataRates.Put(8842, 8842);
        if (dataRates.Size() > 0) {
          *dataRates.GetValuePointerAt(dataRates.Size() - 2) = 8842;
        }
      }
      if (rfm->ToggleMode & 8) {
        dataRates.Put(20000, 20000);
        if (dataRates.Size() > 0) {
          *dataRates.GetValuePointerAt(dataRates.Size() - 2) = 20000;
        }
      }
      *dataRates.GetValuePointerAt(dataRates.Size() - 1) = dataRates.GetKeyAt(0);

      *dataRate = dataRates.Get(rfm->GetDataRate(), 0);
      if (*dataRate == 0) {
        *dataRate = 17241;
      }

      SetDataRate(rfm, *dataRate);
      *lastToggle = millis();

    }
  }
}

void HandleProgressRequest(byte action, unsigned long offset, unsigned long maxValue, String message) {
  if (display.IsConnected()) {
    switch (action) {
    case 1:
      display.PushContent();
      display.ShowProgress(maxValue, message);
      break;

    case 2:
      display.MoveProgress(offset);
      break;

    case 3:
      display.PopContent();
      break;

    default:
      break;
    }

  }

}

void TryConnectWIFI(String ctSSID, String ctPass, byte nbr, uint timeout) {
  if (ctSSID.length() > 0 && ctSSID != "---") {
    unsigned long startMillis = millis();

    WiFi.begin(ctSSID.c_str(), ctPass.c_str());
    logger.println("Connect " + String(timeout) + " seconds to an AP (SSID " + String(nbr) + ")");
    esp.SwitchLed(true, true);
    if (display.IsConnected()) {
      display.ShowProgress(timeout * 2, "Connect WiFi (" + String(nbr) + ")");
    }
    if (nextion.IsConnected()) {
      nextion.ShowProgress(timeout * 2, "Connect WiFi (" + String(nbr) + ")");
    }
    byte retryCounter = 0;
    while (retryCounter < timeout * 2 && WiFi.status() != WL_CONNECTED) {
      retryCounter++;
      delay(500);
      logger.print(".");
      esp.SwitchLed(retryCounter % 2 == 0, true);
      if (display.IsConnected()) {
        display.MoveProgress();
      }
      if (nextion.IsConnected()) {
        nextion.MoveProgress();
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      stateManager.SetWiFiConnectTime((millis() - startMillis) / 1000.0);
    }

    esp.SwitchLed(false, true);
    if (display.IsConnected()) {
      display.HideProgress();
    }
    if (nextion.IsConnected()) {
      ////nextion.HideProgress();
    }
    
  }

}

static bool StartWifi(Settings settings) {
  bool result = false;

  logger.println("Start WIFI_STA");
  WiFi.disconnect();
  WiFi.mode(WiFiMode::WIFI_STA);

  String hostName = settings.Get("HostName", "LaCrosseGateway");
  logger.print("HostName is: ");
  logger.println(hostName);
  WiFi.hostname(hostName);
  stateManager.SetHostname(hostName);

  String staticIP = settings.Get("staticIP", "");
  String staticMask = settings.Get("staticMask", "");
  String staticGW = settings.Get("staticGW", "");

  if (staticIP.length() < 7 || staticMask.length() < 7){
    logger.println("Using DHCP");
  }
  else {
    logger.println("Using static IP");
    logger.print("IP: ");
    logger.println(staticIP);
    logger.print("Mask: ");
    logger.println(staticMask);
    logger.print("Gateway: ");
    logger.println(staticGW);

    if (staticGW.length() < 7) {
      WiFi.config(HTML::IPAddressFromString(staticIP),
      HTML::IPAddressFromString(staticGW),
      (uint32_t)0);
    }
    else {
      WiFi.config(HTML::IPAddressFromString(staticIP),
                  HTML::IPAddressFromString(staticGW),
                  HTML::IPAddressFromString(staticMask));
    }
  }

  TryConnectWIFI(settings.Get("ctSSID", "---"), settings.Get("ctPASS", "---"), 1, settings.GetInt("Timeout1", 15));
  String ctSSID2 = settings.Get("ctSSID2", "---");
  if (WiFi.status() != WL_CONNECTED && ctSSID2.length() > 0 && ctSSID2 != "---") {
    delay(1000);
    TryConnectWIFI(ctSSID2, settings.Get("ctPASS2", "---"), 2, settings.GetInt("Timeout2", 15));
  }

  if (display.IsConnected()) {
    display.Print("LGW V" + stateManager.GetVersion(), DisplayArea_Line1, OLED::Alignments::Center);
    display.Print(WiFi.localIP().toString(), DisplayArea_Line2, OLED::Alignments::Center);
  }
  if (nextion.IsConnected()) {
    nextion.Print("LGW V" + stateManager.GetVersion() + "\r\n" + WiFi.localIP().toString());
  }

  if (WiFi.status() == WL_CONNECTED) {
    result = true;
    logger.println();
    logger.println("connected :-)");
    logger.print("SSID: ");
    logger.println(WiFi.SSID());
    logger.print("IP: ");
    logger.println(WiFi.localIP().toString());
    if (display.IsConnected()) {
      display.SetWifiFlag(true);
    }

    if (settings.GetBool("UseMDNS")) {
      logger.println("Starting MDNS");
      MDNS.begin("esp8266-ota", WiFi.localIP());
      MDNS.addService("arduino", "tcp", OTA_PORT);
      MDNS.addService("http", "tcp", FRONTEND_PORT);
    }

  }
  else {
    logger.println();
    logger.println("We got no connection :-(");
    // Open access point for 15 minutes
    accessPoint.Begin(900);
    esp.SwitchLed(true, true);
  }

  logger.println("Starting frontend");
  frontend.SetPassword(settings.Get("frontPass", ""));
  frontend.SetCommandCallback([] (String command) {
    CommandHandler(command);
  });

  frontend.SetHardwareCallback([]() {
    HardwarePageBuilder builder;
    return builder.Build(
      &rfm1, &rfm2, &rfm3, &rfm4, &rfm5,
      &ownSensors, &sc16is750, &sc16is750_2, &digitalPorts,
      &display, &dataPort1, &dataPort2, &dataPort3,
      &serialBridge, &serialBridge2, &softSerialBridge, &analogPort, &nextion
    );
  });

  logger.println("Starting OTA");
  ota.Begin(frontend.WebServer());
  
  uint p1 = settings.GetInt("DataPort1", 81);
  uint p2 = settings.GetInt("DataPort2", 0);
  uint p3 = settings.GetInt("DataPort3", 0);
  if (p1 > 0) {
    logger.print("Starting data port 1 on ");
    logger.println(p1);
    dataPort1.Begin(p1);
    dataPort1.SetLogItemCallback([](String logItem){
      logger.println(logItem);
    });
  }
  if (p2 > 0) {
    logger.print("Starting data port 2 on ");
    logger.println(p2);
    dataPort2.Begin(p2);
    dataPort2.SetLogItemCallback([](String logItem){
      logger.println(logItem);
    });
  }
  if (p3 > 0) {
    logger.print("Starting data port 3 on ");
    logger.println(p3);
    dataPort3.Begin(p3);
    dataPort3.SetLogItemCallback([](String logItem){
      logger.println(logItem);
    });
  }

  if (useSerialBridge && sc16is750.IsConnected()) {
    int serialBridgePort = settings.GetInt("SerialBridgePort", 0);
    unsigned long serialBridgeBaud = settings.GetUnsignedLong("SerialBridgeBaud", 57600ul);
    if (serialBridgePort > 0 && serialBridgeBaud > 0) {
      serialBridge.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
        HandleProgressRequest(action, currentValue, maxValue, message);
      });

      logger.println("Starting serial bridge on port " + String(serialBridgePort) + " with " + serialBridgeBaud + " baud");
      serialBridge.Begin(serialBridgePort, frontend.WebServer());
      serialBridge.SetBaudrate(serialBridgeBaud);
    }
  }

  if (useSerialBridge2 && sc16is750_2.IsConnected()) {
    int serialBridgePort = settings.GetInt("SerialBridge2Port", 0);
    unsigned long serialBridgeBaud = settings.GetUnsignedLong("SerialBridge2Baud", 57600ul);
    if (serialBridgePort > 0 && serialBridgeBaud > 0) {
      serialBridge2.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
        HandleProgressRequest(action, currentValue, maxValue, message);
      });

      logger.println("Starting serial bridge 2 on port " + String(serialBridgePort) + " with " + serialBridgeBaud + " baud");
      serialBridge2.Begin(serialBridgePort, frontend.WebServer());
      serialBridge2.SetBaudrate(serialBridgeBaud);
    }
  }

  USE_WIFI = 1;
  
  return result;
}

static void StopWifi() {
  WiFi.mode(WiFiMode::WIFI_OFF);
  USE_WIFI = 0;
}

String CommandHandler(String command) {
  command = HTML::UTF8ToASCII(command);
  for (uint i = 0; i < command.length(); i++) {
    HandleSerialPort(command.charAt(i));
  }
  stateManager.ResetLastFullKVPUpdate();
  return "";
}


void setup(void) {
  Serial.begin(57600);
  delay(1000);
  Serial.println();
  
  SetDebugMode(DEBUG);
  
  Settings settings;
  settings.Read();
  
  pinMode(D7, INPUT);
  if (digitalRead(D7)) {
    USE_WIFI = false;
  }
  pinMode(D7, OUTPUT);
  
  if (!settings.GetBool("UseWiFi", true)) {
    USE_WIFI = false;
  }
  if (!USE_WIFI) {
    logger.Disable();
    esp.Blink(20);
  }
  
  logger.Clear();
  logger.println("***CLEARLOG***");
  logger.print(PROGNAME);
  logger.print(" V");
  logger.println(PROGVERS);
  logger.print("Free heap: ");
  logger.print(ESP.getFreeHeap());
  logger.print(" Flash size: ");
  logger.print(ESP.getFlashChipSizeByChipId());
  logger.print(" Core: ");
  logger.print(ESP.getCoreVersion());
  logger.print(" SDK: ");
  logger.println(ESP.getSdkVersion());
  logger.print("Reset: ");
  logger.println(ESP.getResetReason());
  logger.println(ESP.getResetInfo());
 
  ownSensors.SetID(settings.GetByte("ISID", 0));

  useSerialBridge = settings.GetInt("SerialBridgePort", 0) > 0;
  useSerialBridge2 = settings.GetInt("SerialBridge2Port", 0) > 0;

  bool isSC16IS750Clone = settings.GetBool("Is750Clone");
  unsigned long i2cClock = isSC16IS750Clone ? 100000ul : 400000ul;
  
  logger.print("Starting I2C with ");
  logger.print(String(i2cClock/1000));
  logger.println(" kHz");
  Wire.begin();
  Wire.setClock(i2cClock);
  Wire.setClockStretchLimit(1000);
  
  stateManager.Begin(PROGVERS, settings.Get("KVIdentity", String(ESP.getChipId())));
  
  dataPort1.SetConnectCallback([](bool isConnected) {
    dataPort1Connected = isConnected;
    if (display.IsConnected()) {
      display.SetFhemFlag(isConnected);
    }
  });
  dataPort2.SetConnectCallback([](bool isConnected) {
    dataPort2Connected = isConnected;
    if (display.IsConnected()) {
      display.SetFhemFlag(isConnected);
    }
  });
  dataPort3.SetConnectCallback([](bool isConnected) {
    dataPort3Connected = isConnected;
    if (display.IsConnected()) {
      display.SetFhemFlag(isConnected);
    }
  });
  serialBridge.SetConnectCallback([](bool isConnected) {
    bridge1Connected = isConnected;
    if (display.IsConnected()) {
      display.SetAddonFlag(isConnected);
    }
  });
  serialBridge2.SetConnectCallback([](bool isConnected) {
    bridge3Connected = isConnected;
    if (display.IsConnected()) {
      display.SetAddonFlag(isConnected);
    }
  });

  int displayStartMode = -3;
  String dsmSetting = settings.Get("oledStart", "on");
  if (dsmSetting == "on") {
    displayStartMode = -1;
  }
  else if (dsmSetting == "off") {
    displayStartMode = 0;
  }
  else {
    displayStartMode = settings.GetInt("oledStart", 1);
  }

  if(display.Begin(&stateManager, displayStartMode, settings.GetBool("oled13", false) ? OLED::Controllers::SH1106 : OLED::Controllers::SSD1306)) {
    logger.println("OLED found");
    delay(500);
  }
  
  delay(250);
  sc16is750.Begin(57600, isSC16IS750Clone);
  if (sc16is750.IsConnected()) {
    logger.print("SC16IS750");
    if (sc16is750.IsClone()) {
      logger.print("-Clone");
    }
    logger.println(" (0x90) found");

    if (!useSerialBridge) {
      subProcessor.SetLogItemCallback([](String logItem, bool newLine) {
        if (newLine) {
          logger.println(logItem);
        }
        else {
          logger.print(logItem);
        }
      });

      subProcessor.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
        HandleProgressRequest(action, currentValue, maxValue, message);
      });

      subProcessor.Begin(frontend.WebServer());
      logger.println("SubProcessor Reset");
      subProcessor.Reset();
    }
    
    serialBridge.SetLogItemCallback([](String logItem, bool newLine) {
      if (newLine) {
        logger.println(logItem);
      }
      else {
        logger.print(logItem);
      }
    });
  }

  if (sc16is750.IsConnected()) {
    alarm.Begin();
  }

  sc16is750_2.Begin(57600, isSC16IS750Clone);
  if (sc16is750_2.IsConnected()) {
    logger.print("SC16IS750");
    if (sc16is750_2.IsClone()) {
      logger.print("-Clone");
    }
    logger.println(" (0x92) found");

    if (!useSerialBridge2) {
      subProcessor2.SetLogItemCallback([](String logItem, bool newLine) {
        if (newLine) {
          logger.println(logItem);
        }
        else {
          logger.print(logItem);
        }
      });

      subProcessor2.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
        HandleProgressRequest(action, currentValue, maxValue, message);
      });

      subProcessor2.Begin(frontend.WebServer());
      logger.println("SubProcessor2 Reset");
      subProcessor2.Reset();
    }
    
    serialBridge2.SetLogItemCallback([](String logItem, bool newLine) {
      if (newLine) {
        logger.println(logItem);
      }
      else {
        logger.print(logItem);
      }
    });
  }

  rfms[0] = &rfm1;
  rfms[1] = &rfm2;
  rfms[2] = &rfm3;
  rfms[3] = &rfm4;
  rfms[4] = &rfm5;

  rfm1.Begin();  
  rfm2.Begin();
  rfm3.Begin();
  if (sc16is750.IsConnected()) {
    rfm4.Begin();
    rfm5.Begin();
  }

  rfm1.ToggleMode = TOGGLE_MODE_R1; rfm1.ToggleInterval = TOGGLE_INTERVAL_R1;
  rfm2.ToggleMode = TOGGLE_MODE_R2; rfm2.ToggleInterval = TOGGLE_INTERVAL_R2;
  rfm3.ToggleMode = TOGGLE_MODE_R3; rfm3.ToggleInterval = TOGGLE_INTERVAL_R3;
  rfm4.ToggleMode = TOGGLE_MODE_R4; rfm4.ToggleInterval = TOGGLE_INTERVAL_R4;
  rfm5.ToggleMode = TOGGLE_MODE_R5; rfm5.ToggleInterval = TOGGLE_INTERVAL_R5;

  ownSensors.TryInitialize(!rfm3.IsConnected(), !rfm2.IsConnected() && !rfm3.IsConnected());

  int altitude = settings.GetInt("Altitude", 0);
  logger.println("Configured altitude: " + String(altitude));
  ownSensors.SetAltitudeAboveSeaLevel(altitude);

  ownSensors.SetCorrections(settings.Get("CorrT", "0"), settings.Get("CorrH", "0"));

  if (ownSensors.HasBMP180()) {
    logger.println("BMP180 found");
  }
  if (ownSensors.HasBME280()) {
    logger.println("BME280 found");
  }
  if (ownSensors.HasDHT22()) {
    logger.println("DHT22 found");
  }
  if (ownSensors.HasLM75()) {
    logger.println("LM75 found");
  }
  if (ownSensors.HasSHT75()) {
    logger.println("SHT75 found");
  }

  KVINTERVAL = settings.GetInt("KVInterval", KVINTERVAL);

  if (settings.Get("Flags", "").indexOf("NO_FRONTEND_LOG") != -1) {
    logger.Disable();
  }

  esp.EnableLED(ENABLE_ACTIVITY_LED);
  lastToggleR1 = millis();
  lastToggleR2 = millis();

  esp.Blink(5, true);
  
  accessPoint.SetLogItemCallback([](String logItem){
    logger.println("AccessPoint: " + logItem);
  });

  if (settings.Get("Flags", "").indexOf("LOG.PCA301") != -1) {
    pca301.EnableLogging(true);
  }
  pca301.SetLogItemCallback([](String logItem){
    logger.println("PCA301: "+ logItem);
  });

  if (digitalPorts.Begin(
    [](String data) {
    Dispatch(data);
  },
    [](String command) {
    CommandHandler(command);
  },
    settings)) {
    logger.println("MCP23008 found");
  }
 
  int softSerialBridgePort = settings.GetInt("SSBridgePort", 0);
  if (USE_WIFI && !rfm2.IsConnected() && !rfm3.IsConnected() && !ownSensors.HasSHT75() && softSerialBridgePort > 0) {
    unsigned long softSerialBridgeBaud = settings.GetUnsignedLong("SSBridgeBaud", 57600);
    softSerialBridge.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
      HandleProgressRequest(action, currentValue, maxValue, message);
    });

    softSerialBridge.SetConnectCallback([](bool isConnected) {
      bridge2Connected = isConnected;
      if (display.IsConnected()) {
        display.SetAddonFlag(isConnected);
      }

      logger.println("SoftSerialBridge Connect: " + String(isConnected));
    });

    logger.println("Soft serial bridge port:" + String(softSerialBridgePort) + " baud:" + softSerialBridgeBaud);
    softSerialBridge.Begin(softSerialBridgePort, softSerialBridgeBaud, frontend.WebServer());

    if (settings.GetBool("IsNextion", false)) {
      nextion.SetProgressCallback([](byte action, unsigned long currentValue, unsigned long maxValue, String message) {
        HandleProgressRequest(action, currentValue, maxValue, message);
      });
      if (nextion.Begin(frontend.WebServer(), softSerialBridge.GetSoftSerial(), softSerialBridgeBaud, settings.GetBool("AddUnits", false))) {
        logger.println("Nextion initialized");
      }
    }

  }

  // Start wifi
  if (USE_WIFI) {
    int startupDelay = settings.GetInt("StartupDelay", 0);
    if (startupDelay > 0) {
      logger.println("Startup delay: " + String(startupDelay) + " seconds");
      delay(startupDelay * 1000);
    }
    logger.println("Starting wifi");
    StartWifi(settings);
  }
  else {
    WiFi.mode(WiFiMode::WIFI_OFF);
  }

  logger.println("Searching RFMs and Sensors");

  if (rfm1.IsConnected()) {
    rfm1.InitializeLaCrosse();
    rfm1.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm1, DATA_RATE_R1);
    rfm1.EnableReceiver(true);
    logger.print("Radio #1 found: ");
    logger.println(rfm1.GetRadioName());
  }

  if (rfm2.IsConnected()) {
    rfm2.InitializeLaCrosse();
    rfm2.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm2, DATA_RATE_R2);
    rfm2.EnableReceiver(true);
    logger.print("Radio #2 found: ");
    logger.println(rfm2.GetRadioName());
  }

  if (rfm3.IsConnected()) {
    rfm3.InitializeLaCrosse();
    rfm3.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm3, DATA_RATE_R3);
    rfm3.EnableReceiver(true);
    logger.print("Radio #3 found: ");
    logger.println(rfm3.GetRadioName());
  }

  if (rfm4.IsConnected()) {
    rfm4.InitializeLaCrosse();
    rfm4.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm4, DATA_RATE_R4);
    rfm4.EnableReceiver(true);
    logger.print("Radio #4 found: ");
    logger.println(rfm4.GetRadioName());
  }

  if (rfm5.IsConnected()) {
    rfm5.InitializeLaCrosse();
    rfm5.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm5, DATA_RATE_R5);
    rfm5.EnableReceiver(true);
    logger.print("Radio #5 found: ");
    logger.println(rfm5.GetRadioName());
  }
  
  if(settings.GetBool("SendAnalog")) {
    analogPort.TryInitialize();
  }

  frontend.Begin(&stateManager, &logger);

  // FHEM needs this information
  delay(1000);
  logger.println("Sending init String to FHEM");
  HandleCommandV();

  if (display.IsConnected()) {
    display.Command("mode=s");
    ownSensors.Measure();
    lastSensorMeasurement = millis();
    display.Handle(ownSensors.GetDataFrame(), stateManager.GetFramesPerMinute(), stateManager.GetVersion(), WiFi.RSSI());
  
    String oledMode = settings.Get("oledMode", "");
    if (oledMode.length() > 0) {
      display.Command("mode=" + oledMode);
    }

    if (nextion.IsConnected()) {
      delay(1000);
      nextion.SendCommand("page 0");
    }
  }
  
  if (!rfm2.IsConnected() && !ownSensors.HasSHT75() && !softSerialBridge.IsEnabled()) {
    pinMode(D4, OUTPUT);
    digitalWrite(D4, HIGH);
  }

  watchdog.Begin([](String message) {
    Dispatch(message);  
  });

  logger.println("Setup completely done");
}

byte HandleDataReception() {
  byte receivedPackets = 0;
  if (rfm1.IsConnected()) {
    rfm1.Receive(); 
    if (rfm1.PayloadIsReady()) {
      if (HandleReceivedData(&rfm1)) {
        receivedPackets++;
      }
    }
  }
  if (rfm2.IsConnected()) {
    rfm2.Receive();
    if (rfm2.PayloadIsReady()) {
      if(HandleReceivedData(&rfm2)) {
        receivedPackets++;
      }
    }
  }
  if (rfm3.IsConnected()) {
    rfm3.Receive();
    if (rfm3.PayloadIsReady()) {
      if (HandleReceivedData(&rfm3)) {
        receivedPackets++;
      }
    }
  }
  if (rfm4.IsConnected()) {
    rfm4.Receive();
    if (rfm4.PayloadIsReady()) {
      if (HandleReceivedData(&rfm4)) {
        receivedPackets++;
      }
    }
  }
  if (rfm5.IsConnected()) {
    rfm5.Receive();
    if (rfm5.PayloadIsReady()) {
      if (HandleReceivedData(&rfm5)) {
        receivedPackets++;
      }
    }
  }

  return receivedPackets;
}

void HandleDataRate() {
  if (rfm1.IsConnected()) {
    HandleDataRateToggle(&rfm1, &lastToggleR1, &DATA_RATE_R1);
  }
  if (rfm2.IsConnected()) {
    HandleDataRateToggle(&rfm2, &lastToggleR2, &DATA_RATE_R2);
  }
  if (rfm3.IsConnected()) {
    HandleDataRateToggle(&rfm3, &lastToggleR3, &DATA_RATE_R3);
  }
  if (rfm4.IsConnected()) {
    HandleDataRateToggle(&rfm4, &lastToggleR4, &DATA_RATE_R4);
  }
  if (rfm5.IsConnected()) {
    HandleDataRateToggle(&rfm5, &lastToggleR5, &DATA_RATE_R5);
  }
}

// **********************************************************************
void loop(void) {
  if (serialPortFlasher.IsUploading()) {
    if (Serial.available()) {
      serialPortFlasher.Add(Serial.read());
    }
    serialPortFlasher.Handle();
  }
  else {
    stateManager.SetLoopStart();

    // Handle the commands from the serial port
    // ----------------------------------------
    if (Serial.available()) {
      HandleSerialPort(Serial.read());
    }

    // Periodically send own sensor data
    // ---------------------------------
    if (millis() < lastSensorMeasurement) {
      lastSensorMeasurement = 0;
    }
    if (millis() > lastSensorMeasurement + 10000) {
      ownSensors.Measure();
      String data = ownSensors.GetFhemDataString();
      if (data.length() > 0) {
        Dispatch(data);
      }
      lastSensorMeasurement = millis();
    }

    // Handle the data reception
    // -------------------------
    byte receivedPackets = HandleDataReception();

    // Handle PCA301
    if (pca301.IsInitialized()) {
      pca301.Handle();
    }

    // Periodically send some info about us
    // ------------------------------------
    stateManager.Handle(receivedPackets);
    if (KVINTERVAL > 1) {
      String kvData = stateManager.GetKVP(KVINTERVAL);
      if (kvData.length() > 0) {
        Dispatch(kvData);
      }
    }

    // Handle the data rate
    // --------------------
    HandleDataRate();

    if (USE_WIFI) {
      // Handle the web fronted and the access point
      // -------------------------------------------
      frontend.Handle();
      accessPoint.Handle();

      // Handle the ports for FHEM
      // -------------------------
      dataPort1.Handle(CommandHandler);
      dataPort2.Handle(CommandHandler);
      dataPort3.Handle(CommandHandler);
    }

    // Handle OTA
    // ----------
    if (WiFi.status() == WL_CONNECTED) {
      ota.Handle();
    }

    // Handle Alarm
    if (sc16is750.IsConnected()) {
      alarm.Handle();
    }

    // Handle DigitalPorts
    if (digitalPorts.IsConnected()) {
      digitalPorts.Handle();
    }

    if (sc16is750.IsConnected()) {
      // Handle SerialBridge
      // -------------------
      if (useSerialBridge) {
        serialBridge.Handle();
      }

      // Handle SubProcessor
      // -------------------
      if (!useSerialBridge) {
        String subData = subProcessor.Handle();
        if (subData.length() > 0) {
          Dispatch("OK VALUES " + subData.substring(3));
        }
      }
    }

    if (sc16is750_2.IsConnected()) {
      // Handle SerialBridge
      // -------------------
      if (useSerialBridge2) {
        serialBridge2.Handle();
      }

      // Handle SubProcessor
      // -------------------
      if (!useSerialBridge2) {
        String subData = subProcessor2.Handle();
        if (subData.length() > 0) {
          Dispatch("OK VALUES " + subData.substring(3));
        }
      }
    }

    // Handle soft serial bridge
    // -------------------------
    if (softSerialBridge.IsEnabled()) {
      softSerialBridge.Handle();
    }
    
    WSBase::Frame frame = ownSensors.GetDataFrame();
    
    // Handle Nextion
    // --------------
    if (nextion.IsConnected()) {
      nextion.Handle(frame, stateManager.GetFramesPerMinute(), stateManager.GetVersion(), WiFi.RSSI(), dataPort1Connected || dataPort2Connected || dataPort3Connected, bridge1Connected, bridge2Connected);
    }

    // Handle OLED
    // -----------
    if (display.IsConnected()) {
      display.Handle(frame, stateManager.GetFramesPerMinute(), stateManager.GetVersion(), WiFi.RSSI());
      stateManager.SetDisplayStatus(display.IsOn() ? "on" : "off");
    }

    // Watchdog
    // --------
    watchdog.Handle();

    // Analogport
    // ----------
    String analogData = analogPort.GetFhemDataString();
    if (analogPort.IsEnabled() && analogData.length() > 0) {
      Dispatch(analogData);
    }

    stateManager.SetLoopEnd();
  }
}
