

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_mac.h"
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include "WiFi.h"

#include <WebSocketsServer.h>


#include "AES.h"
#include "Fourier.h"
#include "VariableDefine.h"


//-------------------------------------------------------Variable


//-----------------------------packet
static volatile uint8_t rcv_idx = 0;
static volatile int PktState = 0;
static volatile int TransmitState = 0;
volatile bool rfMode = true;
uint8_t PktLen[KEY];            // Assuming uint8_t
uint8_t PayloadData[KEY][PKT];  // Assuming uint8_t
float PktRssi[KEY];

static volatile int WsFlag = 0;

//-----------------------------LORA


LLCC68 radio = new Module(NssPin, DIO1Pin, RstPin, BusyPin);


//-----------------------------websocket
//PktStruct *WsQueue;



WebSocketsServer webSocket = WebSocketsServer(81);

uint8_t PhoneType;
uint8_t PhoneCaller;
bool PhoneTalking = true;
uint8_t lt = 0;
uint8_t sessionId[OCT];
uint8_t deqUncipher[PKT];
uint8_t deqCiphered[PKT];
uint8_t uncipher_len = 0;
uint8_t cipher_len = 0;
uint8_t tmpIV[HKEY];
uint8_t sharedKey[KEY];
uint8_t myPublicKey[KEY];
uint8_t myPrivateKey[KEY];

uint8_t SndIndex = 0;
uint8_t enque_idx = 0;
//-----------------------------webserver

AsyncWebServer httpServer(80);
//-------------------------------------------------------ISR


//-----------------------------LORA



void IRAM_ATTR dio1Trigger() {

  // //Serial.printf("\n rcv irq \n");

  if (rfMode) {
    PktLen[rcv_idx] = radio.getPacketLength();
    PktState = radio.readData(&PayloadData[rcv_idx][0], PktLen[rcv_idx]);
    if (PktState == RADIOLIB_ERR_NONE) {
      rcv_idx = (rcv_idx + 1) & 31;
      xTaskNotifyGive(transformTaskHandle);
    }
  } else {
    ////Serial.printf("\n transFinish \n");
    xTaskNotifyGive(transformTaskHandle);
  }
}


void IRAM_ATTR clearLoraFlagBit() {
  rfMode = false;
}

void IRAM_ATTR dio1Rssi() {
  if (rfMode) {
    ////Serial.println("Bit is currently ON!");
    PktLen[rcv_idx] = radio.getPacketLength();
    PktState = radio.readData(&PayloadData[rcv_idx][0], PktLen[rcv_idx]);

    if (PktState == RADIOLIB_ERR_NONE) {
      PktRssi[rcv_idx] = radio.getRSSI();
      rcv_idx = (rcv_idx + 1) & 31;
      xTaskNotifyGive(transformTaskHandle);
      ////Serial.printf("\n rcvReadData Notify  \n");
    }

  } else {
    xTaskNotifyGive(transformTaskHandle);
  }
}


//-----------------------------LORA

void radioConfigUpdate() {
  if (abs(FTconfig.Bandwidth - HDRB) < 100) {
    if (abs(FTconfig.Frequency - ASISM) > 5 && abs(FTconfig.Frequency - EUISM) > 5 && abs(FTconfig.Frequency - 915) > 5) {
      radio.setFrequency(FTconfig.Frequency);
      radio.setBandwidth(FTconfig.Bandwidth);
      PhonePermit = true;
    } else {
      //Serial.printf("     center channel is not allow 500mhz except SOS \n");
      PhonePermit = false;
    }
  } else {
    radio.setFrequency(FTconfig.Frequency);
    radio.setBandwidth(FTconfig.Bandwidth);
  }
  radio.setSpreadingFactor(FTconfig.SpreadFactor);
  radio.setCodingRate(FTconfig.CodeRate);
  radio.setSyncWord(FTconfig.SyncWord);
  radio.setOutputPower(FTconfig.RadioPower);
}

//-------------------------------------------------------local
//-----------------------------file

bool resetConfig(uint8_t option) {

  LittleFS.remove("/SystemConfig.bin");
  //Serial.println("removed SystemConfig.bin  ");

  if (option == 1) {
    LittleFS.remove("/Session.bin");
    //Serial.println("removed Session.bin   ");
  }
}

bool saveConfig() {
  File file = LittleFS.open("/SystemConfig.bin", "w");
  if (!file) {
    //Serial.println("Failed to save SystemConfig file    ");
    return false;
  }

  file.write((byte *)&FTconfig, sizeof(SystemConfig));
  file.close();
  vTaskDelay(200 / portTICK_PERIOD_MS);
  //Serial.println("  SystemConfig file  saved successful  ");
  return true;
}

bool loadConfig() {
  File file = LittleFS.open("/SystemConfig.bin", "r");
  if (!file) {
    //Serial.println("Failed to load SystemConfig file    ");
    return false;
  }

  file.read((byte *)&FTconfig, sizeof(SystemConfig));

  file.close();
  //Serial.println("  SystemConfig file  loaded successful  ");
  return true;
}


// Function to load session data from LittleFS
bool loadSessionList(ContactStruct *whisperList, uint16_t &whisperSize, ContactStruct *meetingList, uint16_t &meetingSize) {
  File file = LittleFS.open("/Session.bin", "r");
  if (!file) {
    //Serial.println("Failed to open file for reading");
    return false;
  }

  // Read sizes as uint16_t
  file.read((uint8_t *)&whisperSize, sizeof(uint16_t));
  file.read((uint8_t *)&meetingSize, sizeof(uint16_t));

  WhisperNum = whisperSize;
  MeetingNum = meetingSize;
  WhisperTopIndex = whisperSize;
  MeetingTopIndex = meetingSize;

  // Read whisperList data
  if (whisperSize > 0) {
    file.read((uint8_t *)whisperList, sizeof(ContactStruct) * whisperSize);
  }

  // Read meetingList data
  if (meetingSize > 0) {
    file.read((uint8_t *)meetingList, sizeof(ContactStruct) * meetingSize);
  }

  file.close();
  //Serial.println("Session loaded successfully");
  return true;
}

void sessionToFront() {
  JsonDocument sessionArrayJson;

  // JsonArray speechArray = sessionArrayJson["speechUserList"].to<JsonArray>();
  sessionArrayJson["dataType"] = 2;
  //Serial.printf("\n WhisperNum %d , WhisperNew %d , WhisperTopIndex %d   \n", WhisperNum, WhisperNew, WhisperTopIndex);
  if (WhisperNum > 0) {
    JsonArray whisperArray = sessionArrayJson["WhisperList"].to<JsonArray>();
    for (int ArrayStart = 0; ArrayStart <= WhisperTopIndex; ArrayStart++) {

      if (WhisperList[ArrayStart].status == 3) {
        JsonDocument doc;
        String front_Name1 = String((char *)WhisperList[ArrayStart].name);
        //Serial.printf("front_Name1    %d \n.", ArrayStart);
        //Serial.println(front_Name1);
        doc["name"] = front_Name1;
        doc["msgId"] = WhisperList[ArrayStart].msgId;
        doc["sessionId"] = ArrayStart;
        whisperArray.add(doc);
        WhisperMsgId = WhisperList[ArrayStart].msgId;
      }
    }
  }

  //Serial.printf("\n MeetingNum %d , MeetingNew %d , MeetingTopIndex %d   \n", MeetingNum, MeetingNew, MeetingTopIndex);
  if (MeetingNum > 0) {
    JsonArray meetingArray = sessionArrayJson["MeetingList"].to<JsonArray>();
    for (int ArrayStart = 0; ArrayStart <= MeetingTopIndex; ArrayStart++) {


      if (MeetingList[ArrayStart].status == 3) {
        JsonDocument doc;
        String front_Name2 = String((char *)MeetingList[ArrayStart].name);
        //Serial.printf("front_Name2    %d \n.", ArrayStart);
        //Serial.println(front_Name2);
        doc["name"] = front_Name2;
        doc["msgId"] = MeetingList[ArrayStart].msgId;
        doc["sessionId"] = ArrayStart;
        meetingArray.add(doc);
        MeetingMsgId = MeetingList[ArrayStart].msgId;
      }
    }
  }
  String SessionListStr = "";
  serializeJson(sessionArrayJson, SessionListStr);
  webSocket.broadcastTXT(SessionListStr);
}

void settingToFront() {
  JsonDocument writeJson;
  writeJson["dataType"] = 1;
  writeJson["WhisperFlag"] = (FTconfig.PktBits & BIT_WSP) ? true : false;
  writeJson["MeetingFlag"] = (FTconfig.PktBits & BIT_MET) ? true : false;
  writeJson["SpeechFlag"] = (FTconfig.PktBits & BIT_SPH) ? true : false;
  writeJson["GreetingFlag"] = (FTconfig.PktBits & BIT_SSN) ? true : false;
  writeJson["AlertFlag"] = (FTconfig.PktBits & BIT_ALT) ? true : false;
  writeJson["MyName"] = String(reinterpret_cast<char *>(MyName));
  writeJson["SandboxFlag"] = SandboxFlag;
  writeJson["SSID"] = SSID;
  writeJson["Password"] = Password;
  writeJson["WifiMode"] = WifiMode;
  writeJson["Frequency"] = FTconfig.Frequency;
  writeJson["Bandwidth"] = FTconfig.Bandwidth;
  writeJson["SpreadFactor"] = FTconfig.SpreadFactor;
  writeJson["CodeRate"] = FTconfig.CodeRate;
  writeJson["RadioPower"] = FTconfig.RadioPower;
  writeJson["SyncWord"] = FTconfig.SyncWord;
  writeJson["PreambleLength"] = FTconfig.PreambleLength;
  writeJson["ForwardGroup"] = FTconfig.ForwardGroup;
  writeJson["ForwardRSSI"] = FTconfig.ForwardRSSI;
  String configJson;
  serializeJson(writeJson, configJson);
  webSocket.broadcastTXT(configJson);
}

// Function to write session data to LittleFS
bool writeSessionList(ContactStruct *whisperList, uint16_t whisperSize, ContactStruct *meetingList, uint16_t meetingSize) {
  File file = LittleFS.open("/Session.bin", "w");
  if (!file) {
    //Serial.println("Failed to open file for writing");
    return false;
  }

  // Properly write sizes as uint16_t
  //file.write((uint8_t *)&whisperSize, sizeof(uint16_t));
  //file.write((uint8_t *)&meetingSize, sizeof(uint16_t));
  uint16_t tempWhisperSize = whisperSize;
  uint16_t tempMeetingSize = meetingSize;
  file.write((uint8_t *)&tempWhisperSize, sizeof(uint16_t));
  file.write((uint8_t *)&tempMeetingSize, sizeof(uint16_t));


  // Write whisperList data
  int count = 0;
  for (int i = 0; i < whisperSize; i++) {
    if (whisperList[i].status == 3) {
      file.write((uint8_t *)&whisperList[i], sizeof(ContactStruct));
      count++;
    }
  }

  // Write meetingList data
  count = 0;
  for (int i = 0; i < meetingSize; i++) {
    if (meetingList[i].status == 3) {
      file.write((uint8_t *)&meetingList[i], sizeof(ContactStruct));
      count++;
    }
  }

  file.close();
  //Serial.println("Session saved successfully");
  return true;
}

void jsonReceived(String jsonStr)  //===================================
{
  JsonDocument jsonObj;
  deserializeJson(jsonObj, jsonStr);
  uint8_t dataType = jsonObj["dataType"];
  switch (dataType) {
    case 0:  // system command
      {
        uint8_t cmdType = jsonObj["cmdType"];
        if (cmdType == 1)
          ESP.restart();
        if (cmdType == 2)
          resetConfig(1);
        // if (cmdType == 3) export configuration and contact;
      }
      break;
    case 1:  // ---------------------------------------update config
      {
        bool WhisperFlag = jsonObj["WhisperFlag"];
        bool MeetingFlag = jsonObj["MeetingFlag"];
        bool SpeechFlag = jsonObj["SpeechFlag"];
        bool GreetingFlag = jsonObj["GreetingFlag"];
        bool AlertFlag = jsonObj["AlertFlag"];

        if (WhisperFlag) {
          FTconfig.PktBits |= BIT_WSP;
        } else {
          FTconfig.PktBits &= ~BIT_WSP;  // Clear Bit 3
        }
        if (MeetingFlag) {
          FTconfig.PktBits |= BIT_MET;
        } else {
          FTconfig.PktBits &= ~BIT_MET;  // Clear Bit 3
        }
        if (SpeechFlag) {
          FTconfig.PktBits |= BIT_SPH;
        } else {
          FTconfig.PktBits &= ~BIT_SPH;  // Clear Bit 3
        }
        if (GreetingFlag) {
          FTconfig.PktBits |= BIT_SSN;
        } else {
          FTconfig.PktBits &= ~BIT_SSN;
        }
        if (AlertFlag) {
          FTconfig.PktBits |= BIT_ALT;
        } else {
          FTconfig.PktBits &= ~BIT_ALT;  // Clear Bit 3
        }

        SandboxFlag = jsonObj["SandboxFlag"];
        WifiMode = jsonObj["WifiMode"];
        String nameTmp = jsonObj["MyName"].as<String>();
        MyNameLen = nameTmp.length();
        memset(MyName, 0, KEY);
        nameTmp.getBytes(MyName, MyNameLen);
        SSID = jsonObj["SSID"].as<String>();
        Password = jsonObj["Password"].as<String>();
        FTconfig.Frequency = jsonObj["Frequency"];
        FTconfig.Bandwidth = jsonObj["Bandwidth"];
        FTconfig.SpreadFactor = jsonObj["SpreadFactor"];
        FTconfig.CodeRate = jsonObj["CodeRate"];
        FTconfig.RadioPower = jsonObj["RadioPower"];
        FTconfig.SyncWord = jsonObj["SyncWord"];
        FTconfig.PreambleLength = jsonObj["PreambleLength"];
        //FTconfig.SignalInfo = jsonObj["SignalInfo"];
        int ForwardGroupIdx = jsonObj["ForwardGroup"];
        FTconfig.ForwardRSSI = jsonObj["ForwardRSSI"];
        PKT_BITS = FTconfig.PktBits;
        ForwardRSSI == jsonObj["ForwardRSSI"];
        if (ForwardGroupIdx > 0) {
          ForwardGroup = MeetingList[ForwardGroupIdx].msgId;
          FTconfig.ForwardGroup = ForwardGroup;
        }

        bool SavingConfig = jsonObj["SavingConfig"];
        bool SavingNewContact = jsonObj["SavingNewContact"];
        bool needReboot = jsonObj["needReboot"];
        radioConfigUpdate();
        //Serial.printf("\n SandboxFlag %d \n", SandboxFlag);
        if (!SandboxFlag) {

          FTconfig.MyNameLen = MyNameLen;
          memset(FTconfig.MyName, 0, KEY);
          nameTmp.getBytes(FTconfig.MyName, MyNameLen);

          uint8_t PasswordLen = Password.length();
          Password.getBytes(FTconfig.Password, PasswordLen);

          uint8_t ssidLen = SSID.length();
          SSID.getBytes(FTconfig.SSID, ssidLen);

          saveConfig();
          writeSessionList(WhisperList, WhisperNum, MeetingList, MeetingNum);
        }
      }
      break;
    case 2:  // leave session
      {
        uint8_t sessionGroup = jsonObj["sessionGroup"];
        uint8_t sessionIndex = jsonObj["sessionIndex"];
        if (sessionGroup == 0) {  // leave whisper
          memset(&WhisperList[sessionIndex], 0, sizeof(WhisperList[sessionIndex]));
          WhisperNum--;
        }
        if (sessionGroup == 1) {  // leave meeting
          memset(&MeetingList[sessionIndex], 0, sizeof(MeetingList[sessionIndex]));
          MeetingNum--;
        }
        webSocket.broadcastTXT(ReloadStr);
      }
      break;
    case 3:  // nickName session
      {
        uint8_t sessionGroup = jsonObj["sessionGroup"];
        uint8_t sessionIndex = jsonObj["sessionIndex"];
        int nickLen;  //= nickTmp.length();
        String nickTmp = jsonObj["nickName"].as<String>();
        nickLen = nickTmp.length() + 1;
        if (nickLen > 30) {
          nickTmp = "Rename Pls";
          nickLen = 11;
        }

        if (sessionGroup == 0 && sessionIndex >= 0) {
          WhisperList[sessionIndex].nameLen = nickLen;
          memset(&WhisperList[sessionIndex].name, 0, KEY);
          nickTmp.getBytes(WhisperList[sessionIndex].name, nickLen);
          //Serial.printf("WhisperList namd %s     %d.\n", WhisperList[sessionIndex].name, WhisperList[sessionIndex].nameLen);
          //Serial.printf("\n WhisperNum %d , WhisperNew %d , WhisperTopIndex %d   \n", WhisperNum, WhisperNew, WhisperTopIndex);
        }
        if (sessionGroup == 1 && sessionIndex >= 0) {
          memset(&MeetingList[sessionIndex].name, 0, KEY);
          nickTmp.getBytes(MeetingList[sessionIndex].name, nickLen);
          MeetingList[sessionIndex].nameLen = nickLen;
          //Serial.printf("MeetingList namd %s     %d.    \n", MeetingList[sessionIndex].name, MeetingList[sessionIndex].nameLen);
          //Serial.printf("\n MeetingNum %d , MeetingNew %d , MeetingTopIndex %d   \n", MeetingNum, MeetingNew, MeetingTopIndex);
        }
        webSocket.broadcastTXT(ReloadStr);
      }
      break;
    case 4:  // keyUpdate
      {
        //Greeting = true;
        keyUpdate();
      }
      break;
    case 5:
      {
        PhoneCaller = jsonObj["PhoneCaller"];
        PhoneType = jsonObj["PhoneType"];
        //phoneInit();
        if (PhoneTalking == false) {

          //dateIvStr.getBytes(DateIV, OCT);
          PhoneTalking = true;
          //Serial.println("phone start  initialization  !");

          //Serial.println("phone finish initialization  !");
        } else {

          PhoneTalking = false;
        }
      }
      break;
    case 6:
      {
        uint16_t sensorGroupId = jsonObj["sensorGroupId"];
        uint16_t sensorMsgId = jsonObj["sensorMsgId"];
        if (MeetingList[sensorGroupId].msgId == sensorMsgId) {
          keyData = jsonObj["key"].as<String>();
          uint8_t beforeRsa[40];
          memcpy(beforeRsa, MeetingList[sensorGroupId].sessionId, 8);
          memcpy(&beforeRsa[8], MeetingList[sensorGroupId].sharedKey, 32);
          uint8_t afterRsa[256];
          //Serial.printf("\n beforeRsa  %d  sensorMsgId is %d\n", sensorGroupId, sensorMsgId);
          for (int i = 0; i < 40; i++)  //Serial.printf("%d:%02X ", i, beforeRsa[i]);
            //encryptRSA(beforeRsa, afterRsa, keyData);
            //Serial.printf("\n afterRsa \n");
            for (int i = 0; i < 256; i++)  //Serial.printf("%d:%02X ", i, afterRsa[i]);
              keyData = "";
          keyData = String((char *)afterRsa, 256);


          //httpServer.on("/download", HTTP_GET, handleKeyDownload);
        }
      }
      break;
  }
}

//-----------------------------AES
void keyUpdate() {
  //if (Greeting) {
  GenerateKeyPairs(myPrivate, myPublic);
  //}
}


void RandomGenerate(byte *array, uint8_t arrayLen) {
  for (uint8_t i = 0; i < arrayLen; i++) {
    array[i] = random(0, 255);  // Random value between 0 and 255
  }
}

//-----------------------------websocket

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {  // code,sessionIndex,msgType,payload


  uint8_t enque_idx = 0;
  if (payload != nullptr) {

    switch (type) {
      case WStype_DISCONNECTED:
        {
          Serial.printf("Client #%u disconnected\n", num);
          //WifiConnected = false;
        }

        break;
      case WStype_CONNECTED:
        {
          //WifiConnected = true;
          IPAddress ip = webSocket.remoteIP(num);
          Serial.printf("Client #%u connected from %s\n", num, ip.toString().c_str());
          vTaskDelay(pdMS_TO_TICKS(100));
          settingToFront();
          vTaskDelay(pdMS_TO_TICKS(500));
          if (MeetingNum + WhisperNum)
            sessionToFront();
          vTaskDelay(pdMS_TO_TICKS(100));
          String speechListJson;
          File file = LittleFS.open("/speechList.json", "r");
          if (!file) {
            Serial.println("Failed to open file");
            return;
          }
          while (file.available()) {
            speechListJson += (char)file.read();
          }
          file.close();
          webSocket.broadcastTXT(speechListJson);
        }
        break;
      case WStype_TEXT:
        {
          Serial.printf("WStype_TEXT from browser #%u: %s\n", length, payload);
          jsonReceived((char *)payload);
        }
        break;

      case WStype_BIN:
        {

          if (payload[0] == 1) {
            memcpy(SndPkt[lt].payloadData, payload, length);
            SndPkt[lt].pktLen = length;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
            Serial.printf("\n stream  talking   %d \n.", lt);
          } else {
            Serial.printf("\nWStype_BIN from browser #%u  \n", length);
            WsQueue[enque_idx].pktLen = length;
            memcpy(&WsQueue[enque_idx].payloadData, payload, length);
            xQueueSend(wsEventQueue, &enque_idx, portMAX_DELAY);
            enque_idx = (enque_idx + 1) & 31;
          }
        }

        break;
    }
  }
}

//-----------------------------http
void handleRoot(AsyncWebServerRequest *request) {
  request->send(LittleFS, "/index.html", "text/html");
  Serial.println("httpdTask handleRoot.");
}

void handleCSS(AsyncWebServerRequest *request) {
  request->send(LittleFS, "/styles.css", "text/css");
}



void handleChatJS(AsyncWebServerRequest *request) {
  request->send(LittleFS, "/chat.js", "application/javascript");
}


//-------------------------------------------------------api

//-----------------------------infrasture

bool initInfrast() {
  bool fsResult = false;
  fsResult = LittleFS.begin();
  if (!fsResult) {
    //Serial.println("Failed to mount LittleFS. Formatting...");

    fsResult = LittleFS.format();
    if (!fsResult) {
      fsResult = true;
      //Serial.println("LittleFS formatted successfully.");
    } else {
      //Serial.println("LittleFS formatting failed!");
      fsResult = false;
      ESP.restart();
    };
  }



  if (fsResult) {
    //Serial.println("LittleFS mounted successfully.");

    memset(&FTconfig, 0, sizeof(SystemConfig));
    fsResult = loadConfig();
    if (!fsResult) {

      FTconfig.Frequency = Frequency;
      FTconfig.Bandwidth = Bandwidth;
      FTconfig.SpreadFactor = SpreadFactor;
      FTconfig.CodeRate = CodeRate;
      FTconfig.RadioPower = RadioPower;
      FTconfig.SyncWord = SyncWord;
      FTconfig.PreambleLength = PreambleLength;
      FTconfig.PktBits |= BIT_PHO;
      FTconfig.PktBits |= BIT_ALT;
      FTconfig.PktBits |= BIT_SSN;
      FTconfig.PktBits |= BIT_SEN;
      FTconfig.PktBits |= BIT_WSP;
      FTconfig.PktBits |= BIT_MET;
      FTconfig.PktBits |= BIT_SPH;
      memset(FTconfig.MyName, 0, KEY);
      memcpy(FTconfig.MyName, "FT", 2);
      FTconfig.MyNameLen = 3;
      saveConfig();
      Serial.print("\n first save config \n");
      ESP.restart();

    } else {

      TurnOnWifi = FTconfig.Mode;
      keyUpdate();
      //Serial.println(" loading SessionList .");
      uint16_t tempWhisperNum = WhisperNum;
      uint16_t tempMeetingNum = MeetingNum;
      loadSessionList(WhisperList, tempWhisperNum, MeetingList, tempMeetingNum);


      esp_efuse_mac_get_default(ApMAC);

      esp_read_mac(ApMAC, ESP_MAC_WIFI_SOFTAP);


      return true;
    }
  }
}

void wifiMode() {

  if (WifiMode == 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, Password);
    //Serial.println(" wifi-STA started\n");
  }
  if (WifiMode == 1) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, Password);
    //Serial.println(" wifi-AP started\n");
  }
}


bool testWifi(void) {
  int connectAttmempt = 0;
  while (connectAttmempt < 60) {

    if (WifiMode == 0) {
      if (WiFi.status() == WL_CONNECTED) {
        //Serial.println("  Wifi  STA CONNECTED ");
        return true;
      }
    }
    if (WifiMode == 1) {
      if (WiFi.softAPgetStationNum() > 0) {
        //Serial.println("    softAP CONNECTED ");
        return true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    connectAttmempt++;
    // esp_task_wdt_reset();
  }
  return false;
}


bool compareID(byte *array1, byte *array2, uint8_t cmpLength) {
  for (uint8_t i = 0; i < cmpLength; i++) {
    if (array1[i] ^ array2[i]) {  // XOR checks if any bit is different
      return false;               // Return false immediately if there is a mismatch
    }
  }
  return true;  // Return true if all bytes are identical
}



void loraInit() {
  Serial.print(F("llcc68 Initializing ... "));
  int ResetInitState = 0;  // = radio.reset();
  vTaskDelay(pdMS_TO_TICKS(2000));
  if (ResetInitState == RADIOLIB_ERR_NONE) {
    //Serial.println(F("\n Reset lora success!\n "));
  } else {
    //Serial.printf("\n Reset lora failed !\t code: %d\t  ",ResetInitState);
  }
  radio.setTCXO(false);
  vTaskDelay(pdMS_TO_TICKS(2000));
  int loraInitState = 0;
  if (abs(FTconfig.Frequency - ASISM) < 5 && abs(FTconfig.Frequency - EUISM) < 5 && abs(FTconfig.Frequency - NAISM) < 5) {
    loraInitState = radio.begin(FTconfig.Frequency, MDRB, FTconfig.SpreadFactor, FTconfig.CodeRate, FTconfig.SyncWord, FTconfig.RadioPower, FTconfig.PreambleLength, 0, false);
    PhonePermit = false;
  } else {
    loraInitState = radio.begin(FTconfig.Frequency, FTconfig.Bandwidth, FTconfig.SpreadFactor, FTconfig.CodeRate, FTconfig.SyncWord, FTconfig.RadioPower, FTconfig.PreambleLength, 0, false);
    PhonePermit = true;
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  if (loraInitState == RADIOLIB_ERR_NONE) {
    //Serial.println(F("loraInit  success!"));
  } else {
    Serial.print(F("failed, code "));
    //Serial.println(loraInitState);
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  //radio.setIrqFlags(RADIOLIB_SX126X_IRQ_RX_DONE);
  if (ForwardGroup == 0)
    radio.setDio1Action(dio1Trigger);
  else {
    radio.setDio1Action(dio1Rssi);
    PhonePermit = false;
  }


  vTaskDelay(pdMS_TO_TICKS(2000));
  if (radio.setDio2AsRfSwitch() != RADIOLIB_ERR_NONE) {
    //Serial.println(F("Failed to set DIO2 as RF switch!"));
  }
  pinMode(DIO2Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIO2Pin), clearLoraFlagBit, RISING);  // Trigger on falling edge
  radio.startReceive();
  Serial.print(F("radio Task inited "));
}

bool idMatch(uint32_t highPart1, uint32_t lowPart1, uint32_t highPart2, uint32_t lowPart2) {
  return (highPart1 ^ highPart2) == 0 && (lowPart1 ^ lowPart2) == 0;  // Compare all bits at once
}



//-------------------------------------------------------task


void transformTask(void *pvParameters) {

  //Packet
  uint8_t tk_idx = 0; 
  uint8_t RcvBuddy[OCT];
  uint32_t RcvLow;
  uint32_t RcvHigh;
  uint16_t RcvIndex = 0;
  uint16_t RcvMeetingIndex = 0;
  bool volatile WhsCheck = false;
  bool volatile MetCheck = false;
  int i = 0;  //meeting buddy check
  //AES
  uint8_t myPrivateKey[KEY];
  uint8_t rcv_ciphered[PKT];
  uint8_t rcv_decipher[PKT];
  uint8_t rcv_IV[HKEY];
  uint8_t cipher_len = 0;
  uint8_t decipher_len = 0;
  //OPUS
  uint8_t frameIdx = 0;
  uint8_t firstOpus = 0;
  uint8_t phoneEnable;
  phoneEnable = EnableOLED;
  Serial.println("transformTask.");


  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (rfMode) {

      Serial.printf("  rcv code %d . PktLen    %d \n.", PayloadData[tk_idx][0], PktLen[tk_idx]);

      switch (PayloadData[tk_idx][0] & PKT_BITS) {  //       wsSendBin(to_web, to_web_len);

        case BIT_PHO:  // PayloadData:code,frameLen1,frameLen2,frame1_v,frame2_v
          {
            if (TurnOnWifi) {
              webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
            } else {
              Serial.printf(" listening    %d \n.", PktLen[tk_idx]);
              firstOpus = PayloadData[tk_idx][1];
              memcpy(frame_buffer[frameIdx], &PayloadData[tk_idx][3], firstOpus);
              frame_len[frameIdx] = firstOpus;
              xQueueSend(listenQueue, &frameIdx, portMAX_DELAY);
              frameIdx = (frameIdx + 1) & 31;
              frame_len[frameIdx] = PayloadData[tk_idx][2];
              memcpy(frame_buffer[frameIdx], &PayloadData[tk_idx][firstOpus + 3], PayloadData[tk_idx][2]);
              xQueueSend(listenQueue, &frameIdx, portMAX_DELAY);
              frameIdx = (frameIdx + 1) & 31;
            }
          }
          break;
        case BIT_ALT:  // PayloadData:code,direction{up/down},originMac_6,SrcMac_6,DstMac_6,type{SOS/alertBinary/text/webp/opus/sensorData/rescueCommand},metadata_v
          {
            if (TurnOnWifi) {
              webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
            } else {
              //show in oled
            }
          }
          break;
        case BIT_SSN:  // PayloadData:code,subType{GRT/CFM/TNL/IVM},metadata_v{cipheredKeyExchangeData/uncipherdPublicKey}
          {
            if (TurnOnWifi) {
              uint8_t gretType = PayloadData[tk_idx][1];
              switch (gretType) {
                case GRT:
                  {

                    uint8_t grtLen = PayloadData[tk_idx][42];
                    if (grtLen == PktLen[tk_idx] - 43) {
                      to_web[0] = SSN;
                      to_web[1] = GRT;  // older version front  is 41
                      to_web[2] = GreetingIndex;
                      to_web_len = grtLen + 3;
                      memcpy(&to_web[3], &PayloadData[tk_idx][43], grtLen);
                      memcpy(&GreetingList[GreetingIndex].invitation, &PayloadData[tk_idx][2], OCT);
                      memcpy(&GreetingList[GreetingIndex].publicKey, &PayloadData[tk_idx][10], KEY);
                      //Serial.println("rcvGreeting GreetingCode  FT.");
                      GreetingList[GreetingIndex].status = 1;
                      GreetingIndex = (GreetingIndex + 1) & 15;
                      webSocket.broadcastBIN(to_web, to_web_len);
                    }
                  }
                  break;

                case CFM:
                  {
                    //Serial.println("rcvConfirm  FT.");
                    uint8_t rcvCmf[OCT];
                    uint8_t cfmSharedKey[KEY];
                    uint8_t rcvKey[KEY];
                    cipher_len = PktLen[tk_idx] - 58;
                    memcpy(rcvCmf, &PayloadData[tk_idx][2], OCT);
                    memcpy(myPrivateKey, myPrivate, KEY);
                    bool invCheck = false;
                    invCheck = compareID(rcvCmf, GreetingCode, OCT);
                    if (invCheck) {
                      memcpy(rcvKey, &PayloadData[tk_idx][10], KEY);
                      memcpy(rcv_IV, &PayloadData[tk_idx][42], HKEY);
                      memcpy(rcv_ciphered, &PayloadData[tk_idx][58], cipher_len);
                      GreetingIndex = (GreetingIndex + 1) & 15;
                      ConfirmList[ConfirmIndex].status = 1;
                      GenerateSharedKey(myPrivateKey, rcvKey, cfmSharedKey);
                      memcpy(&ConfirmList[ConfirmIndex].sharedKey, cfmSharedKey, KEY);
                      aes_decipher(rcv_ciphered, cipher_len, rcv_decipher, cfmSharedKey, rcv_IV, &decipher_len);
                      memcpy(&ConfirmList[ConfirmIndex].idIndex, rcv_decipher, 2);
                      memcpy(&ConfirmList[ConfirmIndex].sessionId, &rcv_decipher[2], OCT);
                      memcpy(&to_web[3], &rcv_decipher[10], decipher_len - 10);
                      to_web[0] = SSN;
                      to_web[1] = CFM;  //older v is 2
                      to_web[2] = ConfirmIndex;
                      to_web_len = decipher_len - OCT;
                      webSocket.broadcastBIN(to_web, to_web_len);
                    }
                  }
                  break;
                case TNL:
                  {
                    //Serial.println("rcvTunnel  FT.");
                    uint8_t rcvTunIdx = PayloadData[tk_idx][3];
                    memcpy(RcvBuddy, &PayloadData[tk_idx][4], OCT);
                    bool invCheck = false;
                    invCheck = compareID(RcvBuddy, WhisperList[rcvTunIdx].sessionId, OCT);
                    if (invCheck & WhisperList[WhisperIndex].status == 1) {
                      uint8_t rcv_IV[HKEY];
                      uint8_t rcv_ciphered[KEY];
                      uint8_t rcv_decipher[KEY];
                      uint8_t decipher_len = 0;
                      uint8_t cipher_len = PktLen[tk_idx] - 28;
                      memcpy(rcv_IV, &PayloadData[tk_idx][12], HKEY);
                      memcpy(rcv_ciphered, &PayloadData[tk_idx][28], cipher_len);
                      aes_decipher(rcv_ciphered, cipher_len, rcv_decipher, WhisperList[WhisperIndex].sharedKey, rcv_IV, &decipher_len);
                      memcpy(&WhisperList[WhisperIndex].buddyIndex, rcv_decipher, 2);
                      memcpy(&WhisperList[WhisperIndex].buddy, &rcv_decipher[2], OCT);
                      //Serial.printf("rcvTunnel  prepare WhisperAdd %d.\n", WhisperAdd);
                      //Serial.printf("rcvTunnel namd%s     %d.\n", WhisperList[WhisperIndex].name, WhisperList[WhisperIndex].nameLen);
                      while (WhisperAdd)
                        vTaskDelay(pdMS_TO_TICKS(10));
                      WhisperAdd = true;
                      WhisperNum++;
                      WhisperMsgId++;
                      WhisperList[WhisperIndex].msgId = WhisperMsgId;
                      WhisperList[WhisperIndex].status = 3;
                      // send new whisper json
                      JsonDocument whisperNotify;
                      String whisperNotifyStr = "";
                      JsonDocument sessionArrayJson;
                      whisperNotify["dataType"] = 2;
                      JsonArray whisperArray = whisperNotify["WhisperList"].to<JsonArray>();
                      String created_Name = String((char *)WhisperList[WhisperNew].name);  // "new buddy";//
                      sessionArrayJson["name"] = created_Name;
                      sessionArrayJson["msgId"] = WhisperList[WhisperNew].msgId;
                      sessionArrayJson["sessionId"] = WhisperNew;
                      whisperArray.add(sessionArrayJson);
                      serializeJson(whisperNotify, whisperNotifyStr);
                      webSocket.broadcastTXT(whisperNotifyStr);
                      WhisperAdd = false;
                      WhisperTopIndex = (WhisperTopIndex + 1) & 255;
                      WhisperNew = 0;
                      memcpy(&WhisperList[RcvIndex].lowId, WhisperList[RcvIndex].sessionId, 4);
                      memcpy(&WhisperList[RcvIndex].highId, &WhisperList[RcvIndex].sessionId[4], 4);
                    }
                  }
                  break;

                case IVM:
                  {

                    //Serial.println("rcv IVM   .");
                    RcvIndex = PayloadData[tk_idx][3];
                    memcpy(&RcvLow, &PayloadData[tk_idx][4], 4);   // Copy first 4 bytes
                    memcpy(&RcvHigh, &PayloadData[tk_idx][8], 4);  // Copy last 4 bytes
                    WhsCheck = idMatch(RcvLow, RcvHigh, WhisperList[RcvIndex].lowId, WhisperList[RcvIndex].highId);
                    if (WhsCheck) {

                      memcpy(rcv_IV, &PayloadData[tk_idx][12], HKEY);
                      aes_decipher(&PayloadData[tk_idx][28], PktLen[tk_idx] - 28, to_web, WhisperList[RcvIndex].sharedKey, rcv_IV, &decipher_len);
                      //WhisperList[SndIndex].sessionId
                      uint8_t whs_Buddy[OCT];
                      memcpy(whs_Buddy, &to_web[40], OCT);
                      bool ivm_check = false;
                      ivm_check = compareID(whs_Buddy, WhisperList[RcvIndex].buddy, OCT);
                      if (ivm_check) {
                        //Serial.println("match   IVM.");
                        while (MeetingAdd)
                          vTaskDelay(pdMS_TO_TICKS(10));
                        MeetingAdd = true;
                        while (MeetingList[MeetingTopIndex].status == 3)
                          MeetingTopIndex = (MeetingTopIndex + 1) & 255;
                        MeetingMsgId++;
                        MeetingNew = MeetingTopIndex;
                        MeetingList[MeetingNew].msgId = MeetingMsgId;
                        MeetingList[MeetingNew].status = 3;
                        memcpy(&MeetingList[MeetingNew].sessionId, to_web, OCT);
                        memcpy(&MeetingList[MeetingNew].sharedKey, &to_web[8], KEY);
                        memcpy(&MeetingList[MeetingNew].name, WhisperList[RcvIndex].name, WhisperList[RcvIndex].nameLen);
                        MeetingNum++;
                        JsonDocument meetingNotify;
                        String meetingNotifyStr = "";
                        JsonDocument sessionArrayJson;
                        meetingNotify["dataType"] = 2;
                        JsonArray meetingArray = meetingNotify["MeetingList"].to<JsonArray>();
                        String created_Name = String((char *)MeetingList[MeetingNew].name);
                        sessionArrayJson["name"] = created_Name;
                        sessionArrayJson["msgId"] = MeetingList[MeetingNew].msgId;
                        sessionArrayJson["sessionId"] = MeetingNew;
                        meetingArray.add(sessionArrayJson);
                        serializeJson(meetingNotify, meetingNotifyStr);
                        webSocket.broadcastTXT(meetingNotifyStr);
                        MeetingAdd = false;
                        MeetingNew = 0;
                        memcpy(&MeetingList[RcvIndex].lowId, MeetingList[RcvIndex].sessionId, 4);
                        memcpy(&MeetingList[RcvIndex].highId, &MeetingList[RcvIndex].sessionId[4], 4);
                      }
                    }
                    break;
                  }
              }
              break;
            }
          }
        case BIT_WSP:  // PayloadData:code,buddyIdx_2,buddyId_8,IV_16,cipheredData{type,len,metadata_v}
          {
            //Serial.println("rcvWhisper  FT.");

            memcpy(&RcvLow, &PayloadData[tk_idx][3], 4);   // Copy first 4 bytes
            memcpy(&RcvHigh, &PayloadData[tk_idx][7], 4);  // Copy last 4 bytes
            WhsCheck = idMatch(RcvLow, RcvHigh, WhisperList[RcvIndex].lowId, WhisperList[RcvIndex].highId);

            if (WhsCheck) {
              to_web[0] = WSP;
              to_web[1] = RcvIndex;
              memcpy(rcv_IV, &PayloadData[tk_idx][11], HKEY);
              aes_decipher(&PayloadData[tk_idx][27], PktLen[tk_idx] - 27, &to_web[2], WhisperList[RcvIndex].sharedKey, rcv_IV, &decipher_len);
              to_web_len = decipher_len + 2;
              if (TurnOnWifi) {
                webSocket.broadcastBIN(to_web, to_web_len);
              } else {
                //show in oled
              }
              //Serial.println("rcvWhisper  match.");
            }
          }
          break;
        case BIT_MET:  // PayloadData:code,buddyId_8,IV_16,cipheredData{type,len,metadata_v}
          {
            //Serial.println("rcvMeeting  FT.");
            memcpy(&RcvLow, &PayloadData[tk_idx][1], 4);   // Copy first 4 bytes
            memcpy(&RcvHigh, &PayloadData[tk_idx][5], 4);  // Copy last 4 bytes
            MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[LastMeetingB].lowId, MeetingList[LastMeetingB].highId);
            RcvMeetingIndex = LastMeetingB;
            if (!MetCheck) {
              MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[LastMeetingA].lowId, MeetingList[LastMeetingA].highId);
              RcvMeetingIndex = LastMeetingA;

              if (!MetCheck) {
                RcvMeetingIndex = 0;
                while (RcvMeetingIndex < MeetingTopIndex) {
                  RcvMeetingIndex++;
                  //MetCheck = compareID(RcvBuddy, MeetingList[RcvMeetingIndex].sessionId, OCT);
                  MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[RcvMeetingIndex].lowId, MeetingList[RcvMeetingIndex].highId);

                  if (MetCheck)
                    break;
                }
              }
            }
            if (MetCheck) {

              to_web[0] = MET;
              to_web[1] = RcvMeetingIndex;
              memcpy(rcv_IV, &PayloadData[tk_idx][9], HKEY);
              aes_decipher(&PayloadData[tk_idx][25], PktLen[tk_idx] - 25, &to_web[2], MeetingList[RcvMeetingIndex].sharedKey, rcv_IV, &decipher_len);
              to_web_len = decipher_len + 2;
              LastMeetingB = RcvMeetingIndex;
              if (TurnOnWifi) {
                webSocket.broadcastBIN(to_web, to_web_len);
              } else {
                //show in oled
              }
              if(to_web[3]==GPS_MSG){
                
              }

              if (ForwardGroup > 0) {
                //Serial.printf("\n  rcvMet ForwardGroup: %d \n", ForwardGroup);
                if (ForwardRSSI > PktRssi[tk_idx] && MeetingList[RcvMeetingIndex].msgId == ForwardGroup) {
                  webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
                }
              }
            }
          }
          break;
        case BIT_SPH:  // PayloadData:code,metadata_v{name,content}
          {
            //Serial.printf("rcvSpeech  FT! tk_idx  %d length is  %d\t data is  %s\n", tk_idx, PktLen[tk_idx], &PayloadData[tk_idx]);  //ToWebPkt

            webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
          }
          break;

        case BIT_SEN:  // PayloadData:code,direction{up/down},SrcMac,DstMac,type{timeReq/timeAck/ciphered},metadata
          {
            //Serial.println("rcv sensor  FT.");
            //preserve for sensor / relay
            if (RelayNum < 254)
              RelayNum++;
          }
          break;
      }



      tk_idx = (tk_idx + 1) & 31;
    } else {
      radio.finishTransmit();
      radio.startReceive();
      rfMode = true;
    }
    if (TrafficDensity < 11)
      TrafficDensity = (TrafficDensity + 1) & 15;
  }
}


void electricTask(void *pvParameters) {
  uint8_t loraIdx;
  Serial.println("electricTask  successfully.");
  while (1) {
    if (xQueueReceive(loraQueue, &loraIdx, portMAX_DELAY)) {
      while (digitalRead(BusyPin) == HIGH || rfMode == false) {
        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      //for (int i = 0; i < SndPkt[loraIdx].pktLen; i++) //Serial.printf("%d:%02X ", i, SndPkt[loraIdx].payloadData[i]);
      TransmitState = radio.startTransmit(SndPkt[loraIdx].payloadData, SndPkt[loraIdx].pktLen);
      //Serial.printf("\n loraQueue idx : %d \t length:  %d \t TransmitState :%d\n", loraIdx, SndPkt[loraIdx].pktLen, TransmitState);
    }
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}



void httpdTask(void *pvParameters) {
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/styles.css", HTTP_GET, handleCSS);
  httpServer.on("/chat.js", HTTP_GET, handleChatJS);

  httpServer.begin();
  Serial.println("httpdTask  create successfully.");
  while (1) {


    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}


void magneticTask(void *pvParameters) {
  uint8_t deq_idx;
  uint8_t sessionId[OCT];
  uint8_t deqUncipher[PKT];
  uint8_t deqCiphered[PKT];
  uint8_t uncipher_len = 0;
  uint8_t cipher_len = 0;
  uint8_t tmpIV[HKEY];
  uint8_t sharedKey[KEY];
  uint8_t myPublicKey[KEY];
  uint8_t myPrivateKey[KEY];

  while (1) {
    // Wait for new data indefinitely (sleeps when queue is empty)
    if (xQueueReceive(wsEventQueue, &deq_idx, portMAX_DELAY)) {
      Serial.printf("\n wsEventQueue data: %d", deq_idx);
      uint8_t SndIndex = 0;
      switch (WsQueue[deq_idx].payloadData[0]) {
          //Serial.printf("WsFlag  in magneticTask   %d and flag %d \n.", WsFlag, WsQueue[deq_idx].payloadData[0]);

        case PHO:
          {
            memcpy(SndPkt[lt].payloadData, WsQueue[deq_idx].payloadData, SndPkt[lt].pktLen);
            SndPkt[lt].pktLen = WsQueue[deq_idx].pktLen;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
            //Serial.printf("\n stream  talking   %d \n.", lt);
            vTaskDelay(pdMS_TO_TICKS(50));
          }
          break;
        case ALT:
          {
            Serial.printf("snd ALT\n");
            SndPkt[lt].pktLen = WsQueue[deq_idx].pktLen;
            memcpy(&SndPkt[lt].payloadData, WsQueue[deq_idx].payloadData, WsQueue[deq_idx].pktLen);
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            Serial.printf("\n loraQueue enque #%u: %s\n", SndPkt[lt].pktLen, SndPkt[lt].payloadData);
            for (int i = 0; i < SndPkt[lt].pktLen; i++) Serial.printf("%d:%02X ", i, SndPkt[lt].payloadData[i]);
            lt = (lt + 1) & 31;
          }
          break;
        case SSN:
          {
            uint8_t gretType = WsQueue[deq_idx].payloadData[1];
            switch (gretType) {
              case GRT:  //4,1,GreetingCode_8,myPublic_32,grtLen,grtMsg_vl
                {
                  uint8_t grtLen = WsQueue[deq_idx].pktLen - 2;
                  SndPkt[lt].payloadData[0] = GRT;
                  SndPkt[lt].payloadData[1] = 1;
                  for (int i = 0; i < OCT; i++) {
                    GreetingCode[i] = random(0, 255);  // Replace with a cryptographically secure random function
                  }
                  memcpy(&SndPkt[lt].payloadData[2], GreetingCode, OCT);
                  memcpy(&SndPkt[lt].payloadData[10], myPublic, KEY);
                  SndPkt[lt].payloadData[42] = grtLen;
                  memcpy(&SndPkt[lt].payloadData[43], &WsQueue[deq_idx].payloadData[1], grtLen);
                  SndPkt[lt].pktLen = grtLen + 43;
                  Serial.println("sndGreeting  FT.");
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;

              case CFM:  //4,2,invitation_8,myPublic_32,MyIV_16,ciphered(sessionId_8,cfmMsg_vl)
                {
                  Serial.println("sndConfirm  FT.");
                  uint8_t gretIndex = WsQueue[deq_idx].payloadData[2];
                  uint8_t cfmLen = WsQueue[deq_idx].pktLen - 3;
                  while (WhisperList[WhisperIndex].status != 0)
                    WhisperIndex = (WhisperIndex + 1) & 255;
                  if (cfmLen < KEY)
                    WhisperList[WhisperIndex].nameLen = cfmLen;
                  else
                    WhisperList[WhisperIndex].nameLen = KEY;
                  // generate sharedKey and     sessionId
                  RandomGenerate(sessionId, OCT);
                  memcpy(myPublicKey, GreetingList[gretIndex].publicKey, KEY);
                  memcpy(myPrivateKey, myPrivate, KEY);
                  GenerateSharedKey(myPrivateKey, myPublicKey, sharedKey);
                  WhisperList[WhisperIndex].status = 1;  // confirm:1
                  memcpy(&WhisperList[WhisperIndex].name, &WsQueue[deq_idx].payloadData[3], WhisperList[WhisperIndex].nameLen);
                  memcpy(&WhisperList[WhisperIndex].sessionId, sessionId, OCT);
                  memcpy(&WhisperList[WhisperIndex].sharedKey, sharedKey, KEY);
                  deqUncipher[0] = WhisperIndex / ULIST;
                  deqUncipher[1] = WhisperIndex % ULIST;
                  memcpy(&deqUncipher[2], sessionId, OCT);
                  memcpy(&deqUncipher[10], &WsQueue[deq_idx].payloadData[3], cfmLen);
                  memcpy(tmpIV, MyIV, HKEY);
                  aes_cipher(deqUncipher, cfmLen + 10, deqCiphered, sharedKey, tmpIV, &cipher_len);
                  // send pkt
                  SndPkt[lt].payloadData[0] = 4;
                  SndPkt[lt].payloadData[0] = 2;
                  memcpy(&SndPkt[lt].payloadData[2], &GreetingList[gretIndex].invitation, OCT);
                  memcpy(&SndPkt[lt].payloadData[10], myPublic, KEY);
                  memcpy(&SndPkt[lt].payloadData[42], MyIV, HKEY);
                  memcpy(&SndPkt[lt].payloadData[58], deqCiphered, cipher_len);
                  SndPkt[lt].pktLen = cipher_len + 58;
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;

              case TNL:  //4,3,buddyIndex_2,buddy_8,MyIV_16,ciphered(sessionId_8)
                {
                  //Serial.println("sndTunnel  FT.");
                  uint8_t tunIndex = WsQueue[deq_idx].payloadData[2];
                  if (ConfirmList[tunIndex].status == 1) {
                    while (WhisperList[WhisperIndex].status != 0)  // infinite loop trigger reboot
                      WhisperIndex = (WhisperIndex + 1) & 255;
                    WhisperList[WhisperIndex].nameLen = WsQueue[deq_idx].pktLen - 3;
                    memcpy(&WhisperList[WhisperIndex].name, &WsQueue[deq_idx].payloadData[3], WhisperList[WhisperIndex].nameLen);
                    memcpy(&WhisperList[WhisperIndex].buddy, &ConfirmList[tunIndex].sessionId, OCT);
                    memcpy(&WhisperList[WhisperIndex].buddyIndex, &ConfirmList[tunIndex].idIndex, 2);
                    memcpy(&WhisperList[WhisperIndex].sharedKey, &ConfirmList[tunIndex].sharedKey, KEY);
                    RandomGenerate(WhisperList[WhisperIndex].sessionId, OCT);
                    memset(&ConfirmList[tunIndex], 0, sizeof(ConfirmList[tunIndex]));
                    memcpy(&WhisperList[WhisperIndex].lowId, WhisperList[WhisperIndex].sessionId, 4);
                    memcpy(&WhisperList[WhisperIndex].highId, &WhisperList[WhisperIndex].sessionId[4], 4);
                    // cipher
                    deqUncipher[0] = WhisperIndex / ULIST;
                    deqUncipher[1] = WhisperIndex % ULIST;
                    memcpy(&deqUncipher[2], WhisperList[WhisperIndex].sessionId, OCT);
                    memcpy(tmpIV, MyIV, HKEY);
                    Serial.printf("sndTunnel name%s   nameLen  %d\n.", WhisperList[WhisperIndex].name, WhisperList[WhisperIndex].nameLen);
                    aes_cipher(deqUncipher, 10, deqCiphered, WhisperList[WhisperIndex].sharedKey, tmpIV, &cipher_len);
                    SndPkt[lt].payloadData[0] = 4;
                    SndPkt[lt].payloadData[0] = 3;
                    memcpy(&SndPkt[lt].payloadData[2], &WhisperList[WhisperIndex].buddyIndex, 2);
                    memcpy(&SndPkt[lt].payloadData[4], &WhisperList[WhisperIndex].buddy, 8);
                    memcpy(&SndPkt[lt].payloadData[12], MyIV, HKEY);
                    memcpy(&SndPkt[lt].payloadData[28], deqCiphered, cipher_len);
                    SndPkt[lt].pktLen = cipher_len + 28;
                    xQueueSend(loraQueue, &lt, portMAX_DELAY);
                    lt = (lt + 1) & 31;
                    while (WhisperAdd)
                      vTaskDelay(pdMS_TO_TICKS(10));
                    WhisperAdd = true;
                    WhisperNum++;
                    //WhisperTopIndex++;
                    WhisperNew = WhisperIndex;
                    WhisperMsgId++;
                    WhisperList[WhisperIndex].msgId = WhisperMsgId;
                    WhisperList[WhisperIndex].status = 3;
                    // WsFlag = 2;  // send new whisper json
                  }
                }
                break;
              case IVM:  //4,4,buddyIndex_2,buddy_8,MyIV_16,ciphered(mtSessionId_8,mtSharedKey_32,wsSessionId_8)
                {
                  Serial.println("invite   meeting.");
                  uint8_t metIndex;
                  uint8_t tmp[TM];
                  SndIndex = WsQueue[deq_idx].payloadData[2];
                  metIndex = WsQueue[deq_idx].payloadData[3];
                  SndPkt[lt].payloadData[0] = SSN;
                  SndPkt[lt].payloadData[1] = IVM;
                  memcpy(&SndPkt[lt].payloadData[2], &WhisperList[SndIndex].buddyIndex, 2);
                  memcpy(&SndPkt[lt].payloadData[4], &WhisperList[SndIndex].buddy, OCT);
                  memcpy(&SndPkt[lt].payloadData[12], MyIV, HKEY);
                  memcpy(tmpIV, MyIV, HKEY);
                  memcpy(tmp, &MeetingList[metIndex].sessionId, OCT);
                  memcpy(&tmp[8], &MeetingList[metIndex].sharedKey, KEY);
                  memcpy(&tmp[40], &WhisperList[SndIndex].sessionId, OCT);
                  Serial.println("invite   data.");
                  for (int i = 0; i < TM; i++) Serial.printf("%d:%02X\t", i, tmp[i]);
                  aes_cipher(tmp, TM, &SndPkt[lt].payloadData[28], WhisperList[SndIndex].sharedKey, tmpIV, &uncipher_len);
                  SndPkt[lt].pktLen = uncipher_len + 28;
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;
            }
            break;
          }
          break;
        case WSP:
          {
            Serial.println("sndWhisper  FT.");
            SndIndex = WsQueue[deq_idx].payloadData[1];
            SndPkt[lt].payloadData[0] = WSP;
            memcpy(&SndPkt[lt].payloadData[1], &WhisperList[SndIndex].buddyIndex, 2);
            memcpy(&SndPkt[lt].payloadData[3], &WhisperList[SndIndex].buddy, 8);
            memcpy(&SndPkt[lt].payloadData[11], MyIV, HKEY);
            memcpy(tmpIV, MyIV, HKEY);
            aes_cipher(&WsQueue[deq_idx].payloadData[2], WsQueue[deq_idx].pktLen - 2, &SndPkt[lt].payloadData[27], WhisperList[SndIndex].sharedKey, tmpIV, &uncipher_len);
            SndPkt[lt].pktLen = uncipher_len + 27;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
          }
          break;
        case MET:
          {
            Serial.println("sndMeeting  FT.");
            SndIndex = WsQueue[deq_idx].payloadData[1];
            SndPkt[lt].payloadData[0] = MET;
            //memcpy(&SndPkt[lt].payloadData[1], &MeetingList[SndIndex].buddyIndex, 2);
            memcpy(&SndPkt[lt].payloadData[1], &MeetingList[SndIndex].sessionId, 8);
            memcpy(&SndPkt[lt].payloadData[9], MyIV, HKEY);
            memcpy(tmpIV, MyIV, HKEY);
            aes_cipher(&WsQueue[deq_idx].payloadData[2], WsQueue[deq_idx].pktLen - 2, &SndPkt[lt].payloadData[25], MeetingList[SndIndex].sharedKey, tmpIV, &uncipher_len);
            SndPkt[lt].pktLen = uncipher_len + 25;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
            if (LastMeetingB != SndIndex)
              LastMeetingA = SndIndex;
          }
          break;
        case SPH:
          {
            Serial.printf("sndSpeech\n");
            SndPkt[lt].pktLen = WsQueue[deq_idx].pktLen;
            memcpy(&SndPkt[lt].payloadData, WsQueue[deq_idx].payloadData, WsQueue[deq_idx].pktLen);
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
          }
          break;
        case MCT:
          {
            uint8_t meeting_room_len = WsQueue[deq_idx].payloadData[1];
            uint8_t meet_data_len = 0;
            uint8_t tmp_ciphered[TM];
            uint8_t tmp_cipherLen;
            uint8_t tmp_SessionID[OCT];

            while (MeetingAdd)
              vTaskDelay(pdMS_TO_TICKS(10));
            MeetingAdd = true;
            while (MeetingList[MeetingTopIndex].status == 3)
              MeetingTopIndex = (MeetingTopIndex + 1) & 255;  //              MeetingTopIndex++;
            MeetingList[MeetingTopIndex].status = 3;
            if (WsQueue[deq_idx].pktLen < 40) {
              Serial.printf(" safe create  %d \n.", WsQueue[deq_idx].pktLen);
              meeting_room_len = WsQueue[deq_idx].pktLen - 2;

              randomSeed(millis() + WsQueue[deq_idx].payloadData[meeting_room_len]);
              for (uint8_t i = 0; i < OCT; i++) {
                //randomSeed(analogRead(32));
                tmp_SessionID[i] = random(0, 255);  // Random value between 0 and 255
              }
              randomSeed(millis() + WsQueue[deq_idx].payloadData[meeting_room_len - 2]);
              for (uint8_t i = 0; i < 45; i++) {
                tmp_ciphered[i] = random(0, 255);  // Random value between 0 and 255
              }

            } else {
              Serial.printf(" temp create  %d \n.", WsQueue[deq_idx].pktLen);
              uint8_t tmp_room[KEY];
              uint8_t tmp_key[KEY];
              uint8_t tmp_iv[HKEY];
              uint8_t tmp_uncipher[TM];
              meet_data_len = WsQueue[deq_idx].pktLen - 2 - meeting_room_len;
              memcpy(tmp_room, &WsQueue[deq_idx].payloadData[2], meeting_room_len);
              memcpy(tmp_key, &WsQueue[deq_idx].payloadData[meeting_room_len], KEY);
              memcpy(tmp_iv, &WsQueue[deq_idx].payloadData[meeting_room_len + KEY], 16);
              memcpy(tmp_uncipher, &WsQueue[deq_idx].payloadData[1], TM);
              aes_cipher(tmp_uncipher, TM, tmp_ciphered, tmp_key, tmp_iv, &tmp_cipherLen);
              memcpy(tmp_SessionID, tmp_ciphered, OCT);
            }

            memcpy(&MeetingList[MeetingTopIndex].sessionId, tmp_SessionID, OCT);
            memcpy(&MeetingList[MeetingTopIndex].sharedKey, &tmp_ciphered[8], KEY);
            memcpy(&MeetingList[MeetingTopIndex].name, &WsQueue[deq_idx].payloadData[2], meeting_room_len);
            MeetingList[MeetingTopIndex].nameLen = meeting_room_len;
            MeetingMsgId++;
            MeetingList[MeetingTopIndex].msgId = MeetingMsgId;
            MeetingNum++;
            MeetingNew = MeetingTopIndex;
            // WsFlag = 3;

            memcpy(&MeetingList[MeetingTopIndex].lowId, MeetingList[MeetingTopIndex].sessionId, 4);
            memcpy(&MeetingList[MeetingTopIndex].highId, &MeetingList[MeetingTopIndex].sessionId[4], 4);
          }
          break;


          memset(&WsQueue[deq_idx], 0, 257);
      }
    }
  }
}

void websocketTask(void *pvParameters) {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("websocketTask  successfully.");
  while (1) {
    webSocket.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
