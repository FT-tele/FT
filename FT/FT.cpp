
#include <esp_heap_caps.h>
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
uint8_t myPrivateKey[KEY];

uint8_t buddyPublicKey[KEY];


uint8_t SndIndex = 0;
uint8_t wsRcvIdx = 0;
uint8_t enque_idx = 0;

//uint8_t SndIndex = 0;
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
  sessionArrayJson["T"] = 2;
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
  writeJson["MAC_0"] = FavoriteMAC[0][0];
  writeJson["MAC_1"] = FavoriteMAC[0][1];
  writeJson["MAC_2"] = FavoriteMAC[0][2];
  writeJson["MAC_3"] = FavoriteMAC[0][3];
  writeJson["MAC_4"] = FavoriteMAC[0][FOUR];
  writeJson["MAC_5"] = FavoriteMAC[0][5];
  writeJson["T"] = 1;
  writeJson["SavingConfig"] = 0;
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
  uint8_t dataType = jsonObj["T"];
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
        //webSocket.broadcastTXT(ReloadStr);
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
        //webSocket.broadcastTXT(ReloadStr);
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
          String keyData = jsonObj["key"].as<String>();
          uint8_t beforeRsa[40];
          memcpy(beforeRsa, MeetingList[sensorGroupId].sessionId, OCT);
          memcpy(&beforeRsa[OCT], MeetingList[sensorGroupId].sharedKey, 32);
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


  //uint8_t enque_idx = 0;
  if (payload != nullptr) {

    switch (type) {
      case WStype_DISCONNECTED:
        {
          Serial.printf("Client #%u disconnected\n", num);
          //WifiConnected = false;
          // vTaskResume(httpdTaskHandle);
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
          // vTaskSuspend(httpdTaskHandle);
        }
        break;
      case WStype_TEXT:
        {
          // Serial.printf("WStype_TEXT from browser #%u: %s\n", length, payload);
          memset(&WsText, 0, length);
          memcpy(&WsText, payload, length);
          WsTextLen = 1;
          jsonReceived((char *)payload);

          //xQueueSend(wsEventQueue, &enque_idx, portMAX_DELAY);
        }
        break;

      case WStype_BIN:
        {
          //vTaskDelay(pdMS_TO_TICKS(100));
          if (payload[0] < 5) {
            memcpy(SndPkt[lt].payloadData, payload, length);
            SndPkt[lt].pktLen = length;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
          } else {
            WsQueue[enque_idx].pktLen = length;
            memcpy(WsQueue[enque_idx].payloadData, payload, length);

            xQueueSend(wsEventQueue, &enque_idx, portMAX_DELAY);
            enque_idx = (enque_idx + 1) & 31;
            Serial.printf("\nWStype_BIN   . %d\n", enque_idx);
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

      //get the MAC address as  uniq identification


      return true;
    }
    
      esp_efuse_mac_get_default(FavoriteMAC[0]);
      esp_read_mac(FavoriteMAC[0], ESP_MAC_WIFI_SOFTAP);
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
  uint8_t rcv_ciphered[PKT];
  uint8_t rcv_decipher[PKT];
  uint8_t rcv_IV[HKEY];
  uint8_t cipher_len = 0;
  uint8_t decipher_len = 0;
  //OPUS
  uint8_t frameIdx = 0;
  uint8_t firstOpus = 0;
  uint8_t phoneEnable;
  uint8_t alt_offset = 0;
  uint8_t alt_src_mac[6];
  bool checkFavorite = false;
  uint8_t checkFavList = 0;  //index of FavoriteList
  char jsonStr[PKT];
  uint8_t alertIdx = 0;
  uint8_t wspIdx = 0;
  uint8_t metIdx = 0;
  uint8_t sphIdx = 0;

  uint8_t groupGpsIdx = 0;


  uint8_t sosType = 0;
  uint8_t checkFav = 0;
  uint8_t jsonStrtrLen = 0;






  //phoneEnable = EnableOLED;
  Serial.println("transformTask.");


  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (rfMode) {

      //Serial.printf("  rcv  . PktLen    %d \n.", PktLen[tk_idx]);


      Serial.printf("  \n.");
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
        case BIT_ALT:  // PayloadData:code,direction,type{button_SOS 0/page_SOS 1/find 2/ackFind 4},SrcMac_6,DstMac_6,originMac_6,metadata_v
          {



            if (TurnOnWifi) {
              webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
            }
            //show in oled


            if (PayloadData[tk_idx][1] == 0)
              alt_offset = 0;
            else
              alt_offset = 12;

            memcpy(alt_src_mac, &PayloadData[tk_idx][3 + alt_offset], 6);
            //check is my favorite contact
            for (checkFav = 0; checkFav < OCT; checkFav++) {
              checkFavorite = compareID(alt_src_mac, FavoriteMAC[checkFav], 6);
              if (checkFavorite) {
                 checkFavList = checkFav;
                break;
              } else {
                checkFavList = 9;
              }
            }

            if (PayloadData[tk_idx][1] < 4) {
              sosType = PayloadData[tk_idx][2];
              switch (sosType) {


                case 2:
                  {
                    // find  rgb beeper
                    if (checkFavList == 0) {
                      //locate flash and beeper ,report ack sensor(GPS\HBR)

                      // EnterSOS = 1;
                      digitalWrite(BEEPER, HIGH);

                      Serial.printf("  search sos : %d\n", PayloadData[tk_idx][2]);
                      LedRed = PayloadData[tk_idx][10 + alt_offset];
                      LedGreen = PayloadData[tk_idx][11 + alt_offset];
                      LedBlue = PayloadData[tk_idx][12 + alt_offset];
                      vTaskDelay(pdMS_TO_TICKS(3000));
                      digitalWrite(BEEPER, LOW);

                      String sosStr = "";
                      memset(WsQueue[0].payloadData, 0, PKT);
                      WsQueue[0].payloadData[0] = 2;
                      WsQueue[0].payloadData[1] = 0;
                      WsQueue[0].payloadData[2] = 3;
                      WsQueue[0].payloadData[10] = '*';
                      GPSjson["W"] = 1;  //sos gps
                      memcpy(&WsQueue[0].payloadData[3], FavoriteMAC[0], 6);
                      serializeJson(GPSjson, sosStr);
                      WsQueue[0].pktLen = sosStr.length() + 1;
                      sosStr.getBytes((unsigned char *)&WsQueue[0].payloadData[11], WsQueue[0].pktLen);
                      WsQueue[0].pktLen = WsQueue[0].pktLen + 11;

                      while (digitalRead(BusyPin) == HIGH || rfMode == false) {
                        taskYIELD();
                        vTaskDelay(pdMS_TO_TICKS(100));
                      }
                      TransmitState = radio.startTransmit(WsQueue[0].payloadData, WsQueue[0].pktLen);
                    }
                  }
                  break;


                case 3:
                  {
                    //taking HBR 
                  }
                  break;

                case 4:  //ack
                  {
                    if (checkFavList == 0) {  //get trigger ACK
                      Serial.printf("  ACK sos : %d\n", PayloadData[tk_idx][2]);
                      EnterSOS = 2;
                      digitalWrite(BEEPER, HIGH);
                      vTaskDelay(pdMS_TO_TICKS(1000));
                      digitalWrite(BEEPER, LOW);
                      vTaskDelay(pdMS_TO_TICKS(1000));
                      digitalWrite(BEEPER, HIGH);
                      vTaskDelay(pdMS_TO_TICKS(1000));
                      digitalWrite(BEEPER, LOW);
                    }
                  }
                  break;

                default:  //0 or 1
                  {
                    sprintf(jsonStr, "%02X%02X%02X%02X%02X%02X ",

                            PayloadData[tk_idx][3 + alt_offset],
                            PayloadData[tk_idx][4 + alt_offset],
                            PayloadData[tk_idx][5 + alt_offset],
                            PayloadData[tk_idx][6 + alt_offset],
                            PayloadData[tk_idx][7 + alt_offset],
                            PayloadData[tk_idx][8 + alt_offset]);


                    String sosMessage = "";

                    if (checkFavList == 9) {
                      //not my favorite ,beep
                      sosMessage = " stranger ";
                    } else {
                      sosMessage = " favorite:" + checkFavList;
                    }

                    sosMessage = sosMessage + " MAC:" + String(jsonStr);
                    uint8_t jsonStrtrLen = 0;
                    if (sosType == 0) {
                      sosMessage = sosMessage + "phoneSOS"
                                   + ";type:" + PayloadData[tk_idx][9 + alt_offset] + ";fatal:" + PayloadData[tk_idx][10 + alt_offset]
                                   + ";Trapped:" + PayloadData[tk_idx][11 + alt_offset] + ";Happened:" + PayloadData[tk_idx][12 + alt_offset];
                      jsonStrtrLen = PktLen[tk_idx] - 13 - alt_offset;
                      memcpy(jsonStr, &PayloadData[tk_idx][13 + alt_offset], jsonStrtrLen);

                    } else {
                      sosMessage = sosMessage + "watchSOS!";
                      jsonStrtrLen = PktLen[tk_idx] - 10 - alt_offset;
                      memcpy(jsonStr, &PayloadData[tk_idx][10 + alt_offset], jsonStrtrLen);
                    }
                    String gpsDataJson = String((char *)&jsonStr);
                    sosMessage = sosMessage + " GPS:" + gpsDataJson;
                    memset(MsgLoop[alertIdx], 0, PKT);
                    jsonStrtrLen = min((size_t)(sosMessage.length() + 1), (size_t)(PKT - 1));
                    sosMessage.getBytes((unsigned char *)MsgLoop[alertIdx], sosMessage.length() + 1);
                    alertIdx = (alertIdx + 1) & 3;

                    //button_SOS
                  }
                  break;
              }
            }


            //
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
                    Serial.printf("rcvGreeting %d  FT.PayloadData:%s  ", grtLen, PayloadData[tk_idx]);
                    if (grtLen == PktLen[tk_idx] - 43) {
                      to_web[0] = SSN;
                      to_web[1] = GRT;  // older version front  is 41
                      to_web[2] = GreetingIndex;
                      to_web_len = grtLen + 3;
                      memcpy(&to_web[3], &PayloadData[tk_idx][43], grtLen);
                      memcpy(GreetingList[GreetingIndex].invitation, &PayloadData[tk_idx][2], OCT);
                      memcpy(GreetingList[GreetingIndex].publicKey, &PayloadData[tk_idx][10], KEY);
                      GreetingList[GreetingIndex].status = 1;
                      GreetingIndex = (GreetingIndex + 1) & 15;
                      webSocket.broadcastBIN(to_web, to_web_len);
                    }
                  }
                  break;

                case CFM:
                  {

                    uint8_t rcvCmf[OCT];
                    uint8_t cfmSharedKey[KEY];
                    uint8_t rcvKey[KEY];
                    cipher_len = PktLen[tk_idx] - 58;
                    memcpy(rcvCmf, &PayloadData[tk_idx][2], OCT);
                    memcpy(myPrivateKey, myPrivate, KEY);
                    bool invCheck = false;
                    invCheck = compareID(rcvCmf, GreetingCode, OCT);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    //Serial.printf(" GreetingCode .invCheck  %d \n.", invCheck);
                    if (invCheck) {
                      memcpy(rcvKey, &PayloadData[tk_idx][10], KEY);
                      memcpy(rcv_IV, &PayloadData[tk_idx][42], HKEY);
                      memcpy(rcv_ciphered, &PayloadData[tk_idx][58], cipher_len);
                      ConfirmIndex = (ConfirmIndex + 1) & 15;
                      ConfirmList[ConfirmIndex].status = 1;
                      GenerateSharedKey(myPrivateKey, rcvKey, cfmSharedKey);
                      vTaskDelay(50 / portTICK_PERIOD_MS);
                      memcpy(ConfirmList[ConfirmIndex].sharedKey, cfmSharedKey, KEY);
                      aes_decipher(rcv_ciphered, cipher_len, rcv_decipher, cfmSharedKey, rcv_IV, &decipher_len);
                      Serial.printf(" after aes_decipher %d \n.", decipher_len);
                      memcpy(ConfirmList[ConfirmIndex].idIndex, rcv_decipher, 2);
                      memcpy(ConfirmList[ConfirmIndex].sessionId, &rcv_decipher[2], OCT);
                      memcpy(&to_web[3], &rcv_decipher[10], decipher_len - 10);
                      to_web[0] = SSN;
                      to_web[1] = CFM;  //older v is 2
                      to_web[2] = ConfirmIndex;
                      to_web_len = decipher_len - 7;
                      webSocket.broadcastBIN(to_web, to_web_len);
                    }
                  }
                  break;
                case TNL:
                  {
                    Serial.println("rcvTunnel  FT.");
                    uint8_t rcvTunIdx = PayloadData[tk_idx][3];
                    memcpy(RcvBuddy, &PayloadData[tk_idx][FOUR], OCT);
                    bool invCheck = false;
                    invCheck = compareID(RcvBuddy, WhisperList[rcvTunIdx].sessionId, OCT);
                    if (invCheck && WhisperList[rcvTunIdx].status == 1) {
                      uint8_t rcv_IV[HKEY];
                      uint8_t rcv_ciphered[KEY];
                      uint8_t rcv_decipher[KEY];
                      uint8_t decipher_len = 0;
                      uint8_t cipher_len = PktLen[tk_idx] - 28;
                      memcpy(rcv_IV, &PayloadData[tk_idx][12], HKEY);
                      memcpy(rcv_ciphered, &PayloadData[tk_idx][28], cipher_len);
                      aes_decipher(rcv_ciphered, cipher_len, rcv_decipher, WhisperList[rcvTunIdx].sharedKey, rcv_IV, &decipher_len);
                      memcpy(WhisperList[rcvTunIdx].buddyIndex, rcv_decipher, 2);
                      memcpy(WhisperList[rcvTunIdx].buddy, &rcv_decipher[2], OCT);
                      while (WhisperAdd)
                        vTaskDelay(pdMS_TO_TICKS(10));
                      WhisperAdd = true;
                      WhisperNum++;
                      WhisperMsgId++;
                      WhisperList[rcvTunIdx].msgId = WhisperMsgId;
                      WhisperList[rcvTunIdx].status = 3;
                      memcpy(&WhisperList[rcvTunIdx].lowId, WhisperList[rcvTunIdx].sessionId, FOUR);
                      memcpy(&WhisperList[rcvTunIdx].highId, &WhisperList[rcvTunIdx].sessionId[FOUR], FOUR);
                      vTaskDelay(pdMS_TO_TICKS(10));
                      // send new whisper json
                      WhisperAdd = false;
                    }
                  }
                  break;

                case IVM:
                  {

                    //Serial.println("rcv IVM   .");
                    RcvIndex = PayloadData[tk_idx][3];
                    memcpy(&RcvLow, &PayloadData[tk_idx][FOUR], FOUR);  // Copy first 4 bytes
                    memcpy(&RcvHigh, &PayloadData[tk_idx][OCT], FOUR);  // Copy last 4 bytes
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
                        memcpy(MeetingList[MeetingNew].sessionId, to_web, OCT);
                        memcpy(MeetingList[MeetingNew].sharedKey, &to_web[OCT], KEY);
                        memcpy(MeetingList[MeetingNew].name, WhisperList[RcvIndex].name, WhisperList[RcvIndex].nameLen);
                        MeetingList[MeetingNew].nameLen = WhisperList[RcvIndex].nameLen;
                        MeetingNum++;
                        JsonDocument meetingNotify;
                        String meetingNotifyStr = "";
                        JsonDocument sessionArrayJson;
                        meetingNotify["T"] = 2;
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
                        memcpy(&MeetingList[RcvIndex].lowId, MeetingList[RcvIndex].sessionId, FOUR);
                        memcpy(&MeetingList[RcvIndex].highId, &MeetingList[RcvIndex].sessionId[FOUR], FOUR);
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

            memcpy(&RcvLow, &PayloadData[tk_idx][3], FOUR);   // Copy first 4 bytes
            memcpy(&RcvHigh, &PayloadData[tk_idx][7], FOUR);  // Copy last 4 bytes
            WhsCheck = idMatch(RcvLow, RcvHigh, WhisperList[RcvIndex].lowId, WhisperList[RcvIndex].highId);

            if (WhsCheck) {

              to_web[0] = WSP;
              to_web[1] = RcvIndex;
              memcpy(rcv_IV, &PayloadData[tk_idx][11], HKEY);
              aes_decipher(&PayloadData[tk_idx][27], PktLen[tk_idx] - 27, &to_web[2], WhisperList[RcvIndex].sharedKey, rcv_IV, &decipher_len);
              to_web_len = decipher_len + 2;
              if (TurnOnWifi) {

                webSocket.broadcastBIN(to_web, to_web_len);
              }

              if (oledMsgShow == 1) {  //show in oled

                // Serial.printf("whisper show in oled  to_web[3]. %d \n ", to_web[2]);
                memset(MsgLoop[wspIdx], 0, PKT);
                if (to_web[2] == TXT_MSG || to_web[2] == NTF_MSG) {
                  memcpy(MsgLoop[wspIdx + FOUR], &to_web[2], to_web_len - 2);
                  jsonStrtrLen = WhisperList[RcvIndex].nameLen;
                  memcpy(MsgLoop[wspIdx + FOUR], WhisperList[RcvIndex].name, jsonStrtrLen);
                  MsgLoop[wspIdx + FOUR][jsonStrtrLen] = ':';
                  memcpy(&MsgLoop[wspIdx + FOUR][jsonStrtrLen + 1], &to_web[2], to_web_len - 2);
                  wspIdx = (wspIdx + 1) & 3;
                }
              }
              //if (to_web[3] == GPS_MSG){}
            }
          }
          break;
        case BIT_MET:  // PayloadData:code,buddyId_8,IV_16,cipheredData{type,len,metadata_v}
          {
            //Serial.println("rcvMeeting  FT.");
            memcpy(&RcvLow, &PayloadData[tk_idx][1], FOUR);   // Copy first 4 bytes
            memcpy(&RcvHigh, &PayloadData[tk_idx][5], FOUR);  // Copy last 4 bytes
            MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[LastMeetingB].lowId, MeetingList[LastMeetingB].highId);
            RcvMeetingIndex = LastMeetingB;
            if (!MetCheck) {
              MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[LastMeetingA].lowId, MeetingList[LastMeetingA].highId);
              RcvMeetingIndex = LastMeetingA;

              if (!MetCheck) {
                RcvMeetingIndex = 0;
                while (RcvMeetingIndex < MeetingTopIndex) {
                  RcvMeetingIndex++;
                  MetCheck = idMatch(RcvLow, RcvHigh, MeetingList[RcvMeetingIndex].lowId, MeetingList[RcvMeetingIndex].highId);

                  if (MetCheck)
                    break;
                }
              }
            }
            if (MetCheck) {
              memcpy(rcv_IV, &PayloadData[tk_idx][9], HKEY);
              aes_decipher(&PayloadData[tk_idx][25], PktLen[tk_idx] - 25, &to_web[2], MeetingList[RcvMeetingIndex].sharedKey, rcv_IV, &decipher_len);
              to_web_len = decipher_len + 2;
              LastMeetingB = RcvMeetingIndex;
              if (TurnOnWifi) {

                to_web[0] = MET;
                to_web[1] = RcvMeetingIndex;
                if (to_web[2] == GPS_MSG) {

                  if (groupGpsIdx < OCT) {
                    memcpy(&GoupGpsList[groupGpsIdx], &to_web[3], sizeof(gpsStruct));
                    groupGpsIdx++;
                  } else {
                    for (int i = 1; i < OCT; i++) {
                      GoupGpsList[i - 1] = GoupGpsList[i];
                    }
                    memcpy(&GoupGpsList[OCT - 1], &to_web[3], sizeof(gpsStruct));
                  }

                } else {
                  webSocket.broadcastBIN(to_web, to_web_len);
                }
              }
              if (oledMsgShow == 1) {  //show in oled
                if (to_web[2] == TXT_MSG || to_web[2] == NTF_MSG) {
                  memset(MsgLoop[metIdx + OCT], 0, PKT);
                  jsonStrtrLen = MeetingList[RcvMeetingIndex].nameLen;
                  memcpy(MsgLoop[metIdx + OCT], MeetingList[RcvMeetingIndex].name, jsonStrtrLen);
                  MsgLoop[metIdx + OCT][jsonStrtrLen] = ':';
                  memcpy(&MsgLoop[metIdx + OCT][jsonStrtrLen + 1], &to_web[2], to_web_len - 2);
                  metIdx = (metIdx + 1) & 3;
                }
              }


              if (ForwardGroup > 0) {
                if (ForwardRSSI > PktRssi[tk_idx] && MeetingList[RcvMeetingIndex].msgId == ForwardGroup) {
                  while (digitalRead(BusyPin) == HIGH || rfMode == false) {
                    taskYIELD();
                    vTaskDelay(pdMS_TO_TICKS(100));
                  }
                  TransmitState = radio.startTransmit(PayloadData[tk_idx], PktLen[tk_idx]);
                }
              }
            }
          }
          break;
        case BIT_SPH:  // PayloadData:code,metadata_v{name,content}
          {
            if (TurnOnWifi) {
              webSocket.broadcastBIN(PayloadData[tk_idx], PktLen[tk_idx]);
            }
            if (oledMsgShow == 1) {
              if (PayloadData[tk_idx][1] == 0) {
                //     Serial.printf("speech show in oled  to_web[3]. %d \n ", to_web[3]);
                memset(MsgLoop[sphIdx + 12], 0, PKT);
                memcpy(MsgLoop[sphIdx + 12], &PayloadData[tk_idx][2], PktLen[tk_idx] - 1);
                sphIdx = (sphIdx + 1) & 3;
              }
            }
          }

          break;

        case BIT_SEN:  // PayloadData:code,direction,DstMac,SrcMac,type{timeReq/timeAck/ciphered},metadata
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
      TrafficDensity++;
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
      Serial.printf("\n electricTask   : %d\n", SndPkt[loraIdx].pktLen);
      //for (int i = 0; i < SndPkt[loraIdx].pktLen; i++) Serial.printf("%d:%02X ", i, SndPkt[loraIdx].payloadData[i]);
      TransmitState = radio.startTransmit(SndPkt[loraIdx].payloadData, SndPkt[loraIdx].pktLen);
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


    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


void sessionCipherTask(void *pvParameters) {
  uint8_t deq_idx;
  uint8_t sessionId[OCT];
  uint8_t deqUncipher[PKT];
  uint8_t deqCiphered[PKT];
  uint8_t uncipher_len = 0;
  uint8_t cipher_len = 0;
  uint8_t tmpIV[HKEY];
  uint8_t sharedKey[KEY];
  uint8_t buddyPublicKey[KEY];
  uint8_t SndIndex = 0;
  uint8_t cipherResult = 0;
  Serial.println("sessionCipherTask  successfully.");
  while (1) {
    // Wait for new data indefinitely (sleeps when queue is empty)
    if (xQueueReceive(wsEventQueue, &deq_idx, portMAX_DELAY)) {

      switch (WsQueue[deq_idx].payloadData[0]) {
        case WSP:
          {
            Serial.println("sndWhisper  FT.");
            SndIndex = WsQueue[deq_idx].payloadData[1];
            SndPkt[lt].payloadData[0] = WSP;
            memcpy(&SndPkt[lt].payloadData[1], &WhisperList[SndIndex].buddyIndex, 2);
            memcpy(&SndPkt[lt].payloadData[3], &WhisperList[SndIndex].buddy, OCT);
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
            Serial.println("\n sndMeeting  FT. \n ");
            SndIndex = WsQueue[deq_idx].payloadData[1];
            SndPkt[lt].payloadData[0] = MET;
            memcpy(&SndPkt[lt].payloadData[1], &MeetingList[SndIndex].sessionId, OCT);
            memcpy(&SndPkt[lt].payloadData[9], MyIV, HKEY);
            memcpy(tmpIV, MyIV, HKEY);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            cipherResult = aes_cipher(&WsQueue[deq_idx].payloadData[2], WsQueue[deq_idx].pktLen - 2, &SndPkt[lt].payloadData[25], MeetingList[SndIndex].sharedKey, tmpIV, &uncipher_len);
            SndPkt[lt].pktLen = uncipher_len + 25;
            xQueueSend(loraQueue, &lt, portMAX_DELAY);
            lt = (lt + 1) & 31;
            if (LastMeetingB != SndIndex)
              LastMeetingA = SndIndex;
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
              MeetingTopIndex = (MeetingTopIndex + 1) & 255;


            MeetingList[MeetingTopIndex].status = 3;
            if (WsQueue[deq_idx].pktLen < 40) {
              Serial.printf("\n safe %d #  create  %d \n.", MeetingTopIndex, WsQueue[deq_idx].pktLen);
              meeting_room_len = WsQueue[deq_idx].pktLen - 2;

              randomSeed(millis() + WsQueue[deq_idx].payloadData[meeting_room_len]);
              for (uint8_t i = 0; i < OCT; i++) {
                tmp_SessionID[i] = random(0, 255);  // Random value between 0 and 255
              }
              randomSeed(millis() + WsQueue[deq_idx].payloadData[meeting_room_len - 2]);
              for (uint8_t i = 0; i < 45; i++) {
                tmp_ciphered[i] = random(0, 255);  // Random value between 0 and 255
              }

            } else {
              Serial.printf("\n temp create  %d \n.", WsQueue[deq_idx].pktLen);
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
            vTaskDelay(10 / portTICK_PERIOD_MS);
            memcpy(MeetingList[MeetingTopIndex].sessionId, tmp_SessionID, OCT);
            memcpy(MeetingList[MeetingTopIndex].sharedKey, &tmp_ciphered[OCT], KEY);
            memcpy(MeetingList[MeetingTopIndex].name, &WsQueue[deq_idx].payloadData[2], meeting_room_len);
            MeetingList[MeetingTopIndex].nameLen = meeting_room_len;
            MeetingMsgId++;
            MeetingList[MeetingTopIndex].msgId = MeetingMsgId;
            MeetingNum++;
            MeetingNew = MeetingTopIndex;
            memcpy(&MeetingList[MeetingTopIndex].lowId, MeetingList[MeetingTopIndex].sessionId, FOUR);
            memcpy(&MeetingList[MeetingTopIndex].highId, &MeetingList[MeetingTopIndex].sessionId[FOUR], FOUR);
            vTaskDelay(10 / portTICK_PERIOD_MS);
          }
          break;



        case SSN:
          {


            switch (WsQueue[deq_idx].payloadData[1]) {



              case GRT:  //SSN,GRT,GreetingCode_8,myPublic_32,grtLen,grtMsg_vl
                {

                  uint8_t grtLen = WsQueue[deq_idx].pktLen - 2;
                  SndPkt[lt].payloadData[0] = SSN;
                  SndPkt[lt].payloadData[1] = GRT;
                  for (int i = 0; i < OCT; i++) {
                    GreetingCode[i] = random(0, 255);  // Replace with a cryptographically secure random function
                  }
                  memcpy(&SndPkt[lt].payloadData[2], GreetingCode, OCT);
                  memcpy(&SndPkt[lt].payloadData[10], myPublic, KEY);
                  SndPkt[lt].payloadData[42] = grtLen;
                  memcpy(&SndPkt[lt].payloadData[43], &WsQueue[deq_idx].payloadData[2], grtLen);
                  SndPkt[lt].pktLen = grtLen + 43;
                  Serial.printf(" \n sndGreeting %d FT:%d. \n", WsQueue[deq_idx].pktLen, SndPkt[lt].pktLen);
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;

              case CFM:  //SSN,CFM,invitation_8,myPublic_32,MyIV_16,ciphered(buddyId_2,sessionId_8,cfmMsg_vl)
                {
                  Serial.println("sndConfirm  FT.");
                  uint8_t gretIndex = WsQueue[deq_idx].payloadData[2];
                  uint8_t cfmLen = WsQueue[deq_idx].pktLen - 3;
                  while (WhisperList[WhisperIndex].status != 0) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    WhisperIndex = (WhisperIndex + 1) & 255;
                  }
                  if (cfmLen < KEY)
                    WhisperList[WhisperIndex].nameLen = cfmLen;
                  else
                    WhisperList[WhisperIndex].nameLen = KEY;
                  // generate sharedKey and     sessionId
                  RandomGenerate(sessionId, OCT);
                  memcpy(buddyPublicKey, GreetingList[gretIndex].publicKey, KEY);
                  memcpy(myPrivateKey, myPrivate, KEY);
                  GenerateSharedKey(myPrivateKey, buddyPublicKey, sharedKey);
                  WhisperList[WhisperIndex].status = 1;  // confirm:1
                  memcpy(WhisperList[WhisperIndex].name, &WsQueue[deq_idx].payloadData[3], WhisperList[WhisperIndex].nameLen);
                  memcpy(WhisperList[WhisperIndex].sessionId, sessionId, OCT);
                  memcpy(WhisperList[WhisperIndex].sharedKey, sharedKey, KEY);
                  deqUncipher[0] = WhisperIndex / ULIST;
                  deqUncipher[1] = WhisperIndex % ULIST;
                  memcpy(&deqUncipher[2], sessionId, OCT);
                  memcpy(&deqUncipher[10], &WsQueue[deq_idx].payloadData[3], cfmLen);
                  memcpy(tmpIV, MyIV, HKEY);

                  aes_cipher(deqUncipher, cfmLen + 10, deqCiphered, sharedKey, tmpIV, &cipher_len);

                  SndPkt[lt].payloadData[0] = SSN;
                  SndPkt[lt].payloadData[1] = CFM;
                  memcpy(&SndPkt[lt].payloadData[2], GreetingList[gretIndex].invitation, OCT);
                  memcpy(&SndPkt[lt].payloadData[10], myPublic, KEY);
                  memcpy(&SndPkt[lt].payloadData[42], MyIV, HKEY);
                  memcpy(&SndPkt[lt].payloadData[58], deqCiphered, cipher_len);




                  SndPkt[lt].pktLen = cipher_len + 58;
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;

              case TNL:  //SSN,TNL,buddyIndex_2,buddy_8,MyIV_16,ciphered(buddyId,sessionId_8)
                {
                  //Serial.println("sndTunnel  FT.");
                  if (WhisperNum < MAX_LIST) {

                    while (WhisperAdd)
                      vTaskDelay(pdMS_TO_TICKS(10));
                    WhisperAdd = true;
                    WhisperNum++;

                    uint8_t tunIndex = WsQueue[deq_idx].payloadData[2];
                    if (ConfirmList[tunIndex].status == 1) {
                      while (WhisperList[WhisperIndex].status != 0)  // infinite loop trigger reboot
                      {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        WhisperIndex = (WhisperIndex + 1) & 255;
                      }
                      if (WsQueue[deq_idx].pktLen > 34)
                        WhisperList[WhisperIndex].nameLen = KEY;
                      else
                        WhisperList[WhisperIndex].nameLen = WsQueue[deq_idx].pktLen - 3;

                      memcpy(WhisperList[WhisperIndex].name, &WsQueue[deq_idx].payloadData[3], WhisperList[WhisperIndex].nameLen);
                      memcpy(&WhisperList[WhisperIndex].buddy, &ConfirmList[tunIndex].sessionId, OCT);
                      memcpy(WhisperList[WhisperIndex].buddyIndex, &ConfirmList[tunIndex].idIndex, 2);
                      memcpy(WhisperList[WhisperIndex].sharedKey, &ConfirmList[tunIndex].sharedKey, KEY);
                      RandomGenerate(WhisperList[WhisperIndex].sessionId, OCT);
                      memset(&ConfirmList[tunIndex], 0, sizeof(ConfirmList[tunIndex]));
                      memcpy(&WhisperList[WhisperIndex].lowId, WhisperList[WhisperIndex].sessionId, FOUR);
                      memcpy(&WhisperList[WhisperIndex].highId, &WhisperList[WhisperIndex].sessionId[FOUR], FOUR);
                      vTaskDelay(10 / portTICK_PERIOD_MS);
                      // cipher
                      deqUncipher[0] = WhisperIndex / ULIST;
                      deqUncipher[1] = WhisperIndex % ULIST;
                      memcpy(&deqUncipher[2], WhisperList[WhisperIndex].sessionId, OCT);
                      memcpy(tmpIV, MyIV, HKEY);
                      Serial.printf("sndTunnel name      %d\n.", WhisperList[WhisperIndex].nameLen);

                      // WhisperList[WhisperIndex].name

                      aes_cipher(deqUncipher, 10, deqCiphered, WhisperList[WhisperIndex].sharedKey, tmpIV, &cipher_len);
                      SndPkt[lt].payloadData[0] = SSN;
                      SndPkt[lt].payloadData[1] = TNL;
                      memcpy(&SndPkt[lt].payloadData[2], &WhisperList[WhisperIndex].buddyIndex, 2);
                      memcpy(&SndPkt[lt].payloadData[FOUR], &WhisperList[WhisperIndex].buddy, OCT);
                      memcpy(&SndPkt[lt].payloadData[12], MyIV, HKEY);
                      memcpy(&SndPkt[lt].payloadData[28], deqCiphered, cipher_len);
                      SndPkt[lt].pktLen = cipher_len + 28;
                      xQueueSend(loraQueue, &lt, portMAX_DELAY);

                      lt = (lt + 1) & 31;

                      if (WhisperTopIndex < MAX_LIST)
                        WhisperTopIndex++;
                      WhisperMsgId++;
                      WhisperList[WhisperIndex].msgId = WhisperMsgId;
                      WhisperList[WhisperIndex].status = 3;
                      vTaskDelay(pdMS_TO_TICKS(10));
                      WhisperAdd = false;
                    }
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
                  memcpy(&SndPkt[lt].payloadData[FOUR], &WhisperList[SndIndex].buddy, OCT);
                  memcpy(&SndPkt[lt].payloadData[12], MyIV, HKEY);
                  memcpy(tmpIV, MyIV, HKEY);
                  memcpy(tmp, &MeetingList[metIndex].sessionId, OCT);
                  memcpy(&tmp[OCT], &MeetingList[metIndex].sharedKey, KEY);
                  memcpy(&tmp[40], &WhisperList[SndIndex].sessionId, OCT);
                  aes_cipher(tmp, TM, &SndPkt[lt].payloadData[28], WhisperList[SndIndex].sharedKey, tmpIV, &uncipher_len);
                  SndPkt[lt].pktLen = uncipher_len + 28;
                  xQueueSend(loraQueue, &lt, portMAX_DELAY);
                  lt = (lt + 1) & 31;
                }
                break;
            }
          }
      }
      memset(&WsQueue[deq_idx], 0, 257);
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
    vTaskDelay(pdMS_TO_TICKS(10));
    //taskYIELD();
  }
}
