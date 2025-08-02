
//--------------------------------------------------------------------------------------------------------#include

#include <Arduino.h>
#include <ArduinoJson.h>
#include "esp_timer.h"
//--------------------------------------------------------------------------------------------------------#define

//-------------------------------------------------------hardware

#define CORE0 0
#define CORE1 0

//-----------------------------LLCC68
#define NssPin 10
#define RstPin 8
#define BusyPin 9
#define DIO1Pin 38
#define DIO2Pin 46

//-----------------------------I2C
#define SDA_PIN 40
#define SCK_PIN 39
//-----------------------------GPS UART
#define TX_PIN 43
#define RX_PIN 44

#define MIN_MILL 6000
//-----------------------------RGB ws2812
#define LED_PIN 48
#define BRIGHTNESS 40



#define MENU_BTN 15  //long press SOS
#define PAGE_BTN 5
#define BEEPER 47
#define GPS_INT 1000000


//-------------------------------------------------------software
//-----------------------------BITWISE
#define BIT_PHO (1 << 0)  // Bit 0              // opus frame
#define BIT_ALT (1 << 1)  // Bit 1              // alert :SOS ,audio,webp
#define BIT_SPH (1 << 2)  // Bit 2              // greeting,confirm,tunnel,meetingJoin
#define BIT_SEN (1 << 3)  // Bit 3              // sensor req/ack/set/timeSyn/i2cConfig / relayExist
//#define BIT_ (1 << 4)  // Bit 4              //  preserve
#define BIT_WSP (1 << 5)  // Bit 5              // whisper
#define BIT_MET (1 << 6)  // Bit 6              // meeting
#define BIT_SSN (1 << 7)  // Bit 7              // speech

//-----------------------------SIZE
#define FOUR 4
#define OCT 8
#define HKEY 16
#define KEY 32
#define TM 64
#define DTM 128
#define PAYLOAD 192
#define PKT 256
#define ULIST 256
#define MAX_LIST 256

#define BUFFER_SIZE 4096

//-----------------------------CODE
#define PHO 1  // phoneTalking
#define ALT 2
//alert : direction{trigger 0/up 1/down 2/triggerAck 3/upAck 4/downAck 5},type{},dstMac_6/firstRelayMac,srcMac_6,metaData

/*
case 0:// watch  SOS
MAC_6;binary_4,GPS_V;;msg

case 1:// dock SOS
MAC_6;GPS_V;


case 2:// search / request beeper / RGB flash / GPS /HBR
MAC_6;cmd,operator

case 3://  
MAC_6;json

case 4:// opus
MAC_6;opus frame

case 5:// text
MAC_6; string

 

*/

#define SPH 4  // speech
//updateLeft(leftNotify/LeftNoAck/Deploy)
#define SEN 8  // sensor :direction{up/down},dstMac_6,ciphered(type{Set/report/forward},cmdCode{measure/Time/Reboot},objLen, obj_v) iv16 is today date
//#define PRE 16   // preserve
#define WSP 32  // whisper
#define MET 64  // meeting

#define MCT 65  // meeting

#define SSN 128  // session
//-----------------------------MSG_TYPE
#define CMD_MSG 0  // setting //JSON:reset ,reboot,ackFreqSet,loraCfgSet ,mode switch(interval,ack,repeater),
#define NTF_MSG 1  // notification banner {oled / web }
#define TXT_MSG 2  // text msg
#define IMG_MSG 3  //  3 = WebP
#define GPS_MSG 4  //

#define GRT 5  // greeting
#define CFM 6  // confirm
#define TNL 7  // tunnel

#define IVM 9  // invite join meeting

#define REQ 17  // request sensor info:I2C , GPS,voltage
#define ACK 18  // sensor info
#define RLY 19  // turn on or turn off buddy relay
//#define LTR 20   // request buddy location and beeper
#define DRS 21  // report SOS to specific contact



//-----------------------------LORA   freq  & bw
#define ASISM 470  // asia center freq
#define EUISM 868  // eu center freq
#define NAISM 915  // north america center freq

#define LDRB 125  // low date rate bandwidth
#define MDRB 250  // middle date rate bandwidth
#define HDRB 500  // high date rate bandwidth



//-----------------------------language font

#define LATIN 0
#define CHINESE 1
#define CYRILLIC 2
#define GREEK 3
#define HEBREW 4
#define JAPANESE 5
#define KOREAN 6


//--------------------------------------------------------------------------------------------------------variable






//-------------------------------------------------------hardware

//-----------------------------peripherals
extern uint8_t EnableOLED;    // if  peripherals attached
extern uint8_t AmplitudeIR;   // if  peripherals attached 

extern uint8_t menuButton;  //loop menu ,pageButton=0;
extern uint8_t pageButton;  //loop page ,long press sos

extern bool ButtonIR;  //speak PTT
extern uint8_t EnterSOS;
extern uint8_t takingHBR;
extern uint8_t LocationSaveFreq;  //

extern uint8_t oledMsgShow;          //
extern uint8_t RelayNum;             //from the last PathGpsList position count , received sensorRelay broadcast
extern uint8_t FavoriteMAC[OCT][6];  //emergency contact  MAC,first is selfMac

extern volatile bool TurnOnWifi;  //0 off / 1 on

extern uint8_t LedRed;
extern uint8_t LedGreen;
extern uint8_t LedBlue;

extern char MsgLoop[HKEY][PKT];
extern uint8_t scrollSpeed;
extern StaticJsonDocument<200> GPSjson;



extern int Satellites;
//-------------------------------------------------------software

//-----------------------------OPUS
extern uint8_t frame_buffer[KEY][DTM];
extern size_t frame_len[KEY];


//-----------------------------settings

extern volatile uint8_t Role;      //FT / Relay(PktDensity<6&&rssi< x Dbm) / Sensor / Gateway(IOT manager) /Camera /Kids
extern volatile uint8_t Mode;      //0 FIT / 1 TOP /2  Forward  /3  sensorRelay /4  SOS
extern volatile uint8_t WifiMode;  //AP / STA
extern volatile bool PhonePermit;  //allow send opus audio out,250 && center freq only permit SOS mode
extern volatile int timeZone;      //timeZone offset UTC
extern volatile uint8_t MissionGroup;
extern uint8_t ForwardGroup;
extern float ForwardRSSI;

extern String SSID;
extern String Password;


extern volatile uint8_t TrafficDensity;
extern volatile uint8_t PKT_BITS;



//-----------------------------LORA

extern float Frequency;
extern float Bandwidth;
extern uint8_t SpreadFactor;
extern uint8_t CodeRate;
extern int8_t RadioPower;
extern uint8_t SyncWord;
extern uint16_t PreambleLength;


//-----------------------------web

extern String ReloadStr;
//-----------------------------AES

extern uint8_t myPrivate[KEY];
extern uint8_t myPublic[KEY];
extern uint8_t MyIV[HKEY];

//-----------------------------FreeRTOS
extern TaskHandle_t transformTaskHandle;
extern TaskHandle_t electricTaskHandle;

extern TaskHandle_t sessionCipherTaskHandle;

extern TaskHandle_t httpdTaskHandle;

extern TaskHandle_t websocketTaskHandle;

extern TaskHandle_t gpsTaskHandle;
extern TaskHandle_t oledTaskHandle;
extern TaskHandle_t max3010xTaskHandle;


extern TaskHandle_t listenTaskHandle;
extern TaskHandle_t phoneTaskHandle;


extern esp_timer_handle_t timer;


extern QueueHandle_t loraQueue;
extern QueueHandle_t listenQueue;

extern QueueHandle_t wsEventQueue;  // Define the queue

//-----------------------------timer


extern volatile bool taskReady;
//-----------------------------session
extern volatile uint8_t GreetingIndex;
extern volatile uint8_t ConfirmIndex;


extern bool Greeting;  // true after greeting .update publicKey ->false

extern volatile bool WhisperAdd;
extern volatile bool MeetingAdd;


extern volatile uint16_t WhisperTopIndex;
extern volatile uint16_t MeetingTopIndex;


extern volatile uint16_t WhisperNew;
extern volatile uint16_t MeetingNew;

extern volatile uint8_t LastMeetingA;  // last meeting id for active
extern volatile uint8_t LastMeetingB;  // last meeting id for active


extern uint8_t GreetingCode[OCT];
extern bool SandboxFlag;



//-----------------------------Contact

extern volatile uint16_t WhisperNum;
extern volatile uint16_t MeetingNum;


extern volatile uint16_t WhisperMsgId;
extern volatile uint16_t MeetingMsgId;


extern uint8_t to_web[PKT];
extern uint8_t to_web_len;

extern uint8_t MyNameLen;
extern uint8_t MyName[KEY];
//extern uint8_t WhisperMap[ULIST];
//extern uint8_t MeetingMap[ULIST];  // if (meeting[0]>0) match(buddy) until i ==meeting[0]


extern volatile uint16_t WhisperIndex;
extern volatile uint16_t MeetingIndex;


extern uint8_t WsText[512];

extern volatile uint8_t WsTextLen;

//--------------------------------------------------------------------------------------------------------struct
//-----------------------------struct define

typedef struct __attribute__((aligned(4))) PktStruct {
  uint8_t pktLen;
  uint8_t payloadData[PKT];  // code[0],idIndex[1],dataType[2],data[3+N]
};
typedef struct SystemConfig {
  uint8_t PktBits;
  uint8_t MyNameLen;
  uint8_t MyName[KEY];
  uint8_t Mode;  //FT / Repeater / Sensor / IOT manager /Camera
  uint8_t ForwardGroup;
  uint8_t allowFound;
  uint8_t oledLanguage;
  //-----------------------------radio setting
  float Frequency;
  float Bandwidth;
  uint8_t SpreadFactor;
  uint8_t CodeRate;
  int8_t RadioPower;
  uint8_t SyncWord;
  uint16_t PreambleLength;
  float ForwardRSSI;
  //-----------------------------WIFI
  uint8_t SSID[KEY];
  uint8_t Password[KEY];
  int WifiTimeOut;
};

typedef struct ContactStruct {
  int msgId;
  uint8_t status;  // 0 ready;1 confirm;2 tunnel;3 running;4 leaving;
  uint8_t sharedKey[KEY];
  uint8_t sessionId[OCT];  //sessionId[0]  as rssi level in meeting
  uint32_t highId;
  uint32_t lowId;
  uint8_t buddy[OCT];     // whisper sessionId,meeting rssi
  uint8_t buddyIndex[2];  // basic user only for uint8_t[1],pro for uint8_t[1]~[2]
  uint8_t nameLen;
  uint8_t name[KEY];
};

typedef struct greetingStruct {
  uint8_t status;
  uint8_t publicKey[KEY];
  uint8_t invitation[OCT];
};

typedef struct confirmStruct {
  uint8_t status;  // 0 received ,1 toWeb,2 tunnel
  uint8_t sharedKey[KEY];
  uint8_t sessionId[OCT];
  uint8_t idIndex[2];
};


typedef struct gpsStruct {
  int latitude;
  int longitude;
  float altitude;
  float speed;
  uint8_t relayNum;
};





//-----------------------------instance

extern PktStruct WsQueue[KEY];  // ws_BIN received
extern PktStruct SndPkt[KEY];

extern greetingStruct GreetingList[OCT];
extern confirmStruct ConfirmList[OCT];


extern ContactStruct WhisperList[ULIST];
extern ContactStruct MeetingList[ULIST];


extern gpsStruct PathGpsList[KEY];
extern gpsStruct GoupGpsList[OCT];
extern SystemConfig FTconfig;



//--------------------------------------------------------------------------------------------------------StaticContent


//extern const char PGM_SpeechListJson[] PROGMEM;
extern const char index_html[] PROGMEM;
extern const char styles_css[] PROGMEM;
extern const char chat_js[] PROGMEM;
extern const char jszip_js[] PROGMEM;
