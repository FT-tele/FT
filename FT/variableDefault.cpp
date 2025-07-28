
#include "VariableDefine.h"
//--------------------------------------------------------------------------------------------------------variable

//-------------------------------------------------------hardware

//-----------------------------peripherals
uint8_t EnableOLED = 1;    // if  peripherals attached
uint8_t AmplitudeIR = 60;  // if  peripherals attached
uint8_t LanguageFont = 1;  // if  peripherals attached ,msg langunge
bool ButtonIR = false;     //speak PTT
uint8_t EnterSOS = 0;      //waiting for ack
uint8_t takingHBR = 0;
uint8_t LocationSaveFreq = 1;
uint8_t RelayNum = 0;
uint8_t oledMsgShow = 1;
volatile bool TurnOnWifi = 0;  //0 off / 1 on
uint8_t FavoriteMAC[OCT][6];   //MAC address of AP

uint8_t LedRed = 0;
uint8_t LedGreen = 0;
uint8_t LedBlue = 0;

char MsgLoop[HKEY][PKT];
uint8_t scrollSpeed = 2;
StaticJsonDocument<200> GPSjson;

int Satellites = 0;
//-------------------------------------------------------software

//-----------------------------OPUS
uint8_t frame_buffer[KEY][DTM];
size_t frame_len[KEY];


//-----------------------------settings

volatile uint8_t Role = 0;      //FT / Relay(PktDensity<6&&rssi< x Dbm) (broadcast MAC and time every 5 min)  / Sensor/ Gateway(IOT manager) /Camera /Kids
volatile uint8_t Mode = 2;      //0 Watch / 1 Dock /2  FT  /3  Forward /4  SOS
volatile uint8_t WifiMode = 0;  //AP / STA
volatile bool PhonePermit = false;
extern volatile int timeZone = 8;  //timeZone offset UTC
volatile uint8_t MissionGroup = 0;
uint8_t ForwardGroup = 0;
float ForwardRSSI = 0;


String SSID = "AC";
String Password = "Or^Password";
String GPSjsonStr;

uint8_t volatile TrafficDensity = 0;
volatile uint8_t PKT_BITS = 0b11111111;


//-----------------------------LORA

float Frequency = NAISM;
float Bandwidth = MDRB;
uint8_t SpreadFactor = 7;
uint8_t CodeRate = 5;
int8_t RadioPower = 10;
uint8_t SyncWord = 67;
uint16_t PreambleLength = 10;


//-----------------------------web

String ReloadStr = "AP1";
//-----------------------------AES

uint8_t myPrivate[KEY];
uint8_t myPublic[KEY];
uint8_t MyIV[HKEY];

//-----------------------------FreeRTOS
TaskHandle_t transformTaskHandle;
TaskHandle_t electricTaskHandle;

TaskHandle_t sessionCipherTaskHandle;
TaskHandle_t httpdTaskHandle;

TaskHandle_t websocketTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t oledTaskHandle;
TaskHandle_t max3010xTaskHandle;


TaskHandle_t listenTaskHandle;
TaskHandle_t phoneTaskHandle;




esp_timer_handle_t timer;


QueueHandle_t loraQueue;
QueueHandle_t listenQueue;
QueueHandle_t wsEventQueue;
//-----------------------------timer


volatile bool taskReady = false;
//-----------------------------session
volatile uint8_t GreetingIndex = 0;
volatile uint8_t ConfirmIndex = 0;


bool Greeting = 0;  // true after greeting .update publicKey ->false

volatile bool WhisperAdd = false;
volatile bool MeetingAdd = false;


volatile uint16_t WhisperTopIndex = 0;
volatile uint16_t MeetingTopIndex = 0;


volatile uint16_t WhisperNew = 0;
volatile uint16_t MeetingNew = 0;

volatile uint8_t LastMeetingA = 0;  // last meeting id for active
volatile uint8_t LastMeetingB = 0;  // last meeting id for active


uint8_t GreetingCode[OCT];
bool SandboxFlag = true;

//-----------------------------Contact

volatile uint16_t WhisperNum = 0;
volatile uint16_t MeetingNum = 0;


volatile uint16_t WhisperMsgId = 0;
volatile uint16_t MeetingMsgId = 0;


uint8_t to_web[PKT];
uint8_t to_web_len = 0;

uint8_t MyNameLen = 0;
uint8_t MyName[KEY];

volatile uint16_t WhisperIndex = 0;
volatile uint16_t MeetingIndex = 0;





uint8_t WsText[512];

volatile uint8_t WsTextLen;

//-----------------------------instance

PktStruct WsQueue[KEY];  // ws_BIN received
PktStruct SndPkt[KEY];


ContactStruct WhisperList[ULIST];
ContactStruct MeetingList[ULIST];


greetingStruct GreetingList[OCT];
confirmStruct ConfirmList[OCT];


gpsStruct PathGpsList[KEY];
gpsStruct GoupGpsList[OCT];

SystemConfig FTconfig;
