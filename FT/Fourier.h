
#include <Arduino.h>

#define WDT_TIMEOUT_SECONDS 30  // Example: 30 seconds


const char PGM_SpeechListJson[] PROGMEM = R"rawliteral(
{
  "dataType": 2,
  "SpeechList": [
    {"name": "🔔💬", "sessionId": 0, "msgId": 0},
    {"name": "🍰🥤", "sessionId": 1, "msgId": 1},
    {"name": "🛒🛍️", "sessionId": 2, "msgId": 2},
    {"name": "💊💉", "sessionId": 3, "msgId": 3},
    {"name": "🖨️🖥️", "sessionId": 4, "msgId": 4},
    {"name": "🚕🚚", "sessionId": 5, "msgId": 5},
    {"name": "🎓🎒", "sessionId": 6, "msgId": 6},
    {"name": "🎶🎮", "sessionId": 7, "msgId": 7},
    {"name": "✡️✝️", "sessionId": 8, "msgId": 8},
    {"name": "🛠️🏛️", "sessionId": 9, "msgId": 9},
    {"name": "✈️🏝️", "sessionId": 10, "msgId": 10},
    {"name": "🚴🏀", "sessionId": 11, "msgId": 11},
    {"name": "💲💳", "sessionId": 12, "msgId": 12},
    {"name": "👠💎", "sessionId": 13, "msgId": 13},
    {"name": "🛡️🔒", "sessionId": 14, "msgId": 14},
    {"name": "🐱🐠", "sessionId": 15, "msgId": 15}
  ]
}
)rawliteral";



//-------------------------------------------------------api
bool initInfrast();
bool compareID(byte *array1, byte *array2, uint8_t cmpLength);

//-----------------------------wifi
bool testWifi();
void wifiMode();


//-----------------------------file

bool loadConfig();
bool resetConfig(uint8_t option);


bool saveConfig();
void sessionToFront();  //send session lists to web browser
void settingToFront();
bool idMatch(uint32_t highPart1, uint32_t lowPart1, uint32_t highPart2, uint32_t lowPart2);

void radioConfigUpdate();
void keyUpdate();
void loraInit();


//-------------------------------------------------------task

void electricTask(void *pvParameters);
void transformTask(void *pvParameters);
void forwardTask(void *pvParameters);
void httpdTask(void *pvParameters);

void websocketTask(void *pvParameters);

void magneticTask(void *pvParameters);