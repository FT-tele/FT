
#include <Arduino.h>
#include <Wire.h>
#define SENSOR_RATE 100
#define RATE_SIZE 4 

#define OLED_PREVIEW 0 
#define OLED_MSG 1
#define OLED_PATH 2 
#define OLED_HBR 3 

void onTimer(void *arg);
void IRAM_ATTR onFalling();
void IRAM_ATTR onRising(); 
void gpsTask(void *pvParameters);
void oledTask(void *pvParameters);

void max3010xTask(void *pvParameters);
 
 /*
//-------------------------------------------------------OLED menu loop
//-----------------------------Preview
1.time(satelites Alert) \trip \speed
2.altitude \latitude \longtitude \satelites \date
3.total path map ,last relay rcv  time
4.turn off screen
//-----------------------------Msg
1.whisper text
2.favoriteMeeting text
3.alert text
4.SOS Menu
//-----------------------------Map
1.latest 10 coordinate path ( per 3 min )
2.latest 20 coordinate path
3.total path
4.favoriteMeeting nearby buddy (5)

//-----------------------------HBR
1.only HBR

 
//-------------------------------------------------------buttonIR


//-----------------------------Preview
send opus only if  colorLevels < 2  || except off center frequency (433\ 868\ 915)

//-----------------------------Msg
send opus

//-----------------------------Map
send opus

//-----------------------------HBR
HBR test
*/