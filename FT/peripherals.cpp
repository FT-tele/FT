#include <Arduino.h>
#include "esp_heap_caps.h"
#include <FastLED.h>
#include "heartRate.h"
#include "MAX30105.h"
#include <TinyGPS++.h>
#include <U8g2lib.h>



#include "Fourier.h"
#include "Sensor.h"
#include "VariableDefine.h"




//-------------------------------------------------------static


//-----------------------------GPS


TinyGPSPlus gps;

double Latitude = 0;
double Longitude = 0;
double Altitude = 0;
double Speed = 0;
uint8_t pathIndex = 0;
//StaticJsonDocument<200> GPSjson;

// Variables to track distance
double TotalDistanceMeters = 0.0;
char timeBuffer[9];
//-----------------------------OLED


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


uint8_t msgIndex = 0;
int xPos = KEY;
//-----------------------------MAX3010x

MAX30105 particleSensor;

float temperature = 0;
float beatsPerMinute;
int beatAvg;
long irValue = 0;



//-----------------------------RGB led


const uint32_t colorLevels[16] = {
  0x000000, 0x00FF00, 0x33FF00, 0x66FF00,
  0x99FF00, 0xCCFF00, 0xFFFF00, 0xFFCC00,
  0xFF9900, 0xFF6600, 0xFF3300, 0xFF0000,  // Pure GREEN to Red

  0x0000FF,  // blue
  0x00FFFF,  // Aqua
  0xCC00FF,  // purple
  0xFFFFFF   // Pure White
};

CRGB leds[1];


//-----------------------------button

volatile int64_t startTime = 0;
volatile int64_t pulseDuration = 0;
uint8_t sosResponse = 0;  // MENU_BTN click ackonwledge


//-------------------------------------------------------ISR


//-----------------------------timer


void onTimer(void *arg) {
  taskReady = true;  // Set a flag when the timer fires

  // Serial.printf("timer %d\n", millis());
  //relay broadcast time  ,FT check relay num
}


//-----------------------------button


void IRAM_ATTR onFalling() {
  startTime = esp_timer_get_time();                                    // Start time in µs
  attachInterrupt(digitalPinToInterrupt(MENU_BTN), onRising, RISING);  // Swap to release
}

void IRAM_ATTR onRising() {
  pulseDuration = esp_timer_get_time() - startTime;
  if (pulseDuration < 5000000) {  // Short press
    menuButton = (menuButton + 1) & 3;
    pageButton = 0;
    if (menuButton == 3) {
      takingHBR = 1;
    } else {
      takingHBR = 0;
    }

  } else {  // Long press SOS
    menuButton = 1;
    pageButton = 3;
    EnterSOS = 1;
    //takingHBR = 1;
  }
  attachInterrupt(digitalPinToInterrupt(MENU_BTN), onFalling, FALLING);  // Swap back to press
}

void IRAM_ATTR pageLoop() {
  pageButton = (pageButton + 1) & 3;
  //xPos = 0;
}


//-------------------------------------------------------PSRAM

//-----------------------------MAX3010x


//-----------------------------GPS



//-----------------------------OLED



//-------------------------------------------------------task

void max3010xTask(void *pvParameters) {

  Serial.println("max30102Task  successfully.");

  byte rates[RATE_SIZE];  //Array of heart rates
  byte rateSpot = 0;
  long lastBeat = 0;  //Time at which the last beat occurred

  long delta;


  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                  //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeIR(AmplitudeIR);
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED
  particleSensor.enableDIETEMPRDY();         //Enable the temp ready interrupt. This is required.




  vTaskDelay(3000 / portTICK_PERIOD_MS);

  while (1) {
    irValue = particleSensor.getIR();
    if (EnterSOS == 1) {
      ButtonIR = false;
      //takingHBR = 1;
      ////alert : direction{trigger 0},type{1},srcMac_6,GPSjson
      String sosStr = "";
      memset(WsQueue[0].payloadData, 0, PKT);
      WsQueue[0].payloadData[0] = 2;
      WsQueue[0].payloadData[1] = 0;
      WsQueue[0].payloadData[2] = 1;
      WsQueue[0].payloadData[10] = '*';
      GPSjson["W"] = 1;  //sos gps

      //for (int i = 0; i < 6; i++) Serial.printf("%d:%02X ", i, FavoriteMAC[0][i]);
      memcpy(&WsQueue[0].payloadData[3], FavoriteMAC[0], 6);
      serializeJson(GPSjson, sosStr);
      WsQueue[0].pktLen = sosStr.length() + 1;
      sosStr.getBytes((unsigned char *)&WsQueue[0].payloadData[11], WsQueue[0].pktLen);
      WsQueue[0].pktLen = WsQueue[0].pktLen + 11;
      //WsQueue[0].payloadData[WsQueue[0].pktLen] = '0';
      memcpy(SndPkt[takingHBR].payloadData, WsQueue[0].payloadData, WsQueue[0].pktLen);
      SndPkt[takingHBR].pktLen = WsQueue[0].pktLen;
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      //for (int i = 0; i < SndPkt[takingHBR].pktLen; i++) Serial.printf("%d:%02X ", i, SndPkt[takingHBR].payloadData[i]);
      xQueueSend(loraQueue, &takingHBR, portMAX_DELAY);
    }

    if (takingHBR == 1) {


      if (checkForBeat(irValue) == true) {
        //We sensed a beat!
        delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
          rateSpot %= RATE_SIZE;                     //Wrap variable

          //Take average of readings
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
          Serial.print("\n IR=");
          Serial.print(irValue);
          Serial.print(", Avg BPM=");
          Serial.print(beatAvg);
        }
        if (beatAvg == 0) beatsPerMinute == 0;
      }
      temperature = particleSensor.readTemperature();






    } else {
      if (PhonePermit) {
        if (irValue > 5000) {
          ButtonIR = true;
        } else {
          ButtonIR = false;
        }
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}




void gpsTask(void *pvParameters) {

  bool located = false;
  int hour = 0, minute = 0, second = 0;
  int day = 0, month = 0, year = 0;
  int local_hour = 8;

  uint8_t buffer[128];
  size_t length;

  double lastLat = 0.0;
  double lastLon = 0.0;
  bool hasLastLocation = false;
  double segment = 0;

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  unsigned long lastPathSaving = 0;
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  Serial.println("gpsTask  successfully.");
  while (true) {


    length = Serial1.readBytes(buffer, sizeof(buffer));
    for (size_t i = 0; i < length; i++) {
      gps.encode(buffer[i]);
    }

    if (hasLastLocation) {
      segment = TinyGPSPlus::distanceBetween(

        Latitude, Longitude, lastLat, lastLon);

      if (segment >= 5.0) {
        TotalDistanceMeters += segment;
      }
      hasLastLocation = false;
    }

    if (gps.location.isValid()) {
      located = true;
      Latitude = gps.location.lat();
      Longitude = gps.location.lng();

      lastLat = Latitude;
      lastLon = Longitude;
      hasLastLocation = true;
      if ((millis() - lastPathSaving) > LocationSaveFreq * MIN_MILL) {

        if (pathIndex == 31) {
          for (int i = 1; i < HKEY; i++) {
            PathGpsList[i - 1] = PathGpsList[i];
          }
          PathGpsList[pathIndex].latitude = int(Latitude * GPS_INT);
          PathGpsList[pathIndex].longitude = int(Longitude * GPS_INT);
          PathGpsList[pathIndex].altitude = float(Altitude);
          PathGpsList[pathIndex].speed = float(Speed);
          PathGpsList[pathIndex].relayNum = RelayNum;

        } else {
          PathGpsList[pathIndex].latitude = int(Latitude * GPS_INT);
          PathGpsList[pathIndex].longitude = int(Longitude * GPS_INT);
          PathGpsList[pathIndex].altitude = float(Altitude);
          PathGpsList[pathIndex].speed = float(Speed);
          PathGpsList[pathIndex].relayNum = RelayNum;
          pathIndex++;
        }


        lastPathSaving = millis();

        // pathIndex = (pathIndex + 1) & 15;

        GPSjson["T"] = 5;
        GPSjson["W"] = 0;  //self
        GPSjson["La"] = Latitude;
        GPSjson["Ln"] = Longitude;
        GPSjson["Al"] = Altitude;
        GPSjson["Sp"] = Speed;
        GPSjson["Ry"] = RelayNum;
        GPSjson["St"] = Satellites;
        GPSjson["Td"] = TotalDistanceMeters;

        if (beatAvg > 0) {
          GPSjson["HB"] = beatAvg;
          GPSjson["TP"] = temperature;
          GPSjson["IR"] = irValue;
        }
      }

    } else {
      located = false;
    }


    if (gps.altitude.isValid())
      Altitude = gps.altitude.meters();

    if (gps.speed.isValid())
      Speed = gps.speed.kmph();

    if (gps.satellites.isValid())
      Satellites = gps.satellites.value();

    if (gps.time.isValid()) {
      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();

      local_hour = (hour + timeZone) % 24;
      sprintf(timeBuffer, "%02d:%02d:%02d", local_hour, minute, second);
    }

    if (gps.date.isValid()) {
      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();
    }



    if (takingHBR == 1) {
      vTaskDelay(pdMS_TO_TICKS(15000));

      // Serial.printf(" testing hbr \n");
    }


    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
//
void oledTask(void *pvParameters) {

  uint8_t LanguageFont = FTconfig.oledLanguage;  // if  peripherals attached ,msg langunge


  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(BEEPER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MENU_BTN), onFalling, FALLING);


  pinMode(PAGE_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PAGE_BTN), pageLoop, FALLING);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, 1);
  FastLED.setBrightness(BRIGHTNESS);


  u8g2.begin();
  char buffer[HKEY];
  char *row[HKEY];
  u8g2.enableUTF8Print();
  u8g2.setFontDirection(0);
  u8g2.setFontMode(0);
  u8g2.clearBuffer();
  //booting LOGO
  for (int x = 0; x < 128; x++) {
    float radians = (2 * PI * (127 - x)) / 128.0;
    int y = 32 + int(sin(radians) * 24);
    u8g2.drawPixel(x, y + 7);
    u8g2.drawPixel(x, y + 8);
  }

  if (LanguageFont == CHINESE || LanguageFont == JAPANESE || LanguageFont == KOREAN) {
    u8g2.setFont(u8g2_font_wqy16_t_gb2312);
    u8g2.setCursor(25, 45);
    u8g2.print("未");
    u8g2.setCursor(88, 45);
    u8g2.print("雨");

  } else {
    u8g2.setFont(u8g2_font_helvR24_te);
    u8g2.setCursor(22, 52);
    u8g2.print("F");
    u8g2.setCursor(84, 52);
    u8g2.print("T");
  }

  u8g2.sendBuffer();
  // Start off-screen to the right
  // Pixels per frame

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  int maxLen = 0;
  int pixelIdx = 0;
  int pixelTop = HKEY;
  //char row[HKEY];
  for (pixelIdx = 0; pixelIdx < pixelTop; pixelIdx++)
    row[pixelIdx] = (char *)heap_caps_malloc(PKT, MALLOC_CAP_SPIRAM);

  Serial.println("oled psram  successfully.");


  float minAlt = PathGpsList[0].altitude;
  float maxAlt = PathGpsList[0].altitude;
  int minLat = PathGpsList[0].latitude;
  int maxLat = PathGpsList[0].latitude;
  int minLon = PathGpsList[0].longitude;
  int maxLon = PathGpsList[0].longitude;
  float minSpeed = PathGpsList[0].speed;
  float maxSpeed = PathGpsList[0].speed;

  int x = 0;  // Map  to screen width
  int y = 0;  // Map to screen height


  int nextX = 0;
  int nextY = 0;

  while (true) {

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR14_te);



    switch (menuButton) {





      case OLED_PREVIEW:
        {

          u8g2.setPowerSave(0);
          switch (pageButton) {
            case 0:  //trip /speed/time
              {




                // Speed
                dtostrf(Speed, 7, 2, buffer);
                u8g2.setCursor(0, 60);
                u8g2.print("V:");
                u8g2.print(&buffer[strspn(buffer, " ")]);
                u8g2.print("km/h");


                if (TotalDistanceMeters > 99999) {
                  TotalDistanceMeters = TotalDistanceMeters / 1000;
                  itoa(TotalDistanceMeters, buffer, 10);
                  u8g2.setCursor(0, 40);
                  u8g2.print("Trip:");
                  u8g2.setCursor(42, 40);
                  buffer[5] = '\0';
                  u8g2.print(buffer);
                  u8g2.print("km");
                } else {
                  itoa(TotalDistanceMeters, buffer, 10);
                  u8g2.setCursor(0, 40);
                  u8g2.print("Trip:");
                  u8g2.setCursor(42, 40);
                  u8g2.print(buffer);
                  u8g2.print("m");
                }




                u8g2.setFont(u8g2_font_lubB19_te);
                u8g2.setCursor(0, 22);
                u8g2.print(timeBuffer);
              }
              break;
            case 1:  //altitude /lat & lng and date
              {

                // Altitude
                dtostrf(Altitude, 7, 2, buffer);
                u8g2.setCursor(0, 30);
                u8g2.print("Alt:");
                u8g2.print(&buffer[strspn(buffer, " ")]);
                u8g2.print("m");

                // Latitude
                dtostrf(Latitude, 11, 6, buffer);
                u8g2.setCursor(0, 46);
                u8g2.print("Lat:");
                u8g2.print(&buffer[strspn(buffer, " ")]);

                // Longitude
                dtostrf(Longitude, 11, 6, buffer);
                u8g2.setCursor(0, 62);
                u8g2.print("Lng:");
                u8g2.print(&buffer[strspn(buffer, " ")]);

                // Satellites
                itoa(Satellites, buffer, 8);
                u8g2.setCursor(75, 14);
                u8g2.print("Sat:");
                u8g2.print(buffer);
              }
              break;
            case 2:  //path
              {


                if (pathIndex > 15) {
                  u8g2.setCursor(0, HKEY);
                  u8g2.print("path map 0");

                  // ⬇️ Start drawing path using PathGpsList
                  for (int i = 0; i < pathIndex; i++) {
                    x = map(PathGpsList[i].latitude, minLat, maxLat, 0, 127);  // Map latitude to screen width
                    y = map(PathGpsList[i].longitude, minLon, maxLon, 0, 63);  // Map longitude to screen height

                    u8g2.drawCircle(x, y, 1, U8G2_DRAW_ALL);  // Draw small dot

                    if (i < (pathIndex - 1)) {
                      nextX = map(PathGpsList[i + 1].latitude, minLat, maxLat, 0, 127);
                      nextY = map(PathGpsList[i + 1].longitude, minLon, maxLon, 0, 63);
                      u8g2.drawLine(x, y, nextX, nextY);  // Connect to next dot
                    }
                  }
                } else {
                  u8g2.setCursor(0, HKEY);
                  u8g2.print("less than 15 point");
                }
              }
              break;
            case 3:  //dim
              {

                u8g2.setPowerSave(1);
              }
              break;
          }
        }
        break;


      case OLED_MSG:
        {
          u8g2.setPowerSave(0);
          switch (LanguageFont) {
            case LATIN:
              {  //latin
                u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
              }
              break;
            case CHINESE:
              {
                u8g2.setFont(u8g2_font_wqy16_t_gb2312b);
                //u8g2.setFont(noto_tc_font);
              }
              break;
            case CYRILLIC:
              {
                u8g2.setFont(u8g2_font_unifont_t_cyrillic);
              }
              break;
            case GREEK:
              {
                u8g2.setFont(u8g2_font_unifont_t_greek);
              }
              break;
            case HEBREW:
              {
                u8g2.setFont(u8g2_font_unifont_t_hebrew);
              }
              break;
            case JAPANESE:
              {
                u8g2.setFont(u8g2_font_b10_t_japanese1);
              }
              break;
            case KOREAN:
              {
                u8g2.setFont(u8g2_font_unifont_t_korean1);
              }
              break;
          }

          pixelIdx = pageButton * 4;
          maxLen = 0;
          //pageButton
          for (uint8_t oledMsgIdx = 0; oledMsgIdx < 4; oledMsgIdx++) {

            if (maxLen < u8g2.getUTF8Width(MsgLoop[pixelIdx + oledMsgIdx]))
              maxLen = u8g2.getUTF8Width(MsgLoop[pixelIdx + oledMsgIdx]);
          }
          u8g2.drawUTF8(-xPos, HKEY, (const char *)MsgLoop[pixelIdx]);
          u8g2.drawUTF8(-xPos, KEY, (const char *)MsgLoop[pixelIdx + 1]);
          u8g2.drawUTF8(-xPos, 48, (const char *)MsgLoop[pixelIdx + 2]);
          u8g2.drawUTF8(-xPos, TM, (const char *)MsgLoop[pixelIdx + 3]);

          if ((maxLen - xPos) > 100) {
            xPos += OCT * scrollSpeed;
          } else {
            xPos = 0;
          }
        }
        break;




      case OLED_PATH:
        {
          u8g2.setPowerSave(0);
          for (int i = 1; i < 31; i++) {
            if (PathGpsList[i].altitude < minAlt) minAlt = PathGpsList[i].altitude;
            if (PathGpsList[i].altitude > maxAlt) maxAlt = PathGpsList[i].altitude;

            if (PathGpsList[i].latitude < minLat) minLat = PathGpsList[i].latitude;
            if (PathGpsList[i].latitude > maxLat) maxLat = PathGpsList[i].latitude;

            if (PathGpsList[i].longitude < minLon) minLon = PathGpsList[i].longitude;
            if (PathGpsList[i].longitude > maxLon) maxLon = PathGpsList[i].longitude;

            if (PathGpsList[i].speed < minSpeed) minSpeed = PathGpsList[i].speed;
            if (PathGpsList[i].speed > maxSpeed) maxSpeed = PathGpsList[i].speed;
          }
          switch (pageButton) {
            case 0:  //latest 16 check point
              {


                if (pathIndex > 15) {

                  // ⬇️ Start drawing path using PathGpsList
                  for (int i = 0; i < 15; i++) {
                    x = map(PathGpsList[i].latitude, minLat, maxLat, 0, 127);  // Map latitude to screen width
                    y = map(PathGpsList[i].longitude, minLon, maxLon, 0, 63);  // Map longitude to screen height

                    //u8g2.drawCircle(x, y, 1, U8G2_DRAW_ALL);  // Draw small dot
                    u8g2.drawBox(x - 1, y - 1, 3, 3);  // Bold dot (3×3 square)
                    if (i < (15 - 1)) {
                      nextX = map(PathGpsList[i + 1].latitude, minLat, maxLat, 0, 127);
                      nextY = map(PathGpsList[i + 1].longitude, minLon, maxLon, 0, 63);
                      u8g2.drawLine(x, y, nextX, nextY);  // Connect to next dot
                    }
                  }
                } else {
                  u8g2.setCursor(0, HKEY);
                  u8g2.print("only  ");
                  u8g2.print(pathIndex);
                  u8g2.setCursor(0, KEY);
                  u8g2.print(" check point");
                }
              }
              break;
            case 1:  //latest 32 check point
              {


                if (pathIndex == 31) {
                  u8g2.setCursor(0, HKEY);

                  // ⬇️ Start drawing path using PathGpsList
                  for (int i = 0; i < 31; i++) {
                    x = map(PathGpsList[i].latitude, minLat, maxLat, 0, 127);  // Map latitude to screen width
                    y = map(PathGpsList[i].longitude, minLon, maxLon, 0, 63);  // Map longitude to screen height

                    u8g2.drawCircle(x, y, 1, U8G2_DRAW_ALL);  // Draw small dot

                    if (i < 30) {
                      nextX = map(PathGpsList[i + 1].latitude, minLat, maxLat, 0, 127);
                      nextY = map(PathGpsList[i + 1].longitude, minLon, maxLon, 0, 63);
                      u8g2.drawLine(x, y, nextX, nextY);  // Connect to next dot
                    }
                  }
                } else {
                  u8g2.setCursor(0, HKEY);
                  u8g2.print("only  ");
                  u8g2.print(pathIndex);
                  u8g2.setCursor(0, KEY);
                  u8g2.print(" check point");
                }
              }
              break;
            case 2:
              {
                u8g2.setFont(u8g2_font_helvR08_te);
                u8g2.setCursor(KEY, 8);
                u8g2.print("Altitude:m");

                // Compute min and max altitude
                float minAlt = PathGpsList[0].altitude;
                float maxAlt = PathGpsList[0].altitude;
                for (int i = 1; i < 31; i++) {
                  if (PathGpsList[i].altitude < minAlt) minAlt = PathGpsList[i].altitude;
                  if (PathGpsList[i].altitude > maxAlt) maxAlt = PathGpsList[i].altitude;
                }

                for (int i = 0; i < 31; i++) {
                  x = map(i, 0, 30, 0, 127);                                // X: evenly spaced across screen
                  y = map(PathGpsList[i].altitude, minAlt, maxAlt, 63, 0);  // Y: scaled altitude (inverted)

                  u8g2.drawBox(x - 1, y - 1, 3, 3);  // Bold dot (3×3 square)
                  if (PathGpsList[i].altitude == minAlt) {
                    u8g2.print(minAlt);
                  }
                  if (PathGpsList[i].altitude == maxAlt) {
                    u8g2.print(maxAlt);
                  }
                  if (i < 30) {
                    nextX = map(i + 1, 0, 30, 0, 127);
                    nextY = map(PathGpsList[i + 1].altitude, minAlt, maxAlt, 63, 0);
                    u8g2.drawLine(x, y, nextX, nextY);
                  }
                }
              }

              break;
            case 3:
              {
                u8g2.setFont(u8g2_font_helvR08_te);
                u8g2.setCursor(KEY, 8);
                u8g2.print("Speed:km/h");



                for (int i = 0; i < 31; i++) {
                  x = map(i, 0, 30, 0, 127);
                  y = map(PathGpsList[i].speed, minSpeed, maxSpeed, 63, 0);

                  u8g2.drawBox(x - 1, y - 1, 2, 2);  // Small dot (2×2 square)
                  if (PathGpsList[i].speed == minSpeed) {
                    u8g2.print(minSpeed);
                  }
                  if (PathGpsList[i].speed == maxSpeed) {
                    u8g2.print(maxSpeed);
                  }


                  if (i < 30) {
                    nextX = map(i + 1, 0, 30, 0, 127);
                    nextY = map(PathGpsList[i + 1].speed, minSpeed, maxSpeed, 63, 0);
                    u8g2.drawLine(x, y, nextX, nextY);
                  }
                }
              }

              break;
          }
        }
        break;

      case OLED_HBR:
        {
          switch (pageButton) {
            case 0:
              {
                u8g2.setPowerSave(0);
                u8g2.setCursor(0, 14);
                u8g2.print(timeBuffer);

                u8g2.setCursor(0, 30);
                u8g2.print("IR:");
                u8g2.setCursor(30, 30);
                u8g2.print(irValue);


                u8g2.setCursor(0, 62);
                u8g2.print("HBR:");
                u8g2.print(beatAvg);



                u8g2.setCursor(0, 46);
                u8g2.print("T° : ");
                u8g2.print(temperature);
                u8g2.print(" °C");
              }
              break;
            case 1:
              {

                u8g2.setCursor(0, HKEY);
                if (TurnOnWifi == false) {

                  u8g2.print("switch to Dock");
                } else {

                  u8g2.print("switch to watch");
                }


                u8g2.setCursor(0, 40);
                u8g2.print("  click twice ");
                //
                u8g2.setFont(u8g2_font_helvR18_te);
                u8g2.setCursor(0, 60);
                u8g2.print("require reboot  ");
              }
              break;
            case 2:
              {



                u8g2.setFont(u8g2_font_helvR18_te);

                u8g2.setCursor(0, 24);
                u8g2.print("reboot?");

                u8g2.setCursor(0, 50);
                u8g2.print("click again");
              }
              break;
            case 3:
              {
                u8g2.setFont(u8g2_font_t0_22_te);
                u8g2.setCursor(0, KEY);
                u8g2.print("rebooting");
                u8g2.sendBuffer(); 

                if (TurnOnWifi == false) {

                  FTconfig.Mode = 1;
                } else {

                  FTconfig.Mode = 0;
                }
                if (saveConfig()) {

                  ESP.restart();
                }
              }
              break;
          }
        }
        break;
    }


    u8g2.sendBuffer();

    if (EnterSOS == 0) {


      leds[0] = colorLevels[TrafficDensity];  // Assign raw 0xRRGGBB value
      FastLED.show();
      if (TrafficDensity > 0) TrafficDensity--;
    } else {
      leds[0] = CRGB(LedRed, LedGreen, LedBlue);  // Set LED color
      FastLED.show();
      TrafficDensity = 0;
    }

    if (takingHBR == 1)
      vTaskDelay(pdMS_TO_TICKS(800));
    else
      vTaskDelay(pdMS_TO_TICKS(400));
  }
}
//
