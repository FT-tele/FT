
#include <esp_heap_caps.h>

#include "esp_mac.h"
#include "WiFi.h"

#include "AES.h"
#include "Fourier.h"
#include "Phone.h"
#include "Sensor.h"
#include "VariableDefine.h"








void setup() {


  Serial.begin(115200);


  esp_efuse_mac_get_default(FavoriteMAC[0]);
  esp_read_mac(FavoriteMAC[0], ESP_MAC_WIFI_SOFTAP);
  initInfrast();

  for (int i = 0; i < 6; i++) Serial.printf("%d:%02X ", i, FavoriteMAC[0][i]);


  Serial.print("\n   setup   boot \n");
  //vTaskDelay(pdMS_TO_TICKS(3000));
  listenQueue = xQueueCreate(KEY, sizeof(uint8_t));
  loraQueue = xQueueCreate(KEY, sizeof(uint8_t));

  wsEventQueue = xQueueCreate(HKEY, sizeof(uint8_t));
 



  xTaskCreatePinnedToCore(electricTask, "electricTask", 4096, NULL, 8, &electricTaskHandle, CORE1);
  //vTaskDelay(5000 / portTICK_PERIOD_MS);




  xTaskCreatePinnedToCore(transformTask, "transformTask", 4096, NULL, 9, &transformTaskHandle, CORE1);
  //vTaskDelay(5000 / portTICK_PERIOD_MS); 


  Wire.begin(SDA_PIN, SCK_PIN);
  Wire.setClock(400000);

  xTaskCreatePinnedToCore(oledTask, "oledTask", 4096, NULL, 4, &oledTaskHandle, CORE1);
  //vTaskDelay(6000 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore(max3010xTask, "max3010xTask", 4096, NULL, 5, &max3010xTaskHandle, CORE0);
  //vTaskDelay(2000 / portTICK_PERIOD_MS);

  if (FTconfig.EnableGPS == 1) {

    xTaskCreatePinnedToCore(gpsTask, "gpsTask", 4096, NULL, 5, &gpsTaskHandle, CORE0);
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
  }






  loraInit();

  GenerateKeyPairs(myPrivate, myPublic);
  Serial.println("GenerateKeyPairs  successfully.");

  // Serial.printf(" FTconfig.WifiMode%d .", FTconfig.WifiMode);

  if (TurnOnWifi) {

    Serial.print("\n   start wifi      \n");
    wifiMode();

    Serial.print("\n    wifi    finish \n");
  }

  esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "15min_timer"
  };

  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 60000000);
  if (TurnOnWifi) {


    xTaskCreatePinnedToCore(httpdTask, "httpdTask", 4096, NULL, 4, &httpdTaskHandle, CORE0);

    xTaskCreatePinnedToCore(sessionCipherTask, "sessionCipherTask", 8192, NULL, 9, &sessionCipherTaskHandle, CORE1);


    xTaskCreatePinnedToCore(websocketTask, "websocketTask", 8192, NULL, 7, &websocketTaskHandle, CORE0);




  } else {



    //vTaskDelay(5000 / portTICK_PERIOD_MS);

    initI2S();




    xTaskCreatePinnedToCore(phoneTask, "phoneTask", 32000, NULL, 6, &phoneTaskHandle, CORE1);
    //vTaskDelay(5000 / portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(listenTask, "listenTask", 20480, NULL, 7, &listenTaskHandle, CORE1);
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
  }


  
}

void loop() {
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}
