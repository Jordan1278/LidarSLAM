#include <M5Stack.h>
//#include <esp_now.h>
//#include "espnow.h"
#include "lidarcar.h"
#include "rprtrack.h"
#include "iic.h"
#include "AccessService.h"

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include "WiFi.h"  



I2C i2c;
//Espnow espnow;
LidarCar lidarcar;
AccessService service;

const char* ssid = "IP";
const char* password = "password"; 
extern const unsigned char gImage_logo[];

//groote van de JSON array 
//const size_t CAPACITY = JSON_ARRAY_SIZE(180);


//server 
WiFiServer wifiServer(30000);
WiFiClient client;

void Service(void * pvParameters) {

    for(;;) {
        service.Listen();
        vTaskDelay(2 / portTICK_RATE_MS); 
    }
    vTaskDelete(NULL);
}

void setup() {
  m5.begin();
  Serial1.begin(230400, SERIAL_8N1, 16, 2);  //Lidar
  Serial2.begin(115200);                     //motor
    
  // Connect to wifi
  WiFi.begin(ssid, password); //, password);
  Serial.println(WiFi.localIP());
  M5.Lcd.println(WiFi.localIP());
  while (WiFi.status() != WL_CONNECTED) {
    delay(10000);
    Serial.println("Connecting to WiFi..");
    Serial.println(WiFi.localIP());
    M5.Lcd.println(WiFi.localIP()); 
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  M5.Lcd.println(WiFi.localIP());

  // Start server
  wifiServer.begin();
  
  //!esp
  //espnow.BotInit();
  //esp_now_register_recv_cb(OnDataRecv);
  //esp_now_register_send_cb(OnDataSent);

  //!service
  service.Init();
  
  
  //!Motor
  lidarcar.Init();

  //!Camrea
  i2c.master_start(); // TODO: Enkel bij lidar

  //!Service
  
  xTaskCreatePinnedToCore(
                    Service,
                    "Service",
                    40960,
                    NULL, 
                    5,        
                    NULL,
                    0); 
   
}

//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  
//}
int flag = 0;
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
 // if(espnow.OnBotRecv(mac_addr,data,data_len)){
 //   return;
 // }
  
 if((data_len == 3) && (!flag)) {
    lidarcar.ControlWheel(data[0], data[1], data[2]);
 }

 if((data_len == 4) && (!flag)) {
    lidarcar.LedShow();
 }
  
}

void loop()
{
  //espnow.BotConnectUpdate();
  
  lidarcar.MapDisplay();

  //lidarcar.ControlMode();
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("Client connected");
    StaticJsonDocument<2800> doc;
    JsonArray array = doc.to<JsonArray>();
    for (int i=0; i<172; i++){
      array.add(lidarcar.mapdata[i]);
     } 
    char output[2800];  
    serializeJson(doc, output);
    client.write(output);
    client.write("\n");
    Serial.println("Data transmitted");
  } else {
    client = wifiServer.available();
  }
  
  
  if(digitalRead(37) == LOW){
   flag++;
   if(flag >= 4) flag = 0;
   while(digitalRead(37) == LOW);
  }
  
  if(flag == 0){ 
    i2c.master_hangs();
    //esp_now_send(espnow.peer_addr, lidarcar.mapdata, 180);
    //esp_err_t addStatus = esp_now_send(espnow.peer_addr, lidarcar.mapdata, 180);
    //if(addStatus != ESP_OK){
      //lidarcar.ControlWheel(0, 0, 0);
    //}
    M5.Lcd.setCursor(240, 0);    
    M5.Lcd.printf("Remote");
  }
  
  if(flag == 1) {
    i2c.master_hangs();
    lidarcar.CarMaze();
    M5.Lcd.setCursor(240, 0);    
    M5.Lcd.printf("Maze  ");
 }
                 
  if(flag == 3) {
    i2c.master_recovery();
    lidarcar.CarCamera();
    M5.Lcd.setCursor(240, 0);    
    M5.Lcd.printf("Camera  ");
  }

   if(flag == 2) {
    i2c.master_hangs();
    lidarcar.TrackControl();
    M5.Lcd.setCursor(240, 0);    
    M5.Lcd.printf("Track  ");
  }
                 
}