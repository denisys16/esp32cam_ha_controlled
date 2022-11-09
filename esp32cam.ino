/*********
  The code based on Rui Santos sample project.
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  Camera Module communicates with Home Assistant via “home-assistant-integration” library of Dawid Chyrzynski. 
  Now Home Assistant can control:
  - CPU Core temperature;
  - Flash LED Control;
  - Internal board red LED control;
  - Restart board.

  Before compile the sketch you need to create header file "esp32cam_config.h" with imortant defines.
  For example:
    #define WIFI_SSID             "SSID"  
    #define WIFI_PASSWORD         "PASSWORD"
    #define MQTT_SERVER           "192.168.1.100"
    #define MQTT_USER             "mqtt_user"
    #define MQTT_PASSWORD         "mqtt_password"
    #define CPU_TEMP_INTERVAL_S   3
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include <ArduinoHA.h>
#include "esp32cam_config.h"

//Replace with your network credentials in esp32cam_config.h
#ifndef WIFI_SSID
  #define WIFI_SSID             "TEST_SSID"  
#endif
#ifndef WIFI_PASSWORD
  #define WIFI_PASSWORD         "TEST_PASSWORD"
#endif 
#ifndef MQTT_SERVER
  #define MQTT_SERVER           "192.168.1.100"
#endif
#ifndef MQTT_USER
  #define MQTT_USER             "mqtt_user"
#endif
#ifndef MQTT_PASSWORD
  #define MQTT_PASSWORD         "mqtt_password"
#endif
#ifndef CPU_TEMP_INTERVAL_S
  #define CPU_TEMP_INTERVAL_S   5
#endif


#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
   
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22

  #define FLASH_LIGHT_GPIO_NUM    4
  #define RED_LED_GPIO_NUM        33
#else
  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

char *deviceUniqueID;
WiFiClient ha_wifi_client;
HADevice *pHADevice;
HAMqtt *pHAMqtt;
HASensorNumber *pCPUTempSensor;
HASwitch *pFlashLight;
HASwitch *pRedLED;
HASensor *pStreamURL;
HAButton *pResetButton;

void onFlashLightSwitchCommand(bool state, HASwitch* sender)
{
    digitalWrite(FLASH_LIGHT_GPIO_NUM, (state ? HIGH : LOW));
    sender->setState(state); // report state back to the Home Assistant
}

void onRedLedSwitchCommand(bool state, HASwitch* sender)
{
    digitalWrite(RED_LED_GPIO_NUM, (state ? LOW : HIGH));
    sender->setState(state); // report state back to the Home Assistant
}

void onResetCommand(HAButton* sender)
{
    if (sender == pResetButton) {
      ESP.restart();
    }
}

String strVideoStreamURL;
void onMqttConnected() {
  pStreamURL->setValue(strVideoStreamURL.c_str());
}

char *createUniqueID(const char *name, const byte *mac)
{
  int len = 12+strlen(name)+2; // Additional chars '_' and EOL
  char *id = new char[len];
  snprintf(id, len, "%s_%02x%02x%02x%02x%02x%02x", name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);  
  return id;
}

char *createSensorUniqueID(const char *sensor_name, const char *device_name)
{
  int len = strlen(sensor_name) + strlen(device_name) + 2; // Additional chars '_' and EOL
  char *id = new char[len];
  snprintf(id, len, "%s_%s", device_name, sensor_name);  
  return id;
}

void setup_HADevice() {

  byte wifi_mac[6];                     // the MAC address of your Wifi shield
  WiFi.macAddress(wifi_mac);

  // Initilize LED
  pinMode(RED_LED_GPIO_NUM, OUTPUT);
  digitalWrite(RED_LED_GPIO_NUM, HIGH);
  // Initilize Flash Light
  pinMode(FLASH_LIGHT_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_LIGHT_GPIO_NUM, LOW);

  
  
  // set HA device's details (optional)
  deviceUniqueID = createUniqueID("esp32cam", wifi_mac);
  pHADevice = new HADevice(deviceUniqueID);
  //--pHADevice.setUniqueId(wifi_mac, sizeof(wifi_mac));
  pHADevice->setName(createSensorUniqueID("AI_THINKER Module", deviceUniqueID));
  pHADevice->setSoftwareVersion("1.0.0");
  pHADevice->setManufacturer("Custom code for AI_THINKER Camera");
  pHADevice->setModel("Esp32Cam AI_THINKER");  
  pHADevice->enableSharedAvailability();
  pHADevice->enableLastWill();

  pHAMqtt = new HAMqtt(ha_wifi_client, *pHADevice);
  pHAMqtt->setDataPrefix("ESP32Cameras");

  // configure HA sensors
  pCPUTempSensor = new HASensorNumber(createSensorUniqueID("cpu_temperature", deviceUniqueID), HASensorNumber::PrecisionP1);
  pCPUTempSensor->setDeviceClass("temperature");
  pCPUTempSensor->setName(createSensorUniqueID("CPU Temperature", deviceUniqueID));
  pCPUTempSensor->setUnitOfMeasurement("°C");

  // configure HA switches
  //
  pFlashLight = new HASwitch(createSensorUniqueID("flash_light", deviceUniqueID));
  pFlashLight->setIcon("mdi:flash");
  pFlashLight->setName(createSensorUniqueID("Camera Flash Light", deviceUniqueID));
  pFlashLight->onCommand(onFlashLightSwitchCommand);
  //
  pRedLED = new HASwitch(createSensorUniqueID("red_led", deviceUniqueID));
  pRedLED->setIcon("mdi:led-on");
  pRedLED->setName(createSensorUniqueID("Camera Red LED", deviceUniqueID));
  pRedLED->onCommand(onRedLedSwitchCommand);

 
  pStreamURL = new HASensor(createSensorUniqueID("stream_url", deviceUniqueID));
  pStreamURL->setIcon("mdi:lan");
  pStreamURL->setName(createSensorUniqueID("Video Stream URL", deviceUniqueID));

  pResetButton = new HAButton(createSensorUniqueID("reset", deviceUniqueID));  
  pResetButton->setIcon("mdi:restart");
  pResetButton->setName(createSensorUniqueID("Reset Camera Module", deviceUniqueID));
  pResetButton->onCommand(onResetCommand);

  // MQTT broker connection (use your data here)
  pHAMqtt->onConnected(onMqttConnected);
  pHAMqtt->begin(MQTT_SERVER, MQTT_USER, MQTT_PASSWORD);
}

#ifdef __cplusplus
  extern "C" {
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
  }
#endif

void updateCPUTemperature()
{
  // Convert raw esp32 chip temperature in F to Celsius degrees
    float cpu_temperature = (temprature_sens_read() - 32) / 1.8;
    pCPUTempSensor->setValue(cpu_temperature);
}

void loop_HADevice() {
  static unsigned long lastUpdateAt = 0;

  pHAMqtt->loop();

  if ((millis() - lastUpdateAt) > (CPU_TEMP_INTERVAL_S*1000)) { // N * 1000ms debounce time
    updateCPUTemperature();
    lastUpdateAt = millis();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();

  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 3);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 2);       // -2 to 2
  s->set_aec_value(s, 400);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 30);      // 0 to 30
  s->set_gainceiling(s, GAINCEILING_128X);  // GAINCEILING_2X to GAINCEILING_128X
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  // Wi-Fi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  strVideoStreamURL = "http://" + WiFi.localIP().toString();
  Serial.println("");
  Serial.println("WiFi connected");
  
  Serial.println("Camera Stream Ready!");
  Serial.println("Go to: " + strVideoStreamURL);
  Serial.println("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  setup_HADevice();

  // Start streaming web server
  startCameraServer();
}


void loop() {
  loop_HADevice();
}
