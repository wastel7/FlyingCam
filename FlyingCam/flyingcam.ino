
// Use AI-Thinker ESP32-CAM Module as Board for ESP32CAM
// or XIAO_ESP32S3 for Seeedstudio esp32s3 sense
// Partition scheme: minimal spiffs (1.9mb with ota) 

/****************************
Für OTA on XIAO_ESP32S3 add this to your boards.txt:
XIAO_ESP32S3.menu.PartitionScheme.min_spiffs=Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
XIAO_ESP32S3.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
XIAO_ESP32S3.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080

Alternative: OTA-Update in Filesystem mode
*****************************/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ElegantOTA.h>

/******************************************
*  change acourding to your local network
******************************************/
const char* ssid = "homewifinetwork";
const char* password = "supersecretpassphrase";

#define AP_SSID "FlyingCam"
#define AP_PASS "kgodg5+-.TEM#98fghts" // change !!!

IPAddress ap_ip(192, 168, 168, 31);

/*****************************************/

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#define CLKRC               0x11
#define CLKRC_2X            0x80

IPAddress ap_mask(255, 255, 255, 0);
IPAddress ap_leaseStart(192, 168, 168, 2);
IPAddress ap_dns(8, 8, 4, 4);

const char* ota_login = "flyingcam";
const char* ota_password = "change this";

bool APmode = false;
bool ota_running = false;

const int connect_timeout = 20*2; // 20s
unsigned long previousMillis = 0;
unsigned long reconn_interval = 20000;

#ifdef CAMERA_MODEL_AI_THINKER
const int switchPin = 12;    // the number of the AP switch
#endif
#ifdef CAMERA_MODEL_XIAO_ESP32S3
const int switchPin = 8;    // the number of the AP switch
const int fanPin = 7;    // D8
const unsigned long fanIntervall = 120*1000;
const unsigned long fanRunningTime = 20*1000;
unsigned long lastFanRun = 0;
bool fanOn = false;
#endif
int switchState = HIGH;
int lastSwitchState = HIGH;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 750;    // the debounce time; increase if the output flickers

uint8_t otaDone = 0;
WebServer server(81);

void startCameraServer();

bool waitForWifiConnect()
{
  int conn_cnt = 1;
  while (WiFi.status() != WL_CONNECTED && conn_cnt < connect_timeout) 
  {
    delay(500);
    Serial.print(".");
    conn_cnt++;
  }
  
  if(WiFi.status() != WL_CONNECTED)
  {
      Serial.println("Couldn't connect to WiFi!!!");
      return false;
  }
  else
  {
    Serial.println("");
    Serial.println("WiFi connected");
    if(!WiFi.config(ap_ip))
        Serial.println("Failed to configure Static IP");
    else
        Serial.println("Static IP configured!");
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    return true;
  }
}

static esp_err_t changeXCLK(camera_config_t config) {
  //since the original setup doesnt create over 20MHz clock, we do it forcefully
  if (config.xclk_freq_hz <= 20 * 1000000) return ESP_OK;
  esp_err_t res = ESP_OK;
  // Deinitialize the existing LEDC configuration
  ledc_stop(LEDC_LOW_SPEED_MODE, config.ledc_channel, 0);
  delay(5);
  // Configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,
    .timer_num = config.ledc_timer,
    .freq_hz = config.xclk_freq_hz,
    .clk_cfg = LEDC_AUTO_CLK
  };
  res = ledc_timer_config(&ledc_timer);
  if (res != ESP_OK) {
    Serial.printf("Failed to configure timer %d", res);
    return res;
  }
  // Configure the LEDC channel
  ledc_channel_config_t ledc_channel = {
    .gpio_num = XCLK_GPIO_NUM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = config.ledc_channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = config.ledc_timer,
    .duty = 1,  // 50% duty cycle for 1-bit resolution
    .hpoint = 0
  };
  res = ledc_channel_config(&ledc_channel);
  if (res != ESP_OK) {
    Serial.printf("Failed to configure channel %d", res);
    return res;
  }
  delay(300); // base on datasheet, it needs < 300 ms for configuration to settle in. we just put 200ms. it doesnt hurt.
  return res;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(switchPin, INPUT_PULLUP);

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  config.frame_size = FRAMESIZE_SVGA; //FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_LATEST; //CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM; //CAMERA_FB_IN_PSRAM;
  config.fb_count = 1;

#ifdef CAMERA_MODEL_AI_THINKER
  config.jpeg_quality = 14;
  config.xclk_freq_hz = 16000000;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 3;
#else
  config.jpeg_quality = 14;
#endif  

#ifdef CAMERA_MODEL_XIAO_ESP32S3
  config.fb_count = 2;
  pinMode(fanPin, OUTPUT);
  lastFanRun = millis();
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  delay(300);

#ifdef CAMERA_MODEL_XIAO_ESP32S3
 // changeXCLK(config);
#endif

  sensor_t *s = esp_camera_sensor_get();

  if(s->id.PID == OV2640_PID)
  {
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_reg(s, CLKRC|0x100, CLKRC_2X, CLKRC_2X);  // Set cam CLK 2X
    s->set_res_raw(s, 1/*OV2640_MODE_SVGA*/, 0/*unused*/, 0/*unused*/, 0/*unused endY*/, 0, 0, 1600/2, 1200/2, 800, 480, false, false);
//    s->set_vflip(s, 1);
//    s->set_hmirror(s, 1);
    s->set_saturation(s, 1);
  }
  else if(s->id.PID == OV5640_PID)
  {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    // Pwr consumption: 250-260mA, wihtoutup to 280mA
    s->set_reg(s, 0x3031, 0xff, 0x08);// disable SC PWC (internal regulator)
    s->set_reg(s, 0x3035, 0xff, 0x21);// set clock divider
    s->set_reg(s, 0x3036, 0xff, 0x69);// set clock divider 

    s->set_res_raw(s, 0, 0, 2623, 1951, 32/2, 16/2, 2060, 1968/2, 800, 480, true, true);
    delay(300);

    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, 1);
  }
  else if(s->id.PID == OV3660_PID)
  {
      s->set_brightness(s, 1);   // up the brightness just a bit
      s->set_saturation(s, 1);  
      s->set_hmirror(s, 1);
      s->set_res_raw(s, 0, 0, 2079, 1547, 8, 2, 2300, 1564/2+1, 800, 480, true, true);
  }

  switchState = digitalRead(switchPin);
  lastSwitchState = switchState;
  if(switchState)
    wifi_connect_home();
  else
    wifi_start_ap();

  startCameraServer();

  Serial.print("Camera Ready ! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  ElegantOTA.begin(&server, ota_login, ota_password);    // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  server.begin();
  Serial.print("Web Server ready to get updates at http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81");
}

void onOTAStart() 
{
  Serial.println("OTA update started!");
  ota_running = true;
}

void wifi_connect_home()
{
    WiFi.AP.end();
    Serial.println("Reconnecting to WiFi...");
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    WiFi.reconnect();
    waitForWifiConnect();
    APmode = false;
}

void wifi_start_ap()
{
    WiFi.disconnect();
    WiFi.enableSTA(false);
    WiFi.setSleep(false);
    Serial.println("Starting AP...");
    WiFi.AP.begin();
    WiFi.AP.config(ap_ip, ap_ip, ap_mask, ap_leaseStart, ap_dns);
    WiFi.AP.create(AP_SSID, AP_PASS);
    if (!WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000)) 
    {
      Serial.println("Failed to start AP!");
      return;
    }
    Serial.println("AP started");
    APmode = true;
}

void switchEvent()
{
  int reading = digitalRead(switchPin);
  if (reading != lastSwitchState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != switchState)
    {
      switchState = reading;
      
      if (switchState == LOW)
      {
          wifi_start_ap();
          delay(1000);
      }
      else
      {
          wifi_connect_home();
          delay(1000);
      }
    }
  }
  lastSwitchState = reading;
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  if(!ota_running)
  {
      delay(250);
      unsigned long currentMillis = millis();

      switchEvent();

      if(!APmode)
      {
        if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=reconn_interval)) {
          Serial.print(millis());
          Serial.println("Reconnecting to WiFi...");
          WiFi.disconnect();
          WiFi.reconnect();
          waitForWifiConnect();
          previousMillis = currentMillis;
        }
      }
  }
#ifdef CAMERA_MODEL_XIAO_ESP32S3
  unsigned long currentMillis = millis();
  if((lastFanRun + fanIntervall) <= currentMillis)
  {
      if(fanOn)
      {
          if((lastFanRun + fanIntervall + fanRunningTime) < currentMillis)
          {
              lastFanRun = currentMillis;
              digitalWrite(fanPin, LOW);
              fanOn = false;
          }
      }
      else
      {
          digitalWrite(fanPin, HIGH);
          fanOn = true;
      }
  }
#endif

  server.handleClient();
  ElegantOTA.loop();
}
