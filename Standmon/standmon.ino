// Use "esp32s3 dev module" as Board
// Flash: 16mb Psram: OPI PSRAM

#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <ESP32_JPEG_Library.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <lwip/sockets.h>
#include "esp_wifi.h"

/******************************************
*  change acourding to your local network
******************************************/
const char* ssid = "homewifinetwork";
const char* password = "supersecretpassphrase";

const char* ssid_ap = "FlyingCam";
const char* password_ap = "kgodg5+-.TEM#98fghts"; // change !!!

const String cam_ip = "192.168.168.31";
/*****************************************/

#define TFT_BL 2
#define DIRECT_MODE // Uncomment to enable full frame buffer
//#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */,
    0 /* hsync_polarity */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 12 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800 /* width */, 480 /* height */, bus, 0 /* rotation */, true /* auto_flush */);

#include "touch.h"

LV_IMG_DECLARE(iconHome_200x200);
LV_IMG_DECLARE(iconAP_200x200);

/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;
lv_obj_t *status_line;
extern bool http_started;

typedef enum
{
    STATE_WIFI_HOME_CAM_CONNECTING,
    STATE_WIFI_HOME_CAM_GETTING,
    STATE_WIFI_HOME_CAM_SHOWING,
    STATE_WIFI_AP_CAM_CONNECTING,
    STATE_WIFI_AP_CAM_GETTING,
    STATE_WIFI_AP_CAM_SHOWING,
}T_STATE;

T_STATE current_state = STATE_WIFI_HOME_CAM_CONNECTING;

const int switchPin = 17;    // the number of the AP switch
int switchState = HIGH;
int lastSwitchState = HIGH;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 750;    // the debounce time; increase if the output flickers

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
#ifndef DIRECT_MODE
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);

  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
#endif // #ifndef DIRECT_MODE

  /*Call it to tell LVGL you are ready*/
  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print( "Data x " );
      Serial.println( data->point.x );
      Serial.print( "Data y " );
      Serial.println( data->point.y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void)
{
    return millis();
}

void setup()
{
  Serial.begin(115200);
  // while (!Serial);
  Serial.println("Flyingcam Standmon");
  cam_view_setup();

  // Init Display
  gfx->begin();
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); 
#endif
  pinMode(switchPin, INPUT);
  switchState = digitalRead(switchPin);
  lastSwitchState = switchState;

  lv_init();
 /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick);

  // Init touch device
  pinMode(TOUCH_GT911_RST, OUTPUT);
  digitalWrite(TOUCH_GT911_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_GT911_RST, HIGH);
  delay(10);
  touch_init();
//  touch.setTouch( calData );

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef DIRECT_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 40;
#endif

#ifdef ESP32
#if defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL))
  disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
#else  // !(defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL)))
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf)
  {
    // remove MALLOC_CAP_INTERNAL flag try again
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  }
#endif // !(defined(DIRECT_MODE) && (defined(CANVAS) || defined(RGB_PANEL)))
#else // !ESP32
  Serial.println("LVGL disp_draw_buf heap_caps_malloc failed! malloc again...");
  disp_draw_buf = (lv_color_t *)malloc(bufSize * 2);
#endif // !ESP32

//  disp_draw_buf = (lv_color_t *)gfx->getFramebuffer();
  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
#ifdef DIRECT_MODE
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    /*Initialize the (dummy) input device driver*/
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);
  }

  create_image_button(&iconHome_200x200, 1, btn_home_cb);
  create_image_button(&iconAP_200x200, 2, btn_ap_cb);

  status_line = lv_textarea_create(lv_screen_active());
  lv_textarea_set_one_line(status_line, true);
  lv_obj_set_size(status_line, 350, 45);
  lv_obj_align(status_line, LV_ALIGN_BOTTOM_MID, 0, -47); //140px unten frei

  if(switchState)
    wifi_connect_home();
  else
    wifi_start_ap();
}

void wifi_connect_home()
{
    Serial.println("btn_home_cb clicked");
    if(current_state != STATE_WIFI_HOME_CAM_GETTING/* &&
       current_state != STATE_WIFI_HOME_CAM_SHOWING &&
       current_state != STATE_WIFI_AP_CAM_SHOWING*/)
    {
        lv_textarea_set_text(status_line, "Connecting to home WiFi...");
        current_state = STATE_WIFI_HOME_CAM_CONNECTING;
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);  
    }
}

void wifi_start_ap()
{
    Serial.println("btn_ap_cb clicked");
    if(current_state != STATE_WIFI_AP_CAM_GETTING/* &&
       current_state != STATE_WIFI_AP_CAM_SHOWING &&
       current_state != STATE_WIFI_HOME_CAM_SHOWING*/)
    {
        lv_textarea_set_text(status_line, "Connecting to FlyingCam WiFi...");
        current_state = STATE_WIFI_AP_CAM_CONNECTING;
        WiFi.disconnect();
        WiFi.begin(ssid_ap, password_ap);
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
}

void btn_home_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED)
  {
      wifi_connect_home();
  }
}

void btn_ap_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if(code == LV_EVENT_CLICKED)
  {
      wifi_start_ap();
  }
}

void create_image_button(const void *img, int pos, lv_event_cb_t event_cb)
{
    /*Create a transition animation on width transformation and recolor.*/
    static lv_style_prop_t tr_prop[] = {LV_STYLE_TRANSFORM_WIDTH, LV_STYLE_IMAGE_RECOLOR_OPA, 0};
    static lv_style_transition_dsc_t tr;
    lv_style_transition_dsc_init(&tr, tr_prop, lv_anim_path_linear, 200, 0, NULL);

    static lv_style_t style_def;
    lv_style_init(&style_def);
    lv_style_set_text_color(&style_def, lv_color_white());
    lv_style_set_transition(&style_def, &tr);

    /*Darken the button when pressed and make it wider*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);
    lv_style_set_image_recolor_opa(&style_pr, LV_OPA_30);
    lv_style_set_image_recolor(&style_pr, lv_color_black());
//    lv_style_set_transform_width(&style_pr, -20);

    /*Create an image button*/
    lv_obj_t * imagebutton1 = lv_imagebutton_create(lv_screen_active());
    lv_imagebutton_set_src(imagebutton1, LV_IMAGEBUTTON_STATE_RELEASED, nullptr, img,
                           nullptr);
    lv_obj_add_style(imagebutton1, &style_def, 0);
    lv_obj_add_style(imagebutton1, &style_pr, LV_STATE_PRESSED);

    //lv_obj_set_width(imagebutton1, 160);
    int32_t x = (800-2*200)/3*pos + (pos-1)*200;
    lv_obj_align(imagebutton1, LV_ALIGN_LEFT_MID, x, 0);

    lv_obj_add_event_cb(imagebutton1, event_cb, LV_EVENT_ALL, NULL);
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
          delay(100);
      }
      else
      {
          wifi_connect_home();
          delay(100);
      }
    }
  }
  lastSwitchState = reading;
}

void loop()
{
  if(current_state != STATE_WIFI_HOME_CAM_SHOWING && 
     current_state != STATE_WIFI_AP_CAM_SHOWING)
  {
#ifdef DIRECT_MODE
#if defined(CANVAS) || defined(RGB_PANEL)
    gfx->flush();
#else // !(defined(CANVAS) || defined(RGB_PANEL))
    gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif // !(defined(CANVAS) || defined(RGB_PANEL))
#else  // !DIRECT_MODE
#ifdef CANVAS
    gfx->flush();
#endif
#endif // !DIRECT_MODE
    delay(5);
    lv_timer_handler(); /* let the GUI do its work */

  }
  else
  {
      delay(50);
  }
  switchEvent();
  switch(current_state)
  {
      case STATE_WIFI_HOME_CAM_CONNECTING:
        if(WiFi.status() == WL_CONNECTED)
        {
          current_state = STATE_WIFI_HOME_CAM_GETTING;
          lv_textarea_set_text(status_line, "Home WiFi connected, fetching image...");
          start_tasks();
        }
        break;
      case STATE_WIFI_HOME_CAM_GETTING:
        if(http_started)
          current_state = STATE_WIFI_HOME_CAM_SHOWING;
        break;
      case STATE_WIFI_HOME_CAM_SHOWING:
        break;

      case STATE_WIFI_AP_CAM_CONNECTING:
        if(WiFi.status() == WL_CONNECTED)
        {
          current_state = STATE_WIFI_HOME_CAM_GETTING;
          lv_textarea_set_text(status_line, "FlyingCam WiFi connected, fetching image...");
          start_tasks();
        }
        break;
      case STATE_WIFI_AP_CAM_GETTING:
        if(http_started)
          current_state = STATE_WIFI_AP_CAM_SHOWING;
        break;
      case STATE_WIFI_AP_CAM_SHOWING:
        break;
      default:
        break;
  }
}
