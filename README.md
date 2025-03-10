![FlyingCam Logo](http://www.sebastian-duell.de/img/flyingcam/logo.png)

# FlyingCam

Ever wanted to watch something simple on a webcam like when the soup is cooking or when the bathtub is full, but you didn't want to fiddle around with webcam-positioning, power supply's, cables or smartphone-apps?
Then FlyingCam is made for You!!
FlyingCam is using a Ikea NÄVLINGE(TM) for it's flexible arm so the camera can easily point in every direction. With the attached USB-Power Bank it is highly portable because it needs no extra power supply. The parts used are easy to get or 3D printed, the hole system is build around ESP32 which is used also for the display.
For the camera the well known ESP32-CAM or the Seeed Studio XIAO ESP32S3 Sense(recommended) can be used. The display uses the ESP32-8048S070 which is a 7" TFT with integrated ESP32-S3.
FlyingCam can be used standalone with the build-in wifi AP or in a local wifi network.

![FlyingCam](http://www.sebastian-duell.de/img/flyingcam/FlyingCam.jpg)

## Building the Software

Just download the repo and open FlyingCam or Standmon in the latest Arduino-IDE and upload it to the board.
You've to configure your WiFi-credentials and the ip-address according to your local network in flyingcam.ino and standmon.ino. You also have to select the correct esp camera-model in flyingcam.ino.
Support boards are the common ESP32-CAM (CAMERA_MODEL_AI_THINKER) and the Seeed Studio XIAO ESP32S3 Sense (CAMERA_MODEL_XIAO_ESP32S3).

### Libraries/Boards needed
#### General

* arduino-esp32

#### FlyingCam

* ElegantOTA

#### Standmon

* lvgl 9.2.2
* Arduino_GFX Library
* ESP32_JPEG_Library ([https://github.com/esp-arduino-libs/ESP32_JPEG](https://github.com/esp-arduino-libs/ESP32_JPEG))

## 3D-parts

The 3D-parts can be found here: [https://www.printables.com/model/1224319-flyingcam](https://www.printables.com/model/1224319-flyingcam)




More Info: [http://www.sebastian-duell.de/en/flyingcam/index.html](http://www.sebastian-duell.de/en/flyingcam/index.html)
