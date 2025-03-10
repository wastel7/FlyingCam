#include <WiFi.h>
#include <HTTPClient.h>
#include <lwip/sockets.h>
#include <ESP32_JPEG_Library.h>
#include "esp_wifi.h"
#include "FastString.h"

const uint16_t cam_port_stream = 80;
const String cam_url_stream = "/stream";
const uint16_t cam_port_config = 80;

const unsigned long timeout = 30000; // 30 seconds
const int connect_timeout = 20*2; // 20s

bool http_started = false;
bool tasks_started = false;

struct bufSlot
{
    uint8_t *picBuffer;
    int len;
    int offset;
    int align_offset;
    bool free;
};

const int picBufferSize = 62*1024;//70*1024;

#define BUF_COUNT 2
bufSlot buffers[BUF_COUNT];

#define CHUNK_SIZE 2000

static TaskHandle_t getter_task_handle = NULL;
static TaskHandle_t tft_task_handle = NULL;

bool readImage2(HTTPClient &http, uint32_t slotnum);
void tftTask(void* pParam);

void cam_view_setup() 
{
    int i;
    for(i=0;i<BUF_COUNT;i++)
    {
      buffers[i].picBuffer = (uint8_t*)heap_caps_malloc(picBufferSize+16, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT); //MALLOC_CAP_DMA);
      buffers[i].len = 0;
      buffers[i].offset = 0;
      buffers[i].align_offset = 0;
      buffers[i].free = true;
    }
}

void start_tasks()
{
    if(!tasks_started)
    {
      tasks_started = true;
      xTaskCreatePinnedToCore(getterTask, "getterTask", 10000, NULL, 4, &getter_task_handle,  0);
      xTaskCreatePinnedToCore(tftTask, "tftTask", 10000, NULL, 2, &tft_task_handle,  1);
    }
}

uint32_t jpgTimeEnd=0;
uint32_t getTime=0;
uint32_t jpgTime=0;

void getterTask(void* pParam)
{
    bool first = true;

    uint32_t slotnum = 0;

    int fps;
    uint32_t fpsTime = millis();

    HTTPClient http;
    while(1)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            WiFi.reconnect();
            int conn_cnt = 1;
            while(WiFi.status() != WL_CONNECTED && conn_cnt < connect_timeout) 
            {
              vTaskDelay(500 / portTICK_RATE_MS); //portTICK_PERIOD_MS
              conn_cnt++;
            }
        }
        else
        {
            Serial.println("OK");
            getTime = millis();
            if(!http_started)
            {
                http.begin(cam_ip, cam_port_stream, cam_url_stream);
                int httpCode = http.GET();
                if (httpCode <= 0)
                {
                    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
                    http_started = false;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                else
                {
                    if (httpCode != HTTP_CODE_OK)
                    {
                        Serial.printf("[HTTP] Not OK!\n");
                        http_started = false;
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    else
                    {
                        http_started = true;
                    }
                }
            }
            if(!http_started || !http.connected())
            {
                http.end();
                http_started = false;
            }
            else
            {
                if(!readImage(http, slotnum))
                {
                    Serial.println("readImage failed");
                    http.end();
                    http_started = false;
                    continue;
                }
            }

            uint32_t httpTimeEnd = millis();
            Serial.printf("getTime:%u httpTimeEnd:%u httptime:%u\r\n",getTime, httpTimeEnd, (httpTimeEnd-getTime));
          
            if(httpTimeEnd-fpsTime >=1000)
            {
                Serial.printf("\n\nHTTP FPS:%d\n\n", fps);
                fps = 0;
                fpsTime = millis();
            }
            else
            {
                fps++;
            }
            bool free = false;
            bool wait = false;
            while(!free)
            {
              slotnum++;
              if(slotnum >= BUF_COUNT)
              {
                  slotnum = 0;
                  if(wait)
                  {
                      vTaskDelay(pdMS_TO_TICKS(10));
                      wait = false;
                  }
                  else
                    wait = true;
              }
              if(buffers[slotnum].free)
                free = true;
            }
        }
    }
}

bool readImage(HTTPClient &http, uint32_t slotnum)
{
    int _size = -1;
    int idxJpegStart = -1;
    bool ret = false;

    WiFiClient * stream = http.getStreamPtr();
    stream->setTimeout(1000);

    while(_size == -1 && http.connected())
    {
        int s = stream->readBytes(buffers[slotnum].picBuffer+buffers[slotnum].align_offset, CHUNK_SIZE);
        if(!s)
          return false;

        FastString header((char*)buffers[slotnum].picBuffer+buffers[slotnum].align_offset, CHUNK_SIZE, false);
        int idx = header.indexOf("Content-Length");
        int idxEnd = header.indexOf('\n', idx);
        if(idx != -1 && idxEnd != -1)
        {
            String contentLen = header.substring(idx+strlen("Content-Length: "), idxEnd-1);
            if(contentLen.length())
            {
                _size = contentLen.toInt();
          Serial.printf("[HTTP] Content-Length: %d\n", _size);
                idxJpegStart = idxEnd+2;
                for(int i=0;i<4;i++)
                  idxJpegStart = header.indexOf('\n', idxJpegStart+2);
                idxJpegStart++;  
                buffers[slotnum].offset = idxJpegStart;
            }
        }
    }
    if(!http.connected())
      return ret;

    // read all data from server
    uint8_t* p = buffers[slotnum].picBuffer+buffers[slotnum].align_offset+CHUNK_SIZE;
    buffers[slotnum].len = _size;
    int l = _size-(CHUNK_SIZE-idxJpegStart);
    if(l+CHUNK_SIZE > picBufferSize)
    {
        Serial.println("ERROR: image size too big");
        while(l && http.connected())
        {
            int readlen = (l>picBufferSize)?picBufferSize:l;
            int s = stream->readBytes(buffers[slotnum].picBuffer, readlen);
            if(s > 0)
              l -= s;
        }
        return true;
    }
    int s = stream->readBytes(p, l);
    if(!http.connected())
        return ret;

    buffers[slotnum].free = false;
    ret = true;

  xTaskNotify(tft_task_handle, slotnum, eSetValueWithoutOverwrite);
  return ret;
}

jpeg_error_t jpg_decoder_one_image(uint8_t *input_buf, int len, uint8_t *output_buf)
{
    jpeg_error_t ret = JPEG_ERR_OK;
    int inbuf_consumed = 0;

    // Generate default configuration
    jpeg_dec_config_t config = {
        .output_type = JPEG_RAW_TYPE_RGB565_LE,
        .rotate = JPEG_ROTATE_0D,
    };

    // Empty handle to jpeg_decoder
    jpeg_dec_handle_t *jpeg_dec = NULL;

    // Create jpeg_dec
    jpeg_dec = jpeg_dec_open(&config);

    // Create io_callback handle
    jpeg_dec_io_t *jpeg_io = (jpeg_dec_io_t *)calloc(1, sizeof(jpeg_dec_io_t));
    if (jpeg_io == NULL) {
        return JPEG_ERR_MEM;
    }

    // Create out_info handle
    jpeg_dec_header_info_t *out_info = (jpeg_dec_header_info_t *)calloc(1, sizeof(jpeg_dec_header_info_t));
    if (out_info == NULL) {
        return JPEG_ERR_MEM;
    }
    // Set input buffer and buffer len to io_callback
    jpeg_io->inbuf = input_buf;
    jpeg_io->inbuf_len = len;

    // Parse jpeg picture header and get picture for user and decoder
    ret = jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info);
    if (ret < 0) {
        Serial.println("JPEG decode parse failed");
        goto _exit;
    }

    jpeg_io->outbuf = output_buf;
    inbuf_consumed = jpeg_io->inbuf_len - jpeg_io->inbuf_remain;
    jpeg_io->inbuf = input_buf + inbuf_consumed;
    jpeg_io->inbuf_len = jpeg_io->inbuf_remain;

    // Start decode jpeg raw data
    ret = jpeg_dec_process(jpeg_dec, jpeg_io);
    if (ret < 0) {
        Serial.println("JPEG decode process failed");
        goto _exit;
    }

_exit:
    // Decoder deinitialize
    jpeg_dec_close(jpeg_dec);
    free(out_info);
    free(jpeg_io);
    return ret;
}

void tftTask(void* pParam)
{
    uint32_t slotnum = 0;
    BaseType_t xResult;
    uint32_t jpgStartTime = 0;
    uint32_t frameCnt = 0;

    uint8_t *image_jpeg;
    uint8_t *buf = (uint8_t*)gfx->getFramebuffer();

    while(1)
    {
        xResult = xTaskNotifyWait(0X00, 0x00, &slotnum,  pdMS_TO_TICKS(10000));
        if(xResult == pdPASS)
        {
            if(slotnum < BUF_COUNT && !buffers[slotnum].free)
            {
                if(jpgStartTime == 0)
                  jpgStartTime = millis();
                Serial.printf("tftTask slotnum:%u", slotnum);
                jpgTime = millis();

                jpeg_error_t ret = JPEG_ERR_OK;

                uint8_t *image_jpeg_ptr = buffers[slotnum].picBuffer+buffers[slotnum].offset+buffers[slotnum].align_offset;
                if(((uint32_t)image_jpeg_ptr)%16)
                {
                  buffers[slotnum].align_offset = ((uint32_t)image_jpeg_ptr)%16;
                  uint32_t off = ((uint32_t)buffers[slotnum].picBuffer)%16;

                  memmove(buffers[slotnum].picBuffer + off, image_jpeg_ptr, buffers[slotnum].len);
                  image_jpeg_ptr = buffers[slotnum].picBuffer + off;
                }
                
                ret = jpg_decoder_one_image(image_jpeg_ptr, buffers[slotnum].len, buf);//(uint8_t*)(gfx->getFramebuffer()));
                if (ret != JPEG_ERR_OK) {
                    Serial.printf("JPEG decode failed - %d\n", (int)ret);
                    buffers[slotnum].free = true;
                    continue;
                }
                buffers[slotnum].free = true;

                frameCnt++;
                jpgTimeEnd = millis();
                Serial.printf("jpgTime:%u jpgTimeEnd:%u drawtime:%u\r\n",jpgTime,jpgTimeEnd, (jpgTimeEnd-jpgTime));
                uint32_t dispTime = jpgTimeEnd-jpgStartTime;
                if(dispTime > 1000)
                  Serial.printf("jpeg fps:%u\r\n",frameCnt/(dispTime/1000));
            }
            //xTaskNotify(getter_task_handle, slotnum, eSetValueWithoutOverwrite);
        }
    }
}
