/* ======================================== Including the libraries. */
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include "esp_http_server.h"
/* ======================================== */

/* ======================================== Select camera model */
#define CAMERA_MODEL_AI_THINKER
/* ======================================== */
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


// LEDs GPIO
#define LED_OnBoard 4
#define LED_Green   12
#define LED_Blue    13


/* ======================================== Replace with your network credentials */
const char* ssid = "Iam Disguised";
const char* password = "gunasree";

/* ======================================== */

/* ======================================== */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
/* ======================================== */

/* ======================================== Empty handle to esp_http_server */
httpd_handle_t index_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
/* ======================================== */

/* ======================================== HTML code for index / main page */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>

  <head>
  
    <title>ESP32-CAM QR Code Reader Stream Web Server</title>
    
    <meta name="viewport" content="width=device-width, initial-scale=1">
    
    <style>

      /* ----------------------------------- Stream Viewer */
      img {
        width: auto ;
        max-width: 100% ;
        height: auto ; 
      }
      /* ----------------------------------- */
      
    </style>
    
  </head>
  
  <body>
  
    <h3>ESP32-CAM QR Code Reader Stream Web Server</h3>
    
    <img src="" id="vdstream">
    
    <br><br>
    
    <script>
      /* ----------------------------------- Calls the video stream link and displays it */ 
      window.onload = document.getElementById("vdstream").src = window.location.href.slice(0, -1) + ":81/stream";
      /* ----------------------------------- */
    </script>
  
  </body>
  
</html>
)rawliteral";
/* ======================================== */

/* ________________________________________________________________________________ Index handler function to be called during GET or uri request */
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ stream handler function to be called during GET or uri request. */
static esp_err_t stream_handler(httpd_req_t *req){
  Serial.print("stream_handler running on core ");
  Serial.println(xPortGetCoreID());
  
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  /* ---------------------------------------- Loop to show streaming video from ESP32 Cam camera and read QR Code. */
  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed (stream_handler)");
      res = ESP_FAIL;
    } else {
      if(fb->width > 200){
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
  }
  /* ---------------------------------------- */
  return res;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ cmd handler function to be called during GET or uri request. */
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
   
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  int res = 0;
  if(res){
    return httpd_resp_send_500(req);
  }
 
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}
/* ________________________________________________________________________________ */


/* ________________________________________________________________________________ Subroutine for starting the web server / startCameraServer. */
void startCameraWebServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
    
  if (httpd_start(&index_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(index_httpd, &index_uri);
      httpd_register_uri_handler(index_httpd, &cmd_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(stream_httpd, &stream_uri);
  }

  Serial.println();
  Serial.println("Camera Server started successfully");
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  Serial.println();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ VOID SETUP() */
void setup() {
  // put your setup code here, to run once:

  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* ---------------------------------------- Init serial communication speed (baud rate). */
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  /* ---------------------------------------- */

  pinMode(LED_OnBoard, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);

  /* ---------------------------------------- Camera configuration. */
  Serial.println("Start configuring and initializing the camera...");
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
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;
  
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  Serial.println("Configure and initialize the camera successfully.");
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Connect to Wi-Fi. */
  WiFi.mode(WIFI_STA);
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  /* ::::::::::::::::: The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router. */
  /*
   * The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
   * If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
   * I made this condition because on my ESP32-CAM, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.
   */
  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_OnBoard, HIGH);
    delay(250);
    digitalWrite(LED_OnBoard, LOW);
    delay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }
  digitalWrite(LED_OnBoard, LOW);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("------------");
  Serial.println("");
  /* ::::::::::::::::: */
  /* ---------------------------------------- */

  // Start camera web server
  startCameraWebServer(); 
  
}
/* ________________________________________________________________________________ */

void loop() {
  // put your main code here, to run repeatedly:
  
}
/* ________________________________________________________________________________ */