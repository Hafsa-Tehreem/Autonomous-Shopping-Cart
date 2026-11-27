#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// Replace with your Wi-Fi credentials
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// AI-Thinker ESP32-CAM Pinout
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

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// Web interface HTML with live video and touch/mouse buttons
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>ESP32-CAM RC Car</title>
  <style>
    body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; margin: 0; padding: 10px; }
    img { width: 100%; max-width: 480px; border-radius: 8px; border: 2px solid #444; }
    .grid { display: grid; grid-template-columns: repeat(3, 85px); gap: 10px; justify-content: center; margin-top: 15px; }
    button { height: 70px; font-size: 20px; font-weight: bold; border: none; border-radius: 10px; background: #007bff; color: white; touch-action: manipulation; }
    button:active { background: #0056b3; }
    .stop { background: #dc3545; }
  </style>
</head>
<body>
  <h3>ESP32-CAM RC Car</h3>
  <img id="stream" src="">
  <div class="grid">
    <div></div>
    <button onmousedown="send('F')" onmouseup="send('S')" ontouchstart="send('F')" ontouchend="send('S')">▲</button>
    <div></div>
    <button onmousedown="send('L')" onmouseup="send('S')" ontouchstart="send('L')" ontouchend="send('S')">◀</button>
    <button class="stop" onclick="send('S')">■</button>
    <button onmousedown="send('R')" onmouseup="send('S')" ontouchstart="send('R')" ontouchend="send('S')">▶</button>
    <div></div>
    <button onmousedown="send('B')" onmouseup="send('S')" ontouchstart="send('B')" ontouchend="send('S')">▼</button>
    <div></div>
  </div>
  <script>
    window.onload = () => { document.getElementById("stream").src = window.location.protocol + "//" + window.location.hostname + ":81/stream"; };
    function send(cmd) { fetch("/cmd?go=" + cmd); }
  </script>
</body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char buf[32];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char param[8];
    if (httpd_query_key_value(buf, "go", param, sizeof(param)) == ESP_OK) {
      Serial.write(param[0]); // Sends F, B, L, R, or S directly to Arduino Uno
    }
  }
  httpd_resp_send(req, "OK", 2);
  return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part_buf, 64, "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
  httpd_uri_t cmd_uri   = { .uri = "/cmd", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }

  config.server_port = 81;
  config.ctrl_port += 1;
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(9600); // Must match Arduino Uno's Serial.begin(9600)

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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; // Low latency for smooth RC driving
  config.jpeg_quality = 12;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) return;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Note: Watch your router's DHCP list or temporarily set Serial to 115200 to view the IP
  startCameraServer();
}

void loop() {
  delay(10);
}
