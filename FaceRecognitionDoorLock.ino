#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

const char* ssid = "WIFI_NAME"; // diisi wifi_name
const char* password = "WIFI_PASSWORD"; // diisi wifi password

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// Select camera model
#define CAMERA_MODEL_AI_THINKER // camera model
#include "camera_pins.h"
#define Relay 2 // relay IO2

using namespace websockets;
WebsocketsServer socket_server;

static int AUTO_MODE = 0;
static int MANUAL_MODE = 1;
static int OPENED = 2;
static int CLOSED = 3;

int doorMode; // mode lock (AUTO_MODE / MANUAL MODE)
int controlMode; // solenoid lock state (OPENED / CLOSED)
boolean activateRelay; // relay state (true / false)
long prevMillis=0;
int interval = 5000; // interval auto closed untuk solenoid lock-nya

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

void app_facenet_main();
void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;


static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

void setup() { // awalnya solenoid lock dan relaynya dalam kondisi tertutup & MANUAL_MODE lock
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, LOW);
  controlMode = CLOSED;
  activateRelay = false;
  doorMode = MANUAL_MODE;

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
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82); // port web socket

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP()); // untuk mengoutput IP address yang akan diakses
  Serial.println("' to connect");
}

static esp_err_t index_handler(httpd_req_t *req) { // menampilkan interface HTML
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) // fungsi untuk capture wajah
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client) // mengirim semua wajah yang telah terdaftar
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
}

static esp_err_t delete_all_faces(WebsocketsClient &client) // menghapus semua wajah yang telah terdaftar
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg) // mengambil data dari interface HTML
{
  if (msg.data().substring(0, 8) == "capture:") { // kondisi menerima data "capture/mendaftarkan" wajah
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data().substring(0, 7) == "remove:") { // kondisi menerima data "remove" wajah
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all") { // kondisi menerima data "delete all" wajah
    delete_all_faces(client);
  }
  
  if (msg.data() == "auto_door_lock") { // kondisi menerima data "auto door lock" untuk mengubah state menjadi auto_door_lock
    g_state = START_RECOGNITION;
    
    doorMode = AUTO_MODE;
    activateRelay = false;
    controlMode = CLOSED;
    digitalWrite(Relay,LOW);
    
    client.send("AUTO DOOR MODE");
  }
  if (msg.data() == "manual_door_lock") { // kondisi menerima data "manual door lock" untuk mengubah state menjadi manual_door_lock
    g_state = START_STREAM;
    
    doorMode = MANUAL_MODE;
    activateRelay = false;
    controlMode = CLOSED;
    digitalWrite(Relay,LOW);
    
    client.send("MANUAL DOOR MODE");
  }
  if (msg.data() == "open_door" && activateRelay == false && doorMode == MANUAL_MODE && controlMode == CLOSED) { // kondisi menerima data "open door" dan akan membuka door lock apabila state adalah manual_door_lock dan lock dalam kondisi closed
    activateRelay = true;
    controlMode = OPENED;
    digitalWrite(Relay,HIGH);
    client.send("DOOR OPENED");
  }
  if(msg.data() == "close_door" && activateRelay == true && doorMode == MANUAL_MODE && controlMode == OPENED) { // kondisi menerima data "close door" dan akan membuka door lock apabila state adalah manual_door_lock dan lock dalam kondisi opened
    activateRelay = false;
    controlMode = CLOSED;
    digitalWrite(Relay,LOW);
    client.send("DOOR CLOSED");
  }
}

void loop() {
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  send_face_list(client);
  client.send("STREAMING");

  while (client.available()) {
    client.poll();

    fb = esp_camera_fb_get();

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes) // apabila wajah terdeteksi
      {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");
          }

          if (g_state == START_ENROLL) // kondisi untuk capture/mendaftarkan wajah
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0) && doorMode == AUTO_MODE && controlMode == CLOSED && activateRelay == false) // akan mengenali wajah apabila state adalah AUTO_MODE 
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f) // apabila wajah terkenali maka solenoid lock akan terbuka
            {
              controlMode = OPENED;
              activateRelay = true;
              digitalWrite(Relay,HIGH);
              prevMillis = millis();
              
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPENED RECOGNISED %s", f->id_name);
              client.send(recognised_message);
            }
            else // apabila wajah tidak terkenali maka solenoid lock tidak akan terbuka dan akan mengirim pesan "INTRUDER ALERT!"
            {
              client.send("INTRUDER ALERT!");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }
      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("PLEASE SHOW FACES");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;

    if(doorMode == AUTO_MODE && controlMode == OPENED && activateRelay == true && millis()-prevMillis > interval) { // apabila solenoid lock dalam kondisi tertutup dan state adalah AUTO_MODE dan sudah melewati 5 detik, maka solenoid lock akan otomatis tertutup
       controlMode == CLOSED;
       activateRelay = false;
       digitalWrite(Relay,LOW);
    }
  }
}
