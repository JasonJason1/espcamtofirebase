#include <WiFi.h>
#include "esp_camera.h"
#include <Arduino.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>
#include "NTPClient.h"
#include <WifiUDP.h>

//Replace with your network credentials
#define ssid "Ye"
#define password "1234asdf"
#define FIREBASE_HOST "https://triot-d7e6e-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "p6MtqXkstPUvFZLr61JAjAD4DAMUD97Spp4kE3PC"

// // Insert Authorized Email and Corresponding Password
// #define USER_EMAIL "REPLACE_WITH_THE_AUTHORIZED_USER_EMAIL"
// #define USER_PASSWORD "REPLACE_WITH_THE_AUTHORIZED_USER_PASSWORD"

// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "triot-d7e6e.appspot.com"

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/data/photo.jpg"

#define HIGH 1
#define LOW 0

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
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
#define LED_FLASH         4
#define REED_SWITCH       13
#define BUZZER            15
#define PIN_LED           2


//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig fbConfig;
FirebaseData fbdoStream;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

bool buzzerState = false, taskCompleted = false, readyToSendDoorState = false;
int doorState = 0, beforeDoorState = 1, lockState = 0;
String formattedDate = "", timeStamp = "", dayStamp = "";

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs(int flashOnOff) {
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly
  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    digitalWrite(LED_FLASH, flashOnOff);
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      ESP.restart();
    }
    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    if (flashOnOff == HIGH) {
      flashOnOff = LOW;
    }
    digitalWrite(LED_FLASH, flashOnOff);
    formattedDate = timeClient.getFormattedDate();
    int timeIndex = formattedDate.indexOf("T");
    dayStamp = formattedDate.substring(0, timeIndex);
    timeStamp = formattedDate.substring(timeIndex + 1, formattedDate.length() - 1);

    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while ( !ok );
}

void initWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(500);
    ESP.restart();
  }  
  Serial.print("System connected with IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d\n", WiFi.RSSI());
}

void initSPIFFS(){
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera(){
 // OV2640 camera module
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
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
    ESP.restart();
  } 
}

void onFirebaseStream(FirebaseStream data) {
  Serial.printf("onFirebaseStream: %s %s %s %s\n", data.streamPath().c_str(),
                data.dataPath().c_str(), data.dataType().c_str(),
                data.stringData().c_str());
  if (data.dataType() == "int") {
    int value = data.intData();
    if (data.dataPath() == "/LockState" && value <= 1 && value >= 0)
      lockState = value;
  }
}

void Firebase_BeginStream(const String& streamPath) {
  String path = streamPath;
  if (Firebase.RTDB.beginStream(&fbdoStream, path.c_str()))
  {
    Serial.println("Firebase stream on "+ path);
    Firebase.RTDB.setStreamCallback(&fbdoStream, onFirebaseStream, 0);
  }
  else
    Serial.println("Firebase stream failed: "+fbdoStream.errorReason());
}

void Firebase_EndStream(const String& streamPath) {
  if(Firebase.RTDB.endStream(&fbdoStream))
    Serial.println("Firebase stream end");
}

void Firebase_Init(const String& streamPath) {
  FirebaseAuth fbAuth;
  fbConfig.host = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  fbConfig.token_status_callback = tokenStatusCallback;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

  // fbdo.setResponseSize(2048);
  // Firebase.RTDB.setwriteSizeLimit(&fbdo, "medium");
  while (!Firebase.ready())
  {
    Serial.println("Connecting to firebase...");
    delay(1000);
  }
  Serial.println("Connected to firebase");
  Firebase_BeginStream(streamPath);
}

void taskTakePhoto(void *pvParameter ){
  while (true) {
    if (doorState == HIGH) {
      readyToSendDoorState = false;
      Firebase_EndStream("Receive");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      capturePhotoSaveSpiffs(HIGH);

      while (!taskCompleted) {
        String path = "/" + dayStamp + "/" + dayStamp + " " + timeStamp + ".jpg";
        Serial.print("Uploading picture... ");

        //MIME type should be valid to avoid the download problem.
        //The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
        if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, FILE_PHOTO /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, path /* path of remote file stored in the bucket */, "image/jpeg" /* mime type */)){
          // Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
          Serial.println("Upload Success");
          taskCompleted = true;
          readyToSendDoorState = true;
          Firebase_BeginStream("Receive");
        }
        else{
          Serial.println(fbdo.errorReason());
          taskCompleted = false;
        }
      }
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskBuzzer(void *pvParameter) {
  while (true) {
    if (taskCompleted == true && buzzerState == true) {
      digitalWrite(BUZZER, LOW);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      if(doorState == LOW){
        buzzerState = false;
        digitalWrite(BUZZER, HIGH);
      }
    } else {
      digitalWrite(BUZZER, HIGH);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskSendDoorState(void *pvParameter) {
  while (true) {
    if (doorState != beforeDoorState && readyToSendDoorState == true) {
      if (doorState == HIGH) {
        Firebase.RTDB.setString(&fbdo, "/Send/DoorState", "Open");
        Serial.println("Door Open Send");
      } else if (doorState == LOW) {
        Firebase.RTDB.setString(&fbdo, "/Send/DoorState", "Close");
        Serial.println("Door Close Send");
      }
      beforeDoorState = doorState;
    } 
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_FLASH, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  pinMode(REED_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  initWiFi();

  timeClient.begin();
  timeClient.setTimeOffset(25200);
  while(!timeClient.update()) {
    Serial.println("Waiting for time");
    timeClient.forceUpdate();
  }

  initSPIFFS();
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();
  // Test Capture Camera
  capturePhotoSaveSpiffs(LOW);

  //Firebase
  // Assign the api key
  Firebase_Init("Receive");

  xTaskCreatePinnedToCore(taskTakePhoto, "taskTakePhoto", 10000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskBuzzer, "taskBuzzer", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskSendDoorState, "taskSendDoorState", 10000, NULL, 1, NULL, 1);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  if(lockState == 0) {
    doorState = digitalRead(REED_SWITCH);
    Serial.println(doorState);
    if (doorState == HIGH) {
      buzzerState = true;
      taskCompleted = false;
    }
  } else {
    Serial.println("Is Unlocked");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
