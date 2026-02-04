const char *server_address = "https://webofiot.com/dryer/kasese/server.php";

const char *serverName = "https://webofiot.com/dryer/kasese/server.php";

const char *devicename = "Dryer_Controller";
char *OTA_PASS = "1234";
#define FW_VERSION "v1.3.5-Dryer_Datalogger"

// Combined harvester for weather data in Rwebi
// 20th NOVEMBER 2025
#include <Arduino.h>
#include <stdlib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#include "Wire.h"
#include "SPI.h"

#include "ArduinoOTA.h"
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include "RTClib.h"

#include <Adafruit_BMP280.h>

#include "Adafruit_GFX.h"
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"


#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "ThingSpeak.h"

#include "credentials.h"
#include "WiFi_Manager.h"
#include "upload.h"
#include "Buzzer.h"
#include "timer_keeper.h"

#include <ArduinoJson.h>
StaticJsonDocument<2000> JSON_data;
//JsonDocument JSON_data; // for dynamic amounts of data
StaticJsonDocument<4096> JSON_sendable;
          char payload[4096];

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> 

#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

//SF2578908
//PRESSURE AND TEMP SENSOR
Adafruit_BMP280 bmp; // I2C

// File system instance 
//extern fs::FS &fs;  // Could be SD, SPIFFS, or LittleFS


// For debug logging
#define LOG(msg)  Serial.println(msg)


// Power states with precise voltage thresholds
typedef enum {
    POWER_CRITICAL = 0,  // < 11.0V - Deep Sleep (2 hours)
    POWER_LOW,           // 11.0 - 11.8V - Light Sleep (15 mins)  
    POWER_MODERATE,      // 11.8V - 12.5V - Light Sleep (5 mins)
    POWER_EXCELLENT      // > 12.5V - Active (10 seconds)
} power_state_t;

// Sleep durations in microseconds
const uint64_t power_critical_sleep_duration = 2ULL * 60ULL * 60ULL * 1000000ULL; // 2 hours
const uint64_t power_low_sleep_duration = 15ULL * 60ULL * 1000000ULL;             // 15 minutes
const uint64_t power_mod_sleep_duration = 5ULL * 60ULL * 1000000ULL;              // 5 minutes
const uint64_t power_excellent_delay = 10ULL * 1000ULL;                           // 10 seconds (milliseconds)

power_state_t current_power = POWER_EXCELLENT; // Start optimistic
uint64_t dynamic_interval = power_excellent_delay;


//JASON
//JsonDocument JSON_data;

//REALTIME CLOCK
RTC_DS3231 real_time;


char apiKeyValue[20] = "DRYER_006"; //RWEBITABA TOKEN
char SysID[25] = "Kasese";

char DateTime_str[32] = "24/8/25, 11:52:00";


uint8_t batteryPin = 36; //27; // ADC 2 pins cannot work concurrently with WiFi


// base class GxEPD2_GFX can be used to pass references or pointers to the LCD instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0



// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,
  uint8_t CS_ = 5,  DC_ = 2, RES_ = 15, BUSY_ = 4;


// 4.2'' EPD Module
 //  GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> LCD(GxEPD2_420_GDEY042T81(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683
     GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> LCD(GxEPD2_420c_GDEY042Z98(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683
 
         
bool wifi_connected = false;
uint32_t WiFi_Strength = 0;
char ota_log[150] = "...";
volatile bool otaFinished = false;
volatile bool otaStarted = false;
volatile bool otaError = false;


uint16_t attempts = 0;
uint16_t upload_fails = 0;

const uint64_t upload_frequency_ms   = (10ULL * 60ULL * 1000ULL); // 10 minutes
uint64_t last_upload_time_ms = 0;

char upload_error_code[500] = "";

unsigned long Channel_ID = SECRET_CH_ID; // from credential.h
const char * WriteAPIKey= SECRET_WRITE_APIKEY; // from credential.h

int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;


uint8_t blinker = 26;

uint8_t buzzerpin = 14;
uint8_t wifi_led = 32;

uint8_t server_led = 12; // our own server

// Power management pins
uint8_t sensorsRelay = 25; // Example pin for sensor power control

uint8_t cooling_fan_pin = 26;
uint8_t humidity_fan_pin = 27;

float cabin_temperature = 0.0f;

uint8_t innerFAN = 33;
uint8_t night_Light = 27;
bool innerFan_ON = false;
bool night_Light_ON = false;

static char internals_log[250] = "NULL"; // nker relocation issue that the ESP32 toolchain sometimes throws when you put large-ish global/static arrays into flash/DRAM in a certain way.


float average_temp = 0.0f; char average_temp_str[10] = "..."; // 35.855
float average_humi = 0.0f; char average_humi_str[10] = "..."; // 75.695
float average_press = 0.0, average_elev = 0.0;


upload Uploader(server_led, server_address);

//example datapack
char httpsData[2000] = "{\"PassKey\":\"Rwebi_Weather\", \"Air_Speed\":14.7, \"Air_Temperature\":35.1, \"Air_Humidity\":56.7, \"Sunlight\":1500}";




uint8_t hr = 0, mint = 0, sec = 0, day_ = 0, mth = 0, yr = 0; uint16_t mwaka = 2000; 

char ShortDate[24] = "26-08-2025"; 



// Your global variables – untouched
float temp1 = 0.00, temp2 = 0.00, temp_fara = 0.00;
float heat_index = 0.00;
float humidity = 0.00;
float atm_pressure = 0.00;
float elevation = 0.00;



char temp_str[10] = "__";  // no reading yet
char humi_str[10] = "__"; // no reading yet
char temp_str_log[50] = "...";
char humi_str_log[50] = "...";
char temp_log[320] = "";
char humi_log[320] = "";


char printable_temp_resp[10] = "_";
char printable_humi_resp[10] = "_";



uint8_t active_sensors = 0;
float temperature_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float humidity_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float pressure_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float sensor_1_temp_readings[30];
float sensor_1_humi_readings[30];
float sensor_1_press_readings[30];


uint64_t last_seen[12] = {0, 0, 0, 0, 0, 0};

char sensor_1_last_seen[12] = "HH:MM:SS";
char sensor_2_last_seen[12] = "HH:MM:SS";
char sensor_3_last_seen[12] = "HH:MM:SS";
char sensor_4_last_seen[12] = "HH:MM:SS";
char sensor_5_last_seen[12] = "HH:MM:SS";
char sensor_6_last_seen[12] = "HH:MM:SS";

char sensor_7_last_seen[12] = "HH:MM:SS";
char sensor_8_last_seen[12] = "HH:MM:SS";
char sensor_9_last_seen[12] = "HH:MM:SS";
char sensor_10_last_seen[12] = "HH:MM:SS";
char sensor_11_last_seen[12] = "HH:MM:SS";
char sensor_12_last_seen[12] = "HH:MM:SS";



float sensor_1_time_series_readings[30]; // up to 10 readings ago
char sensor_1_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

float sensor_2_time_series_readings[30]; // up to 30 readings
char sensor_2_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};


char voltage_string[10] = ""; 
double voltage = 0.00;
uint8_t no_of_batts = 1; // 1 for 12V, 2 batts for 24V, 4 batts for 48V

char initializer[100] = "Now Booting...";
bool flipped = false;
uint8_t currentScreen = 1;


uint8_t DISP_MODE = 0;

//BATCH 1
float sensor_1_temp, sensor_2_temp, sensor_3_temp, sensor_4_temp, sensor_5_temp, sensor_6_temp;
float sensor_1_humidity, sensor_2_humidity, sensor_3_humidity, sensor_4_humidity, sensor_5_humidity, sensor_6_humidity;
float sensor_1_pressure, sensor_2_pressure, sensor_3_pressure, sensor_4_pressure, sensor_5_pressure, sensor_6_pressure;
float sensor_1_elevation, sensor_2_elevation, sensor_3_elevation, sensor_4_elevation, sensor_5_elevation, sensor_6_elevation;
float sensor_1_h_index, sensor_2_h_index, sensor_3_h_index, sensor_4_h_index, sensor_5_h_index, sensor_6_h_index;

// BATCH 2
float sensor_7_temp, sensor_8_temp, sensor_9_temp, sensor_10_temp, sensor_11_temp, sensor_12_temp;
float sensor_7_humidity, sensor_8_humidity, sensor_9_humidity, sensor_10_humidity, sensor_11_humidity, sensor_12_humidity;
float sensor_7_pressure, sensor_8_pressure, sensor_9_pressure, sensor_10_pressure, sensor_11_pressure, sensor_12_pressure;
float sensor_7_elevation, sensor_8_elevation, sensor_9_elevation, sensor_10_elevation, sensor_11_elevation, sensor_12_elevation;
float sensor_7_h_index, sensor_8_h_index, sensor_9_h_index, sensor_10_h_index, sensor_11_h_index, sensor_12_h_index;

char  sensor_1_transmissions[32]; char  sensor_2_transmissions[32];   char  sensor_3_transmissions[32];   char  sensor_4_transmissions[32]; 
char  sensor_5_transmissions[32]; char  sensor_6_transmissions[32];   char  sensor_7_transmissions[32];   char   sensor_8_transmissions[32]; 
char  sensor_9_transmissions[32]; char  sensor_10_transmissions[32];  char  sensor_11_transmissions[32];  char  sensor_12_transmissions[32]; 

//function prototypes
void read_temperatures();
void query_rtc();
void update_display();
void upload_to_web_of_iot();
bool save_to_sd_card();


char dataPack[250] = ""; // receivable
void extract_readings();

uint8_t priority_index = 1; uint8_t dryer_1_fanning_level = 0; uint8_t dryer_2_fanning_level = 0;
 // dryer_2_cool_fan_ON = false, dryer_2_humi_fan_ON = false;

uint8_t Battery_Level_Counter = 0;
bool inner_fan_status = false;


void initializeOTA();
void MonitorBattery();
void update_power_settings();


Buzzer buzzer(buzzerpin); // pin 12
WiFi_Manager wifi_obj(wifi_led); // pin 2

bool can_send = false; bool data_sent = false; // how to log missed sends
char upload_log[150] = "Nothing Yet Uploaded!";
char ShortTime[12] = "12:45"; char ShortTime_am_pm[32] = "12:45pm"; char SystemTime[50] = "13:45:21";
char SystemDate[32] = "Mon 18th August, 2025";; // char fullDate[50] = "Wednesday 17th November, 2025";


void flash(uint64_t time_now, uint8_t heartbeat,
           uint16_t ON_1, uint16_t OFF_1,
           uint16_t ON_2, uint16_t OFF_2);

typedef struct dryerData{ // all stringified
      char received_data_bundle[200] = ""; // Payload is limited to 250 bytes.
   }  dryerData; 

dryerData fetch;



bool cooling_fan_on = false; uint32_t cooling_fans_duration = 0;
bool humi_fan_on = false; uint32_t humi_fans_duration = 0;

char humi_fans_start_time_str[12];  char humi_fan_stop_time_str[] = "hh:mm:ss"; char humi_fan_duration[12] = "";
char cooling_fans_start_time_str[12];     char cooling_fan_stop_time_str[] = "HH:MM:SS"; char cooling_fan_duration[12] = "";


uint64_t humi_fans_started = 0, cooling_fans_started = 0;

uint64_t now_now_ms = 0, prev = 0, last_read_time_ms = 0, last_refresh_time_ms = 0; // for DUE, millis() times out at [4.9Bn] 4,294,967.295 seconds which is 49.7 days
uint8_t save_counter = 0;

float solar_radiation = 0.0f;
float wind_speed = 0.0f;

char sendabe_to_cloud_db[2048] = "";
char sendable_to_sd_card[4000] = "";


void setup() { delay(50); // for OTA to fully hand over
       Serial.begin(115200); Serial.print(devicename); Serial.println(" Booting..."); // while (!Serial) delay(100);   // wait for native usb

  // INITIALIZING ALARM
        buzzer.begin();
        xTaskCreatePinnedToCore(BuzzingTask, "BuzzingCheck", 4096, NULL, 4, NULL, 1);
        buzzer.beep(1,50,0);       
  
        pinMode(blinker, OUTPUT); digitalWrite(blinker, HIGH); delay(500); buzzer.beep(1,50,0);

//        Serial2.begin(4800, SERIAL_8N1, 16, 17); delay(500);
  //      Serial2.setTimeout(500); delay(500); // read for 500ms
          Serial2.begin(115200, SERIAL_8N1, 16, 17); //the GSM MOD


        strcpy(initializer, "Initalizing Temperature and Humidity System..."); Serial.println(initializer); 

        LCD.init(115200,true,50,false);  
        strncpy(initializer, "Initializing display!", sizeof(initializer));  Serial.println(initializer);

        Boot();  LCD.hibernate();  

    /*
        Perform the read operation: Use a function that respects the timeout, such as readBytes(), readString(), or parseInt(). 
        If no data is received within the specified timeout period, these functions will return an appropriate indicator 
        (e.g., 0 for readBytes(), an empty string for readString(), or 0 for parseInt() if no valid number is found).
    */

    initialize_sd_card();

    digitalWrite(blinker, HIGH); delay(500);
    digitalWrite(blinker, LOW); delay(500);


    pinMode(server_led, OUTPUT); digitalWrite(server_led, HIGH);

   //initiate_bmp280();    // dht.begin(); Serial.println(F("DHT initiated!"));
      pinMode(batteryPin, INPUT);

    pinMode(innerFAN, OUTPUT); digitalWrite(innerFAN, LOW); 
    pinMode(night_Light, OUTPUT);  digitalWrite(night_Light, LOW);
    
    
    Serial.print("Starting Realtime Clock...");
    initialize_RTC(); delay(100);
    query_rtc(); delay(100);


    for(int i = 0; i < 3; i++){
        digitalWrite(innerFAN, HIGH);
        digitalWrite(night_Light, LOW);
        buzzer.beep(1,50,0);
        delay(2000);

        digitalWrite(innerFAN, LOW);
        digitalWrite(night_Light, HIGH);
        buzzer.beep(1,50,0);
        delay(2000);
    }

    monitor_box_conditions(); // based on RTC: time n temp

          // digitalWrite(innerFAN, LOW); // should be controlled by the contents of the box
         //  digitalWrite(night_Light, LOW);


    digitalWrite(blinker, HIGH); delay(500);
    digitalWrite(blinker, LOW); delay(500);

      
     // pinMode(batteryPin, INPUT); delay(500); // If batteryPin is GPIO34/35/36/39, they MUST NOT use pinMode() at all
    MonitorBattery();

    digitalWrite(blinker, HIGH); delay(500);;
    digitalWrite(blinker, LOW); delay(500);
 
      
      pinMode(cooling_fan_pin, OUTPUT);
      pinMode(humidity_fan_pin, OUTPUT);



  
       
         digitalWrite(server_led, LOW);


         digitalWrite(blinker, HIGH); delay(1000);
         digitalWrite(blinker, LOW);



      
 
  
         digitalWrite(blinker, HIGH); delay(1000);
         digitalWrite(blinker, LOW);

        // WiFi.begin();
  
   
  WiFi.mode(WIFI_STA);    
      
  // Initilize ESP-NOW
      if(esp_now_init() != ESP_OK) {
          Serial.println("Error initializing ESP-NOW");
          return;
      }

      else {
        Serial.println("ESPNOW successfully initialized");
        
        // Register callback function
          esp_err_t result = esp_now_register_recv_cb(esp_now_recv_cb_t(OnSensorData_received)); // Serial.print("CALLBACK: ");Serial.println(result);
          if(result == ESP_OK) Serial.println("Call Back of Call Back successfully set!"); 
      }


       /*
       switch_radio_to_wifi();
     
       //wifi_connected = wifi_obj.ensure_wifi();
       initializeOTA();
       
    
      */
        // Uploader.begin(); // leave it because it uses WiFi
        

       digitalWrite(blinker, HIGH); delay(1000);
       digitalWrite(blinker, LOW);
  

               homepage(); delay(1000);

    // Initialize timers AFTER hardware is ready
      last_read_time_ms = esp_timer_get_time() / 1000ULL;
      last_refresh_time_ms = esp_timer_get_time() / 1000ULL;
      last_upload_time_ms = esp_timer_get_time() / 1000ULL;

         buzzer.beep(2,50,50);  // END OF BOOT
         Serial.println("Done Booting!");
    

}


void BuzzingTask(void * pvParams) { 
  uint8_t checkin_frequency = 10; 
  const TickType_t xDelay = pdMS_TO_TICKS(checkin_frequency);
  while (true) {
    buzzer.update();
    vTaskDelay(xDelay);
  }
}

uint8_t SD_Chip_Select = 5;

void initialize_sd_card(){
      if(!SD.begin(SD_Chip_Select)){
        Serial.println("Card Mount Failed");
        return;
      }
      uint8_t cardType = SD.cardType();

      if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
      }

      Serial.print("SD Card Type: ");
      if(cardType == CARD_MMC){
        Serial.println("MMC");
      } else if(cardType == CARD_SD){
        Serial.println("SDSC");
      } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Size: %lluMB\n", cardSize);

}

bool unset = false;

const uint64_t refresh_rate = (5ULL*60ULL*1000ULL); // refreshing readings or entire screen once 180 seconds
const uint64_t read_duration = 5;// how long it pauses before taking another reading
const uint64_t upload_fequency = (10ULL*60ULL*1000ULL); // every 10 minutes
bool posted = false, uploaded = false;


uint64_t read_frequency = 30ULL * 1000ULL; // 30 seconds if power is good, 60 seconds when not

bool fans_are_togglable = false;
bool logged = false;

uint8_t side_scroll = 1; // default = 1
uint32_t loop_count = 0;

volatile bool packet_received = false;

void loop() {
    now_now_ms = esp_timer_get_time() / 1000ULL; // us to ms

    if(otaStarted)  flash(now_now_ms, blinker, 50, 50, 0, 0); // uploading
    else flash(now_now_ms, blinker, 50, 75, 50, 1500); // normal

    if(packet_received) { buzzer.beep(1,50,0);  extract_readings(); packet_received = false; }
    
    // Sensor readings
    if ((now_now_ms - last_read_time_ms) >= read_frequency) { // 60 seconds if power is good for 24 hours:: 1,440 saved packets/day
        last_read_time_ms = now_now_ms;  // Update FIRST to prevent race conditions
          
          
          query_rtc();
          MonitorBattery(); // battery voltage
          
          monitor_box_conditions(); // internal temp and fan state
          monitor_dryer_readings(); 
          monitor_fans(now_now_ms);
          

          bind_datasets_into_json();

          logged = save_to_sd_card();
        
          Serial.printf("\nLoopCount:%lu\nUptime: %llu, Reading complete! Power mode: %d, Voltage: %.1fV\n",     loop_count, last_read_time_ms/1000, current_power, voltage);
    }
    

    // Display update (less frequent)
    if ((now_now_ms - last_refresh_time_ms) >= refresh_rate) { // 5ULL minutes when day & power is excellent...or...// 15 mins // 30 mins when past midnight and before 6am
          
          if(side_scroll > 3) side_scroll = 1;
          else side_scroll++;

          update_display();

        last_refresh_time_ms = now_now_ms;                  
    }

    if((now_now_ms - last_upload_time_ms) > upload_frequency_ms){ // either every 10mins or at the tenth minute
      if(can_send){
        last_upload_time_ms = now_now_ms;
       // UploadStatus st Uploader.upload_to_web_of_iot(sendabe_to_cloud_db);
    //    Serial.printf("[Upload Task] upload status=%d; report=%s\n", (int)st, Uploader.get_upload_report());
         snprintf(upload_error_code, sizeof(upload_error_code), 
                        "Uploaded at %s on %s", ShortTime_am_pm, SystemDate);
        LOG(upload_error_code);
        can_send = false;

        data_sent = true;
       

      }
    }

  /*
    // OTA handling (only in excellent power mode)
    if (wifi_connected) { // && current_power == POWER_LOW
        ArduinoOTA.handle();
    }

    // Data upload
 // WiFi & OTA handling for excellent power mode
    if (current_power == POWER_LOW || current_power == POWER_MODERATE || current_power == POWER_EXCELLENT) {
        // Check/ensure WiFi periodically for OTA
        static uint64_t last_wifi_check = 0;
          if ((now_now_ms - last_wifi_check) >= 10000) { // Check every 10 seconds
              wifi_connected = wifi_obj.ensure_wifi();
              last_wifi_check = now_now_ms;
          }
     
    }

    // Data upload (only check WiFi here if not already connected from OTA section)
    if (can_send) {
        // If we're in excellent power mode but WiFi check hasn't run recently
        if (current_power == POWER_EXCELLENT && !wifi_connected) {
            wifi_connected = wifi_obj.ensure_wifi();
        }
        
        if (wifi_connected) {
                Serial.println("Uploading data...");
                bool upload_success = post_to_thingsPeak();
                delay(500);
            
            
                prepare_JSON_file();
                delay(500);
               // upload_to_web_of_iot();
               UploadStatus st = Uploader.upload_to_web_of_iot(httpsData);
               Serial.printf("[Upload Task] upload status=%d; report=%s\n", (int)st, Uploader.get_upload_report());

                
                snprintf(upload_log, sizeof(upload_log), 
                        "Uploaded at %s on %s", ShortTime_am_pm, SystemDate);
           // } else {      strncpy(upload_log, "Upload failed", sizeof(upload_log)); }
            
            data_sent = true;
        } else {
            Serial.println("WiFi not available for upload");
        }
        
        can_send = false;
    }

 /*
    // Enter sleep based on power state
    if (current_power != POWER_EXCELLENT) {
        sleep_dynamically();
    } else {
        // For excellent power, use delay instead of sleep
        delay(1000); // Short delay to prevent tight loop
    }
  */
    delay(2); // make the loop less tight
        loop_count++;

}
/// loop()

char save_log[300];

static char *csv_file = "/data.csv";
static char current_json_file[128] = "/data.json";

char new_name[64];

static char json_file[64] = "/data.json"; // each data packet weighs ~2kB ... Should update this when rotating
static const size_t MAX_FILE_SIZE = 20 * 1024 * 1024; // 20 MB per file [stores only weekly data | 20.16MB]
static const size_t MAX_TOTAL_SIZE = 100 * 1024 * 1024;    // 100 MB total limit

bool   save_to_sd_card(){
  /*
  *  OPEN FILE
  *  CHECK ITS SIZE: IF < ESP FILE SYSTEM: APPEND TO IT
  *  IF SIZE IS BIGGER, CREATE ANOTHER FILE (TIMESTAMPED) e.g. data_15_Jan_2026.json
  *  SAVE TO THAT NEW FILE
  *  WHEN NEED BE, SHARE THE FULL FILE  CONTENTS OVER SCREEN OR BT OR WIFI ...

  */


 if (!sendable_to_sd_card || strlen(sendable_to_sd_card) == 0) {
        LOG("[SD] Nothing to write");
        return false;
    }

    // --- Open or create file ---
    File file = SD.open(json_file, FILE_APPEND);

    if (!file) {
        LOG("[SD] Failed to open file for appending");
        return false;
    }

    // --- Size check ---
    size_t currentSize = file.size();
    size_t incomingSize = strlen(sendable_to_sd_card);

    if ((currentSize + incomingSize) > MAX_FILE_SIZE) {
        file.close();
        file = create_another_file();

        if (!file) {
            LOG("[SD] File rotation failed");
            return false;
        }
        currentSize = 0; // new file
    }


    // --- Append JSON to either the old file or the new file  ---
    bool saved = file.println(sendable_to_sd_card); // newline-delimited JSON
    file.close();

    if (!saved) {
        Serial.println("[SD] Write failed");
        return false;
    }


    snprintf(
        save_log,
        sizeof(save_log),
        "[SD] Appended %lu bytes (file size now %lu bytes)",
        (unsigned long)incomingSize,
        (unsigned long)(currentSize + incomingSize)
    );

 LOG(save_log);


    return true;

}


File create_another_file() {

    snprintf(
        new_name, sizeof(new_name),
        "/data_%s_%s.json",
        ShortDate, ShortTime
    );

    // Replace ':' for FAT compatibility
    for (char *p = new_name; *p; p++) {
        if (*p == ':') *p = '-';
    }
    char new_name_log[500];
    snprintf(new_name_log, sizeof(new_name_log), "[SD] Rotating file → %s", new_name);
    LOG(new_name_log);

    strncpy(json_file, new_name, sizeof(json_file));
    json_file[sizeof(json_file) - 1] = '\0';
    // or // snprintf(json_file, sizeof(json_file), "%s", new_name);


    File newFile = SD.open(new_name, FILE_APPEND);
    if (!newFile) {
        LOG("[SD] Failed to create rotated file");
    }

    return newFile;
}



// Optional: Function to list files and manage storage
void manage_sd_storage() {
    /*
    * Optional: Delete old files if storage is getting full
    * Keep only last N files or files from last X days
    */
    File root = SD.open("/");
    if (!root) {
        Serial.println("[SD] Failed to open root directory");
        return;
    }
    
    File file;
    int file_count = 0;
    uint32_t total_size = 0;
    
    while (true) {
    File file = root.openNextFile();
    if (!file) break;

    if (!file.isDirectory()) {
        file_count++;
        total_size += file.size();
        Serial.printf("  %s (%u bytes)\n", file.name(), file.size());
    }
    file.close();
  }

    Serial.printf("[SD] Storage: %d files, %u bytes total\n", file_count, total_size);
    
    // Optional: Delete oldest file if over limit
    if (total_size > MAX_TOTAL_SIZE) {
        delete_oldest_file();
    }

}



// Optional: Helper to delete oldest file
void delete_oldest_file() {
    // Implementation depends on your file naming scheme
    // Could delete by oldest modification time or filename pattern
}

void sync_sd_card() {
    // Force SD card to flush buffers
    #ifdef SD_MMC
        SD_MMC.end();
        SD_MMC.begin();
    #elif defined(SD)
        SD.end();
        SD.begin();
    #endif
}

/*
✔ Only copy
✔ Set a flag
✔ Exit ASAP
This is exactly what ESP-NOW callbacks should do.
*/

void OnSensorData_received(const uint8_t * mac, const uint8_t *incomingData, int len){
     //   memcpy(&fetch, incomingData, min(len, sizeof(dryerData))); // no matching function for call to 'min(int&, unsigned int)'
        // The compiler will complain because len is an int, while sizeof(dryerData) is an unsigned int (size_t).
 // The template deduction for std::min() fails when the two arguments are of different signedness.

        //Since memcpy length must be size_t, it’s best to cast explicitly: ... no template fights.
        size_t weight_of_packet = (len < (int)sizeof(dryerData)) ? (size_t)len : sizeof(dryerData); // either sizeof of len
        memcpy(&fetch, incomingData, weight_of_packet);

        uint32_t size_of_packet = sizeof(dataPack);

  //   memcpy(&fetch, incomingData, sizeof(dryerData)); //If len < sizeof(dryerData), memcpy will copy beyond valid incomingData.
 //    strcpy(dataPack, fetch.received_data_bundle); // If the buffer is not null-terminated, strcpy() will read past the end.

        strncpy(dataPack, fetch.received_data_bundle, size_of_packet-1); // Only copy and queue the packet in the callback:
        dataPack[size_of_packet-1] = '\0';
     // Serial.print("Size of received BUFF: "); Serial.println(size_of_packet);

      //Serial.print("Received => "); Serial.println(dataPack);  delay(50);
      packet_received = true;

      //extract_readings();
      //beep(1, 0); //flash();
    // ESP-NOW callback should be FAST (this is critical)
    //exit ASAP
     
}


//deserialize to assign char[] and floats accordingly
void extract_readings(){
  if(!packet_received) { LOG("No packet received!"); return; }  // this never runs, but just in case 

       DeserializationError err = deserializeJson(JSON_data, dataPack);

        if (err) {
            Serial.print("JSON error: ");
            Serial.println(err.c_str());
            return;
        }

          
    const char * sensor_ID = JSON_data["S_ID"] | "unknown"; /*const char * sensor_2_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_3_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_4_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_5_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_6_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_7_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_8_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_9_ID = JSON_data["S_ID"] | "unknown";  const char * sensor_10_ID = JSON_data["S_ID"] | "unknown";
   
    const char * sensor_11_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_12_ID = JSON_data["S_ID"] | "unknown";
    */

    if(strcmp(sensor_ID, "T_1") == 0){
          sensor_1_temp = JSON_data["Temp"] | NAN;
          sensor_1_humidity = JSON_data["Humi"] | NAN; 
          sensor_1_pressure = JSON_data["Atm_Pre"] | NAN;
          sensor_1_elevation = JSON_data["Elev"] | NAN;
          sensor_1_h_index = JSON_data["Heat_index"] | NAN;
           
       //   temperature_readings[0] = sensor_1_temp; humidity_readings[0] = sensor_1_humidity;  // arrays or indexes
      strncpy(sensor_1_last_seen, ShortTime, sizeof(sensor_1_last_seen));      sensor_1_last_seen[sizeof(sensor_1_last_seen) - 1] = '\0'; 
      strncpy(sensor_1_transmissions,  JSON_data["Sends"], sizeof(sensor_1_transmissions));
    }

    if(strcmp(sensor_ID, "T_2") == 0){
        sensor_2_temp = JSON_data["Temp"] | NAN;
        sensor_2_humidity = JSON_data["Humi"] | NAN; 
        sensor_2_pressure = JSON_data["Atm_Pre"] | NAN;
        sensor_2_elevation = JSON_data["Elev"] | NAN;
        sensor_2_h_index = JSON_data["Heat_index"] | NAN;
    strncpy(sensor_2_last_seen, ShortTime, sizeof(sensor_2_last_seen)); sensor_2_last_seen[sizeof(sensor_2_last_seen) - 1] = '\0';
    strncpy(sensor_2_transmissions, JSON_data["Sends"], sizeof(sensor_2_transmissions));
    }

    if(strcmp(sensor_ID, "T_3") == 0){
        sensor_3_temp = JSON_data["Temp"] | NAN;
        sensor_3_humidity = JSON_data["Humi"] | NAN; 
        sensor_3_pressure = JSON_data["Atm_Pre"] | NAN;
        sensor_3_elevation = JSON_data["Elev"] | NAN;
        sensor_3_h_index = JSON_data["Heat_index"] | NAN;
      strncpy(sensor_3_last_seen, ShortTime, sizeof(sensor_3_last_seen)); sensor_3_last_seen[sizeof(sensor_3_last_seen) - 1] = '\0';
      strncpy(sensor_3_transmissions,  JSON_data["Sends"], sizeof(sensor_3_transmissions));
    }

    if(strcmp(sensor_ID, "T_4") == 0){
        sensor_4_temp = JSON_data["Temp"] | NAN;
        sensor_4_humidity = JSON_data["Humi"] | NAN;
        sensor_4_pressure = JSON_data["Atm_Pre"] | NAN;
        sensor_4_elevation = JSON_data["Elev"] | NAN;
        sensor_4_h_index = JSON_data["Heat_index"] | NAN;
      strncpy(sensor_4_last_seen, ShortTime, sizeof(sensor_4_last_seen)); sensor_4_last_seen[sizeof(sensor_4_last_seen) - 1] = '\0';
      strncpy(sensor_4_transmissions, JSON_data["Sends"], sizeof(sensor_4_transmissions));
    }

    if(strcmp(sensor_ID, "T_5") == 0){
        sensor_5_temp = JSON_data["Temp"] | NAN;
        sensor_5_humidity = JSON_data["Humi"] | NAN;
        sensor_5_pressure = JSON_data["Atm_Pre"] | NAN;
        sensor_5_elevation = JSON_data["Elev"] | NAN;
        sensor_5_h_index = JSON_data["Heat_index"] | NAN;
      strncpy(sensor_5_last_seen, ShortTime, sizeof(sensor_5_last_seen)); sensor_5_last_seen[sizeof(sensor_5_last_seen) - 1] = '\0';
      strncpy(sensor_5_transmissions,  JSON_data["Sends"], sizeof(sensor_5_transmissions));
    }

     if(strcmp(sensor_ID, "T_6") == 0){
        sensor_6_temp = JSON_data["Temp"] | NAN;
        sensor_6_humidity = JSON_data["Humi"] | NAN;
        sensor_6_pressure = JSON_data["Atm_Pre"] | NAN;
        sensor_6_elevation = JSON_data["Elev"] | NAN;
        sensor_6_h_index = JSON_data["Heat_index"] | NAN;
    strncpy(sensor_6_last_seen, ShortTime, sizeof(sensor_6_last_seen)); sensor_6_last_seen[sizeof(sensor_6_last_seen) - 1] = '\0';
    strncpy(sensor_6_transmissions,  JSON_data["Sends"], sizeof(sensor_6_transmissions));
     }

     if(strcmp(sensor_ID, "T_7") == 0){
        sensor_7_temp = JSON_data["Temp"];
        sensor_7_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_7_pressure = JSON_data["Atm_Pre"];
        sensor_7_elevation = JSON_data["Elev"];
        sensor_7_h_index = JSON_data["Heat_index"];
    strncpy(sensor_7_last_seen, ShortTime, sizeof(sensor_7_last_seen)); sensor_7_last_seen[sizeof(sensor_7_last_seen) - 1] = '\0';
    strncpy(sensor_7_transmissions, JSON_data["Sends"], sizeof(sensor_7_transmissions));
     }

     if(strcmp(sensor_ID, "T_8") == 0){
        sensor_8_temp = JSON_data["Temp"];
        sensor_8_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_8_pressure = JSON_data["Atm_Pre"];
        sensor_8_elevation = JSON_data["Elev"];
        sensor_8_h_index = JSON_data["Heat_index"];
    strncpy(sensor_8_last_seen, ShortTime, sizeof(sensor_8_last_seen)); sensor_8_last_seen[sizeof(sensor_8_last_seen) - 1] = '\0';
    strncpy(sensor_8_transmissions, JSON_data["Sends"], sizeof(sensor_8_transmissions));
     }

     if(strcmp(sensor_ID, "T_9") == 0){
        sensor_9_temp = JSON_data["Temp"];
        sensor_9_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_9_pressure = JSON_data["Atm_Pre"];
        sensor_9_elevation = JSON_data["Elev"];
        sensor_9_h_index = JSON_data["Heat_index"];
        strncpy(sensor_9_last_seen, ShortTime, sizeof(sensor_9_last_seen)); sensor_9_last_seen[sizeof(sensor_9_last_seen) - 1] = '\0';
        strncpy(sensor_9_transmissions, JSON_data["Sends"], sizeof(sensor_9_transmissions));
     }

     if(strcmp(sensor_ID, "T_10") == 0){
        sensor_10_temp = JSON_data["Temp"];
        sensor_10_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_10_pressure = JSON_data["Atm_Pre"];
        sensor_10_elevation = JSON_data["Elev"];
        sensor_10_h_index = JSON_data["Heat_index"];
    strncpy(sensor_10_last_seen, ShortTime, sizeof(sensor_10_last_seen)); sensor_10_last_seen[sizeof(sensor_10_last_seen) - 1] = '\0';
    strncpy(sensor_10_transmissions, JSON_data["Sends"], sizeof(sensor_10_transmissions));
     }

     if(strcmp(sensor_ID, "T_11") == 0){
        sensor_11_temp = JSON_data["Temp"];
        sensor_11_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_11_pressure = JSON_data["Atm_Pre"];
        sensor_11_elevation = JSON_data["Elev"];
        sensor_11_h_index = JSON_data["Heat_index"];
    strncpy(sensor_11_last_seen, ShortTime, sizeof(sensor_11_last_seen)); sensor_11_last_seen[sizeof(sensor_11_last_seen) - 1] = '\0';
    strncpy(sensor_11_transmissions, JSON_data["Sends"], sizeof(sensor_11_transmissions));
     }

     if(strcmp(sensor_ID, "T_12") == 0){
        sensor_12_temp = JSON_data["Temp"];
        sensor_12_humidity = JSON_data["Humi"]; // humidity ... moisture in the air
        sensor_12_pressure = JSON_data["Atm_Pre"];
        sensor_12_elevation = JSON_data["Elev"];
        sensor_12_h_index = JSON_data["Heat_index"];
    strncpy(sensor_12_last_seen, ShortTime, sizeof(sensor_12_last_seen)); sensor_12_last_seen[sizeof(sensor_12_last_seen) - 1] = '\0';
    strncpy(sensor_12_transmissions, JSON_data["Sends"], sizeof(sensor_12_transmissions));
     }

  /* the above only update when the respective sensor has ID'd itself, 
  hence no need for the part below... which was preventing the zeroing of other unsent values

    if(sensor_1_temp > 2.00) {  
        last_seen[0] = now_now_ms;    
        strcpy(sensor_1_last_seen, ShortTime_am_pm);  
        sensor_1_humidity  = sensor_1_humidity_;
        sensor_1_pressure  = sensor_1_pressure_;
        sensor_1_elevation = sensor_1_elevation_;
        sensor_1_h_index   = sensor_1_h_index_;
        temperature_readings[0] = sensor_1_temp; humidity_readings[0] = sensor_1_humidity;
      //  sensor_1_transmissions = sensor_1_transmissions_;
       
      }

    if(sensor_2_temp > 2.00) {  
        strcpy(sensor_2_last_seen, ShortTime_am_pm);  
        sensor_2_humidity  = sensor_2_humidity_;
        sensor_2_pressure  = sensor_2_pressure_;
        sensor_2_elevation = sensor_2_elevation_;
        sensor_2_h_index   = sensor_2_h_index_;
        temperature_readings[1] = sensor_2_temp; humidity_readings[1] = sensor_2_humidity;
      //  sensor_2_transmissions = sensor_2_transmissions_;
       
      }

    if(sensor_3_temp > 2.00) {  
        strcpy(sensor_3_last_seen, ShortTime_am_pm);  
        sensor_3_humidity  = sensor_3_humidity_;
        sensor_3_pressure  = sensor_3_pressure_;
        sensor_3_elevation = sensor_3_elevation_;
        sensor_3_h_index   = sensor_3_h_index_;
        temperature_readings[2] = sensor_3_temp; humidity_readings[2] = sensor_3_humidity;
      //  sensor_2_transmissions = sensor_2_transmissions_;
       
      } 

    if(sensor_4_temp > 2.00) {  
        strcpy(sensor_4_last_seen, ShortTime_am_pm);  
        sensor_4_humidity  = sensor_4_humidity_;
        sensor_4_pressure  = sensor_4_pressure_;
        sensor_4_elevation = sensor_4_elevation_;
        sensor_4_h_index   = sensor_4_h_index_;
        temperature_readings[3] = sensor_4_temp; humidity_readings[3] = sensor_4_humidity;
      //  sensor_2_transmissions = sensor_2_transmissions_;
       
      }  
   
       if(sensor_5_temp > 2.00) {  
        strcpy(sensor_5_last_seen, ShortTime_am_pm);  
        sensor_5_humidity  = sensor_5_humidity_;
        sensor_5_pressure  = sensor_5_pressure_;
        sensor_5_elevation = sensor_5_elevation_;
        sensor_5_h_index   = sensor_5_h_index_;
        temperature_readings[4] = sensor_5_temp; humidity_readings[4] = sensor_5_humidity;
      //  sensor_2_transmissions = sensor_2_transmissions_;
       
      }

          if(sensor_6_temp > 2.00) {  
        
        strcpy(sensor_6_last_seen, ShortTime_am_pm);  
        sensor_6_humidity  = sensor_6_humidity_;
        sensor_6_pressure  = sensor_6_pressure_;
        sensor_6_elevation = sensor_6_elevation_;
        sensor_6_h_index   = sensor_6_h_index_;
        temperature_readings[5] = sensor_6_temp; humidity_readings[5] = sensor_6_humidity;
      //  sensor_2_transmissions = sensor_2_transmissions_;
       
      }
      */


}



void monitor_dryer_readings(){
  average_temp = 0.00;
  average_humi = 0.00;
  active_sensors = 0;
  average_press = 0.0;
  average_elev = 0.0;

  
  average_press = sensor_1_pressure + sensor_2_pressure + sensor_3_pressure + sensor_4_pressure + sensor_5_pressure + sensor_6_pressure;
  average_elev = sensor_1_elevation + sensor_2_elevation + sensor_3_elevation + sensor_4_elevation + sensor_5_elevation + sensor_6_elevation;


    //  Serial.print("Readings Stored: [");
      for(int i=0; i<(sizeof(temperature_readings)/sizeof(temperature_readings[0])); i++){
        if(temperature_readings[i] >= 2.00 || !isnan(temperature_readings)){ // !is_nan and is not unexplainably cold            
            active_sensors++;
        }
        // Serial.print(temperature_readings[i]); if(i<5) Serial.print(", ");
      }
    //  Serial.print("] :: Active Sensors: ");   
      Serial.print("Active Sensors: "); Serial.println(active_sensors);
      if(active_sensors <= 0) { average_temp = 0.00; average_humi = 0.00; }
      else {
        average_temp /= (active_sensors);
        average_humi /= (active_sensors);
      }

      Serial.print("Average Temp: "); Serial.println(average_temp);
      Serial.print("Average Humi: "); Serial.println(average_humi);


  average_press /= active_sensors;
  average_elev /=  active_sensors;
}



char fans_log[100] = "...";
uint16_t re_run_attempts = 0;
void monitor_fans(uint64_t time_now_ms){ // ALL DRYER FANS ARE ACTIVE LOW
 Serial.println("Monitoring Fans...");

  if(!fans_are_togglable) {
    strcpy(fans_log, "Night Conditions: Fans IDLE");
    return;
  }

      if(Battery_Level_Counter <= 1){ // < 11.5V ... power is too low
          if(humi_fan_on){   digitalWrite(humidity_fan_pin, HIGH); cooling_fan_on = false; re_run_attempts++; }
          if(cooling_fan_on) {digitalWrite(cooling_fan_pin, HIGH); cooling_fan_on = false; re_run_attempts++;}    
          strcpy(fans_log, "Power too Low: Fans OFF");
          // return;
      }
      else { // START FANS ONLY IF BATTERY IS SUFFICIENT
         re_run_attempts = 0; 
        if(average_humi >= 50.0){ // high humidity, toggle extractive fans
          if(!humi_fan_on){digitalWrite(humidity_fan_pin, LOW); cooling_fan_on = true;  humi_fans_started = time_now_ms; }
          if(humi_fan_on) { humi_fans_duration = (time_now_ms - humi_fans_started)/1000; }
        } 
        else { // if humidity has lowered, jako ezo fan
              if(humi_fan_on){digitalWrite(humidity_fan_pin, HIGH); cooling_fan_on = false; }
            }


        if(average_temp > 45.0){ // turn em ON at high temperatures
          if(!cooling_fan_on) {digitalWrite(cooling_fan_pin, LOW); cooling_fan_on = true;  cooling_fans_started = time_now_ms;}
          if(cooling_fan_on) {cooling_fans_duration = (time_now_ms - cooling_fans_started)/1000;}
          
        }
        else if(average_temp <= 35.0){ // turn em OFF at low temperatures
             if(cooling_fan_on) {digitalWrite(cooling_fan_pin, HIGH); cooling_fan_on = false; } 
        }
        else { // in between 35 - 45 ... do nothing... just monitor the time
             if(cooling_fan_on) {cooling_fans_duration = (time_now_ms - cooling_fans_started)/1000;}
        }
     }
}



bool bind_datasets_into_json(){
  bool bound_successfully = false;
           JSON_sendable.clear();
           save_counter++; // save this into EEPROM or SPIFFS

    // SYSTEM PARAMETERS
        JsonObject System      = JSON_sendable["System"].to<JsonObject>();
        System["FW_Version"]       = FW_VERSION;
        System["Uptime"]           = (now_now_ms/1000); // convert into seconds
        System["Date"] = SystemDate;
        System["Time"] = SystemTime;
        System["Datalog_Count"] = save_counter; 
        System["Voltage"] = voltage;
        System["Internal_Temp"] = cabin_temperature;
        System["Internal_Fan"] = inner_fan_status;
        

        // SNAPSHOTS OF READINGS AT A CERTAIN TIME, NOT CUMMULATED MEDIANS/MEANS
        JsonObject Dryer_Conditions      = JSON_sendable["Dryer_Conditions"].to<JsonObject>();
        Dryer_Conditions["Active_Sensors"] = active_sensors;
        Dryer_Conditions["Average_Temperature"] = average_temp;
        Dryer_Conditions["Average_Humidity"] = average_humi;
        Dryer_Conditions["Average_Pressure"] = average_press;

        Dryer_Conditions["Humidity_Fans"] = humi_fan_on; // state of fan
        Dryer_Conditions["Humidity_Fans_Start_Time"] = humi_fans_start_time_str; // most recent time of ON
        Dryer_Conditions["Humidity_Fans_Stop_Time"] = humi_fan_stop_time_str;
        Dryer_Conditions["Humidity_Fans_ON_Duration"] = humi_fan_duration;

        Dryer_Conditions["Cooling_Fans"] = cooling_fan_on;
        Dryer_Conditions["Cooling_Fans_Start_Time"] = cooling_fan_stop_time_str; 
        Dryer_Conditions["Cooling_Fans_Start_Time"] = cooling_fans_start_time_str; // most recent time of ON
        Dryer_Conditions["Cooling_Fans_Stop_Time"] = cooling_fan_stop_time_str;
        Dryer_Conditions["Cooling_Fans_ON_Duration"] = cooling_fan_duration;
        
        

        //ALL SENSOR COMBINED SENSOR PARAMETERS
        JsonObject All_Sensors      = JSON_sendable["All_Sensors"].to<JsonObject>();
    for(int i = 0; i < active_sensors; i++){ 
        All_Sensors["Temperature_Readings"][i] = temperature_readings[i];
        All_Sensors["Humidity_Readings"][i] = humidity_readings[i];
        All_Sensors["Pressure_Readings"][i] = pressure_readings[i];
    }   

        // ---------------- INDIVIDUAL SENSORS ----------------
        // SENSOR 1 PARAMETERS
      JsonObject Sensor_1    = JSON_sendable["Sensor_1"].to<JsonObject>();
          Sensor_1["Position"]     = "AIR INLET 1";
          Sensor_1["Temp"]      =   sensor_1_temp;
          Sensor_1["Humi"]      =   sensor_1_humidity;
          Sensor_1["Pressure"]  =   sensor_1_pressure;
          Sensor_1["Last_Seen"] =   sensor_1_last_seen;
          Sensor_1["Sends"]     =   sensor_1_transmissions;

  /*
       // time series readings/... we shall do this later
          JsonArray readings1 = Sensor_1["Time_Series_Readings"].to<JsonArray>();
          JsonArray times1    = Sensor_1["Timestamps"].to<JsonArray>();
          for (int i = 0; i < 30; i++) {
              readings1.add(sensor_1_temp_readings[i]);
              readings1.add(sensor_1_humi_readings[i]);
              readings1.add(sensor_1_press_readings[i]);
              times1.add(sensor_1_time_stamps[i]);
          }

  */
        // SENSOR 2 PARAMETERS
      JsonObject Sensor_2    = JSON_sendable["Sensor_2"].to<JsonObject>();
          Sensor_2["Position"]     = "AIR INLET 2";
          Sensor_2["Temp"]      =   sensor_2_temp;
          Sensor_2["Humi"]      =   sensor_2_humidity;
          Sensor_2["Pressure"]  =   sensor_2_pressure;
          Sensor_2["Last_Seen"] =   sensor_2_last_seen;
          Sensor_2["Sends"]     =   sensor_2_transmissions;


        // SENSOR 3 PARAMETERS
      JsonObject Sensor_3    = JSON_sendable["Sensor_3"].to<JsonObject>();
          Sensor_3["Position"]     = "AIR OUTLET 1";
          Sensor_3["Temp"]      =   sensor_3_temp;
          Sensor_3["Humi"]      =   sensor_3_humidity;
          Sensor_3["Pressure"]  =   sensor_3_pressure;
          Sensor_3["Last_Seen"] =   sensor_3_last_seen;
          Sensor_3["Sends"]     =   sensor_3_transmissions;


        // SENSOR 4 PARAMETERS
      JsonObject Sensor_4    = JSON_sendable["Sensor_4"].to<JsonObject>();
          Sensor_4["Position"]     = "AIR OUTLET 2";
          Sensor_4["Temp"]      =   sensor_4_temp;
          Sensor_4["Humi"]      =   sensor_4_humidity;
          Sensor_4["Pressure"]  =   sensor_4_pressure;
          Sensor_4["Last_Seen"] =   sensor_4_last_seen;
          Sensor_4["Sends"]     =   sensor_4_transmissions;


        // SENSOR 5 PARAMETERS
      JsonObject Sensor_5    = JSON_sendable["Sensor_5"].to<JsonObject>();
          Sensor_5["Position"]     = "DRYING BED LOWEST";
          Sensor_5["Temp"]      =   sensor_5_temp;
          Sensor_5["Humi"]      =   sensor_5_humidity;
          Sensor_5["Pressure"]  =   sensor_5_pressure;
          Sensor_5["Last_Seen"] =   sensor_5_last_seen;
          Sensor_5["Sends"]     =   sensor_5_transmissions;

        // SENSOR 5 PARAMETERS
      JsonObject Sensor_6    = JSON_sendable["Sensor_6"].to<JsonObject>();
          Sensor_6["Position"]     = "DRYING BED MIDDLE";
          Sensor_6["Temp"]      =   sensor_6_temp;
          Sensor_6["Humi"]      =   sensor_6_humidity;
          Sensor_6["Pressure"]  =   sensor_6_pressure;
          Sensor_6["Last_Seen"] =   sensor_6_last_seen;
          Sensor_6["Sends"]     =   sensor_6_transmissions;

        // SENSOR 7 PARAMETERS
      JsonObject Sensor_7    = JSON_sendable["Sensor_7"].to<JsonObject>();
          Sensor_7["Position"]     = "DRYING BED TOP";
          Sensor_7["Temp"]      =   sensor_7_temp;
          Sensor_7["Humi"]      =   sensor_7_humidity;
          Sensor_7["Pressure"]  =   sensor_7_pressure;
          Sensor_7["Last_Seen"] =   sensor_7_last_seen;
          Sensor_7["Sends"]     =   sensor_7_transmissions;

        // SENSOR 5 PARAMETERS
      JsonObject Sensor_8    = JSON_sendable["Sensor_8"].to<JsonObject>();
          Sensor_8["Position"]     = "DRYING BED MIDDLE ZONE";
          Sensor_8["Temp"]      =   sensor_8_temp;
          Sensor_8["Humi"]      =   sensor_8_humidity;
          Sensor_8["Pressure"]  =   sensor_8_pressure;
          Sensor_8["Last_Seen"] =   sensor_8_last_seen;
          Sensor_8["Sends"]     =   sensor_8_transmissions;

        // SENSOR 5 PARAMETERS
      JsonObject Sensor_9    = JSON_sendable["Sensor_9"].to<JsonObject>();
          Sensor_9["Position"]     = "DRYING BED ENTRY ZONE";
          Sensor_9["Temp"]      =   sensor_9_temp;
          Sensor_9["Humi"]      =   sensor_9_humidity;
          Sensor_9["Pressure"]  =   sensor_9_pressure;
          Sensor_9["Last_Seen"] =   sensor_9_last_seen;
          Sensor_9["Sends"]     =   sensor_9_transmissions;

        // SENSOR 10 PARAMETERS
      JsonObject Sensor_10    = JSON_sendable["Sensor_10"].to<JsonObject>();
          Sensor_10["Position"]     = "DRYING BED EXIT ZONE";
          Sensor_10["Temp"]      =   sensor_10_temp;
          Sensor_10["Humi"]      =   sensor_10_humidity;
          Sensor_10["Pressure"]  =   sensor_10_pressure;
          Sensor_10["Last_Seen"] =   sensor_10_last_seen;
          Sensor_10["Sends"]     =   sensor_10_transmissions;

        // SENSOR 11 PARAMETERS
      JsonObject Sensor_11    = JSON_sendable["Sensor_11"].to<JsonObject>();
          Sensor_11["Position"]     = "DRYING BED RANDOM POSITION";
          Sensor_11["Temp"]      =   sensor_11_temp;
          Sensor_11["Humi"]      =   sensor_11_humidity;
          Sensor_11["Pressure"]  =   sensor_11_pressure;
          Sensor_11["Last_Seen"] =   sensor_11_last_seen;
          Sensor_11["Sends"]     =   sensor_11_transmissions;

        // SENSOR 12 PARAMETERS
      JsonObject Sensor_12    = JSON_sendable["Sensor_12"].to<JsonObject>();
          Sensor_12["Position"]     = "DRYING BED AMBIENT ZONE";
          Sensor_12["Temp"]      =   sensor_1_temp;
          Sensor_12["Humi"]      =   sensor_12_humidity;
          Sensor_12["Pressure"]  =   sensor_12_pressure;
          Sensor_12["Last_Seen"] =   sensor_12_last_seen;
          Sensor_12["Sends"]     =   sensor_12_transmissions;

          // SOLAR RADIATION DATA // WIND SPEED DATA
      JsonObject Environment    = JSON_sendable["Environment"].to<JsonObject>();
          Environment["Solar_Radiation"]     = solar_radiation;
          Environment["Wind_Speed"]     = wind_speed;
          
          // --- Serialize into a buffer or string ---
           size_t len = serializeJsonPretty(JSON_sendable, sendable_to_sd_card); /// to save to data.json, then as dada.csv
           serializeJson(JSON_sendable, sendabe_to_cloud_db);  // to send via the 4G SIM to upload_to_web_of_iot(sendabe_to_cloud_db)


          if (active_sensors <= 0) { // to prevent garbage writes when sensors fail.
              LOG("[JSON] No active sensors. Skipping bind.");
              return false;
          }

          if (len <= 0) {
              LOG("[JSON] Serialization failed!");
              return false;
          }

               
          if (len > 0) {
            bound_successfully = true;
            
            LOG("[JSON] Serialization successful.");
            LOG("[JSON] Payload size: " + String(len) + " bytes");
            LOG("[JSON] --- Payload Preview ---");
           //   LOG(sendable_to_sd_card); // optional: can be commented out if too large//
          } 
          
  return bound_successfully;

 
}


/* LOG SENSOR DROPOUTS AS WELL
"Events": [
  {"Type":"FAN_ON","Fan":"Cooling","Time":"14:32:10"},
  {"Type":"SENSOR_LOST","Sensor":"S3","Duration_s":120}
]
*/

/*

void prepare_JSON_file(){ // bundle of JOY
strcpy(httpsData, "");

     JSON_data["PassKey"] = "Dryer_Kima";
     JSON_data["Air_Temperature"] = temp1;  // float, env't temp, rs485
     JSON_data["Air_Humidity"] = humidity;  // float, env't humi, rs485
     JSON_data["Air_Speed"] =  wind_speed; // float, env't speed, rs485

     JSON_data["Atmospheric_Pressure"] = atm_pressure;
     JSON_data["Internal_Temp"] = cabin_temperature; 
     JSON_data["Internal_Fan"] = innerFan_ON; 
     JSON_data["Night_Light"] = night_Light_ON;

     JSON_data["Voltage"] =  voltage;

     JSON_data["ON_TIME"] = now_now_ms;
     JSON_data["Hour"] = hr; 
     JSON_data["Minute"] = mint;  
     JSON_data["Second"] =  sec;
     JSON_data["Date"] =  SystemDate;


    serializeJson(JSON_data, httpsData);  //Serial.println(output);

      
}  

*/

power_state_t get_power_state(float voltage) {
    if (voltage <= 11.0f) return POWER_CRITICAL;
    if (voltage <= 11.8f) return POWER_LOW;
    if (voltage <= 12.5f) return POWER_MODERATE;
    return POWER_EXCELLENT;
}

void MonitorBattery() {
    uint16_t readBattery = 0;
    float summation = 0.00f;

    for (int i = 0; i < 10; i++) {
        readBattery = analogRead(batteryPin);
        summation += float(readBattery);
        delay(2);
    }
    
    summation /= 10.0f;
    voltage = no_of_batts * 14.86 * (summation / 3095.0f);
    
    dtostrf(voltage, 5, 1, voltage_string);
    strncat(voltage_string, "V", sizeof(voltage_string) - strlen(voltage_string) - 1);

    Serial.println(voltage_string);

    // Update power state
    power_state_t new_power_state = get_power_state(voltage);
    if (new_power_state != current_power) {
        Serial.printf("Power state changed: %d -> %d (Voltage: %.1fV)\n", 
                     current_power, new_power_state, voltage);
        current_power = new_power_state;
        //update_power_settings();
    }
}

void update_power_settings() {
    switch (current_power) {
        case POWER_CRITICAL:
            read_frequency = 120ULL * 60ULL * 1000ULL; // 120 minutes
            break;
        case POWER_LOW:
            read_frequency = 30ULL * 60ULL * 1000ULL;  // 30 minutes
            break;
        case POWER_MODERATE:
            read_frequency = 10ULL * 60ULL * 1000ULL;  // 10 minutes
            break;
        case POWER_EXCELLENT:
            read_frequency = 10ULL * 1000ULL;          // 10 seconds
            break;
    }
    
    Serial.printf("Power mode %d: Read frequency = %.1f minutes\n", 
                 current_power, (float)read_frequency / 60000.0f);
}

//  Serial.print("Inside Temp: "); Serial.print(cabin_temperature);  Serial.print("\tFAN: ");



void monitor_box_conditions() {

    // --- Read internal temperature ---
    cabin_temperature = real_time.getTemperature();

    // ---------- FAN CONTROL (with hysteresis) ----------
    if (cabin_temperature >= 29.0f) { // turn fan ON
        digitalWrite(innerFAN, HIGH);
        innerFan_ON = true;
        buzzer.beep(1, 50, 0);
    }
    else if (cabin_temperature <= 25.0f) { // turn fan OFF
        digitalWrite(innerFAN, LOW);
        innerFan_ON = false;
       // buzzer.beep(1, 50, 0);
    }

    // ---------- NIGHT LIGHT CONTROL ----------
    // ON from 8:00 → 07:00
    bool isNight = (hr >= 20 || hr < 7);

    if (isNight) { // turn light ON
        digitalWrite(night_Light, HIGH);
        night_Light_ON = true;
        buzzer.beep(1, 100, 0);
    }
    else if (!isNight) { // turn light OFF
        digitalWrite(night_Light, LOW);
        night_Light_ON = false;
        //buzzer.beep(2, 100, 100);
    }

    // ---------- STATUS LOG ----------
    snprintf(
                internals_log,
                sizeof(internals_log),
                "Box Temp: %.1fC | Fan: %s | Night Light: %s",
                cabin_temperature,
                innerFan_ON ? "ON" : "OFF",
                night_Light_ON ? "ON" : "OFF"
           );

    //Serial.println(internals_log);
    
}




// --- Flash pattern generator ---
void flash(uint64_t flash_time, uint8_t heartbeat,
           uint16_t ON_1, uint16_t OFF_1,
           uint16_t ON_2, uint16_t OFF_2) {
    
    // Make these static to persist between calls for each LED
    static uint64_t previe = 0;
    static uint8_t phase = 0;
    static int k = 1;
    
    if(otaStarted) { 
        Serial.println(k); 
        k++; 
    }
    
    uint64_t interval = 0;
    switch (phase) {
        case 0: interval = ON_1; break;
        case 1: interval = OFF_1; break;
        case 2: interval = ON_2; break;
        case 3: interval = OFF_2; break;
    }

    if ((flash_time - previe) >= interval) {
        previe = flash_time;
        phase = (phase + 1) % 4;
        bool ledState = (phase == 0 || phase == 2);
        digitalWrite(heartbeat, ledState ? HIGH : LOW);
    }
}


uint8_t wifi_connect_attempts = 0;
//DRAM_ATTR char wifi_log[200]; // force globals into RAM instead of flash

char IP_Addr[50] = "NULL";


#define WIFI_SWITCH_TIMEOUT_MS 10000
#define WIFI_RECONNECT_DELAY_MS 500

bool switch_radio_to_wifi() {
  
  // WiFi initialization with timeout
  Serial.println("📶 Initializing WiFi...");
  unsigned long startTime = now_now_ms;
  
  WiFi.disconnect(true, true); // Full cleanup
  delay(WIFI_RECONNECT_DELAY_MS);
  WiFi.mode(WIFI_OFF);
  delay(WIFI_RECONNECT_DELAY_MS);
  
  // Set WiFi mode before initialization
  WiFi.mode(WIFI_STA);
  Serial.printf("📊 WiFi channel: %d\n", WiFi.channel());
  
  // Initialize WiFi with timeout protection
  bool wifiSuccess = false;
 
    while (now_now_ms - startTime < WIFI_SWITCH_TIMEOUT_MS) {
    if (wifi_obj.initialize_ESP_WiFi(devicename) == WIFI_MGR_SUCCESS) {
        wifiSuccess = true;
        break;
    }
    
    Serial.printf("⏳ WiFi connection failed, retrying... (%lu ms elapsed)\n", 
                 now_now_ms - startTime);
    delay(1000);
    
    Serial.printf("⏳ WiFi connection failed, retrying... (%lu ms elapsed)\n", 
                 now_now_ms - startTime);
    delay(1000);
  }
  
  if (!wifiSuccess) {
    Serial.println("💥 Failed to initialize WiFi within timeout period");
    
    // Fallback attempt - try with different approach
    Serial.println("🔄 Attempting fallback WiFi initialization...");
    WiFi.begin(); // Try simple connection
    
    unsigned long fallbackStart = now_now_ms;
    while ((WiFi.status() != WL_CONNECTED) && ((now_now_ms - fallbackStart) < 5000)) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ Fallback WiFi connection successful");
      wifiSuccess = true;
    } else {
      Serial.println("\n💥 Fallback WiFi also failed");
      return false;
    }
  }
  
  // Initialize OTA if WiFi is successful
  if (wifiSuccess) {
    Serial.println("🔌 Initializing OTA updates...");
    initializeOTA();
    
    // Verify OTA is ready
    Serial.printf("🎯 WiFi Mode Activated - IP: %s, RSSI: %d dBm\n", 
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  
  return false;
}

// Add dynamic sleep based on conditions
void optimize_power_consumption() {
    // Reduce update frequency during night/low activity
    bool is_night = (hr > 20 || hr < 6);
    bool low_activity = (average_temp < 18.0 && average_humi > 60.0); // go to idle at night
    
    if (is_night && low_activity) {
      read_frequency = 30ULL * 60ULL * 1000ULL;
      fans_are_togglable = false;
        // Extend reading intervals
        // Consider deep sleep between readings
    }
}

void initialize_minimal_systems() {
    // Quick initialization after light sleep
    if (current_power >= POWER_MODERATE) {
        wifi_connected = wifi_obj.ensure_wifi();
        if (wifi_connected) {
            initializeOTA();
        }
    }
}

void LowPower_Screen() {
    // Simple low-power display screen
    LCD.firstPage();
    do {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setFont(&FreeSans9pt7b);
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(50, 100);
        LCD.print("LOW POWER MODE");
        LCD.setCursor(30, 130);
        LCD.print("Sleeping...");
        LCD.setCursor(40, 160);
        LCD.print(voltage_string);
    } while (LCD.nextPage());
}


void initializeOTA() {
    ArduinoOTA.setHostname(devicename);
    ArduinoOTA.setPassword(OTA_PASS);

    ArduinoOTA
        .onStart([]() { otaStarted = true; }) 
        .onEnd([]() { otaFinished = true;}) 
        .onProgress([](unsigned int progress, unsigned int total) {
           // snprintf(ota_log, sizeof(ota_log), "Progress: %u%%", (progress * 100) / total);
           // flash(now_now_ms, blinker, 50, 50, 0, 0);
        })
        .onError([](ota_error_t error) { otaError = true;
            snprintf(ota_log, sizeof(ota_log), "Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                strcat(ota_log, "Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                strcat(ota_log, "Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                strcat(ota_log, "Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                strcat(ota_log, "Receive Failed");
            else if (error == OTA_END_ERROR)
                strcat(ota_log, "End Failed");
        });

    ArduinoOTA.begin();
    //webLog("OTA Ready!");
}






/*
float total_bmp_readings = 0.00;
float total_humi_readings = 0.00;

uint8_t which_temperature = 0;

int i = 0;
void read_temperatures(){
    
  //  for(int i = 0; i < 10; i++){
    while(i < 10){
      cumulated_temp_readings[i] =  bmp.readTemperature();
      if(cumulated_temp_readings[i] >= 5.0) {total_bmp_readings += cumulated_temp_readings[i]; i++; }
   //   delay(5000); // instead take other readings and do other things in the 5 second gap
    }

        temp1 = total_bmp_readings/(i);
 
        average_temp = 0.00;
        temp1 = bmp.readTemperature(); atm_pressure = bmp.readPressure(); elevation = bmp.readAltitude(1013.25);
        temp2 = dht.readTemperature(); humidity = dht.readHumidity(); temp_fara = dht.readTemperature(true); // falanahait

        heat_index = dht.computeHeatIndex(temp2, humidity, false);

        if(temp1 > 0.0 && temp2 > 0.0){ // so we don't get half a value when one is zero
            average_temp = (temp1 + temp2)/2.0;
             dtostrf(average_temp, 3, 1, temp_str); // strcat(temp_str, "C"); 27.8
             dtostrf(humidity, 3, 1, humi_str);
            which_temperature = 1;
        }

        else { // if any iz zero or both
        //if both give zero... impossible here in Africa.... 
          if(temp1 <= 0.0 && temp2 <= 0.0) { 
                strcpy(temp_str_log, "Error Taking Temperature Readings");
                strcpy(humi_str_log, "Error Taking Humidity Readings");
                which_temperature = 6;
             }

          else{


              if(temp1 <= 0.00 || isnan(temp1)) average_temp = temp2; // take the non zero
              if(temp2 <= 0.00 || isnan(temp2)) average_temp = temp1; // take the non zero
                  which_temperature = 2;
              if(temp1 - temp2 >= 10.0){ // too high a disparity
                  average_temp = temp2;  // take the lower  
                  which_temperature = 3;
              }
              if(temp2 - temp1 >= 10.0){ // too high a disparity
                  average_temp = temp1; // take the lower
                  which_temperature = 4; 
              }

          dtostrf(average_temp, 3, 1, temp_str);
          dtostrf(humidity, 3, 1, humi_str);

             } // both are not is_nan
        } // 


 
    Serial.print("Temperature Order: ");    Serial.println(which_temperature);
    Serial.print(F("Temperature 1 = "));    Serial.print(temp1, 2); Serial.println(" °C ");
    Serial.print(F("Temperature 2 = "));    Serial.print(temp2, 3);    Serial.println(F("°C "));
    Serial.print(F("AVG TEMP: "));  Serial.print(temp_str);  Serial.println(F("°C ")); Serial.println();

    Serial.print(F("Pressure = "));    Serial.print(atm_pressure);    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));    Serial.print(elevation);    Serial.println(" m");


    Serial.print(F("Humidity: "));  Serial.println(humidity);
 
    //Serial.print(F("Heat index: "));  Serial.print(heat_index);  Serial.print(F("°C "));


      Serial.println();

}

*/



void initiate_bmp280(){
  
        Serial.println(F("BMP280 test"));
        unsigned status;
        //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
        status = bmp.begin();
        if (!status) {
          Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                            "try a different address!"));
          Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
          Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
          Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
          Serial.print("        ID of 0x60 represents a BME 280.\n");
          Serial.print("        ID of 0x61 represents a BME 680.\n");
          //while (1) delay(10);
        }

        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
  

/*
void enforce_bus_quiet() {
    unsigned long t0 = millis();
    while (millis() - t0 < mbus.busQuietTime) {
        while (Serial2.available()) {
            Serial2.read();   // drain EVERY late byte
            t0 = millis();    // restart timer when any byte arrives
        }
    }
}
*/








uint64_t successful_uploads = 0;
char successful_uploads_c[20] = ""; // 18




/*
void upload_to_web_of_iot(){ // (no leak, better status handling)
  if (!wifi_connected) { Serial.println("Skip POST: WiFi down"); return; }
  if (!hasFreshJson) { Serial.println("Skip POST: no fresh JSON"); return; }

  Serial.printf("\tPOST JSON => %s\n\tTo: %s\n", httpsData, serverName);

  WiFiClientSecure client;   // no heap leak
  client.setInsecure();      // accept all certs (consider proper CA in production)

  HTTPClient https;
  if (https.begin(client, serverName)) {
    https.setTimeout(15000); // 15s network timeout
    https.addHeader("Content-Type", "application/json");

    int httpResponse = https.POST((uint8_t*)httpsData, strlen(httpsData));
    Serial.printf("HTTP Response: %d (%s)\n",
                  httpResponse, https.errorToString(httpResponse).c_str());

    if (httpResponse > 0 && (httpResponse/100) == 2) {
      Serial.println("Uploaded Successfully");
      // Optionally: String body = https.getString(); Serial.println(body);
    } else {
      Serial.println("Upload Failed");
    }
    https.end();
  } else {
    Serial.println("HTTP initialization failed!");
  }
  hasFreshJson = false; // mark consumed
}
*/

void upload_to_web_of_iot(){
   //  Serial.print("\tData to send via HTTP POST => ");    Serial.println(dataPack);    Serial.print("\tTo: "); Serial.println(serverName); 
   
    digitalWrite(server_led, HIGH);
    
    // WiFiClientSecure client;
    //client.setInsecure();
    //HTTPClient https;
   //std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
    // Ignore SSL certificate validation
    //client->setInsecure();
    //setInsecure(); 
    
     //if(WiFi.status()== WL_CONNECTED){
      WiFiClientSecure *client = new WiFiClientSecure; // must delete it once you’re done with the HTTP transaction to avoid a memory leak.
      client->setInsecure(); //don't use SSL certificate
    
      HTTPClient https;
 
      Serial.println("Initializing HTTP...");
      
    //  bool initialized = https.begin(serverName); 
        bool initialized = https.begin(*client, serverName); // Your Domain name with URL path or IP address with path

        if(initialized){ 
           
              Serial.println("HTTP initialized Successfully!"); //Serial.println("Adding HTTP Header: \'Application/x-www-form-urlencoded'");
      // Specify content-type header
              https.addHeader("Content-Type", "application/json");  
              delay(50);
      
      // Send HTTP POST request
             int httpResponse = https.POST(httpsData);
             delay(150);
            
            Serial.printf("HTTP Response: %d (%s)\n", httpResponse, https.errorToString(httpResponse).c_str());  Serial.println();
            if(httpResponse == 200){
              Serial.println("Uploaded Successfully, YeeeeY!"); Serial.println();
            }
            else if(httpResponse >= 600){
              Serial.println("Failed to upload to server!"); Serial.println();
            }

            else if(httpResponse <= 0){
              Serial.println("Insane Response!"); Serial.println("Failed to upload to server!");

            }
            else {
              Serial.println("rESPONSE FUNNy!"); Serial.println("Failed to upload to server!");
            }

        }
        else Serial.println("HTTPS initialization failed!");
             
    
        Serial.println("Ending HTTP");
      // Free resources
      https.end();
          
    // now free the heap allocation
    delete client;
    client = nullptr;  // optional safety a good habit to avoid dangling pointers.
    //}
    //NOTE: If you repeatedly create/destroy clients in loop(), 
    //this will fragment the ESP32 heap over time. 
    //Stack allocation (WiFiClientSecure client;) is usually safer and lighter on small devices.
    
    //lastTime = millis();
  //}

  digitalWrite(server_led, LOW);
}





void safety(){
        // temperature measurements
        // prolonged operations --- 3 hours timeout
        //


}


char SleepPrompt[200];

void sleep_dynamically() {
    // Add activity-based optimization
    bool is_night = (hr > 20 || hr < 6);
    bool low_activity = (average_temp < 18.0 && average_humi > 60.0); // night time: low temp, high humidity
    
    // Extend sleep during low activity periods
    if (is_night && low_activity && current_power != POWER_EXCELLENT) {
        dynamic_interval *= 2; // Double sleep time
        snprintf(SleepPrompt, sizeof(SleepPrompt), 
                "Low activity period: Extended sleep by 2x");
        Serial.println(SleepPrompt);
    }

    switch (current_power) {
        case POWER_CRITICAL: {
            dynamic_interval = power_critical_sleep_duration;
            Serial.println("\n=== ENTERING CRITICAL POWER MODE ===");
            Serial.printf("Deep Sleep: %.1f hours\n", dynamic_interval / 3.6e9);

            // Graceful shutdown sequence
            digitalWrite(sensorsRelay, HIGH); // Turn off sensor power
            digitalWrite(innerFAN, LOW);
            
            // Update display for shutdown
            currentScreen = -1;
            LowPower_Screen();
            LCD.hibernate();
            delay(100); // Ensure SPI transactions complete
            
            // Network teardown
            if (wifi_connected) {
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
                Serial.println("WiFi powered down for deep sleep");
            }

            // Configure wake sources
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            esp_sleep_enable_timer_wakeup(dynamic_interval);
            
            Serial.println("→ Entering deep sleep...");
            Serial.flush();
            delay(100);
            
            esp_deep_sleep_start();
            break;
        }

        case POWER_LOW: {
            dynamic_interval = power_low_sleep_duration;
            enter_light_sleep(dynamic_interval, "LOW POWER: 15min light sleep");
            break;
        }

        case POWER_MODERATE: {
            dynamic_interval = power_mod_sleep_duration;
            enter_light_sleep(dynamic_interval, "MODERATE POWER: 5min light sleep");
            break;
        }

        case POWER_EXCELLENT: {
            // No sleep, just delay
            dynamic_interval = power_excellent_delay;
            Serial.printf("Excellent power: Delaying %.1f seconds\n", 
                         dynamic_interval / 1000.0f);
            delay(dynamic_interval);
            break;
        }
    }
}

void enter_light_sleep(uint64_t sleep_time, const char* mode) {
    Serial.printf("%s - %.1f minutes\n", mode, (float)sleep_time / 6e7);
    
    // Light sleep preparation
    if (wifi_connected) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        esp_wifi_stop();
    }

    // Light sleep preserves RAM, so we don't need full reinitialization
    esp_sleep_enable_timer_wakeup(sleep_time);
    Serial.println("→ Entering light sleep...");
    
    esp_light_sleep_start();
    
    // Upon waking
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    Serial.printf("Woke up (cause=%d). Reinitializing systems...\n", wakeCause);
    
    // Quick reinitialization for light sleep
    initialize_minimal_systems();
}


bool clock_is_working = false;

void initialize_RTC(){
  uint8_t trial = 0;

  while(trial < 10){

      clock_is_working = real_time.begin();
      if(clock_is_working) { Serial.println("RTC FOUND!"); break; }

    trial++; delay(100);
  }

  if(!clock_is_working)  { Serial.println("RTC not found"); return; }

  else {
      if (real_time.lostPower()) {
                Serial.println("RTC lost power, let's set the time!"); // When time needs to be set on a new device, or after a power loss, the
                
                real_time.adjust(DateTime(F(__DATE__), F(__TIME__)));  // This line sets the RTC with an explicit date & time, for example to set 
                // rtc.adjust(DateTime(2025, 9, 4, 3, 0, 0));
                
                Serial.println("Clock Started and time set!");
        }

    }

        //   real_time.adjust(DateTime(2025, 12, 18, 13, 55, 0)); // ONLY SET TIME ONCE

}






void query_rtc(){ //Serial.println("Time Check!"); Serial.println();
    char root[5] = "th";
    char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
    char Moonth[12][12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
    char short_Month[12][10] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};

    char hr_str[5] = ""; char min_str[5] = ""; char sec_str[5] = ""; 
    char day_str[15] = ""; char date_str[10] = ""; 
    char mth_str[15] = ""; char short_mth_str[12] = ""; char yr_str[7] = ""; 
    char zero_holder[4] = "0"; char bucket[4] = "";

    uint8_t datey = 0; uint8_t reminder = 0; // for correcting 24 hour clock

  // if(running_ticks%10 == 0){ //query for time once every 10 seconds
  if(clock_is_working){
            DateTime time_now = real_time.now();

              hr = time_now.hour(); mint = time_now.minute();  sec = time_now.second();
              datey = time_now.day(); mth = time_now.month();  mwaka = time_now.year(); day_ = time_now.dayOfTheWeek();
              cabin_temperature = real_time.getTemperature();
            if(hr > 24){ 
                // DateTime now = real_time.now();
              // while(recursive_counter < 5){ query_rtc(); recursive_counter++;}

                hr = 24; mint = 59; sec = 59;  /*beep(2);*/ Serial.println("Time Chip Failed!");

              }
                  reminder = (hr%12);
                if(hr<=12) itoa(hr, hr_str, 10);  // 00: - 12:
                else   itoa(reminder, hr_str, 10); // 13: - 23:
                itoa(mint, min_str, 10); 
                itoa(sec, sec_str, 10);  
                itoa(datey, date_str, 10); 
                strncpy(day_str, daysOfTheWeek[day_], sizeof(day_str));
                strncpy(mth_str, Moonth[mth-1], sizeof(mth_str));  
                strncpy(short_mth_str, short_Month[mth-1], sizeof(short_mth_str)); 
                itoa(mwaka, yr_str, 10);
  
              //construct the time sequences
        if(hr<=9){  strcpy(ShortTime, zero_holder); strcat(ShortTime, hr_str);  } 
        else {
             strcpy(ShortTime, hr_str); 
        }
        strcat(ShortTime, ":"); 
        if(mint<=9){strcat(ShortTime, zero_holder);} 
                    strcat(ShortTime, min_str); // HH:MM

                    strcpy(SystemTime, ShortTime);  strcat(SystemTime, ":"); strcat(SystemTime, sec_str); // HH:MM:SS

          
          strcpy(ShortTime_am_pm, ShortTime);
          strcat(ShortTime_am_pm, (hr<12)?" am":" pm");


          //construct the date sequence
          strcpy(SystemDate, day_str); strcat(SystemDate, " "); strcat(SystemDate, date_str);
          strcpy(ShortDate, date_str); strcat(ShortDate, "-"); strcat(ShortDate, short_mth_str); strcat(ShortDate, "-"); strcat(ShortDate, yr_str);
        
         if(datey == 1 || datey == 21 || datey == 31) strcpy(root, "st");
         if(datey == 2 || datey == 22 ) strcpy(root, "nd");
         if(datey == 3 || datey == 23 ) strcpy(root, "rd");
         else strcpy(root, "th");
         strcat(SystemDate, root); strcat(SystemDate, " ");
         strcat(SystemDate, mth_str); strcat(SystemDate, ", "); strcat(SystemDate, yr_str);
  
                  Serial.println();
                  Serial.print("Internal Temperature: "); Serial.println(cabin_temperature);
                  Serial.print("Short Time: ");  Serial.println(ShortTime);
                  Serial.print("Short Time AM/PM: ");  Serial.println(ShortTime_am_pm);
                  Serial.print("Full System Time: "); Serial.print(SystemTime); 
                  Serial.print("\tSystem Date: "); Serial.println(SystemDate); Serial.println();
                  Serial.println();
  
       //  if (minute == 0 || minute == 10 || minute == 20 || minute == 30 || minute == 40 || minute == 50){ 
       if (mint % 10 == 0){ // every 10 mins
            if(!data_sent) can_send = true;  // trigger sending once
            else can_send = false; // already sent in this minute
            } 
      else data_sent = false; // reset for the next 10-min cycle
        
       } // if clock is working

       else {
           if(now_now_ms - last_upload_time_ms >= upload_fequency){ // 
            can_send = true;
           // if(!data_sent) can_send = true;
          //  else can_send = false;
            last_upload_time_ms = now_now_ms; 
          }
       }


}





void update_display(){
  
  if(currentScreen == 1) homepage();
  if(currentScreen == 2) dataPage();

  LCD.hibernate();

}


void dataPage(){
  
}





const char Manufacturer[] = "IntelliSys UG";
const char DeviceID[] = "Dryer Monitoring System";
const char Parameter[] = "Susan M PhD Project, MUARIK";

void Boot(){

  LCD.setRotation(2);
  LCD.setFont(&FreeSans12pt7b);
  LCD.setTextColor(GxEPD_BLACK);

  int16_t tbx, tby; uint16_t tbw, tbh;
  LCD.getTextBounds(Manufacturer, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t y = ((LCD.height() - tbh) / 2) - tby;
  LCD.setFullWindow();

  LCD.firstPage(); //FULLY CENTERED TEXTS
  do{
  
       //  LCD.fillScreen(GxEPD_BLACK); delay(2000);
      //   LCD.fillScreen(GxEPD_RED); delay(2000);
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setFont(&FreeSansBold24pt7b);

                LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

        LCD.setCursor(50, 100);    
        
        LCD.print(Manufacturer);

        delay(3000);

        LCD.setFont(&FreeMonoBold9pt7b);
        LCD.getTextBounds(DeviceID, 0, 0, &tbx, &tby, &tbw, &tbh);

        x = ((LCD.width() - tbw) / 2) - tbx;
        LCD.setCursor(x, y+tbh);    LCD.print(DeviceID);
        
        LCD.setCursor(70, 200); LCD.print(Parameter);

        delay(1000);

    }  while (LCD.nextPage());
}


uint16_t box_width  = 150;
uint16_t box_height = 95;

bool is_home = false;

// ---- Layout constants ----
static const uint16_t SCREEN_W = 400;
static const uint16_t SCREEN_H = 300;

static const uint16_t HEADER_H = 27;
static const uint16_t BOX_RADIUS = 10;

static const uint16_t LEFT_X = 30;
static const uint16_t RIGHT_X = LEFT_X + box_width + 30;

static const uint16_t TOP_ROW_Y = 40;
static const uint16_t BOTTOM_ROW_Y = 150;

// ---- Helper to draw a standard info box ----
void drawInfoBox(
  uint16_t x, uint16_t y,
  const char* title,
  uint16_t borderColor,
  uint16_t fillColor
) {
  LCD.drawRoundRect(x, y, box_width, box_height, BOX_RADIUS, borderColor);
  LCD.fillRoundRect(x, y, box_width, 20, BOX_RADIUS, fillColor);
  LCD.fillRect(x, y + 10, box_width, 12, fillColor);

  LCD.setFont(&FreeSans9pt7b);
  LCD.setTextColor(GxEPD_WHITE);
  LCD.setCursor(x + 10, y + 18);
  LCD.print(title);
}

float temp[12];
float humi[12];
uint32_t entry_count = 0;

char time_stamp[12][10];

// ---- Data table layout ----
static const uint16_t TABLE_X = 20;
static const uint16_t TABLE_Y = 40;
static const uint16_t TABLE_W = 360;
static const uint16_t TABLE_H = 230;

static const uint16_t ROW_H = 24;
static const uint8_t  MAX_ROWS = 8;

// Column widths
static const uint16_t COL_NUM_W  = 30;
static const uint16_t COL_TEMP_W = 95;
static const uint16_t COL_HUMI_W = 95;
static const uint16_t COL_TIME_W = 140;


void drawTableFrame() {

  uint16_t headerColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

  // Outer border
  LCD.drawRect(TABLE_X, TABLE_Y, TABLE_W, TABLE_H, GxEPD_BLACK);

  // Header ribbon
  LCD.fillRect(TABLE_X, TABLE_Y, TABLE_W, ROW_H, headerColor);

  // Vertical lines
  uint16_t x = TABLE_X + COL_NUM_W;
  LCD.drawLine(x, TABLE_Y, x, TABLE_Y + TABLE_H, GxEPD_BLACK);

  x += COL_TEMP_W;
  LCD.drawLine(x, TABLE_Y, x, TABLE_Y + TABLE_H, GxEPD_BLACK);

  x += COL_HUMI_W;
  LCD.drawLine(x, TABLE_Y, x, TABLE_Y + TABLE_H, GxEPD_BLACK);

  // Horizontal lines
  for (uint16_t y = TABLE_Y + ROW_H; y < TABLE_Y + TABLE_H; y += ROW_H) {
    LCD.drawLine(TABLE_X, y, TABLE_X + TABLE_W, y, GxEPD_BLACK);
  }

  // ---- Column titles ----
  LCD.setFont(&FreeSans9pt7b);
  LCD.setTextColor(GxEPD_WHITE);

  LCD.setCursor(TABLE_X + 8, TABLE_Y + 17);
  LCD.print("#");

  LCD.setCursor(TABLE_X + COL_NUM_W + 12, TABLE_Y + 17);
  LCD.print("Avg Temp");

  LCD.setCursor(TABLE_X + COL_NUM_W + COL_TEMP_W + 12, TABLE_Y + 17);
  LCD.print("Avg Humi");

  LCD.setCursor(TABLE_X + COL_NUM_W + COL_TEMP_W + COL_HUMI_W + 20, TABLE_Y + 17);
  LCD.print("Time");
}


// ---- Settings page layout ----
static const uint16_t CIRCLE_R = 55;
static const uint16_t CIRCLE_Y = 135;

static const uint16_t CIRCLE1_X = 120;
static const uint16_t CIRCLE2_X = 280;

void drawFanIcon(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color) {

  // Outer housing
  LCD.drawCircle(cx, cy, r, color);
  LCD.drawCircle(cx, cy, r - 2, color);

  // Fan blades (triangles)
  LCD.fillTriangle(cx, cy - 10, cx - 8, cy - r + 15, cx + 8, cy - r + 15, color); // top
  LCD.fillTriangle(cx + 10, cy, cx + r - 15, cy - 8, cx + r - 15, cy + 8, color); // right
  LCD.fillTriangle(cx, cy + 10, cx - 8, cy + r - 15, cx + 8, cy + r - 15, color); // bottom
  LCD.fillTriangle(cx - 10, cy, cx - r + 15, cy - 8, cx - r + 15, cy + 8, color); // left
}


// ---- Switch layout (compact) ----
static const uint16_t SWITCH_W = 70;
static const uint16_t SWITCH_H = 28;
static const uint16_t SWITCH_R = 14;
void drawSwitch(uint16_t x, uint16_t y, bool isOn) {

  uint16_t accentColor = (LCD.epd2.hasColor && isOn) ? GxEPD_RED : GxEPD_BLACK;

  // Outer shell
  LCD.fillRoundRect(x, y, SWITCH_W, SWITCH_H, SWITCH_R, GxEPD_BLACK);

  // Inner body
  LCD.fillRoundRect(
    x + 2, y + 2,
    SWITCH_W - 4, SWITCH_H - 4,
    SWITCH_R - 2,
    GxEPD_WHITE
  );

  // Knob
  uint16_t knobRadius = SWITCH_R - 4;
  uint16_t knobX = isOn
      ? (x + SWITCH_W - SWITCH_R)
      : (x + SWITCH_R);

  LCD.fillCircle(knobX, y + SWITCH_H / 2, knobRadius, accentColor);

  // Text
  LCD.setFont(&FreeSansBold9pt7b);
  LCD.setTextColor(GxEPD_BLACK);

  if (isOn) {
    LCD.setCursor(x + 8, y + SWITCH_H - 7);
    LCD.print("ON");
  } else {
    LCD.setCursor(x + SWITCH_W - 28, y + SWITCH_H - 7);
    LCD.print("OFF");
  }
}



void homepage() {

  LCD.firstPage();
  do {

    LCD.fillScreen(GxEPD_WHITE);
    LCD.drawRect(0, 0, SCREEN_W, SCREEN_H, GxEPD_BLACK);
    LCD.drawRect(1, 1, SCREEN_W - 2, SCREEN_H - 2, GxEPD_BLACK);

    if (side_scroll == 1) { // the sensors page

           

 // VERTICAL SCROLL BAR
        LCD.drawRoundRect(370, 50, 20, 160, 5, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
        LCD.fillRoundRect(375, 60, 10, 80, 12, GxEPD_BLACK);

        LCD.fillTriangle(370, 45, 380, 35, 390, 45, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // UPPER ARROW
        LCD.drawTriangle(370, 215, 380, 225, 390, 215, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // DOWN ARROE
        
      // ---- Header ----
      uint16_t headerColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
      LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, headerColor);
      LCD.fillRoundRect(8, 2, 100, 30, 5, GxEPD_WHITE);

      LCD.setFont(&FreeSans12pt7b);
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setCursor(14, 22);
      LCD.print("Sensors");

      LCD.setFont(&FreeSans9pt7b);
      LCD.setTextColor(GxEPD_WHITE);
      LCD.setCursor(140, 19);
      LCD.print("Data Table");

      LCD.fillRect(250, 5, 2, 20, GxEPD_WHITE);

      LCD.setCursor(290, 19);
      LCD.print("Settings");

      // ---- Average Temperature ----
      drawInfoBox(LEFT_X, TOP_ROW_Y, "Av. Temperature", GxEPD_BLACK, GxEPD_BLACK);

      //DYNAMICALLY RESIZABLE BUBBLE
          if(active_sensors<99){ LCD.fillCircle((25+box_width), 42,  12, GxEPD_RED); }
          else { // should hold 3-5 digits: 100-99,999
                LCD.fillRoundRect((15+box_width), 30,  36, 20, 20, GxEPD_RED); // notification bubble
          } 

         LCD.setFont(); LCD.setTextColor(GxEPD_WHITE); 
         LCD.setCursor((20+box_width), 37); LCD.print(active_sensors);

        LCD.setFont(&FreeMono9pt7b); LCD.setTextColor(GxEPD_BLACK); //FreeMono9pt7b
        LCD.setCursor(60, 128); LCD.print(sensor_1_last_seen);

      
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setFont(&FreeMono12pt7b); LCD.setCursor(142, 80); LCD.print("o");
      LCD.setFont(&FreeSans12pt7b); LCD.setCursor(155, 90); LCD.print("C");

       LCD.setFont(&FreeSansBold24pt7b); LCD.setTextColor(GxEPD_BLACK);   
       LCD.setCursor(50, 110);      LCD.print(average_temp);




      // ---- Humidity Box ----
      drawInfoBox(RIGHT_X, TOP_ROW_Y, "Dryer Humidity", GxEPD_BLACK, GxEPD_BLACK);

      LCD.setFont(&FreeMono12pt7b);
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setCursor(RIGHT_X + 120, 100);      LCD.print("%");

      LCD.setTextColor(GxEPD_WHITE);
       //  LCD.setCursor(((10+box_width)+(1.2*box_width)), 37);          LCD.print(average_humi);

         LCD.setTextColor(GxEPD_BLACK);      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(50+((1.2*box_width)), 110);    LCD.print(average_humi); 

         

          LCD.setFont(&FreeMono9pt7b);  LCD.setTextColor(GxEPD_BLACK); //FreeMono9pt7b
      //LCD.setCursor(85+(30+(1.2*box_width)), 85); LCD.print("%");

        LCD.setCursor(25+(30+(1.2*box_width)), 128); LCD.print(sensor_2_last_seen); 
      
        //DATA
         LCD.setFont(); LCD.setCursor((40+(1.2*box_width)), 66); 
        // LCD.print("PERCENT VOLUME");
         //LCD.print(sensor2_POWER_XTIX);  //LCD.print("PERCENT VOLUME"); // wind speed log
          
         LCD.setTextColor(GxEPD_WHITE);
         LCD.setCursor(((10+box_width)+(1.2*box_width)), 37); 
         LCD.print("snds");


      // ---- Suction Fans ----
      uint16_t fanColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
      drawInfoBox(LEFT_X, BOTTOM_ROW_Y, "Suction Fans", fanColor, fanColor);

     // LCD.setFont(&FreeMonoBold12pt7b);
      LCD.setTextColor(fanColor);
    //  LCD.setCursor(LEFT_X + 60, BOTTOM_ROW_Y + 45);
     

    //   LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(60, (215)); LCD.print(humi_fan_on ? "ON" : "OFF"); 
         LCD.setFont(&FreeMono9pt7b); 
         LCD.setCursor(50, (232)); LCD.print(humi_fan_stop_time_str);
      // ---- Cooling Fans ----
      drawInfoBox(RIGHT_X, BOTTOM_ROW_Y, "Cooling Fans", fanColor, fanColor);

      LCD.setTextColor(fanColor);
      LCD.setCursor(RIGHT_X + 60, BOTTOM_ROW_Y + 45);
      

      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(20+(60+box_width), (215));    LCD.print(cooling_fan_on ? "ON" : "OFF");

         LCD.setFont(&FreeMono9pt7b); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(90+(box_width), (232));  LCD.print(cooling_fan_stop_time_str);
        

    } // side_scroll == 1

    else if(side_scroll == 2){ // the data table page
        // draw a table with black lines: and a red top ribbon
        // col 1: #
        // col 2: Avg Temp
        // col 3: Avg Humi
        // col 4: Time

    // ---- Header ----
  uint16_t headerColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
  LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, headerColor);
  LCD.fillRoundRect(120, 2, 130, 30, 5, GxEPD_WHITE);

  LCD.setFont(&FreeSans12pt7b);
  LCD.setTextColor(GxEPD_BLACK);
  LCD.setCursor(130, 22);
  LCD.print("Data Table");

  LCD.setFont(&FreeSans9pt7b);
  LCD.setTextColor(GxEPD_WHITE);
  LCD.setCursor(20, 19);
  LCD.print("Sensors");

  LCD.fillRect(250, 5, 2, 20, GxEPD_WHITE);

  LCD.setCursor(290, 19);
  LCD.print("Settings");

  // ---- Table ----
  drawTableFrame();

  // ---- Rows ----
  LCD.setFont(&FreeMono9pt7b);
  LCD.setTextColor(GxEPD_BLACK);

 // uint8_t visible_rows = min(entry_count, MAX_ROWS);
  uint8_t visible_rows = min((uint8_t)entry_count, MAX_ROWS);


  for (uint8_t i = 0; i < visible_rows; i++) {

    uint16_t y = TABLE_Y + ROW_H * (i + 1) + 16;

    // #
    LCD.setCursor(TABLE_X + 8, y);
    LCD.print(i + 1);

    // Avg Temp
    LCD.setCursor(TABLE_X + COL_NUM_W + 10, y);
    LCD.print(temp[i]);
    LCD.print("C");

    // Avg Humidity
    LCD.setCursor(TABLE_X + COL_NUM_W + COL_TEMP_W + 10, y);
    LCD.print(humi[i]);
    LCD.print("%");

    // Time
    LCD.setCursor(TABLE_X + COL_NUM_W + COL_TEMP_W + COL_HUMI_W + 10, y);
    LCD.print(time_stamp[0][i]);
      }


    }

    else { // ---- side_scroll == 3 :: Settings Page ----

  uint16_t accentColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

  // ---- Header ----
  LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, accentColor);
  LCD.fillRoundRect(260, 2, 120, 30, 5, GxEPD_WHITE);

  LCD.setFont(&FreeSans12pt7b);
  LCD.setTextColor(GxEPD_BLACK);
  LCD.setCursor(270, 22);
  LCD.print("Settings");

  LCD.setFont(&FreeSans9pt7b);
  LCD.setTextColor(GxEPD_WHITE);
  LCD.setCursor(20, 19);
  LCD.print("Sensors");

  LCD.setCursor(140, 19);
  LCD.print("Data Table");

  // ---- Fan Icons ----
  drawFanIcon(CIRCLE1_X, CIRCLE_Y, CIRCLE_R, accentColor);
  drawFanIcon(CIRCLE2_X, CIRCLE_Y, CIRCLE_R, accentColor);

  // ---- Fan titles ----
  LCD.setFont(&FreeSans9pt7b);
  LCD.setTextColor(GxEPD_BLACK);

  LCD.setCursor(CIRCLE1_X - 35, CIRCLE_Y - CIRCLE_R - 25);
  LCD.print("Suction Fan");

  drawSwitch(
    CIRCLE1_X - SWITCH_W / 2,
    CIRCLE_Y - CIRCLE_R - 10,
    humi_fan_on
  );

  LCD.setCursor(CIRCLE2_X - 35, CIRCLE_Y - CIRCLE_R - 25);
  LCD.print("Cooling Fan");

  drawSwitch(
    CIRCLE2_X - SWITCH_W / 2,
    CIRCLE_Y - CIRCLE_R - 10,
    cooling_fan_on
  );

  // ---- Threshold text ----
  LCD.setFont(&FreeMono9pt7b);
  LCD.setTextColor(GxEPD_BLACK);

  LCD.setCursor(CIRCLE1_X - 50, CIRCLE_Y + CIRCLE_R + 20);
  LCD.print("ON  >= 60%");

  LCD.setCursor(CIRCLE1_X - 50, CIRCLE_Y + CIRCLE_R + 35);
  LCD.print("OFF <= 50%");

  LCD.setCursor(CIRCLE2_X - 55, CIRCLE_Y + CIRCLE_R + 20);
  LCD.print("ON  >= 60C");

  LCD.setCursor(CIRCLE2_X - 55, CIRCLE_Y + CIRCLE_R + 35);
  LCD.print("OFF <= 50C");
}


      footer();
    

  } while (LCD.nextPage());
}


void footer(){     
  /*
            //upward curve
            LCD.fillRect(0, 260, 400, 40, GxEPD_BLACK); // GxEPD_BLACK
            LCD.fillRoundRect(2, 255, 396, 15, 8, GxEPD_WHITE);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);
            
            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_WHITE);
    
            //HOME TAB
          //  LCD.fillRoundRect(5, 275, 62, 20, 10, GxEPD_WHITE); 
            //LCD.setCursor(12, 290);  LCD.print("Home"); 
            
            //DATE
            LCD.setCursor(2, 290);  LCD.print(SystemDate); 

            
            // WIFI & UPLOADS
            
            //NETWORK BARS
            LCD.fillRect(230, 293, 6, 5, GxEPD_WHITE); LCD.fillRect(240, 283, 6, 15, GxEPD_WHITE); LCD.fillRect(250, 274, 6, 25, GxEPD_WHITE);
            LCD.setFont();
            //LCD.setCursor(200, 290); LCD.print("WiFi: "); 
            LCD.setCursor(225, 280); LCD.print(wifi_connected?"ON":"OFF");
            if(successful_uploads) {LCD.setCursor(265, 280); LCD.print("(");LCD.print(successful_uploads_c);LCD.print(")"); }
            
            //BATTERY
            LCD.drawRect(280, 275, 20, 10, GxEPD_WHITE);
            LCD.setCursor(310, 280); LCD.print(voltage_string); LCD.print("V");
            
            
            //SEPARATOR
           // LCD.fillRect(295, 275, 7, 20, GxEPD_WHITE);

            //TIME
            LCD.setFont(&FreeSans9pt7b); //FreeMono12pt7b
            LCD.setCursor(320, 290); LCD.print(ShortTime_am_pm);

          //  LCD.setFont(&FreeMono9pt7b);
          //  LCD.setCursor(385, 292); LCD.print(DISP_MODE);
*/


   //footer with upward curve
            LCD.fillRect(0, 260, 400, 40, GxEPD_BLACK); // GxEPD_BLACK
            LCD.fillRoundRect(2, 255, 396, 15, 8, GxEPD_WHITE);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);


        // LOGS FOR NETWORK
            LCD.setFont();
            LCD.setTextColor(GxEPD_RED);
            LCD.setCursor(40, 250); LCD.print(internals_log);
            LCD.setCursor(40, 260); LCD.print(upload_log);


            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_WHITE);
    
            //HOME TAB
          //  LCD.fillRoundRect(5, 275, 62, 20, 10, GxEPD_WHITE); 
            //LCD.setCursor(12, 290);  LCD.print("Home"); 
            
            //DATE
            LCD.setCursor(2, 290);  LCD.print(SystemDate); 

            
            // WIFI & UPLOADS
            
            //NETWORK BARS
            LCD.fillRect(230, 293, 6, 5, GxEPD_WHITE); LCD.fillRect(240, 283, 6, 15, GxEPD_WHITE); LCD.fillRect(250, 274, 6, 25, GxEPD_WHITE);
            LCD.setFont();
            //LCD.setCursor(200, 290); LCD.print("WiFi: "); 
            LCD.setCursor(224, 280); LCD.print(wifi_connected?"ON":"OFF");
           // if(successful_uploads) {LCD.setCursor(265, 275); LCD.print("(");LCD.print(successful_uploads_c);LCD.print(")"); }
            
            //BATTERY
            LCD.fillRect(272, 282, 3, 6, GxEPD_WHITE); LCD.fillRoundRect(275, 277, 35, 16, 3, GxEPD_WHITE); 
            if(voltage >= 6.00) { LCD.setFont(); LCD.setTextColor(GxEPD_RED);  LCD.setCursor(272, 281); LCD.print(voltage_string);  }
       
            //SEPARATOR
           // LCD.fillRect(295, 275, 7, 20, GxEPD_WHITE);

            //TIME
            LCD.setTextColor(GxEPD_WHITE);
            LCD.setFont(&FreeSans9pt7b); //FreeMono12pt7b
            LCD.setCursor(320, 290); LCD.print(ShortTime_am_pm);

          //  LCD.setFont(&FreeMono9pt7b);
          //  LCD.setCursor(385, 292); LCD.print(DISP_MODE);

 }




char ESP_IDF_VER[50] = "ESP-IDF Version: ";
char ARD_CORE_VER[50] = "Arduino Core Version: ";

void ESPInfo() {
    snprintf(ESP_IDF_VER, sizeof(ESP_IDF_VER), "ESP-IDF Version: %s", esp_get_idf_version());
    snprintf(ARD_CORE_VER, sizeof(ARD_CORE_VER), "Arduino Core Version: %s", ESP.getSdkVersion());
}


