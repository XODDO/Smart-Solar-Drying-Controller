#include "Arduino.h"
#include "RTClib.h"
//#include "DHT.h"


#include <WiFi.h>
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs

#include <esp_wifi_types.h>  // For wifi_interface_t

#include "ArduinoOTA.h"
#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>


#include <ArduinoJson.h>
//StaticJsonDocument<2000> JSON_data;
JsonDocument JSON_data; // for dynamic amounts of data
StaticJsonDocument<2048> JSON_sendable;
//JsonDocument JSON_sendable

#include "packet_handler.h"

#include "buzzer.h"
#include "time_keeper_2.h"
#include "WiFi_Manager.h"


#include <U8g2lib.h>
  #ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
  #endif



//#define DHTTYPE DHT22   // DHT 21 (AM2301)
//#define DHTPIN 15  

const char DeviceID[20] = "Smart Solar Dryer"; 
const char device_name[20] = "Black_Soldier_Fly";
const char system_client[32] = "Busitema University Project"; 
const char sensor_position[30] = "Arapai Campus, Soroti";



OTA_Credentials get;
const char* OTA_PASS = get.secure_ota_password;

constexpr uint8_t buzzingPin = 14, wifi_led = 26; 

constexpr uint8_t reset = 13, chip_select = 5;
constexpr uint8_t ota_button = 32; //25;


constexpr uint8_t cooling_fans = 15; //26;
constexpr uint8_t extraction_fans = 33;


constexpr uint8_t internal_fan = 25; 
constexpr uint8_t night_light = 27;
uint8_t flashing_led = 26;

float box_temp = 0.0f;

bool night_light_on = false;

bool internal_fan_on = false;

U8G2_ST7920_128X64_1_HW_SPI LCD(U8G2_R2, /* CS=*/ chip_select, /* reset=*/ reset);



TimerKeeper sawa;
Buzzer buzzer(buzzingPin);   //uint8_t buzzer = 14;     // buzzer pin
WiFi_Manager wifi_obj(wifi_led); // pin 27

PacketHandler packetHandler;





uint64_t screen_timeout = 0; 
uint8_t currentScreen = 1; 
char current_screen_c[7];

uint8_t backlight_ = 4; //27; 
bool backlit = false;

uint8_t BatteryPin = 36;

float voltage = 0.00f;   


bool esp_now_initialized = false; char esp_now_status_msg[128];
bool clock_is_working = false;
uint8_t now_mins = 0;


//char temp_str[8]; char humi_str[8];
char temp_tens[4] = "tt";     char temp_dec[4] = "t"; 
char humi_tens[4] = "hh";     char humi_dec[4] = "h";

char dot[3] = ".";

uint8_t bod_y_pos = 15;

float drying_temperature = 0.00;
float drying_humidity = 0.00;


bool cooling_fans_on = false;
bool extraction_fans_on = false;

bool duoMode = true;

float outdoor_temperature = 0.0f;
float outdoor_humidity = 0.0f;


float sensor_1_temp, sensor_2_temp, sensor_3_temp, sensor_4_temp, sensor_5_temp, sensor_6_temp;
float sensor_1_humidity, sensor_2_humidity, sensor_3_humidity, sensor_4_humidity, sensor_5_humidity, sensor_6_humidity;
float sensor_1_pressure, sensor_2_pressure, sensor_3_pressure, sensor_4_pressure, sensor_5_pressure, sensor_6_pressure;
float sensor_1_elevation, sensor_2_elevation, sensor_3_elevation, sensor_4_elevation, sensor_5_elevation, sensor_6_elevation;
uint64_t sensor_1_running_time_ms, sensor_2_running_time_ms, sensor_3_running_time_ms, sensor_4_running_time_ms, sensor_5_running_time_ms, sensor_6_running_time_ms;
uint64_t sensor_1_last_seen_ms = 0; uint64_t sensor_2_last_seen_ms = 0; uint64_t sensor_3_last_seen_ms = 0; uint64_t sensor_4_last_seen_ms = 0;
uint64_t sensor_5_last_seen_ms = 0; uint64_t sensor_6_last_seen_ms = 0; uint64_t sensor_7_last_seen_ms = 0; uint64_t sensor_8_last_seen_ms = 0;
size_t sensor_1_packet_size = 0; size_t sensor_2_packet_size = 0; size_t sensor_3_packet_size = 0; size_t sensor_4_packet_size = 0;

char  sensor_1_transmissions[9]; char  sensor_2_transmissions[9];   char  sensor_3_transmissions[9];   char  sensor_4_transmissions[9]; 
char  sensor_5_transmissions[9]; char  sensor_6_transmissions[9];   char  sensor_7_transmissions[9];   char   sensor_8_transmissions[9]; 
char sensor_1_last_seen[12] = "--:--"; char sensor_2_last_seen[12] = "--:--"; char sensor_3_last_seen[12] = "--:--"; char sensor_4_last_seen[12] = "--:--";
char sensor_5_last_seen[12] = "--:--"; char sensor_6_last_seen[12] = "--:--"; char sensor_7_last_seen[12] = "--:--"; char sensor_8_last_seen[12] = "--:--";


const uint32_t buzzer_check_frequency = (50UL); // 50 milliseconds buzzer.beep(multiple of 50)

const uint32_t  OTA_BTN_STACK_SIZE       = 4096;  // 4kB
const uint32_t OTA_TASK_STACK_SIZE      = 8192; // 8kB because WiFi stack activity, TCP, buffers, flash writes, callbacks

const uint8_t OTA_SERVICE_TASK_CORE  = 0;
const uint8_t OTA_BTN_TASK_CORE = 1;

const uint32_t BUZZING_TASK_STACK_SIZE  = 2048; // 4kB
const uint8_t BUZZING_TASK_CORE = 0;

#define BUZZING_TASK_PRIORITY  6  // Highest (short, critical)
#define OTA_TASK_PRIORITY      7  // High for OTA
#define OTA_BTN_PRIORITY       3



TaskHandle_t buzzingTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
TaskHandle_t otaBtnHandle = NULL;
SemaphoreHandle_t otaMutex = NULL;

// Round counters — start at 1 to skip round-0 cold read
uint32_t buzzing_round     = 1;
uint32_t buttoning_round   = 1;
uint32_t ota_service_round = 1;


// Watermark globals — written by each task, read by transmit logic
volatile uint32_t wm_buzzing_bytes  = 0;
volatile uint32_t wm_ota_btn_bytes  = 0;
volatile uint32_t wm_ota_svc_bytes  = 0;
volatile uint32_t wm_sensing_bytes  = 0; 


// Add these enums and globals at the top with your other globals
typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_STARTING,
    OTA_STATE_PROGRESS,
    OTA_STATE_SUCCESS,
    OTA_STATE_ERROR,
    OTA_STATE_TIMEOUT
} OTAState;

OTAState current_ota_state = OTA_STATE_IDLE;


char ota_watermark_str[100] = "OTA Stack not yet allocated";
char buzzing_watermark_char[128] = "BUZZ Stack not yet allocated";
char btn_watermark_char[100] = "BTN STACK NOT ALLOCATED YET";

char stack_report[2048] = "No STACK!"; // SUPER HEAVY BUFFER FOR ALL TASKS

char heap_report[1024] = "No Heap!";

void OtaButtonTask(void *pvParameters);
void BuzzingTask(void * pvParams);
void OtaServiceTask(void *pv);
void MemoryMonitorTask(void *pv);


char clock_status_msg[100] = "Not Started!";

const uint8_t NTP_TIMEOUT = 10;

#define WIFI_SWITCH_TIMEOUT_MS 30000
#define WIFI_RECONNECT_DELAY_MS 1000
uint8_t wifi_connect_attempts = 0;

char wifi_conn_log[512];

// ==================== WIFI & OTA STATUS ====================
bool wifi_connected = false;
uint32_t WiFi_Strength = 0;
char ota_log[128] = "...";
volatile bool otaFinished = false;
volatile bool otaStarted = false;
volatile bool otaError = false;

bool otaModeActive = false;
bool otaTriggered = false;
uint64_t otaStartTime = 0;
const uint64_t OTA_TIMEOUT = 15ULL * 60ULL * 1000ULL;  // 15 minutes
char LastOTAUpdate[60] = "27 Oct 2025 10:10";

volatile uint8_t otaProgress;

uint64_t ota_success_time = 0;
uint64_t ota_error_time = 0;
uint64_t ota_timeout_time = 0;
uint64_t ota_start_time = 0;


bool waiting_for_delivery = false;
uint64_t last_wifi_check = 0;
uint64_t ota_running_time = 0;

constexpr uint8_t  SDA_PIN = 21;//8; // Define your custom SDA pin
constexpr uint8_t  SCL_PIN = 22;//9; // Define your custom SCL pin

bool fanning_activated = false;
uint32_t loop_delay_duration = 0;

char ip_addr[64];
char sensor_ip_address[32] = "0.0.0.0";  // Will store IP as string

// Logging macro
#define LOG(msg) Serial.println(msg)

bool five_min = false, ten_min = false, hour_marked = false;

unsigned long uptime_seconds = 0;
uint8_t hawa = 0, minuto = 0, sekonda = 0;
uint64_t now_now_ms = 0, prev_update = 0;

char DeviceTime[32] = "00:00:00";
char DeviceDate[32] = "1st Jan 2001";

void setup() { delay(100);
      Serial.begin(115200); Serial.print(DeviceID); Serial.println(" Booting..."); 

  
  // INITIALIZING ALARM
        buzzer.begin();
        xTaskCreatePinnedToCore(BuzzingTask, "BuzzChecker", BUZZING_TASK_STACK_SIZE, NULL, BUZZING_TASK_PRIORITY, &buzzingTaskHandle, BUZZING_TASK_CORE);
        buzzer.beep(1,50,0);  


        Serial.print("Crystal Frequency: "); Serial.println(getXtalFrequencyMhz()); Serial.println();

        Serial.print("CPU Frequency Before: "); Serial.println(getCpuFrequencyMhz());

        setCpuFrequencyMhz(80); delay(200);

        Serial.print("CPU Frequency After: "); Serial.println(getCpuFrequencyMhz());

      //DISPLAY    
   // SPI.begin(SCK, MISO, MOSI, SS);
 
        LCD.begin(); delay(100);


         // ===== BOOT SCREEN START =====
       // draw_boot_screen("Booting...", device_name, DeviceID, 5);

  
   //back light  
      pinMode(backlight_, OUTPUT);     digitalWrite(backlight_, HIGH);  backlit = true;
      delay(100);

      
   //ACTUAL LOAD
    pinMode(cooling_fans, OUTPUT);
    pinMode(extraction_fans, OUTPUT);

   //INDICATORS
    pinMode(flashing_led, OUTPUT); digitalWrite(flashing_led, HIGH);

  // OTHER PERIPHERALS
    pinMode(internal_fan, OUTPUT); digitalWrite(internal_fan, HIGH);
    pinMode(night_light, OUTPUT); digitalWrite(night_light, HIGH);


    Boot();

      //0. Starting WiFi
    Serial.println("Connecting to WiFi before creating wifi-dependent tasks...");
    

 // ===== 1. WIFI =====
   // draw_boot_screen("WiFi: Scanning...", "Looking for networks", "", 15);

    wifi_connected = switch_radio_to_wifi();   // this blocks until done

    if (wifi_connected) {
        char ip_buf[20];
        snprintf(ip_buf, sizeof(ip_buf), "IP: %s", WiFi.localIP().toString().c_str());
       // draw_boot_screen("WiFi Connected", wifi_conn_log, ip_addr, 35);
        // optionally show RSSI
        // char rssi_buf[10]; snprintf(rssi_buf, sizeof(rssi_buf), "RSSI: %d dBm", WiFi.RSSI());
        // draw_boot_screen("WiFi Connected", WiFi.SSID().c_str(), rssi_buf, 35);
    } else {
        draw_boot_screen("WiFi Failed", "Check connection", "", 35);
    }
    delay(1000);  // let user read



        Serial.println("Starting RTC...");
              // 2. Initialize RTC
          clock_is_working = sawa.initialize_RTC(SDA_PIN, SCL_PIN);

          // 3. Time initialisation
          if (clock_is_working) {
              // RTC present
              snprintf(clock_status_msg, sizeof(clock_status_msg), "RTC Clock is working.");
              // Sync RTC with NTP if WiFi available (once)
              if (wifi_connected) {
                  wifi_obj.sync_ntp(10800, 0, NTP_TIMEOUT);
                  if (wifi_obj.ntp_synced) {
                      sawa.apply_ntp_time(wifi_obj.ntp_timeinfo);
                      Serial.println("✅ RTC updated from NTP.");
                  } else {
                      Serial.println("⚠️ NTP sync failed – keeping existing RTC time.");
                  }
              }
              // Read RTC time
              sawa.query_rtc();
          } else {
              // RTC absent – use internal software clock
              snprintf(clock_status_msg, sizeof(clock_status_msg), "RTC not found – using software clock.");
        //      draw_boot_screen("RTC: Not found", "Using software clock", "", 60);
              delay(1000);

              if (wifi_connected) {
            //    draw_boot_screen("Syncing time (NTP)...", "Contacting server", "", 70);
                  wifi_obj.sync_ntp(10800, 0, NTP_TIMEOUT);
                  if (wifi_obj.ntp_synced) {
                      sawa.setInternalTime(wifi_obj.ntp_timeinfo);
                      sawa.setLastNtpSyncTime(esp_timer_get_time());   // set the last‑sync timestamp so the hourly NTP check doesn’t trigger immediately:

                      Serial.println("✅ Software clock set from NTP.");
                      // Record last sync time
                      sawa.last_ntp_sync_us = esp_timer_get_time();
                  } else {
                      Serial.println("⚠️ NTP sync failed – starting from default time.");
                      // Optionally set a default (e.g., compile time or 2001-01-01)
                      struct tm default_tm = {0};
                      default_tm.tm_year = 2001 - 1900;
                      default_tm.tm_mon = 0;   // Jan
                      default_tm.tm_mday = 1;
                      default_tm.tm_hour = 0;
                      default_tm.tm_min = 0;
                      default_tm.tm_sec = 0;
                      sawa.setInternalTime(default_tm);
                  }
              } else {
                  // No WiFi – use default
                  struct tm default_tm = {0};
                  default_tm.tm_year = 2001 - 1900;
                  default_tm.tm_mon = 0;
                  default_tm.tm_mday = 1;
                  sawa.setInternalTime(default_tm);
              }
              // Mark that we are in software mode (already done by setInternalTime)
          }

          // 4. After time source is set, copy to global strings
          snprintf(DeviceTime, sizeof(DeviceTime), "%s", sawa.ShortTime); //SystemTime
          snprintf(DeviceDate, sizeof(DeviceDate), "%s", sawa.ShortDate); // SystemDate

          char time_buf[30];
            snprintf(time_buf, sizeof(time_buf), "Time: %s", sawa.SystemTime);
       //     draw_boot_screen("NTP: Synced", time_buf, "", 90);

          // 5. If RTC present, set flags (five_min, ten_min) etc.
          if (clock_is_working) {
              five_min = sawa.is5MinuteMarker();
              ten_min = sawa.is10MinuteMarker();
          }


                LOG(clock_status_msg);

        
        Serial.println("\n=== Creating OTA Service Task ===");

        Serial.printf("  - Free heap before task: %u bytes\n",
                    esp_get_free_heap_size());

        Serial.printf("  - Minimum free heap: %u bytes\n",
                    esp_get_minimum_free_heap_size());

        Serial.printf("  - Largest free block: %u bytes\n",
                    heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // ota service task
   //     BaseType_t ota_result = xTaskCreatePinnedToCore(OtaServiceTask,  "OTA_Service",  OTA_TASK_STACK_SIZE,    NULL,   OTA_TASK_PRIORITY,  &otaTaskHandle,   OTA_SERVICE_TASK_CORE );

        delay(1000);


      switch_radio_to_espnow();

      // 6. ESP-NOW takes over
     //   esp_now_initialized = packetHandler.begin(); // this already exists in the above function
        snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "%s",
            esp_now_initialized ? "ESP-NOW Initialization Successful!"
                                : "Failed to initialize packet handler!");

        


    


       time_check(); delay(1000);
       MonitorFans(); delay(1000);
       



    xTaskCreatePinnedToCore(OtaButtonTask,  "OTA_Button", OTA_BTN_STACK_SIZE,   NULL,   OTA_BTN_PRIORITY,   &otaBtnHandle,  OTA_BTN_TASK_CORE); // polls every 100ms


    //TURN EM OFF
    
   //INDICATORS
     digitalWrite(flashing_led, LOW); delay(500);

  // OTHER PERIPHERALS
    digitalWrite(internal_fan, LOW); delay(500);
    digitalWrite(night_light, LOW); delay(500);

     digitalWrite(cooling_fans, HIGH); delay(500);
     digitalWrite(extraction_fans, HIGH);

       drying_temperature = sensor_1_temp;
       drying_humidity = sensor_1_humidity;

       splicer(); delay(1000);

    
   
    currentScreen = 1;
    update_display();


      buzzer.beep(2,50,50);  
      
    loop_delay_duration = (fanning_activated)?1000:10000; // every sec / 10 secs when active
  
    Serial.println("Done Booting!");




}
 

void BuzzingTask(void * pvParams) { 
    const TickType_t buzzDelay = pdMS_TO_TICKS(buzzer_check_frequency); // 50 milliseconds
    uint32_t last_memory_check = 0;
    uint32_t current_time = 0;
    
    while (true) {
        buzzer.update();
        
        vTaskDelay(buzzDelay); // yield to scheduler
    }

      /*
        // Check memory usage every 5 minutes using millis() - RELIABLE!
        current_time = millis();
        if ((current_time - last_memory_check) >= (5UL * 60UL * 1000UL)) { // 5 minutes
            last_memory_check = current_time;
            
            UBaseType_t wm = uxTaskGetStackHighWaterMark(NULL);
            uint32_t wm_b = wm * sizeof(StackType_t);
            uint32_t used = BUZZING_TASK_STACK_SIZE - wm_b;
            uint32_t percent = (used * 100) / BUZZING_TASK_STACK_SIZE;
            
            wm_buzzing_bytes = wm_b;  // write global for transmission
            
            snprintf(buzzing_watermark_char, sizeof(buzzing_watermark_char),
                "[BUZZ] WM: %.1fkB free | used %u/%u bytes (%u%%)",
                wm_b / 1024.0f, used, BUZZING_TASK_STACK_SIZE, percent);
            
            Serial.println(buzzing_watermark_char);
        }
        */
      
}



void OtaServiceTask(void *pv){
    

    while(true){
    
        if(otaModeActive){
        
            ArduinoOTA.handle();

             ota_running_time = esp_timer_get_time();
             // Manage WiFi connection (every 2 seconds)
            if ((ota_running_time - last_wifi_check) >= (2ULL * 1000ULL)) {
                wifi_connected = wifi_obj.ensure_wifi();
               // process_ota_state_transitions(ota_running_time);

                handle_ota_exit_conditions(ota_running_time);

                last_wifi_check = ota_running_time;
            }

            
            // small delay during OTA, no print or snprintf
            vTaskDelay(pdMS_TO_TICKS(10)); // VERY IMPORTANT:
        }
        else
        {   
             UBaseType_t wm   = uxTaskGetStackHighWaterMark(NULL);
            uint32_t    wm_b = wm * sizeof(StackType_t);

            wm_ota_svc_bytes = wm_b;  // write global for transmission

            snprintf(ota_watermark_str, sizeof(ota_watermark_str),
                "[OTA_SVC] WM: %.1fkB free | stack=%u bytes",
                wm_b / 1024.0f, OTA_TASK_STACK_SIZE);

            Serial.println(ota_watermark_str);

            ota_service_round++;
            vTaskDelay(pdMS_TO_TICKS(30000)); // sleep for half a minute when idle
            
        }
    }
}


void OtaButtonTask(void *pvParameters){

          pinMode(ota_button, INPUT_PULLUP);


    bool lastState = HIGH;
    bool longPressHandled = false;

    uint64_t pressStart = 0;

    const uint32_t button_mem_interval = (5UL*60UL*10UL);

    while(true){

         if (otaModeActive) {
            vTaskSuspend(NULL);  // Suspend this task
            // Will resume when OTA is done
            continue;
        }
    
        bool state = digitalRead(ota_button);

        uint64_t now = esp_timer_get_time();

        // Button pressed
        if(state == LOW && lastState == HIGH)
        {
            pressStart = now;
            longPressHandled = false;
        }

        // Long press detected (2.2s)
        if(state == LOW &&  !longPressHandled && (now - pressStart >= 2000000ULL)){
        
            longPressHandled = true;
   
            Serial.println("OTA button activated");
            if(!otaModeActive)  toggle_ota(); //  buzzer.beep(2, 500, 500); and  initializeOTA(); happen in toggle ota

           // handle_ota(now, true); //  on true calls toggle_ota() which itself calls initializeOTA();

        }

        // Button released
        if(state == HIGH && lastState == LOW)
        {
            longPressHandled = false;
        }

        lastState = state;

          if(buttoning_round % button_mem_interval == 0){  // every ~5mins at 100ms/tick
            UBaseType_t wm = uxTaskGetStackHighWaterMark(NULL);
            uint32_t    wm_b = wm * sizeof(StackType_t);

            wm_ota_btn_bytes = wm_b;  // write global for transmission

            snprintf(btn_watermark_char, sizeof(btn_watermark_char),
                "[OTA_BTN] WM: %.1fkB free | stack=%u bytes",
                wm_b / 1024.0f, OTA_BTN_STACK_SIZE);

            Serial.println(btn_watermark_char);
        }

        buttoning_round++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void toggle_ota() {
    // Enter OTA mode - COMPLETELY stop the sensing task
   // if (otaMutex != NULL)   xSemaphoreTake(otaMutex, portMAX_DELAY);
  
        /*
    // 1. Stop the sensing task
    if (sensingTaskHandle != NULL) {
        vTaskSuspend(sensingTaskHandle);
        sensingTaskSuspended = true;
        Serial.println("Sensing task suspended for OTA");
    }
    */
    
     Serial.println("\n=== OTA Triggered ===");
     buzzer.beep(1, 1000, 0);
    
    esp_err_t result = esp_now_deinit();
    if (result != ESP_OK) {
        Serial.printf("ESP-NOW deinit failed: %d\n", result);
        return;
    }
    
    delay(300);
    
    if (!switch_radio_to_wifi()) {
        Serial.println("WiFi failed. Returning to ESP-NOW.");
        switch_radio_to_espnow(); 
        return;
    }
    
    initializeOTA();
    
    
    
    if (otaMutex != NULL) {
        xSemaphoreGive(otaMutex);
    }
    
    // 4. Set OTA start time
     if (!otaModeActive) otaModeActive = true;
    otaStartTime = esp_timer_get_time() / 1000ULL;
    
    Serial.printf("OTA Mode Active for %llu minutes\n", (OTA_TIMEOUT / 60000));
 
    
    Serial.println("\n=== OTA Mode Activated ===");
    buzzer.beep(3, 50, 50); // now active
    
    // Update display to show OTA screen
    currentScreen = 10;
    update_display();
}


void process_ota_state_transitions(uint64_t running_time) {
    // Handle OTA started state
    if (otaStarted) {
        strcpy(ota_log, "Starting OTA update...");
       // buzzer.beep(2, 300, 200);
        ota_start_time = running_time;
        otaStarted = false;
        current_ota_state = OTA_STATE_STARTING;
    }
    
    // Handle OTA progress state
    if (otaProgress > 0 && otaProgress < 100) {
        snprintf(ota_log, sizeof(ota_log), "Updating... %d%%", otaProgress);
        current_ota_state = OTA_STATE_PROGRESS;
    }
    
    // Handle OTA finished state
    if (otaFinished) {
        strcpy(ota_log, "Update complete!");
        buzzer.beep(2, 500, 500);
        otaFinished = false;
        current_ota_state = OTA_STATE_SUCCESS;
        ota_success_time = running_time;
    }
    
    // Handle OTA error state
    if (otaError) {
        strcpy(ota_log, "Update failed!");
        buzzer.beep(1, 2000, 0);
        otaError = false;
        current_ota_state = OTA_STATE_ERROR;
        ota_error_time = running_time;
    }
}

void handle_ota_exit_conditions(uint64_t running_time) {
    // Exit on timeout
    if (current_ota_state == OTA_STATE_TIMEOUT && 
        (running_time - ota_timeout_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
    
    // Exit on success
    if (current_ota_state == OTA_STATE_SUCCESS && 
        (running_time - ota_success_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
    
    // Exit on error
    if (current_ota_state == OTA_STATE_ERROR && 
        (running_time - ota_error_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
}

void exit_ota_mode() {
    Serial.println("Exiting OTA mode, resuming normal operation");
    
    // Stop OTA
    ArduinoOTA.end();
 
 /*
    // Resume sensing task
    if (sensingTaskSuspended && sensingTaskHandle != NULL) {
        vTaskResume(sensingTaskHandle);
        sensingTaskSuspended = false;
    }
    */
    
    // Reset OTA state
    otaModeActive = false;
    current_ota_state = OTA_STATE_IDLE;
    otaProgress = 0;
}

/*
void handle_ota(uint64_t now, bool button_pressed) {
    static uint64_t last_ota_check = 0;
    
    if (button_pressed && !ota_active) {
        Serial.println("OTA mode activated!");
        ota_active = true;
        
        // 1. Suspend sensor task immediately
        if (sensor_task_handle != NULL) {
            vTaskSuspend(sensor_task_handle);
            Serial.println("Sensor task suspended");
        }
        
        // 2. Power down sensors
        sensor.toggle_sensor(OFF);
        sensor.flash_indicators(false);
        
        // 3. Deinit ESP-NOW
        esp_now_deinit();
        esp_wifi_stop();
        
        // 4. Start OTA in normal WiFi mode
        start_ota_mode();  // Your OTA initialization
        
        last_ota_check = now;
    }
    
    // Handle OTA updates
    if (ota_active) {
        handle_ota_updates();  // Your OTA loop
        
        // Check if OTA timed out (e.g., 5 minutes no activity)
        if ((now - last_ota_check) > (300 * 1000000ULL)) {
            Serial.println("OTA timeout, resuming normal operation");
            exit_ota_mode();
        }
    }
}
*/

char current_activity[50] = "Not sensing Anything";

unsigned long long time_of_last_activity = 0;
unsigned long long auto_turn_off = 18e6; // 5 hours //18 million milliseconds


int64_t last_clocked = 0;  int64_t last_refresh = 0;  int64_t last_input = 0; 
int64_t auto_scan_start = 0; int64_t now_scanning = 0;

bool pressed = false; bool firstPass = false;

uint16_t scanning_interval = 10;
bool auto_scanning = false; bool soil_scan_toggled = false; uint64_t button_refresher;

uint8_t ticks = 0; uint8_t sensing_ticks = 0;

bool power_is_enuf = false;
bool flipped = false; 
bool inverted = false;

uint32_t refresh_interval = 2000; //5UL * 60UL * 1000UL; // also dynamic based on fanning activity

bool started_reading = false;

void loop() {

    if(sensor_1_temp <= 1.0 && sensor_1_humidity <= 1.0) { // change screen readings quickly to prevent static zero's
         drying_temperature = sensor_1_temp;
         drying_humidity = sensor_1_humidity;
         splicer();
  
         time_check();
         MonitorFans();
   
         update_display();

          delay(3000);
      return;
    }

      now_now_ms = esp_timer_get_time()/1000ULL;
  
  if((now_now_ms - prev_update) >= refresh_interval){ // 5 MINUTES or 1 second
       prev_update = now_now_ms;

        
        // MonitorBattery(); 

         drying_temperature = sensor_1_temp;
         drying_humidity = sensor_1_humidity;
         splicer();
  
         
        time_check();
        MonitorFans();
   
        update_display();

        loop_delay_duration = (fanning_activated)?1000UL:(1UL * 60 * 1000UL); // every  sec when active // minute
        refresh_interval = fanning_activated?1000UL:(5UL * 60UL * 1000UL);

   
     if(sawa.is10MinuteMarker()) { if(backlit) {digitalWrite(backlight_, LOW); backlit = false; }}

     if(sawa.isHourlyMarker()) sawa.periodicNtpSync(wifi_obj, 10800, 0);  // Periodic NTP sync for software mode (only if WiFi connected)
     
   //    Serial.println(current_activity); 
       snprintf(current_screen_c, sizeof(current_screen_c), "%u", currentScreen);

   }
    
 
    delay(loop_delay_duration); // sleep for a second when active or for a minute when idle
        
}

void  time_check(){  
    sawa.update();

    uint8_t hour = sawa.getCurrentHour();

    Serial.print("Hour: "); Serial.println(hour);
    Serial.println("Checking whether it is time for lights OFF/ON!");

    if(hour >= 19 || hour <= 6){ // it is night time
        if(!night_light_on) { buzzer.beep(1, 50, 0); digitalWrite(night_light, HIGH); night_light_on = true;  }
            Serial.println("it is night!");

    }
    else { // it is daytime
        if(night_light_on) { buzzer.beep(1, 500, 0); digitalWrite(night_light, LOW); night_light_on = false;  }
                    Serial.println("it is day!");


    }

        box_temp = sawa.getRTCTemperature();
        Serial.print("RTC TEMP: "); Serial.println(box_temp);

    if(box_temp >= 30.0) { 
        if(!internal_fan_on) {   buzzer.beep(1, 50, 0); digitalWrite(internal_fan, HIGH); internal_fan_on = true; } 
      
      }
    else if(box_temp <= 25.0) { 
        if(internal_fan_on) { buzzer.beep(1, 500, 0); digitalWrite(internal_fan, LOW); internal_fan_on = false; } 
      
      }




    snprintf(DeviceTime, sizeof(DeviceTime), "%s", sawa.ShortTime); // SystemTime
    snprintf(DeviceDate, sizeof(DeviceDate), "%s", sawa.ShortDate); // SystemDate

    Serial.print("Time Check: "); Serial.println(DeviceTime);
    Serial.print("Date Check: "); Serial.println(DeviceDate);
}

// if current hour > 20 or < 06 // night hours, do nothing

// if temp > 55.0, turn on cooling fans
// if temp < 50.0 turn off cooling fans

// if humidity > 40, turn off extraction fans
// if humidity < 35, turn off extraction fans

char cooling_fan_turn_on_time[32];
char cooling_fan_turn_off_time[32];

char extraction_fans_turn_on_time[32];
char extraction_fans_turn_off_time[32];


void MonitorFans() {
    // 1. Get current hour (0-23)
    uint8_t hour = sawa.getCurrentHour();
  

  Serial.print("Drying Temperature: ");  Serial.print(drying_temperature);  Serial.println("°C ");
  Serial.print("Drying Humidity: "); Serial.print(drying_humidity); Serial.println("%");
  

  if (isnan(drying_temperature) || isnan(drying_humidity)) {
    Serial.println("⚠️ Sensor read error – keeping fans in current state");
    return;
}

    // 2. Night time (20:00 – 05:59) – turn everything off and exit
    if (hour >= 20 || hour < 6) {
        if (cooling_fans_on) {
            digitalWrite(cooling_fans, HIGH);
            cooling_fans_on = false;
            Serial.println("Turned OFF Cooling Fans, NIGHT TIME");
        }
        if (extraction_fans_on) {
            digitalWrite(extraction_fans, HIGH);
            extraction_fans_on = false;
            Serial.println("Turned OFF Extraction Fans, NIGHT TIME");
        }
        fanning_activated = false;
        Serial.println("\tCan't do no fanning @Night!");
        return;   // do nothing else
    }

   
    // ---------- COOLING FANS ----------
    // On when temp > 55.0°C, off when temp < 50.0°C
    if (drying_temperature > 55.0) {
        if (!cooling_fans_on) {
            digitalWrite(cooling_fans, LOW);
            cooling_fans_on = true;
            fanning_activated = true;
            snprintf(cooling_fan_turn_on_time, sizeof(cooling_fan_turn_on_time), "%s", DeviceTime);
            Serial.print("Turned ON Cooling Fans at: "); Serial.println(cooling_fan_turn_on_time);
            
        }
    } 
    else if (drying_temperature < 50.0) {
        if (cooling_fans_on) {
            digitalWrite(cooling_fans, HIGH);
            cooling_fans_on = false;
            snprintf(cooling_fan_turn_off_time, sizeof(cooling_fan_turn_off_time), "%s", DeviceTime);
            Serial.print("Turned OFF Cooling Fans at: "); Serial.println(cooling_fan_turn_off_time);
            
        }
    }
    // else: stay in current state (hysteresis zone)

    // ---------- EXTRACTION FANS ----------
    // On when humidity > 40.0%, off when humidity < 35.0%
    if (drying_humidity > 40.0) {
        if (!extraction_fans_on) {
            digitalWrite(extraction_fans, LOW);
            fanning_activated = true;
            extraction_fans_on = true;
            snprintf(extraction_fans_turn_on_time, sizeof(extraction_fans_turn_on_time), "%s", DeviceTime);
            Serial.print("Turned ON Extraction Fans at: "); Serial.println(extraction_fans_turn_on_time);            
        }
    } 
    else if (drying_humidity < 35.0) {
        if (extraction_fans_on) {
            digitalWrite(extraction_fans, HIGH);
            extraction_fans_on = false;
            snprintf(extraction_fans_turn_off_time, sizeof(extraction_fans_turn_off_time), "%s", DeviceTime);
            Serial.print("Turned OFF Extraction Fans at: "); Serial.println(extraction_fans_turn_off_time);
            
        }
    }
   // if(!cooling_fans_on && !extraction_fans_on) fanning_activated = false;
    fanning_activated = (cooling_fans_on || extraction_fans_on);
   /*
    else {
      fan_running_time
    }
    */
    // else: stay in current state (hysteresis zone)
    /*
    Serial.println();
    Serial.print("Outdoor Temperature: ");  Serial.print(outdoor_temperature);  Serial.println("°C ");
    Serial.print("Outdoor Humidity: "); Serial.print(outdoor_humidity); Serial.println("%");

    */
}


bool switch_radio_to_wifi() {
    if(esp_now_initialized) { packetHandler.shutdown(); esp_now_initialized = false; }  // to prevent heap fragmentation

    Serial.println("📶 Initializing WiFi...");
    unsigned long startTime = millis();
    
    while ((millis() - startTime) < WIFI_SWITCH_TIMEOUT_MS) {
        WiFiMgrStatus status = wifi_obj.initialize_ESP_WiFi(device_name);
        
        if (status == WIFI_MGR_SUCCESS) {

            update_sensor_ip_address();

            Serial.printf("WiFi:  %d\n", WiFi.status());
        
            snprintf(wifi_conn_log, sizeof(wifi_conn_log), "WiFi ON: [%s]", WiFi.SSID().c_str());
            snprintf(ip_addr, sizeof(ip_addr), "IP Address: %s\n", sensor_ip_address); // WiFi.localIP().toString().c_str()

             Serial.printf("🎯 WiFi Mode Activated - IP: %s, RSSI: %d dBm\n",  WiFi.localIP().toString().c_str(), WiFi.RSSI());

                    Serial.println("🔌 Initializing OTA updates...");
                    initializeOTA();
                   
            
            return true;
        }
        
        Serial.printf("⏳ WiFi connection failed, retrying... (%lu ms elapsed)\n", 
                     millis() - startTime);
        delay(1000);
    }
    
    snprintf(wifi_conn_log, sizeof(wifi_conn_log), "%s", "💥 Failed to initialize WiFi within timeout period");
    return false;
}


#define ESPNOW_MAX_RETRIES 3

bool switch_radio_to_espnow(){
  Serial.println("Switching radio to ESP-NOW...");

    WiFi.disconnect(true);
    delay(250);
    WiFi.mode(WIFI_OFF);
    delay(250);
    
    WiFi.mode(WIFI_STA); Serial.printf("WiFi channel: %d\n", WiFi.channel());

     esp_now_initialized = packetHandler.begin(); // pinned to core 0: has espnow, callbacks, and queue creation inside
    if (!esp_now_initialized) snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "Failed to initialize packet handler!");
        
    else     snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "ESP-NOW Initialization Successful!");
    // HANDLES onDataReceived
    
  /*
      // Proper cleanup
        if (esp_now_deinit() != ESP_OK) {
           Serial.println("Warning: ESP-NOW deinit had issues");
         }
  */  
    
    return esp_now_initialized;
}





// ============================================================
// GET IP ADDRESS AS STRING (safe, non-blocking)
// ============================================================
void update_sensor_ip_address() { // WiFi.localIP().toString().c_str()
    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        snprintf(sensor_ip_address, sizeof(sensor_ip_address), 
                 "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    } else {
        strncpy(sensor_ip_address, "0.0.0.0", sizeof(sensor_ip_address) - 1);
        sensor_ip_address[sizeof(sensor_ip_address) - 1] = '\0';
    }
}

void initializeOTA() {
    ArduinoOTA.setHostname(device_name);
    ArduinoOTA.setPassword(OTA_PASS);
    Serial.println("Hostname and Password set!");

    ArduinoOTA
        .onStart([]() { 
            otaStarted = true; 
            otaModeActive = true;
            otaProgress = 0;
            otaStartTime = esp_timer_get_time() / 1000000ULL;
            
            // ── SINGLE DISPLAY UPDATE AT START ──
            snprintf(ota_log, sizeof(ota_log), "OTA Update started...");
            //  currentScreen = 10;
           // update_display();        // ONE e-paper refresh only
            buzzer.beep(1, 200, 0);
            
            Serial.println("OTA Update started - display updated once");
        }) 
        .onEnd([]() { 
            otaFinished = true;
            snprintf(ota_log, sizeof(ota_log), "Update complete! Rebooting...");
            
            // ── FINAL DISPLAY UPDATE ──

          //  update_display();        // One final refresh before reboot
            buzzer.beep(3, 300, 200);
            
            Serial.println("OTA Update complete - rebooting in 2s");
            delay(2000);            // Give display time to render
            ESP.restart();
        }) 
        .onProgress([](unsigned int progress, unsigned int total) {
            // ── NO DISPLAY UPDATES DURING PROGRESS ──
            // Just update the log in memory for debug
            uint8_t percent = (progress * 100) / total;
            if (percent != otaProgress && percent % 10 == 0) {
                otaProgress = percent;
                snprintf(ota_log, sizeof(ota_log), "Progress: %d%%", percent);
                Serial.printf("OTA Progress: %d%%\n", percent);
                // ⚠️ DO NOT call update_display() here - e-paper is too slow!
            }
        })
        .onError([](ota_error_t error) { 
            otaError = true;
            otaModeActive = false;
            /*
            
            const char* err_msg;
            switch(error) {
                case OTA_AUTH_ERROR: err_msg = "Auth failed"; break;
                case OTA_BEGIN_ERROR: err_msg = "Begin failed"; break;
                case OTA_CONNECT_ERROR: err_msg = "Connect failed"; break;
                case OTA_RECEIVE_ERROR: err_msg = "Receive failed"; break;
                case OTA_END_ERROR: err_msg = "End failed"; break;
                default: err_msg = "Unknown error"; break;
            }
            
            snprintf(ota_log, sizeof(ota_log), "Error: %s", err_msg);

            
            
            // ── DISPLAY ERROR ONCE ──
          //  currentScreen = 20;
          //  update_display();        // One error screen
            buzzer.beep(3, 200, 100);
            
            Serial.printf("OTA Error: %s\n", err_msg);
            */
        });

    ArduinoOTA.begin();
    snprintf(ota_log, sizeof(ota_log), "Ready - waiting for firmware");
    otaModeActive = true;
    
    Serial.println("OTA READY - waiting for updates...");
}


void formatFloatValue(float value, char* intPart, char* decPart) {
    // Capture the integer part ONCE to avoid recasting
    uint16_t intVal = (uint16_t)value; // Truncates decimal

    if (value < 10.0) {
        // 2 decimal places (e.g., 2.18 -> "2" and "18")
        strcpy(intPart, u8x8_utoa(intVal)); // u8x8_utoa expects unsigned int
        uint16_t frac = (uint16_t)(value * 100) - (intVal * 100);
        strcpy(decPart, u8x8_u8toa(frac, 2)); // pad to 2 digits
    } else {
        // 1 decimal place (e.g., 22.1 -> "22" and "1")
        strcpy(intPart, u8x8_u8toa((uint8_t)intVal, 2)); // pad to 2 digits
        uint8_t frac = (uint8_t)(value * 10) - ((uint8_t)intVal * 10);
        strcpy(decPart, u8x8_utoa(frac));
    }
}

void splicer() {
    formatFloatValue(drying_temperature, temp_tens, temp_dec);
    formatFloatValue(drying_humidity, humi_tens, humi_dec);

    // Debug prints (still slightly redundant, but now only 2 lines per variable)
    Serial.print("Temp Tens: "); Serial.println(temp_tens);
    Serial.print("Temp Dec: ");  Serial.println(temp_dec);
    Serial.print("Humi Tens: "); Serial.println(humi_tens);
    Serial.print("Humi Dec: ");  Serial.println(humi_dec);
}

/*
void splicer(){ //temp and humi character slicer
  //NOT dtostrf(drying_temperature, 4, 1, temp_str);  dtostrf(drying_humidity, 4, 1, humi_str);
    
    if(drying_temperature<10.0) {strcpy(temp_tens, u8x8_utoa(drying_temperature)); //2 decimal guaz
          uint16_t carrier = uint16_t(drying_temperature*100) - (uint16_t(drying_temperature))*100; //break apart a 2.18 into 2 and 18
                  strcpy(temp_dec, u8x8_u8toa(carrier, 2)); // the 2 dp
      }
      else{ strcpy(temp_tens, u8x8_u8toa(drying_temperature, 2)); // 1 decimal gua
          uint8_t carrier = uint8_t(drying_temperature*10) - (uint8_t(drying_temperature))*10; //break apart a 22.1 into 22 and 1
                strcpy(temp_dec, u8x8_utoa(carrier));
      }

      Serial.print("Tens: "); Serial.println(temp_tens);
      Serial.print("Zeros: "); Serial.println(temp_dec);

      if(drying_humidity<10.0) {strcpy(humi_tens, u8x8_utoa(drying_humidity)); //2 decimal guaz
          uint16_t carrier = uint16_t(drying_humidity*100) - (uint16_t(drying_humidity))*100; //break apart a 2.18 into 2 and 18
                  strcpy(humi_dec, u8x8_u8toa(carrier, 2)); // the 2 dp
      }
      else{ strcpy(humi_tens, u8x8_u8toa(drying_humidity, 2)); // 1 decimal gua
          uint8_t carrier = uint8_t(drying_humidity*10) - (uint8_t(drying_humidity))*10; //break apart a 22.1 into 22 and 1
                strcpy(humi_dec, u8x8_utoa(carrier));
      }

      Serial.print("Tens: "); Serial.println(humi_tens);
      Serial.print("Zeros: "); Serial.println(humi_dec);




}
*/


char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
char Moonth[12][6] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};


float recent_soil_data[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; //6 slots ...but ... 5 readings
char recent_soil_moistures[6][8] = {"XX", "YY", "ZZ", "", "", ""}; //6 slots for 5 most recent readings e.g. 10.4%
char recent_soil_times[6][8] = {"xx:xx", "yy:yy", "10:35", "", "", ""}; //5 sets of seven each 10:35



char tens[8] = "AM ON"; char dec[3] = "."; 


//void footer(uint8_t activated_carbon = 1);

void update_display(){

    if(currentScreen == 1)      homeScreen();


    if(currentScreen == 2)     fanScreen(); //  shareScreen2();
        
    
    if(currentScreen == 6)      bluetoothScreen();
    


    if(currentScreen == 8){
                if(backlit){
                          digitalWrite(backlight_, LOW); backlit = false; 
                        }
                      sleep();

        }

}




bool bluetooth_state = false;
char bluetooth_status[20] = "Bluetooth ";

char time_str[12] = "4:25:03";

uint8_t active_tab = 1;




void homeScreen(){
     
  LCD.firstPage();
  do{

    header();


  //if(duoMode){

    LCD.setFont(u8g_font_helvR08r); // originally u8g_font_helvB08r but readability of BOLD with BOLD
    LCD.drawStr(5, 23, "Temperature"); /* || */  LCD.drawStr(78, 23, "Humidity");

  
      LCD.setFont(u8g2_font_logisoso22_tn);  // u8g2_font_logisoso22_tn most readable font at 5 meter distance away
   // LCD.setFont(u8g2_font_logisoso20_tn);
     
     
    LCD.drawStr(3, 52, temp_tens);

                  if(drying_temperature < 10.0){
                      LCD.drawStr(15, 42, ".");
                      LCD.drawStr(20, 52, temp_dec);
                  }
                  else {
                      LCD.drawStr(30, 42, dot);
                      LCD.drawStr(36, 52, temp_dec);
                  }

    //   HUMI  

  LCD.drawStr(68, 52, humi_tens);

                  if(drying_humidity < 10.0){
                      LCD.drawStr(81, 42, ".");
                      LCD.drawStr(85, 52, humi_dec);
                  }
                  else {
                      LCD.drawStr(96, 42, dot);
                      LCD.drawStr(102, 52, humi_dec);
                  }


  // Separator
   //  LCD.drawHLine(10, 27, 100);
     LCD.drawFrame(0, 13, 128, 42); LCD.drawHLine(0, 25, 128);
     LCD.drawVLine(66, 14, 40); // LCD.drawVLine(67, 16, 40); //LCD.drawVLine(68, 20, 35);


  // THE SYMBOLOZ
    LCD.setFont(u8g2_font_5x8_t_cyrillic); LCD.drawStr(51, 32, "o");

    LCD.setFont(u8g2_font_6x12_m_symbols); //is easier to read but very heavy on CPU resources
   // LCD.setFont(u8g_font_helvR08r); //u8g_font_helvR08r -- is compact in memory footprint
    LCD.drawStr(55, 37, "C");

    LCD.drawStr(117,36, "%");
    


  //}

  footer(1);


  
  } while(LCD.nextPage());


}



void fanScreen() {
    LCD.firstPage();
    do {
        // Optional: draw a common header (your header() function)
        header();   // if you have one

        // Main title
        LCD.setFont(u8g_font_helvB08r);  // bold for heading
        LCD.drawStr(12, 18, "STATUS OF FANS");

        // === Two columns ===
        int col1_x = 5;
        int col2_x = 68;    // half-width (128/2 + a few pixels)
        int y_start = 26;   // below heading
        int box_height = 30;

        // ---- Column 1: Cooling Fans ----
        LCD.drawFrame(col1_x, y_start, 58, box_height);
        LCD.setFont(u8g_font_helvR08r);
        LCD.drawStr(col1_x + 4, y_start, "Cooling");



        // Status text
        LCD.setFont(u8g2_font_logisoso20_tn);  // large font for ON/OFF
        const char* cooling_status = cooling_fans_on ? "ON" : "OFF";
        // Center the text in the box (roughly)
        int text_width = (cooling_fans_on ? 2 : 3) * 12; // approximate char width
        int text_x = col1_x + (58 - text_width) / 2;
        LCD.drawStr(text_x, y_start + box_height - 4, cooling_status);

        // Optional: filled circle indicator
        drawFanIcon(col1_x+10, 30);


        // ---- Column 2: Extraction Fans ----
        LCD.drawFrame(col2_x, y_start, 58, box_height);
        LCD.setFont(u8g_font_helvR08r);
        LCD.drawStr(col2_x + 2, y_start, "Extraction");

        LCD.setFont(u8g2_font_logisoso20_tn);
        const char* ext_status = extraction_fans_on ? "ON" : "OFF";
        text_width = (extraction_fans_on ? 2 : 3) * 12;
        text_x = col2_x + (58 - text_width) / 2;
        LCD.drawStr(text_x, y_start + box_height - 4, ext_status);

        // Indicator circle
                drawFanIcon(col2_x+10, 30);
  /*
        LCD.setFont(u8g2_font_6x12_m_symbols);
        if (extraction_fans_on) {
              LCD.drawDisc(col2_x + 4, y_start + 8, 3, U8G2_DRAW_ALL);
          } else {
              LCD.drawCircle(col2_x + 4, y_start + 8, 3, U8G2_DRAW_ALL);
          }
          */

        // Optional footer
       // footer(1);   // if you have one

    } while (LCD.nextPage());
}

void drawFanIcon(int x, int y) {
    // 30x30 fan icon, top-left corner at (x, y)
    int cx = x + 15;          // centre X
    int cy = y + 15;          // centre Y

    // Four filled triangles (blades) pointing toward centre
    // Right blade
    LCD.drawTriangle(cx, cy, cx + 11, cy - 6, cx + 11, cy + 6);
    // Down blade
    LCD.drawTriangle(cx, cy, cx + 6, cy + 11, cx - 6, cy + 11);
    // Left blade
    LCD.drawTriangle(cx, cy, cx - 11, cy + 6, cx - 11, cy - 6);
    // Up blade
    LCD.drawTriangle(cx, cy, cx - 6, cy - 11, cx + 6, cy - 11);

    // Central hub (filled circle, radius 4)
    LCD.drawDisc(cx, cy, 4, U8G2_DRAW_ALL);
}

void resultScreen(){
uint8_t page_count  = 1; 

//U8G2_BTN_SHADOW0
//STATICS
 //if(!share_screen_locked){ Serial.println("\tShare Screen Locked in"); share_screen_locked = true;
  
            LCD.firstPage();
            do { //Serial.print("Page: "); Serial.println(page_count);
                  //LCD.drawFrame(5, 10, 123, 54);
            
           
                 //HEAD
                 header();


                  //TABLE OF RESULTS
                 LCD.drawFrame(0, 9, 127, 22);
                 LCD.drawHLine(1, 20, 126);
                 LCD.drawVLine(50, 10, 20);                 LCD.drawVLine(76, 10, 20);                 LCD.drawVLine(102,10, 20);

                  
                 
                 //ROW 1
                  LCD.setFont(u8g_font_6x10);LCD.drawStr(2, 19, "Moisture");
                  LCD.setFont(u8g_font_helvB08); 
                  LCD.drawStr(54, 19, recent_soil_moistures[0]); 
                  LCD.drawStr(80, 19, recent_soil_moistures[1]);
                  LCD.drawStr(105,19, recent_soil_moistures[2]);
            
              //ROW 2
                  LCD.setFont(u8g_font_6x10); LCD.drawStr(10, 29, "Time");
                 // LCD.setFont(u8g_font_4x6); // LCD.setFont(u8g_font_5x8r);
                  //LCD.setFont(u8g_font_5x8); //works and is readable
                  LCD.setFont(u8g_font_5x7);
                  LCD.drawStr(52, 28, recent_soil_times[0]);
                  LCD.drawStr(78, 28, recent_soil_times[1]);
                  LCD.drawStr(103, 28, recent_soil_times[2]);


 //ACTION

                LCD.setColorIndex(1);  LCD.drawRBox(12, 34, 100, 15, 3);    //SHADOW
                LCD.setColorIndex(0);  LCD.drawRBox(11, 33, 100, 15, 3); 
                LCD.setColorIndex(1); LCD.drawRFrame(11, 33, 100, 15, 3);// Text Box
                

                LCD.setColorIndex(1);
                LCD.setFont(u8g_font_helvB08);
                LCD.drawStr(23, 44, "Read Soil Now"); 

   //FOOTER
                active_tab = 2;
                footer(active_tab);
                
                page_count++;
              } while(LCD.nextPage());

 

 //DYNAMIC VALUES

}

uint8_t blinking_cursor = 0;




uint8_t text_width = 40;

uint8_t move_sideways = 1;



uint8_t dj_selekta = 1;

void bluetoothScreen(){

 LCD.firstPage();
 do{
  LCD.setFont(u8g_font_7x13B);    LCD.drawStr(0, 10, "Available Devices"); 
  //LCD.setFont(u8g_font_helvR08);  LCD.drawStr(122, 10, dj_selekta);  // header --- u8g_font_6x10
   
  //LCD.setContrast(10);
 LCD.setFont(u8g_font_lucasfont_alternate);  //font for the nav bar
  //LCD.setPrintPos(104, 63); LCD.print("Back");


  LCD.drawBox(102,55,26,10);
  LCD.setColorIndex(0); LCD.drawStr(104, 63, "Data");
  
  // ... select and cancel should be exactly of the same font & in the same position at all times ... 
 //LCD.setContrast(255);
  LCD.setColorIndex(1);
      
 if(dj_selekta < 4){//flash first screen
  
  
  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,23,"1   Device 1"); 
  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,36,"2   Device 2"); 
  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,49,"3   Device 3"); 
  
  LCD.drawTriangle(60,58, 70,58, 64,63); // --- scroll down
  LCD.setFont(u8g_font_lucasfont_alternate);  LCD.drawBox(0,55,40,9);
  LCD.setColorIndex(0); LCD.drawStr(2, 63, "Select");  LCD.setColorIndex(1);
  
 if(dj_selekta==1){
  LCD.drawVLine(126,14,11); LCD.drawVLine(127,14,11); // -- scroll bar
//highlighter...
LCD.drawHLine(3, 24, 5); LCD.drawHLine(3,25,5); // --underLine
LCD.drawHLine(13, 12, 94); LCD.drawHLine(12, 25, 95);      LCD.drawVLine(11, 13, 12); LCD.drawVLine(107, 13, 12); //--outer border
LCD.drawHLine(12, 13, 96); LCD.drawHLine(12, 24, 96);      LCD.drawVLine(12, 13, 12); LCD.drawVLine(106, 13, 12); //--inner border
  }
  
else if(dj_selekta == 2){
   LCD.drawVLine(126,27,11); LCD.drawVLine(127,27,11); // -- scroll bar
//highlighter...
LCD.drawHLine(3, 37, 5); LCD.drawHLine(3,38,5); // --underLine
LCD.drawHLine(13, 25, 94); LCD.drawHLine(12, 38, 95);      LCD.drawVLine(11, 26, 12); LCD.drawVLine(107, 26, 12); //--outer border
LCD.drawHLine(12, 26, 96); LCD.drawHLine(12, 37, 96);      LCD.drawVLine(12, 26, 12); LCD.drawVLine(106, 26, 12); //--inner border


  }
  
 else if(dj_selekta == 3){
  LCD.drawVLine(126,41,11); LCD.drawVLine(127,41,11);
  //highlighter...
  LCD.drawHLine(3, 50, 5); LCD.drawHLine(3,51,5); // --underLine
  LCD.drawHLine(13, 38, 94); LCD.drawHLine(12, 51, 95);      LCD.drawVLine(11, 39, 12); LCD.drawVLine(107, 39, 12); //--outer border
  LCD.drawHLine(12, 39, 96); LCD.drawHLine(12, 50, 96);      LCD.drawVLine(12, 39, 12); LCD.drawVLine(106, 39, 12); //--inner border
  }

}
else { // if the second scrolled screen is in view port

  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,23,"4  Device 4"); 
  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,37,"5  Device 5"); 
  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(3,51,"6  Device 6"); 
  LCD.drawTriangle(65,58, 60,63, 69,63); // --- scroll up

/*
//highlighter...
LCD.drawHLine(1, 40, 126); LCD.drawHLine(1, 53, 126);      LCD.drawVLine(0, 41, 12); LCD.drawVLine(127, 41, 12); //--outer border
LCD.drawHLine(1, 41, 127); LCD.drawHLine(1, 52, 127);      LCD.drawVLine(1, 41, 12); LCD.drawVLine(126, 41, 12); //--inner border
*/

   if(dj_selekta == 4){
  //  String solenoid = "";
   // solenoid = valve_is_ON?"Switch OFF":"Switch ON";
    
    LCD.drawVLine(126,14,11); LCD.drawVLine(127,14,11); //-- scroll bar
    LCD.drawHLine(3, 24, 5); LCD.drawHLine(3,25,5); // --underLine
    LCD.drawHLine(12, 12, 95); LCD.drawHLine(11, 25, 96);      LCD.drawVLine(10, 13, 12); LCD.drawVLine(107, 13, 12); //--outer border
    LCD.drawHLine(11, 13, 97); LCD.drawHLine(11, 24, 97);      LCD.drawVLine(11, 13, 12); LCD.drawVLine(106, 13, 12); //--inner border
LCD.setFont(u8g_font_lucasfont_alternate);  LCD.drawBox(0,55,56,9); 
//LCD.setPrintPos(2, 63);  LCD.setColorIndex(0); LCD.print(solenoid);  LCD.setColorIndex(1);

    //LCD.setFont(u8g_font_helvR08);LCD.setPrintPos(100, 23);   LCD.print("24C"); 
// --- highlighter...

  }
  else if(dj_selekta == 5){
 //   String submersible = "";
//submersible = pump_is_ON?"Switch OFF":"Switch ON";

    LCD.drawVLine(126,28,11); LCD.drawVLine(127,28,11);
//highlighter...
LCD.drawHLine(3, 38, 5); LCD.drawHLine(3,39,5); // --underLine
LCD.drawHLine(12, 26, 95); LCD.drawHLine(11, 39, 96);      LCD.drawVLine(10, 27, 12); LCD.drawVLine(107, 27, 12); //--outer border
LCD.drawHLine(11, 27, 97); LCD.drawHLine(11, 38, 97);      LCD.drawVLine(11, 27, 12); LCD.drawVLine(106, 27, 12); //--inner border
LCD.setFont(u8g_font_lucasfont_alternate);  LCD.drawBox(0,55,56,9); 
LCD.setColorIndex(0); 
LCD.drawStr(2,63,"submersible");  LCD.setColorIndex(1);

  }

  else if(dj_selekta == 6){
     LCD.drawVLine(126,42,11); LCD.drawVLine(127,42,11);
//highlighter...
LCD.drawHLine(3, 52, 5); LCD.drawHLine(3,53,5); // --underLine
  LCD.drawHLine(12, 40, 95); LCD.drawHLine(11, 53, 96);      LCD.drawVLine(10, 41, 12); LCD.drawVLine(107, 41, 12); //--outer border
  LCD.drawHLine(11, 41, 97); LCD.drawHLine(11, 52, 97);      LCD.drawVLine(11, 41, 12); LCD.drawVLine(106, 41, 12); //--inner border

LCD.setFont(u8g_font_lucasfont_alternate);  LCD.drawBox(0,55,40,9);
LCD.setColorIndex(0); LCD.drawStr(2,63,"Select");  LCD.setColorIndex(1);
  
   }
 }

  
  
}while(LCD.nextPage());

   

}





void header(){

         //LCD.setFont(u8g_font_4x6); //shows but numbers and letters are very tiny to read
    // u8g_font_helvR08r u8g_font_6x10 u8g2_font_6x12_mf   //u8g_font_courR08 == somehow best, u8g_font_helvB08 == second best, u8g_font_helvR08 == third best
         
          LCD.setFont(u8g_font_helvR08); 

          LCD.drawStr(0, 8, DeviceDate); 
          LCD.drawStr(80, 8, DeviceTime);
         

    

                      //HEADER
                  
  /*                //LCD.drawHLine(5, 6, 118);
                  
                 LCD.setFont(u8g_font_5x8r); //u8g2_font_6x12_mf
                  strcpy(bluetooth_status, "Bluetooth ");

            if(bluetooth_state) strcat(bluetooth_status, "ON");
               else strcat(bluetooth_status, "OFF");

                  LCD.drawStr(0, 7, bluetooth_status);

                  LCD.setFont(u8g_font_4x6); 
                  LCD.drawStr(95, 6, time_str);
                  
*/

         // LCD.drawStr(15, 7, batt_percent);
    // uint x_pos = 25;

        


}

void battery_display(uint8_t x_pos) {
             
              LCD.drawVLine(x_pos, 59, 4); LCD.drawFrame((x_pos+1), 58, 15, 6);   

               //if battery is low... BUZZZ wulululuuuuuuu                

       if(voltage >= (3.2*2) && voltage < (3.6*2)){LCD.drawFrame((x_pos+14),59,1,5);} //empty
       if(voltage >= (3.6*2) && voltage < (3.7*2)) LCD.drawBox((x_pos+12),59,3,5);  // if quart 
  else if(voltage >= (3.7*2) && voltage < (3.9*2)) LCD.drawBox((x_pos+9),59,7,5); // if half
  else if(voltage >= (3.9*2) && voltage < (4.1*2)) LCD.drawBox(x_pos+4,1,60,4); // if 3-quarter
  else if(voltage > (4.1*2)){   LCD.drawBox(x_pos+2, 1, 61, 5);   } // if full


                  //LCD.drawHLine(5, 6, 118);
                  
         //         LCD.setFont(u8g_font_5x8r); //u8g2_font_6x12_mf
                 // strcpy(power_status, "Power ");

        //    if(power_is_enuf) strcat(power_status, "Good");
          //     else strcat(power_status, "Low");

            //      LCD.drawStr(0, 7, power_status);




}

void footer(uint8_t activated_carbon){
  //footer

     //LCD.setColorIndex(1);    LCD.drawBox(0, 56, 128, 7);

          char cooling_fans_char[10] = "OFF";
        //  LCD.setFont(u8g_font_5x7r);
          LCD.setFont(u8g_font_4x6);

          LCD.drawStr(10, 63, cooling_fans_on?"FANS(1): ON":"FANS(1): OFF");
          LCD.drawStr(75, 63, extraction_fans_on?"FANS(2): ON":"FANS(2): OFF");

           
          // LCD.drawStr(50, 60, current_screen_c);

        //  battery_display(110); 

          
          /*

  
  if(activated_carbon == 1){

                                    LCD.setColorIndex(0); LCD.drawBox(80, 55, 2, 8); //selector bar ... extreme right
                                    LCD.drawRBox(5, 50, 32, 13, 3); // ACTIVE TAB

                                    LCD.setFont(u8g_font_6x10); //u8g_font_courR08 == somehow best, u8g_font_helvB08 == second best, u8g_font_helvR08 == third best
                                    LCD.setColorIndex(1);
                                    LCD.drawStr(10, 62, "Home");

                                    LCD.setColorIndex(0); 
                                    LCD.setFont(u8g_font_courR08);
                                    LCD.drawStr(45, 62, "Read");
                                    LCD.drawStr(95, 62, "Data");
                                    LCD.setColorIndex(1);

           
  }

  else if(activated_carbon == 2){
                            
                            LCD.setColorIndex(0);         LCD.drawRBox(40, 50, 45, 13, 3);  //active tab
                                    
                            LCD.setFont(u8g_font_courR08); //u8g_font_courR08 == somehow best, u8g_font_helvB08 == second best, u8g_font_helvR08 == third best
                            LCD.setColorIndex(0);
                            LCD.drawStr(10, 62, "Home");
                            LCD.drawStr(95, 62, "Data");

                            LCD.setColorIndex(1);
                            LCD.setFont(u8g_font_6x10);
                            LCD.drawStr(50, 62, "Read");
                            
                            LCD.setColorIndex(1);
   
                            LCD.setFont(u8g_font_4x6); 

  
               

  }

  else {
                                     LCD.setColorIndex(0); LCD.drawBox(40, 55, 2, 8); //selector bar ... extreme left
                                    
                                     LCD.setColorIndex(0); LCD.drawRBox(90, 52, 32, 11, 3); //active tab
                                     //LCD.drawBox(80, 55, 2, 8); //divider
                                    
                                     LCD.setFont(u8g_font_courR08); //u8g_font_courR08 == somehow best, u8g_font_helvB08 == second best, u8g_font_helvR08 == third best
                                     LCD.setColorIndex(0);
                                     LCD.drawStr(10, 62, "Home");
                                     LCD.drawStr(50, 62, "Read");

                                     LCD.setColorIndex(1);
                                     LCD.setFont(u8g_font_6x10);
                                     LCD.drawStr(95, 62, "Data");

                                      

                                       LCD.setColorIndex(1);

                   
                  //  }

                //  LCD.drawBox(0, 53, 128, 11); LCD.setColorIndex(0);
                 // LCD.drawBox(48, 54, 35, 9);
                 // LCD.drawBox(90, 54, 35, 9);
                 //home, save, share, sleep
               
  }
  */
}



void popup(){
            LCD.setColorIndex(0);LCD.drawRBox(18, 18, 92, 38, 4);LCD.setColorIndex(1); LCD.drawRFrame(18, 18, 92, 38, 4); 

            if(move_sideways == 1){
                LCD.drawRBox(24,40,34,12, 2); LCD.setColorIndex(0);  LCD.drawStr(28, 49, "Share"); LCD.setColorIndex(1);
                LCD.drawStr(78, 49, "Quit"); LCD.drawRFrame(70, 40, 34, 12, 2); 
            }
            else{
                LCD.drawRBox(70,40,34,12, 2); LCD.setColorIndex(0);  LCD.drawStr(76, 49, "Open"); LCD.setColorIndex(1);
                LCD.drawStr(26, 49, "Share"); LCD.drawRFrame(22, 40, 34, 12, 2);
                }

}


void saveData(){}


 unsigned long irrig_duration_in_mins = 0; uint16_t irrimin = 0, irrisec = 0;

 uint8_t day = 0, month = 0, year = 0;


uint32_t counter = 0;

char irri_secs[3] = ""; 

void sleep(){ //our LOGO and maybe Makerere LOGO

//  if(backlit) {digitalWrite(backlight_, LOW);  backlit = false; }

  LCD.firstPage();
                do{
                  LCD.setFont(u8g_font_helvR08);  LCD.drawStr(20, 10, "Tap Home to Wake      "); LCD.setFont(u8g_font_courB08); //LCD.drawCircle(119,7,8);   // shade the status bar
                  LCD.setFont(u8g_font_7x13B);  LCD.drawStr(28, 32, "Sleeping...");  //LCD.setClipWindow(35, 11, 74, 27);
                
                  LCD.setFont(u8g_font_helvB08);  LCD.drawStr(2, 62, "Home"); //shade the menu bar 
                  LCD.drawFrame(0,53,32,11);

      }while(LCD.nextPage());
             //LARGE FORMAT CLOCK

}


/*

uint8_t shift_right = 1;
void reset_time(){
  LCD.firstPage();
    do{
      

  LCD.setFont(u8g_font_9x18B); // header --- u8g_font_6x10
     LCD.drawStr(8,12, "Reset the Time "); 
   // body
 // LCD.setPrintPos(1, 25);   LCD.print("HH : MM");  
     LCD.setFont(u8g_font_unifont);  LCD.drawStr(40, 28, "HH : MM");
     LCD.setFont(u8g_font_9x18B);    
     LCD.drawStr(40, 45, (temp_hr>9?temp_hr:("0"+String(temp_hr)))); LCD.print(" : "); LCD.print(temp_min>9?temp_min:("0"+String(temp_min))); 

    if(shift_right == 1) {LCD.drawFrame(38,32,22,15);}
    else if(shift_right == 2){LCD.drawFrame(82,32,22,15);}
  
      
  
  LCD.setFont(u8g_font_lucasfont_alternate); LCD.setPrintPos(2, 62);   LCD.print("Save");   
  LCD.setFont(u8g_font_lucasfont_alternate);    LCD.setPrintPos(104, 62); LCD.print("Back");
  
  LCD.drawFrame(0,53,27,11);
  LCD.drawFrame(101,53, 26, 11); 
  }while(LCD.nextPage());

}
*/


void time_is_set(){
  LCD.firstPage();
  do{

LCD.setColorIndex(0);LCD.drawRBox(18, 8, 92, 48,4);LCD.setColorIndex(1);
LCD.drawRFrame(18, 8, 92, 48, 4); 

LCD.setFont(u8g_font_helvB08);    LCD.drawStr(27, 26, "Data is Sent!"); 

LCD.setFont(u8g_font_lucasfont_alternate);    

  
      
  if(move_sideways == 1){
   LCD.drawBox(24,40,34,12); LCD.setColorIndex(0);  LCD.drawStr(28, 49, "Save"); LCD.setColorIndex(1);
   LCD.drawStr(78, 49, "Quit"); LCD.drawFrame(70, 40, 34, 12); 
  }
  else{
      LCD.drawBox(70,40,34,12); LCD.setColorIndex(0);  LCD.drawStr(76, 49, "Quit"); LCD.setColorIndex(1);
      LCD.drawStr(26, 49, "Save"); LCD.drawFrame(22, 40, 34, 12);
    }

//}

  }while(LCD.nextPage());
  
}


void notification_popup(int x = 18, int y = 8, char title[10] = "Title", char body[10] = "Body", int button_count = 1){
// x_pos otherwise default
// y_pos otherwise default
// Header
// Body
// Call to Action buttons

    char btn1[10] = ""; char btn2[10] = "";

    uint8_t side_shift = 1;

    //the ka boxis
    LCD.setColorIndex(0); LCD.drawRBox(x, y, 92, 48,4); LCD.setColorIndex(1); LCD.drawRFrame(x, y, 92, 48, 4); //the notification box

    //Eddy Mutwe
    LCD.setFont(u8g_font_helvB08);    /*LCD.print(which);*/ LCD.drawStr(40, 20, title); //LCD.drawHLine(19,21,90);

    //The Body
    LCD.setFont(u8g_font_lucasfont_alternate);  LCD.drawStr(22, 30, body);

    //The Mapeesa
    if(button_count == 2){ }// One big center button
    if(side_shift == 1){ } // when the selector is left
      LCD.drawRBox(24,38,34,12,2); LCD.setColorIndex(0); LCD.drawStr(28, 48, btn1); LCD.setColorIndex(1);
      LCD.drawStr(76, 48, btn2); LCD.drawRFrame(70, 38, 34, 12, 2); 
      LCD.setFont(u8g_font_lucasfont_alternate);//}
    /*
      else{
          LCD.drawBox(70,40,34,12); LCD.setColorIndex(0); LCD.setPrintPos(76, 49); LCD.print(btn2); LCD.setColorIndex(1);
      LCD.setPrintPos(26, 49); LCD.print(btn1); LCD.drawFrame(22, 40, 34, 12);
    }  */
}

/*
uint8_t scroll = 1;
void Home(uint8_t hr, uint8_t min, uint8_t sec, byte soil_Tank_field[], String PumpPrompt, String Moisture_Message, unsigned int * MSG, unsigned int * count_down){ // --- screen 9
//title bar -- 1.Network bars, 2.Saved Prompt icon, 3.
    char hawa[10], minat[10], sekond[10];
    hawa = (hr >=0 && hr <= 9)?("0"+String(hr)):String(hr);
    minat = (min >= 0 && min <= 9)?("0"+String(min)):String(min);
    sekond= (sec >= 0 && sec <= 9)?("0"+String(sec)):String(sec);
    //title bar

    LCD.firstPage();
    do{ // -- the status bar --- time, daynight, battery...

   // CLOCK //

  LCD.setFont(u8g_font_04b_03b); LCD.setPrintPos(75, 6);   
  LCD.drawStr(75, 6, hawa); LCD.print(":");
  LCD.print(minat); LCD.print(":");
  LCD.print(sekond);

// --- SAVED AND SENT ICONS --- //
  LCD.drawFrame(25,1,8,6); LCD.drawTriangle(25,1,29,1,27,4); //LCD.setPrintPos(37,6);  //LCD.print("SMS Box");
  
  String figurer;
  if(*MSG != 0){
         if(message_counter>99) figurer = "99+"; 
       else figurer = String(*MSG);
     LCD.setPrintPos(34,7);   LCD.print(figurer);
  }
  
  
  //download icon
  //LCD.drawVLine(42,0,3);   //  LCD.drawVLine(31,0,6); 
  //LCD.drawHLine(39,5,7); LCD.drawHLine(39,6,7); //downbar
  //LCD.drawLine(41,2,41,4); LCD.drawLine(44,2,41,4); // arrows --- LCD.drawLine(32,12,30,7); //LCD.setPrintPos(43,6);  //LCD.print("Arrow");
  

  //  NETWORK BARS   //
  LCD.drawVLine(55,4,2); LCD.drawVLine(56,4,2); 
  LCD.drawVLine(58,2,4); LCD.drawVLine(59,2,4); 
  LCD.drawVLine(61,0,6); LCD.drawVLine(62,0,6); 
  
  
// --- bool save_icon = true, send_icon = true;
// --- if(save_icon) {LCD.drawPixel()}  else {} // save failed!
// --- if(send_icon) {LCD.drawPixel();} else {} // power fail

  // TANK //
  //LCD.setPrintPos(0, 6); LCD.setFont(u8g_font_04b_03); LCD.print("Tank"); // --- LCD.print(soil_tank_field[1]);
 

  LCD.drawFrame(2,1,9,6);  LCD.drawHLine(5,0,3); //tank and head
       if(soil_Tank_field[1] >= 0 && soil_Tank_field[1] <= 10){LCD.drawFrame(2,1,9,6);}
       if(soil_Tank_field[1] > 10 && soil_Tank_field[1] < 25) LCD.drawBox(3,5,7,1);  // if quart 
  else if(soil_Tank_field[1] >= 25 && soil_Tank_field[1] < 50) LCD.drawBox(3,4,7,3); // if half
  else if(soil_Tank_field[1] >= 50 && soil_Tank_field[1] < 75) LCD.drawBox(3,3,7,4); // if 3-quarter
  else if(soil_Tank_field[1] > 75){LCD.drawBox(3,2,7,5);} // if full



//---title
    LCD.setFont(u8g_font_tpssb);  LCD.setPrintPos(36, 21);   LCD.print("irri-kit UI "); LCD.print(dim_screen);

// --- the MAIN SECTION
    LCD.drawFrame(10,22,105,30);LCD.drawFrame(12,24,101,26); // --- ebili muno bichyyuse every maybe 5seconds -> slide away --- 
    

    String  tank_levolo;
   String tanka = String(soil_Tank_field[1])+"%";
   String soyilo = String(soil_Tank_field[0])+"%";
   if(soil_Tank_field[1]==0) {tanka = "No Signal from Tank!";   tank_levolo = "";} //:String(soil_Tank_field[1]+"%");
   else {tank_levolo = "Tank Level: ";}
   if(soil_Tank_field[0]==0) {soyilo = "Sensor Out!"; }//:String(soil_Tank_field[0]+"V");
    
    if(scroll == 1){  
    LCD.drawVLine(110,25,10); LCD.drawVLine(111,25,10); // --- scroll bar
    LCD.setFont(u8g_font_helvR08);   
    LCD.setPrintPos(15, 34);   LCD.print("Soil Moisture: "); LCD.setFont(u8g_font_timR08); LCD.print(soyilo); 
    LCD.setFont(u8g_font_helvR08); 
    LCD.setPrintPos(15, 46);   LCD.print("Valve: ");  LCD.setFont(u8g_font_timR08); LCD.print(Moisture_Message); if(*count_down>0){LCD.print(*count_down);}
       
    LCD.drawTriangle(60,58, 70,58, 64,63); // --- scroll down
  }
    
  else if(scroll == 2){//then display the 2 already pre-customized prompts 
       LCD.drawVLine(110,40,10); LCD.drawVLine(111,40,10); // --- scroll bar
       LCD.setFont(u8g_font_helvR08);  // // u8g_font_helvR08
       LCD.setPrintPos(15, 34);   LCD.print("Pump: ");  LCD.setFont(u8g_font_timR08);  LCD.print("Not Connected"); 
       LCD.setFont(u8g_font_helvR08); 
       LCD.setPrintPos(15, 46);   LCD.print("Tank: "); LCD.setFont(u8g_font_timR08);LCD.print("Not connected"); 
    LCD.drawTriangle(60,58, 70,58, 64,63); // --- scroll down
   }


    LCD.setFont(u8g_font_timR08);  LCD.setPrintPos(92, 52); ////very nice font: u8g_font_fixed_v0
//    Y2 = year%100;

// --- THE TASK BAR
//LCD.setContrast(10);
  LCD.setFont(u8g_font_lucasfont_alternate);  //font for the nav bar
  LCD.drawBox(0,55,30,9);
  LCD.setPrintPos(2, 63);  LCD.setColorIndex(0); LCD.print("Menu");   //LCD.setPrintPos(104, 63); LCD.print("Back");

LCD.setColorIndex(1);
LCD.drawBox(98,55,30,10);
LCD.setPrintPos(100, 63); LCD.setColorIndex(0); LCD.print("Sleep");
  
   LCD.drawStr(30, 26, Moisture_Message); 
LCD.setPrintPos(20,32); LCD.print(*count_down);
LCD.drawRBox(24,36,34,12,2); LCD.setColorIndex(0); LCD.setPrintPos(32, 46); LCD.print("OK"); LCD.setColorIndex(1);
LCD.setPrintPos(72, 46); LCD.print("STOP"); LCD.drawRFrame(70, 36, 34, 12, 2); 
LCD.setFont(u8g_font_lucasfont_alternate);
    }

  
    }

    //if message sent


    
    }while(LCD.nextPage());
}


*/

void message_sent(){
  LCD.firstPage();
    do{



LCD.setColorIndex(0);LCD.drawRBox(18, 8, 92, 48,4);LCD.setColorIndex(1);
LCD.drawRFrame(18, 8, 92, 48, 4); 

LCD.setFont(u8g_font_helvB08);     LCD.drawStr(27, 26, "Shared By Bluetooth"); 

LCD.setFont(u8g_font_helvB14n);    

  
      
  if(move_sideways == 1){
   LCD.drawBox(24,40,34,12); LCD.setColorIndex(0);  LCD.drawStr(28, 49, "View"); LCD.setColorIndex(1);
   LCD.drawStr(78, 49, "Quit"); LCD.drawFrame(70, 40, 34, 12); 
  }
  else{
      LCD.drawBox(70,40,34,12); LCD.setColorIndex(0);  LCD.drawStr(76, 49, "Quit"); LCD.setColorIndex(1);
       LCD.drawStr(26, 49, "View"); LCD.drawFrame(22, 40, 34, 12);
    }

//}

}while(LCD.nextPage());
  
  }

//close the display class


// Draw a boot screen with three lines of text and a progress bar
void draw_boot_screen(const char* line1, const char* line2, const char* line3, int progress) {
    LCD.firstPage();
    do {
        LCD.setFont(u8g_font_7x13B);
        LCD.drawStr(2, 20, line1);
        LCD.drawStr(2, 35, line2);
        LCD.drawStr(2, 50, line3);

        // Progress bar
        LCD.drawFrame(14, 56, 100, 5);
        int bar_width = map(progress, 0, 100, 0, 100);
        if (bar_width > 0) LCD.drawBox(15, 57, bar_width, 4);
    } while (LCD.nextPage());
}


void Boot(){
  // BOOT SCREEN
  int loader = 0; int pos = 0;
  while(loader<99){ loader++; pos++;
    LCD.firstPage();
     do{
         
      if(loader <= 30){
         //LCD.setFont(); LCD.setFont(u8g_font_unifont);  LCD.setFont(u8g_font_9x18B);
            LCD.setFont(u8g_font_7x13B); LCD.drawStr(2, 20, device_name); 
            LCD.setFont(u8g_font_7x13B); LCD.drawStr(2, 40, DeviceID);   //LCD.drawStr(22, 30, "IntelliSys");   
    
      }

    //u8g_font_04b_03b u8g_font_04b_03 u8g_font_5x7 u8g_font_timR08 u8g_font_tpssb u8g_font_courB08
    else if(loader > 30 && loader <= 60){ 
     //LCD.setFont(u8g_font_helvR08);   LCD.drawStr(22, 30, "IntelliSys"); 
       LCD.setFont(u8g_font_5x7);   //   u8g_font_lucasfont_alternate u8g_font_unifont
            LCD.drawStr(0, 15, system_client);
            LCD.drawStr(0, 30, device_name);
            LCD.drawStr(2, 41, sensor_position);  
    }

    else if(loader > 60 && loader <= 80){
        LCD.setFont(u8g_font_7x13B);    
          LCD.drawStr(20, 10,"LEAD TEAM"); 
        LCD.setFont(u8g_font_helvR08);  
          LCD.drawStr(2, 26, "1. PROF EGONYU"); 
          // LCD.drawStr(2, 37, "2. ENG. D. BESIGYE"); 
          // LCD.drawStr(2, 40, "3. PROF PETER-T"); 
    
    }



  else {
        LCD.drawFrame(5,14,120,38); LCD.drawFrame(3,12,124,42);
        LCD.setFont(u8g_font_helvB08);  LCD.drawStr(20, 11,"DRYER SENSING"); 

        LCD.setFont(u8g_font_helvR08);  LCD.drawStr(8, 26, "1. Temperatute Sensing"); 
        LCD.setFont(u8g_font_helvR08);  LCD.drawStr(8, 37, "2. Humidity Sensing"); 
        LCD.setFont(u8g_font_helvR08);  LCD.drawStr(8, 48, "3. Air Flow Control"); 

  }
    LCD.drawFrame(14, 56, 100, 5);
    LCD.drawBox(15, 57, pos, 4);
      
     }while(LCD.nextPage());
  delay(10); //delete me
}

/*
delay(1000);

//display system settings
 LCD.firstPage();
 do{
//---title
    LCD.setFont(u8g_font_helvB08);  LCD.drawStr(26, 10, "Fertility Sensor"); 

// --- the MAIN SECTION
    LCD.drawFrame(10,22,105,30);LCD.drawFrame(12,24,101,26); // --- slide away every maybe 5seconds ->  --- 

    if(scroll == 1){  
      LCD.drawVLine(110,25,10); LCD.drawVLine(111,25,10); // --- scroll bar
      
      LCD.setFont(u8g_font_helvR08); 
        
      LCD.drawStr(15, 34, "1. Soil Sensor");   //LCD.print(" ");  //LCD.setFont(u8g_font_timR08); LCD.print(soyilo);
            
      LCD.drawStr(15, 46, "2. Sensor Data");   //LCD.print("2. : ");  //LCD.setFont(u8g_font_timR08); LCD.print(IrrigationPrompt); if(*count_down>0){LCD.print(*count_down);}
         
      LCD.drawTriangle(60,58, 70,58, 64,63); // --- scroll down triangle
  }
    
  else if(scroll == 2){//then display the 2 already pre-customized prompts 
       LCD.drawVLine(110,32,10); LCD.drawVLine(111,32,10); // --- scroll bar
       
       LCD.drawStr(15, 34, "1. Soil Sensor");   //LCD.print(" ");  //LCD.setFont(u8g_font_timR08); LCD.print(soyilo);
            
       LCD.drawStr(15, 46, "2. Sensor Data");   //LCD.print("2. : ");  //LCD.setFont(u8g_font_timR08); LCD.print(IrrigationPrompt); if(*count_down>0){LCD.print(*count_down);}
       
       LCD.drawTriangle(60,58, 70,58, 64,63); // --- scroll down triangle
   }

   else if(scroll == 3){ // soil_Tank_field[2]==0?"unplugged":soil_Tank_field[2]; // then disp batt and solar status
        LCD.drawVLine(110,39,10); LCD.drawVLine(111, 39, 10); // --- scroll bar
        
        LCD.drawStr(15, 34, "1. Soil Sensor");   //LCD.print(" ");  //LCD.setFont(u8g_font_timR08); LCD.print(soyilo);
            
        LCD.drawStr(15, 46, "2. Sensor Data");   //LCD.print("2. : ");  //LCD.setFont(u8g_font_timR08); LCD.print(IrrigationPrompt); if(*count_down>0){LCD.print(*count_down);}
        
        LCD.drawTriangle(65,58, 60,63, 69,63); // --- scroll up triangle
  }

    

  } while(LCD.nextPage());

*/

}






uint16_t batt_runs = 0; uint8_t Battery_Level_Counter = 0; char batt_percent[10];
void MonitorBattery(){ //Power Manage(battery, solar)
    
    /*
      * 2 VOLTAGES:
      * 1 during charging - charging voltage
      * 1 not charging - nominal voltage
      * when heavily loaded
    */

  Battery_Level_Counter = 0;
  uint16_t readBattery = 0;
  float summation = 0.00; 

  for(int i=0; i<10; i++){ //10 samples
      readBattery = analogRead(BatteryPin);
      summation += float(readBattery);
  }
         summation /= 10.0f;
      // voltage    = 15.00 * (summation/3095.0);
     //  voltage    = 14.50 * (summation/3095.0);
         voltage    = 2 * 4.45  * (summation/3095.0);

  //     also make the reading less jumpy

  // PRINT READINGS //
      //Serial.print("Battery Read Count: "); Serial.println(batt_runs);
      //Serial.print("Raw Reading: "); Serial.println(readBattery);
      
  /*
      char voltage_string[7];   //  dtostrf(voltage, -5, 2, voltage_string);
      dtostrf((voltage), -4, 3, voltage_string);
      Serial.print("Voltage String 1: "); Serial.println(voltage_string);
  */

  if(voltage < 7.00){
          power_is_enuf = false;
  } 
  else { power_is_enuf = true;
      if(voltage < 8.00){
          //
      }
      else {

      }
  }

    if(batt_runs%20 == 0){
        Serial.print("\t\tVoltage: "); Serial.print(voltage, 3); Serial.println("V"); 
        Serial.println(power_is_enuf?"\t\tPower Sufficient!":"\tPower Low!"); Serial.println();
    }

 batt_runs++;
}





