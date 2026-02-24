/* WIRELESS DRYER DATALOGGING SYSTEM FOR SUSAN MBACHO, FOR THE 13mX6m DRYER IN MUARIK*/
const char *server_address = "https://webofiot.com/dryer/muarik/server.php";

const char *devicename = "Dryer_Controller";
const char *OTA_PASS = "12!34";
bool otaModeActive = false;

char apiKeyValue[12] = "DRYER_006"; //MUARIK TOKEN

char ESP_IDF_VER[50] = "ESP-IDF Version: ";
char ARD_CORE_VER[50] = "Arduino Core Version: ";

void ESPInfo() {
    snprintf(ESP_IDF_VER, sizeof(ESP_IDF_VER), "ESP-IDF Version: %s", esp_get_idf_version());
    snprintf(ARD_CORE_VER, sizeof(ARD_CORE_VER), "Arduino Core Version: %s", ESP.getSdkVersion());
}

#define FW_VERSION "v1.3.5-Dryer_Datalogger"
#define USE_VSPI_FOR_EPD
// AND USE HSPI FOR SD and VSPI for E-PAPER

#include <Arduino.h>
#include "Wire.h"
#include <WiFi.h>
#include <esp_now.h>

#include "ArduinoOTA.h"
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include <stdlib.h>
#include <math.h>


#include "RTClib.h"


#include "Adafruit_GFX.h"
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>

#include "SPI.h"
#include "FS.h"
#include "SD.h"


#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include "credentials.h"
#include "WiFi_Manager.h"
#include "upload.h"
#include "Buzzer.h"
#include "timer_keeper.h"

#include <ArduinoJson.h>
StaticJsonDocument<2000> JSON_data;
//JsonDocument JSON_data; // for dynamic amounts of data
StaticJsonDocument<4096> JSON_sendable;

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> 
#include <Fonts/FreeSans24pt7b.h> 

#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

#include <Preferences.h>
Preferences prefs;


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

static const uint16_t SCREEN_W = 400;
static const uint16_t SCREEN_H = 300;

// base class GxEPD2_GFX can be used to pass references or pointers to the LCD instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0


//pins
const uint8_t ota_button  = 12;
const uint8_t buzzerpin = 14;
const uint8_t wifi_led = 32; // this

const uint8_t batteryPin = 36; //27; // ADC 2 pins cannot work concurrently with WiFi
const uint8_t wind_speed_pin = 34;
const uint8_t solar_rad_pin = 35;

const uint8_t blinker = 13;
const uint8_t innerFAN = 32; // this

const uint8_t sensor_indicator = 17;
const uint8_t wifi_toggler = 16;
/*
const uint8_t cooling_fan_pin = ;//12; 12 is also taken
const uint8_t humidity_fan_pin = ;// 14; 14 is taken
const uint8_t night_Light = //27; // 27 is taken
*/





 //HSPI (Second hardware SPI bus)
//SCK → GPIO 14, MISO → GPIO 12, MOSI → GPIO 13, CS → GPIO 15
// REMAPPED TO  MOSI:25, CS:4 SCK:26

#define SCREEN_SCK 26
#define SCREEN_MISO -1 // NOT NEEDED
#define SCREEN_MOSI 25 
#define SCREEN_CS 4
uint8_t  DC_ = 2, RES_ = 33, BUSY_ = 27;
// 4.2 EPD Module
 //  GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> LCD(GxEPD2_420_GDEY042T81(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683
     GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> LCD(GxEPD2_420c_GDEY042Z98(SCREEN_CS, DC_, RES_, BUSY_)); // 400x300, SSD1683
 

SPIClass EPD_SPI(HSPI);


 //VSPI (Default SPI object = SPI)
//SCK → GPIO 18, MISO → GPIO 19, MOSI → GPIO 23, CS → GPIO 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_Chip_Select 5 // USES HW VSPI => 5, 18, 19, 23

// connected to ESP32 VSPI CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,
SPIClass SD_SPI(VSPI);  // SD exciplitely uses the default: VSPI

/*
SD SPI max (stable): 20–25 MHz

Many cards fail above 10–16 MHz Some will mount but fail writes

Pins you should NEVER put SD on
Avoid for CS / CLK / MISO / MOSI:

GPIO 0
GPIO 2
GPIO 12
GPIO 15
*/





bool json_bound_successfully = false;
bool csv_bound_successfully = false;
         
bool wifi_connected = false;
uint32_t WiFi_Strength = 0;
char ota_log[150] = "...";
volatile bool otaFinished = false;
volatile bool otaStarted = false;
volatile bool otaError = false;
volatile bool otaProgress = false;

uint16_t attempts = 0;
uint16_t upload_fails = 0;

const uint64_t upload_frequency_ms   = (10ULL * 60ULL * 1000ULL); // 10 minutes
uint64_t last_upload_time_ms = 0;

char last_upload_time[32] = "Never!";

char upload_error_code[100] = "";


int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;


// Power management pins


float cabin_temperature = 0.0f;
bool innerFan_ON = false;
bool night_Light_ON = false;

static char internals_log[255] = "NULL"; // nker relocation issue that the ESP32 toolchain sometimes throws when you put large-ish global/static arrays into flash/DRAM in a certain way.


float average_temp = 0.0f; char average_temp_str[10] = "x.x"; // 35.855
float average_humi = 0.0f; char average_humi_str[10] = "x.x"; // 75.695
float average_press = 0.0f; 
float average_elev = 0.0f;
float new_average_temp = 0.0f;

// readings for up to 10 minutes ago
float historical_outside_temperatures[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float historical_temperature_readings[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float historical_humidity_readings[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float historical_pressure_readings[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
char  time_stamps[10][12] = {"--:--", "--:--"}; 

char last_json_save_time[32] = "JSON Never Saved!";
char last_csv_save_time[32] = "CSV Never Saved!";

char printable_temp_resp[10] = "_";
char printable_humi_resp[10] = "_";


float average_wind_speed = 0.0f; char wind_speed_str[10] = ".";
float average_solar_radiation = 0.0f; char solar_rad_str[10] = ".";


upload Uploader(wifi_led, server_address);

//example datapack
//char httpsData[2000] = "{\"PassKey\":\"Rwebi_Weather\", \"Air_Speed\":14.7, \"Air_Temperature\":35.1, \"Air_Humidity\":56.7, \"Sunlight\":1500}";

bool storage_initialized = false;
float sd_storage_size = 0.0f;
float freeSpace = 0.0f;
uint64_t usedSpace = 0;

char storage_size_char[64];



uint8_t hr = 0, mint = 0, sec = 0, day_ = 0, mth = 0, yr = 0; uint16_t mwaka = 2000; 

char ShortDate[24] = "26-08-2025"; 



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


// time series averages

uint8_t active_sensors = 0;
float temperature_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float humidity_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float pressure_readings[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float sensor_1_temp_readings[30];
float sensor_1_humi_readings[30];
float sensor_1_press_readings[30];


uint64_t last_seen[12] = {0, 0, 0, 0, 0, 0};




float sensor_1_time_series_readings[30]; // up to 10 readings ago
char sensor_1_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

float sensor_2_time_series_readings[30]; // up to 30 readings
char sensor_2_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};


char voltage_string[10] = ""; 
double voltage = 0.00;
uint8_t no_of_batts = 1; // 1 for 12V, 2 batts for 24V, 4 batts for 48V

char initializer[100] = "Now Booting...";
bool flipped = false;
uint8_t currentScreen = 0; // bootscreen by default


uint8_t DISP_MODE = 0;

// DATASET 1
float sensor_1_temp, sensor_2_temp, sensor_3_temp, sensor_4_temp, sensor_5_temp, sensor_6_temp;
float sensor_7_temp, sensor_8_temp, sensor_9_temp, sensor_10_temp, sensor_11_temp, sensor_12_temp;
// DATASET 2
float sensor_1_humidity, sensor_2_humidity, sensor_3_humidity, sensor_4_humidity, sensor_5_humidity, sensor_6_humidity;
float sensor_7_humidity, sensor_8_humidity, sensor_9_humidity, sensor_10_humidity, sensor_11_humidity, sensor_12_humidity;
// DATASET 3
float sensor_1_pressure, sensor_2_pressure, sensor_3_pressure, sensor_4_pressure, sensor_5_pressure, sensor_6_pressure;
float sensor_7_pressure, sensor_8_pressure, sensor_9_pressure, sensor_10_pressure, sensor_11_pressure, sensor_12_pressure;
// DATASET 4
float sensor_1_elevation, sensor_2_elevation, sensor_3_elevation, sensor_4_elevation, sensor_5_elevation, sensor_6_elevation;
float sensor_7_elevation, sensor_8_elevation, sensor_9_elevation, sensor_10_elevation, sensor_11_elevation, sensor_12_elevation;
// DATASET 5
float sensor_7_h_index, sensor_8_h_index, sensor_9_h_index, sensor_10_h_index, sensor_11_h_index, sensor_12_h_index;
float sensor_1_h_index, sensor_2_h_index, sensor_3_h_index, sensor_4_h_index, sensor_5_h_index, sensor_6_h_index;
// DATASET 6
char  sensor_1_transmissions[16]; char  sensor_2_transmissions[16];   char  sensor_3_transmissions[16];   char  sensor_4_transmissions[16]; 
char  sensor_5_transmissions[16]; char  sensor_6_transmissions[16];   char  sensor_7_transmissions[16];   char   sensor_8_transmissions[16]; 
char  sensor_9_transmissions[16]; char  sensor_10_transmissions[16];  char  sensor_11_transmissions[16];  char  sensor_12_transmissions[16]; 
// DATASET 7
char sensor_1_CPU_freq[12]; char sensor_2_CPU_freq[12]; char sensor_3_CPU_freq[12]; char sensor_4_CPU_freq[12];
char sensor_5_CPU_freq[12]; char sensor_6_CPU_freq[12]; char sensor_7_CPU_freq[12]; char sensor_8_CPU_freq[12];
char sensor_9_CPU_freq[12]; char sensor_10_CPU_freq[12]; char sensor_11_CPU_freq[12]; char sensor_12_CPU_freq[12];
// DATASET 8
uint64_t sensor_1_running_time_ms, sensor_2_running_time_ms, sensor_3_running_time_ms, sensor_4_running_time_ms;
uint64_t sensor_5_running_time_ms, sensor_6_running_time_ms, sensor_7_running_time_ms, sensor_8_running_time_ms;
uint64_t sensor_9_running_time_ms, sensor_10_running_time_ms, sensor_11_running_time_ms, sensor_12_running_time_ms;
// DATASET 9
size_t sensor_1_packet_size = 0; size_t sensor_2_packet_size = 0; size_t sensor_3_packet_size = 0; size_t sensor_4_packet_size = 0;
size_t sensor_5_packet_size = 0; size_t sensor_6_packet_size = 0; size_t sensor_7_packet_size = 0; size_t sensor_8_packet_size = 0;
size_t sensor_9_packet_size = 0; size_t sensor_10_packet_size = 0; size_t sensor_11_packet_size = 0; size_t sensor_12_packet_size = 0;
// DATASET 10
char sensor_1_last_seen[12] = "--:--"; char sensor_2_last_seen[12] = "--:--"; char sensor_3_last_seen[12] = "--:--"; char sensor_4_last_seen[12] = "--:--";
char sensor_5_last_seen[12] = "--:--"; char sensor_6_last_seen[12] = "--:--"; char sensor_7_last_seen[12] = "--:--"; char sensor_8_last_seen[12] = "--:--";
char sensor_9_last_seen[12] = "--:--"; char sensor_10_last_seen[12] = "--:--"; char sensor_11_last_seen[12] = "--:--"; char sensor_12_last_seen[12] = "--:--";

                      

//function prototypes
void read_temperatures();
void query_rtc();
void update_display();
void Boot();
void homepage();
void tablePage();
void graphPage();
void filesPage();
void logsPage();
void otaPage();
void errorPage();
void cloudPage();
void footer();

bool special_call = false;

void upload_to_web_of_iot();
bool initialize_sd_card(); //USES HSPI HARDWARE SERIAL

bool save_csv_to_sd_card();
bool save_json_to_sd_card();
void initialize_RTC();

bool sd_initialized = false;
char save_log[512] = "No JSON Saved!";
char json_save_log[512] = "Not Saved!";
char csv_save_log[512] = "not Saved!";

char dataPack[250] = ""; // receivable
void extract_readings();

void OnSensorData_received(const uint8_t * mac, const uint8_t *incomingData, int len);

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

uint8_t side_scroll = 1; // default = 1


uint64_t humi_fans_started = 0, cooling_fans_started = 0;

uint64_t now_now_ms = 0, prev = 0, last_read_time_ms = 0, last_refresh_time_ms = 0; // for DUE, millis() times out at [4.9Bn] 4,294,967.295 seconds which is 49.7 days
uint64_t otaStartTime = 0; 
const uint64_t OTA_TIMEOUT = 15ULL * 60ULL * 1000ULL;  // 15 minutes
char LastOTAUpdate[60] = "27 Oct 2025 10:10";
//char LastOTAUpdate[60] = "27 Oct 2025 10:10";
volatile uint64_t totalBytes = 0; /// = SD.totalBytes();
volatile uint64_t usedBytes = 0; //  = SD.usedBytes();
volatile uint64_t save_counter = 0; // uint8_t save_counter = 0;

volatile bool packet_received = false;

char sendable_to_cloud_db[3000] = ""; 
char sendable_to_sd_card[4000] = ""; // JSON ~3kiB
char sendable_to_sd_card_csv[3000] = "";
char storage_status_log[512] = "NO SD";

#define PWRKEY 0

void setup() { delay(50); // for OTA to fully hand over
       Serial.begin(115200); Serial.print(devicename); Serial.println(" Booting..."); // while (!Serial) delay(100);   // wait for native usb

  // INITIALIZING ALARM
        buzzer.begin();
        xTaskCreatePinnedToCore(BuzzingTask, "BuzzingCheck", 4096, NULL, 4, NULL, 1);
        buzzer.beep(1,50,0);       
  
        pinMode(blinker, OUTPUT); digitalWrite(blinker, HIGH); delay(500); buzzer.beep(1,50,0);

     //   pinMode(SCREEN_CS, OUTPUT); digitalWrite(SCREEN_CS, HIGH);
     //   pinMode(SD_Chip_Select, OUTPUT); digitalWrite(SD_Chip_Select, HIGH);

  //      Serial2.setTimeout(500); delay(500); // read for 500ms
        //  Serial2.begin(115200, SERIAL_8N1, 16, 17); //the GSM MOD

         // Ensure both CS pins are HIGH
          pinMode(SCREEN_CS, OUTPUT);
          digitalWrite(SCREEN_CS, HIGH);
          pinMode(SD_Chip_Select, OUTPUT);
          digitalWrite(SD_Chip_Select, HIGH);
          delay(100);

        sd_initialized = initialize_sd_card(); //USES HSPI HARDWARE SERIAL


        if (sd_initialized) {
            check_storage_files();
        }
        delay(1000);

            // Initialize Preferences and load counter
        prefs.begin("SAVES", false);
        save_counter = prefs.getUInt("SAVES", 0);  // Load saved value, default 0
        prefs.end();
        
        
        // --- Initialize EPD on VSPI ---
        strcpy(initializer, "Initializing E-Paper Display over VSPI...");
        Serial.println(initializer);


        EPD_SPI.begin(SCREEN_SCK, SCREEN_MISO, SCREEN_MOSI, SCREEN_CS); // CUSTOM SPI PINS
        LCD.epd2.selectSPI(EPD_SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
        
        LCD.init(115200, true, 50, false);  

        strncpy(initializer, "Initialied display!", sizeof(initializer));  Serial.println(initializer);
         special_call = true; // to bypass the timer
         currentScreen = 0;
         update_display();
         // currentScreen = 2;
         delay(1000);
        
          

    /*
        Perform the read operation: Use a function that respects the timeout, such as readBytes(), readString(), or parseInt(). 
        If no data is received within the specified timeout period, these functions will return an appropriate indicator 
        (e.g., 0 for readBytes(), an empty string for readString(), or 0 for parseInt() if no valid number is found).
    */


    digitalWrite(blinker, HIGH); delay(500);
    digitalWrite(blinker, LOW); delay(500);



      pinMode(batteryPin, INPUT);

    pinMode(innerFAN, OUTPUT); digitalWrite(innerFAN, LOW); 
    
    
    Serial.print("Starting Realtime Clock...");
    initialize_RTC(); delay(100);
    query_rtc(); delay(100);




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

        delay(1000);

       /*
       switch_radio_to_wifi();
     
       //wifi_connected = wifi_obj.ensure_wifi();
       initializeOTA();
       
    
      */
        // Uploader.begin(); // leave it because it uses WiFi
        

        digitalWrite(blinker, HIGH); delay(1000);
        digitalWrite(blinker, LOW);

        
    // Initialize timers AFTER hardware is ready
      last_read_time_ms = esp_timer_get_time() / 1000ULL;
      last_refresh_time_ms = esp_timer_get_time() / 1000ULL;
      last_upload_time_ms = esp_timer_get_time() / 1000ULL;
      Serial.println("Schedulers and Timers Set");
      delay(1000);

        Serial.println("Starting Reads");
           // pinMode(batteryPin, INPUT); delay(500); // If batteryPin is GPIO34/35/36/39, they MUST NOT use pinMode() at all
        MonitorBattery();


       for(int i = 0; i < 1; i++){
        digitalWrite(innerFAN, HIGH);
       // buzzer.beep(1,50,0);
        delay(500);

        digitalWrite(innerFAN, LOW);
        buzzer.beep(1,50,0);
        delay(500);

                
    }
        uint16_t poll = 0;
        while(poll<2000){ // keep a bit here for 2 seconds
            delay(1); poll++;
            if(packet_received) { buzzer.beep(1,50,0);  extract_readings(); packet_received = false; }
        }

        monitor_box_conditions();

        monitor_dryer_readings(); 

           update_display(); // HOME SCREEN 
               delay(1000);
            special_call = false;
        Serial.println("Screen set to home page");

          // IF THIS SYSTEM IS EVER TO HAVE FANS
      
  //    pinMode(cooling_fan_pin, OUTPUT);
   //   pinMode(humidity_fan_pin, OUTPUT);

        pinMode(ota_button, INPUT_PULLUP);
        pinMode(wifi_toggler, INPUT_PULLUP);

        powerOnSIM();
        
         buzzer.beep(2,50,50);  // END OF BOOT
         Serial.println("Done Booting!");
    

}


void powerOnSIM() {
  pinMode(PWRKEY, OUTPUT);

  digitalWrite(PWRKEY, HIGH);
  delay(100);

  digitalWrite(PWRKEY, LOW);   // Pull LOW
  delay(1500);                 // Hold 1–2 seconds
  digitalWrite(PWRKEY, HIGH);  // Release
  delay(5000);                 // Wait for boot
}

void BuzzingTask(void * pvParams) { 
  uint8_t checkin_frequency = 10; 
  const TickType_t xDelay = pdMS_TO_TICKS(checkin_frequency);
  while (true) {
    buzzer.update();
    vTaskDelay(xDelay);
  }
}


const char *file_heading = "Report,for,Hybrid,Solar,Dryer,at,Makerere,University,Agricultural,Research,Institute, Kabanyolo,(MUARIK)\n";
char file_creation_date[64] = "File created:,,,";
char logging_interval[52] = "Logging Interval (HH:MM:SS):,,,00:01:00"; // once every minute
const char data_points_log[128] = "Data Points:,,Temperature, Relative Humidity, Barometric Pressure inside the Dryer (1-11) and ambient / outside the dryer (12)";
const char units_of_data[156] = "Temperature is taken in Celcius Degrees (°C) | Relative Humidity in percent (%) | Barometric Pressure in kilopascals (kPa)";
const char additional_entries[156] = "Microclimatic conditions measured: (2) wind speed in metres per second (m/s) and solar radiation in Watts per square metres (W/m²)";

size_t bytesWritten = 0;
bool writeSuccess = false;
  


static char *csv_file = "/data.csv";
char new_name[64];
static char json_log_file[64] = "/data.json"; // primary storage file
static char csv_log_file[64] = "/data.csv"; // secondary storage file

static const size_t MAX_FILE_SIZE = 20 * 1024 * 1024; // 20 MB per file
static const size_t MAX_TOTAL_SIZE = 100 * 1024 * 1024; // 100 MB total limit

size_t jsonSize = 0; // base file ...  before rotating
size_t csvSize  = 0; // base file ...  before rotating

size_t json_currentSize = 0;
size_t csv_currentSize = 0;


void enable_sd_card(){
  
    digitalWrite(SCREEN_CS, HIGH); delay(10);
    digitalWrite(SD_Chip_Select, LOW);        

}

void enable_display(){
    digitalWrite(SD_Chip_Select, HIGH); delay(10);
    digitalWrite(SCREEN_CS, LOW);        

}

    

bool initialize_sd_card() { // “Is the SD card hardware usable?”
    LOG("-------- INITIALIZING ONSITE STORAGE ------------------");

    bool status = false;

    memset(storage_status_log, 0, sizeof(storage_status_log));
    strncpy(storage_status_log, "SD: ", sizeof(storage_status_log));

    // --- Initialize VSPI for SD ---
    /*
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_Chip_Select);

    storage_initialized = SD.begin(SD_Chip_Select, SD_SPI, 4000000);
    */
    storage_initialized = SD.begin(SD_Chip_Select);

    if (!storage_initialized) {
        snprintf(storage_status_log, sizeof(storage_status_log),
                 "SD CARD failed to initialize. Check wiring/power.");
        LOG(storage_status_log);
        return false;
    }

    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        snprintf(storage_status_log, sizeof(storage_status_log),
                 "NO SD CARD detected");
        storage_initialized = false;
        LOG(storage_status_log);
        return false;
    }

    uint64_t cardSize = SD.cardSize();
    if (cardSize == 0) {
        strncat(storage_status_log, "Size: Unknown",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);
        storage_initialized = false;
        LOG(storage_status_log);
        return false;
    }

    sd_storage_size = float(cardSize / (1024.0 * 1024.0));
    sd_storage_size /= 1000.0;

    snprintf(storage_size_char, sizeof(storage_size_char),
             "%.2fGB", sd_storage_size);

    strncat(storage_status_log, storage_size_char,
            sizeof(storage_status_log) - strlen(storage_status_log) - 1);

    LOG(storage_status_log);

    status = true;
    return status;
}

bool check_storage_files() {

    if (!storage_initialized) {
         strncat(storage_status_log, " SD: Init Error. Will retry later",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);
        return false;
    }

    bool status = false;
        /*
                usedSpace = (SD.usedBytes()) / (1024 * 1024);
                //usedSpace /= 1000.0;
                freeSpace = (sd_storage_size*1000) - (float)(usedSpace);
                freeSpace /= 1000.0;
        */

         totalBytes = SD.totalBytes();
         usedBytes  = SD.usedBytes();

        float usedMB = usedBytes / 1048576.0;      // avoids intermediate rounding of  1024*1024
        float freeMB = (totalBytes - usedBytes) / (1024.0 * 1024.0);

        float freeGB = freeMB / 1024.0; // using binary units ... 1073741824.0 from 1024*1024*1024

   

    snprintf(storage_status_log + strlen(storage_status_log),
             sizeof(storage_status_log) - strlen(storage_status_log),
             ", Used: %.2fMB, Free: %.2fGB", usedMB, freeGB);

    // Check/create JSON log file
    if (!SD.exists(json_log_file)) {

        strncat(storage_status_log, " | Creating new log file",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);

        File f = SD.open(json_log_file, FILE_WRITE);
        if (!f) {
            strncat(storage_status_log, " | FAILED to create file",
                    sizeof(storage_status_log) - strlen(storage_status_log) - 1);
            return false;
        }
        f.close();
    }

    File f = SD.open(json_log_file, FILE_READ);
    if (!f) {
        strncat(storage_status_log, " | Cannot open file for size check",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);
        return false;
    }

     json_currentSize = f.size();

    f.close();

    
    File f2 = SD.open(csv_log_file, FILE_READ);
    if (!f2) {
        strncat(storage_status_log, " | Cannot open file for size check",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);
        return false;
    }

    size_t csv_currentSize = f2.size();

    f2.close();

    if (json_currentSize <= MAX_FILE_SIZE) { // at most 20MB 

            float size_of_json_file = ((float)(json_currentSize))/1048576.0; // // 1024*1024
            float size_of_csv_file = ((float)(csv_currentSize))/1048576.0; // 1024*1024

        snprintf(storage_status_log + strlen(storage_status_log),
                 sizeof(storage_status_log) - strlen(storage_status_log), 
                 ", JSON: %.2f MB, CSV: %.2fMB", size_of_json_file, size_of_csv_file);

        status = true;
   
    } else {
        
        strncat(storage_status_log, " | File needs rotation",
                sizeof(storage_status_log) - strlen(storage_status_log) - 1);

        uint64_t totalUsed =  usedBytes; // SD.usedBytes();

        if (totalUsed > MAX_TOTAL_SIZE) {

            strncat(storage_status_log, " | TOTAL STORAGE LIMIT REACHED",
                    sizeof(storage_status_log) - strlen(storage_status_log) - 1);

            return false;
        }

        File newFile = create_another_file();

        if (newFile) {
            strncat(storage_status_log, " | File rotated",
                    sizeof(storage_status_log) - strlen(storage_status_log) - 1);
            newFile.close();
            status = true;
        } else {
            strncat(storage_status_log, " | Rotation FAILED",
                    sizeof(storage_status_log) - strlen(storage_status_log) - 1);
        }

            }

    LOG(storage_status_log);
    return status;
}

File create_another_file() {
    // Make sure ShortDate and ShortTime are defined and valid
    if (strlen(ShortDate) == 0 || strlen(ShortTime) == 0) {
        // Use timestamp as fallback
        snprintf(new_name, sizeof(new_name), "/data_%lu.json", millis());
    } else {
        snprintf(new_name, sizeof(new_name), "/data_%s_%s.json", 
                 ShortDate, ShortTime);
        
        // Replace invalid characters for FAT compatibility
        for (char *p = new_name; *p; p++) {
            if (*p == ':' || *p == '/' || *p == '\\' || *p == '*' || 
                *p == '?' || *p == '"' || *p == '<' || *p == '>' || *p == '|') {
                *p = '-';
            }
        }
    }
    
    // Log the rotation
    char new_name_log[100];
    snprintf(new_name_log, sizeof(new_name_log), 
             "[SD] Rotating to: %s", new_name);
    LOG(new_name_log);
    
    // Update global filename
    strncpy(json_log_file, new_name, sizeof(json_log_file) - 1);
    json_log_file[sizeof(json_log_file) - 1] = '\0';
    
    // Create new file and write metadata
    File newFile = SD.open(new_name, FILE_WRITE);
    if (!newFile) {
        LOG("[SD] ERROR: Failed to create rotated file");
        // Fallback to original filename
        strncpy(json_log_file, "/data.json", sizeof(json_log_file) - 1);
        json_log_file[sizeof(json_log_file) - 1] = '\0';
        newFile = SD.open(json_log_file, FILE_APPEND);
    } else {
        // Write initial metadata to the new file
        write_json_metadata(newFile);
    }
    
    return newFile;
}


bool handle_sd_recovery();
void handle_empty_payload();
void handle_filename_issue();
bool handle_file_creation();
bool handle_large_file_issue(size_t weight_under_review);
void handle_open_failure();
void handle_timeout(size_t bytesWritten);
void handle_separator_corruption(size_t separatorWritten);
void handle_write_failure(size_t bytesWritten, size_t incomingSize);
void handle_periodic_maintenance();

bool timeoutOccurred = false;


bool save_json_to_sd_card() {

    memset(json_save_log, 0, sizeof(json_save_log));
    bool writeSuccess = false;
    timeoutOccurred = false;

    // =========================================================
    // 1. Ensure SD initialized
    // =========================================================
    if (!storage_initialized) {
        if (!handle_sd_recovery()) return false;
    }

    // =========================================================
    // 2. Validate input
    // =========================================================
    if (!sendable_to_sd_card || strlen(sendable_to_sd_card) == 0) {
        handle_empty_payload();
        return false;
    }

    // =========================================================
    // 3. Validate filename
    // =========================================================
    if (strlen(json_log_file) <= 50) {
        handle_filename_issue();
    }

    // =========================================================
    // 4. Ensure file exists
    // =========================================================
    if (!SD.exists(json_log_file)) {
        if (!handle_file_creation()) return false;
    }

    // =========================================================
    // 5. Size check / rotation
    // =========================================================
    size_t incomingSize = strlen(sendable_to_sd_card);

    if ((json_currentSize + incomingSize) > MAX_FILE_SIZE) {
        if (!handle_large_file_issue(json_currentSize + incomingSize))
            return false;
    }

    // =========================================================
    // 6. Open file
    // =========================================================
    File file = SD.open(json_log_file, FILE_APPEND);
    if (!file) {
        handle_open_failure();
        return false;
    }

    // =========================================================
    // 7. Write with timeout protection
    // =========================================================
    const uint32_t WRITE_TIMEOUT_MS = 10000UL;
    uint32_t writeStartTime = millis();

    size_t bytesWritten = file.print(sendable_to_sd_card);

    if ((millis() - writeStartTime) > WRITE_TIMEOUT_MS) {
        timeoutOccurred = true;
        file.close();
        handle_timeout(bytesWritten);
        return false;
    }

    // Write separator
    size_t separatorWritten = file.print(",\n");
    if (separatorWritten != 2) {
        file.flush();
        file.close();
        handle_separator_corruption(separatorWritten);
        return false;
    }

    bytesWritten += separatorWritten;
    writeSuccess = (bytesWritten >= incomingSize);

    // Flush
    file.flush();
    file.close();

    // =========================================================
    // 8. Verify write result
    // =========================================================
    if (!writeSuccess || bytesWritten == 0) {
        handle_write_failure(bytesWritten, incomingSize);
        return false;
    }

    // =========================================================
    // 9. Update state AFTER verified success
    // =========================================================
    json_currentSize += bytesWritten;

    snprintf(last_json_save_time, sizeof(last_json_save_time), "%s", SystemTime);

    save_counter++;

    if (save_counter % 60 == 0) {
        prefs.begin("SAVES", false);
        prefs.putUInt("SAVES", save_counter);
        prefs.end();
    }

    // =========================================================
    // 10. Periodic resync / maintenance
    // =========================================================
    if (save_counter % 120 == 0) {
        handle_periodic_maintenance();
    }

    // =========================================================
    // 11. Success log
    // =========================================================
    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Success: %lu bytes appended to '%s' (total ~%lu)",
             (unsigned long)bytesWritten,
             json_log_file,
             (unsigned long)json_currentSize);

    LOG(json_save_log);

    return true;
}

bool handle_large_file_issue(size_t weight_under_review) {

    snprintf(json_save_log + strlen(json_save_log),
             sizeof(json_save_log) - strlen(json_save_log),
             " [Rotation: incoming %lu > max %lu]",
             (unsigned long)weight_under_review,
             (unsigned long)MAX_FILE_SIZE);

    LOG(json_save_log);

    File newFile = create_another_file();

    if (!newFile) {
        snprintf(json_save_log, sizeof(json_save_log),
                 "[SD] CRITICAL: File rotation failed");
        LOG(json_save_log);
        return false;
    }

    newFile.close();
    json_currentSize = 0;

    return true;
}

bool handle_sd_recovery() {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Storage not initialized, attempting recovery...");
    LOG(json_save_log);

    for (int i = 0; i < 3; i++) {

        storage_initialized = initialize_sd_card();
        delay(1000 * (1 << i));

        if (storage_initialized) {
            snprintf(json_save_log, sizeof(json_save_log),
                     "[SD] Recovered after %d attempts", i + 1);
            LOG(json_save_log);
            return true;
        }
    }

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Failed to initialize after retries");
    LOG(json_save_log);

    return false;
}

void handle_empty_payload() {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Nothing to write (empty or null data)");
    LOG(json_save_log);

    log_dummy_data();
}

void handle_filename_issue() {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] ERROR: Invalid filename, resetting to default");
    LOG(json_save_log);

    strncpy(json_log_file, "/data.json", sizeof(json_log_file) - 1);
    json_log_file[sizeof(json_log_file) - 1] = '\0';
}

void handle_separator_corruption(size_t separatorWritten) {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] WRITE CORRUPTION: Separator incomplete (%lu/2 bytes)",
             (unsigned long)separatorWritten);
    LOG(json_save_log);

    log_dummy_data();
}

bool handle_file_creation() {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] File '%s' does not exist. Creating...",
             json_log_file);
    LOG(json_save_log);

    File f = SD.open(json_log_file, FILE_WRITE);
    if (!f) {
        snprintf(json_save_log, sizeof(json_save_log),
                 "[SD] Failed to create file '%s'",
                 json_log_file);
        LOG(json_save_log);
        return false;
    }

    f.close();
    return true;
}

void handle_open_failure() {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Failed to open '%s' for appending",
             json_log_file);
    LOG(json_save_log);
}

void handle_timeout(size_t bytesWritten) {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] TIMEOUT: Write hung at %lu bytes",
             (unsigned long)bytesWritten);
    LOG(json_save_log);

    char corruptedName[70];
    snprintf(corruptedName, sizeof(corruptedName),
             "%s_CORRUPTED", json_log_file);

    SD.rename(json_log_file, corruptedName);

    log_dummy_data();
}

void handle_write_failure(size_t bytesWritten, size_t incomingSize) {

    snprintf(json_save_log, sizeof(json_save_log),
             "[SD] Write FAILED. Expected %lu, wrote %lu",
             (unsigned long)incomingSize,
             (unsigned long)bytesWritten);
    LOG(json_save_log);

    if (!SD.exists(json_log_file)) {
        storage_initialized = false;
    }

    log_dummy_data();
}

void handle_periodic_maintenance() {

    File f = SD.open(json_log_file, FILE_READ);
    if (f) {
        json_currentSize = f.size();
        f.close();
    }

    if (json_currentSize > MAX_FILE_SIZE) {

        LOG("[SD] Periodic rotation trigger (size exceeded)");

        File newFile = create_another_file();
        if (newFile) {
            newFile.close();
            json_currentSize = 0;
        }
    }

    storage_initialized = false;
    check_storage_files();
}

/*
bool save_csv_to_sd_card() {

    memset(csv_save_log, 0, sizeof(csv_save_log));

    // ================= STEP 1: Check SD Initialization =================
    if (!storage_initialized) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Storage not initialized");
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 2: Validate Data =================
    if (!sendable_to_sd_card_csv || strlen(sendable_to_sd_card_csv) == 0) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Nothing to write (empty buffer)");
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 3: Ensure File Exists =================
    if (!SD.exists(csv_log_file)) {
        File createFile = SD.open(csv_log_file, FILE_WRITE);
        if (!createFile) {
            snprintf(csv_save_log, sizeof(csv_save_log),
                     "[CSV][SD] Failed to create '%s'",
                     csv_log_file);
            LOG(csv_save_log);
            return false;
        }
        createFile.close();
        csv_currentSize = 0;
    }

    // ================= STEP 4: Open for Append =================
    File file = SD.open(csv_log_file, FILE_APPEND);
    if (!file) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Failed to open '%s'",
                 csv_log_file);
        LOG(csv_save_log);
        return false;
    }

    // If cache unknown (e.g., first boot), sync once
    if (csv_currentSize == 0) {
        csv_currentSize = file.size();
    }

    // ================= STEP 5: If File Empty → Write Header =================
    if (csv_currentSize <= 90) {

        snprintf(file_creation_date, sizeof(file_creation_date),
                 "Log File Created:,,,%s,%s", ShortDate, ShortTime_am_pm);

        file.println(file_heading);
        file.println("#########################################################");
        file.println(file_creation_date);
        file.println(logging_interval);
        file.println(data_points_log);
        file.println(units_of_data);
        file.println(additional_entries);
        file.println("---------------------------------------------------------");
        file.println();

        file.print(",");
        file.print(",");
        file.print("Sensor 1,,,");
        file.print("Sensor 2,,,");
        file.print("Sensor 3,,,");
        file.print("Sensor 4,,,");
        file.print("Sensor 5,,,");
        file.print("Sensor 6,,,");
        file.print("Sensor 7,,,");
        file.print("Sensor 8,,,");
        file.print("Sensor 9,,,");
        file.print("Sensor 10,,,");
        file.print("Sensor 11,,,");
        file.print("Sensor 12,,,");
        file.print("Environment,,");
        file.println("System,,");

        file.println(
            "Date,Time,"
            "S1_T (degC),S1_H (%),S1_P (kPa),"
            "S2_T (degC),S2_H (%),S2_P (kPa),"
            "S3_T (degC),S3_H (%),S3_P (kPa),"
            "S4_T (degC),S4_H (%),S4_P (kPa),"
            "S5_T (degC),S5_H (%),S5_P (kPa),"
            "S6_T (degC),S6_H (%),S6_P (kPa),"
            "S7_T (degC),S7_H (%),S7_P (kPa),"
            "S8_T (degC),S8_H (%),S8_P (kPa),"
            "S9_T (degC),S9_H (%),S9_P (kPa),"
            "S10_T (degC),S10_H (%),S10_P (kPa),"
            "S11_T (degC),S11_H (%),S11_P (kPa),"
            "S12_T (degC),S12_H (%),S12_P (kPa),"
            "Solar (W/sq.m),Wind(m/s),"
            "Uptime,Voltage (V)"
        );

        file.flush();

        // Update cached size after header write
        csv_currentSize = file.size();

        snprintf(csv_save_log + strlen(csv_save_log),
                 sizeof(csv_save_log) - strlen(csv_save_log),
                 "[CSV] Header written. ");

        snprintf(last_csv_save_time,
                 sizeof(last_csv_save_time),
                 "%s",
                 SystemTime);
    }

    // ================= STEP 6: Append CSV Row =================
    size_t incomingSize = strlen(sendable_to_sd_card_csv);
    size_t bytesWritten = file.print(sendable_to_sd_card_csv);

    file.flush();
    file.close();

    // ================= STEP 7: Verify Write =================
    if (bytesWritten != incomingSize) {
        snprintf(csv_save_log,
                 sizeof(csv_save_log),
                 "[CSV][SD] Write FAILED (%lu/%lu bytes)",
                 (unsigned long)bytesWritten,
                 (unsigned long)incomingSize);
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 8: Update Cache ONLY After Success =================
    csv_currentSize += bytesWritten;

    snprintf(csv_save_log,
             sizeof(csv_save_log),
             "[CSV][SD] Success: %lu bytes appended (total ~%lu)",
             (unsigned long)bytesWritten,
             (unsigned long)csv_currentSize);

    LOG(csv_save_log);

    return true;
}
*/

bool save_csv_to_sd_card() {

    memset(csv_save_log, 0, sizeof(csv_save_log));

    // =========================================================
    // 1️⃣ SD Initialization (with recovery)
    // =========================================================
    if (!storage_initialized && !handle_csv_sd_recovery())
        return false;

    // =========================================================
    // 2️⃣ Validate Payload
    // =========================================================
    if (!sendable_to_sd_card_csv || strlen(sendable_to_sd_card_csv) == 0) {
        handle_csv_empty_payload();
        return false;
    }

    size_t incomingSize = strlen(sendable_to_sd_card_csv);

    // =========================================================
    // 3️⃣ File Rotation Check (pre-write protection)
    // =========================================================
    if ((csv_currentSize + incomingSize) > MAX_CSV_FILE_SIZE) {
        if (!handle_csv_large_file_issue(incomingSize))
            return false;
    }

    // =========================================================
    // 4️⃣ Ensure File Exists
    // =========================================================
    if (!SD.exists(csv_log_file) && !handle_csv_file_creation())
        return false;

    // =========================================================
    // 5️⃣ Open File
    // =========================================================
    File file = SD.open(csv_log_file, FILE_APPEND);
    if (!file) {
        handle_csv_open_failure();
        return false;
    }

    // Sync cache if unknown (first boot scenario)
    if (csv_currentSize == 0)
        csv_currentSize = file.size();

    // =========================================================
    // 6️⃣ Write Header If Needed
    // =========================================================
    if (csv_currentSize <= 90) {

        snprintf(file_creation_date, sizeof(file_creation_date),
                 "Log File Created:,,,%s,%s", ShortDate, ShortTime_am_pm);

        file.println(file_heading);
        file.println("#########################################################");
        file.println(file_creation_date);
        file.println(logging_interval);
        file.println(data_points_log);
        file.println(units_of_data);
        file.println(additional_entries);
        file.println("---------------------------------------------------------");
        file.println();

        file.print(",");
        file.print(",");
        file.print("Sensor 1,,,");
        file.print("Sensor 2,,,");
        file.print("Sensor 3,,,");
        file.print("Sensor 4,,,");
        file.print("Sensor 5,,,");
        file.print("Sensor 6,,,");
        file.print("Sensor 7,,,");
        file.print("Sensor 8,,,");
        file.print("Sensor 9,,,");
        file.print("Sensor 10,,,");
        file.print("Sensor 11,,,");
        file.print("Sensor 12,,,");
        file.print("Environment,,");
        file.println("System,,");

        file.println(
            "Date,Time,"
            "S1_T (degC),S1_H (%),S1_P (kPa),"
            "S2_T (degC),S2_H (%),S2_P (kPa),"
            "S3_T (degC),S3_H (%),S3_P (kPa),"
            "S4_T (degC),S4_H (%),S4_P (kPa),"
            "S5_T (degC),S5_H (%),S5_P (kPa),"
            "S6_T (degC),S6_H (%),S6_P (kPa),"
            "S7_T (degC),S7_H (%),S7_P (kPa),"
            "S8_T (degC),S8_H (%),S8_P (kPa),"
            "S9_T (degC),S9_H (%),S9_P (kPa),"
            "S10_T (degC),S10_H (%),S10_P (kPa),"
            "S11_T (degC),S11_H (%),S11_P (kPa),"
            "S12_T (degC),S12_H (%),S12_P (kPa),"
            "Solar (W/sq.m),Wind(m/s),"
            "Uptime,Voltage (V)"
        );

        file.flush();
        csv_currentSize = file.size();

        snprintf(csv_save_log + strlen(csv_save_log),
                 sizeof(csv_save_log) - strlen(csv_save_log),
                 "[CSV] Header written. ");

        snprintf(last_csv_save_time,
                 sizeof(last_csv_save_time),
                 "%s",
                 SystemTime);
    }

    // =========================================================
    // 7️⃣ Write CSV Row
    // =========================================================
    size_t bytesWritten = file.print(sendable_to_sd_card_csv);

    file.flush();
    file.close();

    // =========================================================
    // 8️⃣ Verify Write
    // =========================================================
    if (bytesWritten != incomingSize) {
        handle_csv_write_failure(bytesWritten, incomingSize);
        return false;
    }

    // =========================================================
    // 9️⃣ Update Cache After Success
    // =========================================================
    csv_currentSize += bytesWritten;

    snprintf(csv_save_log,
             sizeof(csv_save_log),
             "[CSV][SD] Success: %lu bytes appended (total ~%lu)",
             (unsigned long)bytesWritten,
             (unsigned long)csv_currentSize);

    LOG(csv_save_log);

    return true;
}


bool handle_csv_large_file_issue(size_t incomingSize) {

    snprintf(csv_save_log + strlen(csv_save_log),
             sizeof(csv_save_log) - strlen(csv_save_log),
             " [CSV Rotation: incoming %lu > max %lu]",
             (unsigned long)(csv_currentSize + incomingSize),
             (unsigned long)MAX_CSV_FILE_SIZE);

    LOG(csv_save_log);

    File newFile = create_another_csv_file();

    if (!newFile) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] CRITICAL: Rotation failed");
        LOG(csv_save_log);
        return false;
    }

    newFile.close();
    csv_currentSize = 0;

    return true;
}

bool handle_csv_sd_recovery() {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Not initialized. Attempting recovery...");
    LOG(csv_save_log);

    for (int i = 0; i < 3; i++) {

        storage_initialized = initialize_sd_card();
        delay(1000 * (1 << i));

        if (storage_initialized) {
            snprintf(csv_save_log, sizeof(csv_save_log),
                     "[CSV][SD] Recovered after %d attempts", i + 1);
            LOG(csv_save_log);
            return true;
        }
    }

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Recovery failed");
    LOG(csv_save_log);

    return false;
}

void handle_csv_empty_payload() {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Empty CSV payload");
    LOG(csv_save_log);
}

bool handle_csv_file_creation() {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Creating '%s'...",
             csv_log_file);
    LOG(csv_save_log);

    File f = SD.open(csv_log_file, FILE_WRITE);
    if (!f) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Failed to create '%s'",
                 csv_log_file);
        LOG(csv_save_log);
        return false;
    }

    f.close();
    csv_currentSize = 0;
    return true;
}

void handle_csv_open_failure() {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Failed to open '%s'",
             csv_log_file);
    LOG(csv_save_log);
}


void handle_csv_write_failure(size_t bytesWritten, size_t incomingSize) {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] Write FAILED (%lu/%lu bytes)",
             (unsigned long)bytesWritten,
             (unsigned long)incomingSize);

    LOG(csv_save_log);

    if (!SD.exists(csv_log_file)) {
        storage_initialized = false;
    }
}

void handle_csv_timeout(size_t bytesWritten) {

    snprintf(csv_save_log, sizeof(csv_save_log),
             "[CSV][SD] TIMEOUT at %lu bytes",
             (unsigned long)bytesWritten);
    LOG(csv_save_log);

    char corruptedName[70];
    snprintf(corruptedName, sizeof(corruptedName),
             "%s_CORRUPTED", csv_log_file);

    SD.rename(csv_log_file, corruptedName);
}

void handle_csv_periodic_maintenance() {

    File f = SD.open(csv_log_file, FILE_READ);
    if (f) {
        csv_currentSize = f.size();
        f.close();
    }

    if (csv_currentSize > MAX_CSV_FILE_SIZE) {

        LOG("[CSV][SD] Periodic rotation trigger");

        File newFile = create_another_csv_file();
        if (newFile) {
            newFile.close();
            csv_currentSize = 0;
        }
    }
}



/*
bool save_csv_to_sd_card(){

    memset(csv_save_log, 0, sizeof(csv_save_log));

    // ================= STEP 1: Check SD Initialization =================
    if (!storage_initialized) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Storage not initialized");
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 2: Validate Data =================
    if (!sendable_to_sd_card_csv || strlen(sendable_to_sd_card_csv) == 0) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Nothing to write (empty buffer)");
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 3: Ensure File Exists =================
    bool fileExists = SD.exists(csv_log_file);

    File file = SD.open(csv_log_file, FILE_APPEND);
    if (!file) {
        snprintf(csv_save_log, sizeof(csv_save_log),
                 "[CSV][SD] Failed to open '%s'",
                 csv_log_file);
        LOG(csv_save_log);
        return false;
    }

    // ================= STEP 4: If File Empty → Write Header =================
    size_t currentSize = file.size();

    if (currentSize <= 90) { // each entry weighs about 1kB... SO 90B is considered empty
      snprintf(file_creation_date, sizeof(file_creation_date),
             "Log File Created:,,,%s,%s", ShortDate, ShortTime_am_pm);

        // ================= REPORT METADATA =================
        file.println(file_heading);                  // Title
        file.println("#########################################################");

        file.println(file_creation_date);            // Creation timestamp //  in-col-spacing included
    
        file.println(logging_interval);

        file.println(data_points_log);
        file.println(units_of_data);
        file.println(additional_entries);

        file.println("---------------------------------------------------------");
        file.println();   // Blank line before table

      // ================= GROUP HEADER ROW =================

         // Write the main header rows (two-row header for better organization)
        // Row 1: Main sensor groups (these will span multiple columns when viewed in spreadsheet software)
        file.print(","); // Empty cell for Date/Time column
        file.print(","); // Empty cell for Date/Time column (second empty for the two columns)
        file.print("Sensor 1,,,"); // S1_T, S1_H, S1_P will be under this
        file.print("Sensor 2,,,"); // col-span-3
        file.print("Sensor 3,,,");
        file.print("Sensor 4,,,");
        file.print("Sensor 5,,,");
        file.print("Sensor 6,,,");
        file.print("Sensor 7,,,");
        file.print("Sensor 8,,,");
        file.print("Sensor 9,,,");
        file.print("Sensor 10,,,");
        file.print("Sensor 11,,,");
        file.print("Sensor 12,,,");
        file.print("Environment,,"); // This will span Solar and Wind
        file.println("System,,"); // This will span Uptime and Voltage

      //   file.println();   // small separator before first data row

            // ================= COLUMN HEADER ROW =================

        file.println( // at row 11, column 1, col-span-1 each
            "Date,Time,"
            "S1_T (degC),S1_H (%),S1_P (kPa),"
            "S2_T (degC),S2_H (%),S2_P (kPa),"
            "S3_T (degC),S3_H (%),S3_P (kPa),"
            "S4_T (degC),S4_H (%),S4_P (kPa),"
            "S5_T (degC),S5_H (%),S5_P (kPa),"
            "S6_T (degC),S6_H (%),S6_P (kPa),"
            "S7_T (degC),S7_H (%),S7_P (kPa),"
            "S8_T (degC),S8_H (%),S8_P (kPa),"
            "S9_T (degC),S9_H (%),S9_P (kPa),"
            "S10_T (degC),S10_H (%),S10_P (kPa),"
            "S11_T (degC),S11_H (%),S11_P (kPa),"
            "S12_T (degC),S12_H (%),S12_P (kPa),"
            "Solar (W/sq.m),Wind(m/s),"
            "Uptime,Voltage (V)" //AvgTemp,AvgHumi,AvgPress,"
        ); 

        snprintf(csv_save_log + strlen(csv_save_log),
                 sizeof(csv_save_log) - strlen(csv_save_log),
                 "[CSV] Header written. ");
                     snprintf(last_csv_save_time, sizeof(last_csv_save_time), "%s", SystemTime);

    }

    // ================= STEP 5: Append CSV Row =================
    size_t incomingSize = strlen(sendable_to_sd_card_csv);

    size_t bytesWritten = file.print(sendable_to_sd_card_csv); 

    file.flush();
    file.close();

    // ================= STEP 6: Verify Write =================
    if (bytesWritten != incomingSize) {
        snprintf(csv_save_log,
                 sizeof(csv_save_log),
                 "[CSV][SD] Write FAILED (%lu/%lu bytes)",
                 (unsigned long)bytesWritten,
                 (unsigned long)incomingSize);
        LOG(csv_save_log);
        return false;
    }

    snprintf(csv_save_log,
             sizeof(csv_save_log),
             "[CSV][SD] Success: %lu bytes appended",
             (unsigned long)bytesWritten);

    LOG(csv_save_log);

   
      
        File f = SD.open(csv_log_file, FILE_READ);
        if (f) { csvSize = f.size(); f.close(); }
    


    return true;
}
*/


String read_complete_json_file() {
    if (!storage_initialized || strlen(json_log_file) == 0) {
        return "{}";
    }
    
    File file = SD.open(json_log_file, FILE_READ);
    if (!file) {
        return "{}";
    }
    
    // Read the entire file
    String completeJson;
    while (file.available()) {
        completeJson += (char)file.read();
    }
    file.close();
    
    return completeJson;
}

// Function to get just the data array
String read_json_data_only() {
    String complete = read_complete_json_file();
    
    // Find the "data" array in the JSON
    int dataStart = complete.indexOf("\"data\":[");
    if (dataStart == -1) {
        return "[]";
    }
    
    dataStart += 7; // Move past "\"data\":["
    
    // Find the matching closing bracket
    int bracketCount = 1;
    int dataEnd = dataStart;
    
    for (int i = dataStart; i < complete.length(); i++) {
        if (complete.charAt(i) == '[') bracketCount++;
        if (complete.charAt(i) == ']') bracketCount--;
        
        if (bracketCount == 0) {
            dataEnd = i;
            break;
        }
    }
    
    return complete.substring(dataStart, dataEnd + 1);
}

// Optional: Add a watchdog reset function for RTOS integration
void reset_sd_write_watchdog() {
    // Could be called periodically from a high-priority task
    // to ensure write operations don't hang indefinitely
    static uint32_t lastWriteTime = 0;
    static bool writeInProgress = false;
    
    // This would need integration with the actual write state
    // For RTOS, consider using a semaphore or flag to track write state
}

// Correct way to create a multi-line JSON string in C/C++
void log_dummy_data(){
 snprintf(sendable_to_sd_card, sizeof(sendable_to_sd_card), 
    "{"
    "\"System\": {"
        "\"FW_Version\": \"v1.3.2\","
        "\"Uptime\": 45231,"
        "\"Date\": \"2026-02-04\","
        "\"Time\": \"14:37:19\","
        "\"Datalog_Count\": 128,"
        "\"Voltage\": 12.42,"
        "\"Internal_Temp\": 36.8,"
        "\"Internal_Fan\": true"
    "},"
    
    "\"Dryer_Conditions\": {"
        "\"Active_Sensors\": 6,"
        "\"Average_Temperature\": 54.6,"
        "\"Average_Humidity\": 28.3,"
        "\"Average_Pressure\": 1007.8,"
        
        "\"Humidity_Fans\": true,"
        "\"Humidity_Fans_Start_Time\": \"14:05:00\","
        "\"Humidity_Fans_Stop_Time\": \"14:20:00\","
        "\"Humidity_Fans_ON_Duration\": 900,"
        
        "\"Cooling_Fans\": false,"
        "\"Cooling_Fans_Start_Time\": \"13:40:00\","
        "\"Cooling_Fans_Stop_Time\": \"13:55:00\","
        "\"Cooling_Fans_ON_Duration\": 900"
    "},"
    
    "\"All_Sensors\": {"
        "\"Temperature_Readings\": [52.4, 53.1, 55.7, 56.2, 54.8, 55.3],"
        "\"Humidity_Readings\": [30.1, 29.4, 27.8, 26.9, 28.2, 27.6],"
        "\"Pressure_Readings\": [1008.2, 1007.9, 1007.5, 1007.2, 1008.0, 1007.8]"
    "},"
    
    "\"Sensor_1\": {"
        "\"Position\": \"AIR INLET 1\","
        "\"Temp\": 52.4,"
        "\"Humi\": 30.1,"
        "\"Pressure\": 1008.2,"
        "\"Last_Seen\": \"14:37:17\","
        "\"Sends\": \"1542\""
    "},"
    
    "\"Sensor_2\": {"
        "\"Position\": \"AIR INLET 2\","
        "\"Temp\": 53.1,"
        "\"Humi\": 29.4,"
        "\"Pressure\": 1007.9,"
        "\"Last_Seen\": \"14:37:16\","
        "\"Sends\": \"1519\""
    "},"
    
    "\"Sensor_3\": {"
        "\"Position\": \"AIR OUTLET 1\","
        "\"Temp\": 55.7,"
        "\"Humi\": 27.8,"
        "\"Pressure\": 1007.5,"
        "\"Last_Seen\": \"14:37:18\","
        "\"Sends\": \"1603\""
    "},"
    
    "\"Sensor_4\": {"
        "\"Position\": \"AIR OUTLET 2\","
        "\"Temp\": 56.2,"
        "\"Humi\": 26.9,"
        "\"Pressure\": 1007.2,"
        "\"Last_Seen\": \"14:37:18\","
        "\"Sends\": \"1598\""
    "},"
    
    "\"Sensor_5\": {"
        "\"Position\": \"DRYING BED LOWEST\","
        "\"Temp\": 54.8,"
        "\"Humi\": 28.2,"
        "\"Pressure\": 1008.0,"
        "\"Last_Seen\": \"14:37:15\","
        "\"Sends\": \"1489\""
    "},"
    
    "\"Sensor_6\": {"
        "\"Position\": \"DRYING BED MIDDLE\","
        "\"Temp\": 55.3,"
        "\"Humi\": 27.6,"
        "\"Pressure\": 1007.8,"
        "\"Last_Seen\": \"14:37:14\","
        "\"Sends\": \"1472\""
    "},"
    
    "\"Sensor_7\": {"
        "\"Position\": \"DRYING BED TOP\","
        "\"Temp\": 56.0,"
        "\"Humi\": 26.4,"
        "\"Pressure\": 1007.6,"
        "\"Last_Seen\": \"14:36:58\","
        "\"Sends\": \"1391\""
    "},"
    
    "\"Sensor_8\": {"
        "\"Position\": \"DRYING BED MIDDLE ZONE\","
        "\"Temp\": 55.1,"
        "\"Humi\": 27.1,"
        "\"Pressure\": 1007.9,"
        "\"Last_Seen\": \"14:36:55\","
        "\"Sends\": \"1384\""
    "},"
    
    "\"Sensor_9\": {"
        "\"Position\": \"DRYING BED ENTRY ZONE\","
        "\"Temp\": 53.9,"
        "\"Humi\": 28.9,"
        "\"Pressure\": 1008.3,"
        "\"Last_Seen\": \"14:36:52\","
        "\"Sends\": \"1378\""
    "},"
    
    "\"Sensor_10\": {"
        "\"Position\": \"DRYING BED EXIT ZONE\","
        "\"Temp\": 56.4,"
        "\"Humi\": 25.8,"
        "\"Pressure\": 1007.1,"
        "\"Last_Seen\": \"14:36:49\","
        "\"Sends\": \"1366\""
    "},"
    
    "\"Sensor_11\": {"
        "\"Position\": \"DRYING BED RANDOM POSITION\","
        "\"Temp\": 54.2,"
        "\"Humi\": 29.0,"
        "\"Pressure\": 1008.1,"
        "\"Last_Seen\": \"14:36:44\","
        "\"Sends\": \"1359\""
    "},"
    
    "\"Sensor_12\": {"
        "\"Position\": \"DRYING BED AMBIENT ZONE\","
        "\"Temp\": 36.5,"
        "\"Humi\": 41.2,"
        "\"Pressure\": 1009.4,"
        "\"Last_Seen\": \"14:36:40\","
        "\"Sends\": \"1347\""
    "},"
    
    "\"Environment\": {"
        "\"Solar_Radiation\": 742.5,"
        "\"Wind_Speed\": 2.8"
    "}"
    "}"
  );
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
bool initialize_sd_card() {
    LOG("-------- INITIALIZING ONSITE STORAGE ------------------");
    bool status = false;
    // Clear status log
    memset(storage_status_log, 0, sizeof(storage_status_log));
    strncpy(storage_status_log, "SD Init: ", sizeof(storage_status_log));
    
    // --- Initialize VSPI for SD ---
    SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_Chip_Select);
    // Initialize SD card with robust settings
    storage_initialized = SD.begin(SD_Chip_Select, SD_SPI, 4000000); // Lower frequency, better for reliability
    
    if (!storage_initialized) {
        snprintf(storage_status_log, sizeof(storage_status_log), 
                 "SD CARD failed to initialize. Check wiring/power.");
        LOG(storage_status_log);
        status = false;
        
    }
    else {
    
    // Check card type and presence
    uint8_t cardType = SD.cardType();
    
    if (cardType == CARD_NONE) {
        snprintf(storage_status_log, sizeof(storage_status_log), 
                 "NO SD CARD detected");
        LOG(storage_status_log);
        storage_initialized = false;
        status = false;
    }

    else {
    
          // Report card type
          const char* typeStr = "Unknown";
          switch(cardType) {
              case CARD_MMC: typeStr = "MMC"; break;
              case CARD_SD: typeStr = "SDSC"; break;
              case CARD_SDHC: typeStr = "SDHC/SDXC"; break;
          }
          
          snprintf(storage_status_log + strlen(storage_status_log), 
                  sizeof(storage_status_log) - strlen(storage_status_log),
                  "Type: %s | ", typeStr);
          
          // Get and report card size
          uint64_t cardSize = SD.cardSize();
          if (cardSize == 0) {
              strncat(storage_status_log, "Size: Unknown", 
                      sizeof(storage_status_log) - strlen(storage_status_log) - 1);
              storage_initialized = false;
              status = false;
              return status;
          }
          
          storage = cardSize / (1024 * 1024); // Convert to MB
          snprintf(storage_size_char, sizeof(storage_size_char), 
                  "Size: %lluMB", storage);
          
          // Check if we have enough space
          uint64_t usedSpace = SD.usedBytes() / (1024 * 1024);
          uint64_t freeSpace = storage - usedSpace;
          
          strncat(storage_status_log, storage_size_char,
                  sizeof(storage_status_log) - strlen(storage_status_log) - 1);
          
          snprintf(storage_status_log + strlen(storage_status_log),
                  sizeof(storage_status_log) - strlen(storage_status_log),
                  " | Used: %lluMB | Free: %lluMB", usedSpace, freeSpace);
          
          // Check/create JSON log file with proper error handling
          if (!SD.exists(json_log_file)) {
              strncat(storage_status_log, " | Creating new log file",
                      sizeof(storage_status_log) - strlen(storage_status_log) - 1);
              
              File f = SD.open(json_log_file, FILE_WRITE);
              if (!f) {
                  strncat(storage_status_log, " | FAILED to create file",
                          sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                  storage_initialized = false;
                  status = false;
                  return status;
              }
              f.close();
          }
          
          // Check file size and rotate if needed
          File f = SD.open(json_log_file, FILE_READ);
          if (f) {
              size_t currentSize = f.size();
              f.close();
              
              if (currentSize > MAX_FILE_SIZE) {
                  strncat(storage_status_log, " | File needs rotation",
                          sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                  
                  // Check total storage usage before creating new file
                  uint64_t totalUsed = SD.usedBytes();
                  if (totalUsed > MAX_TOTAL_SIZE) {
                      strncat(storage_status_log, " | TOTAL STORAGE LIMIT REACHED",
                              sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                      // Consider implementing file deletion logic here
                  } else {
                      // Create rotated file
                      File newFile = create_another_file();
                      if (newFile) {
                          strncat(storage_status_log, " | File rotated",
                                  sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                          newFile.close();
                          status = true;
                      } else {
                          strncat(storage_status_log, " | Rotation FAILED",
                                  sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                      }
                  }
              } else {
                  snprintf(storage_status_log + strlen(storage_status_log),
                          sizeof(storage_status_log) - strlen(storage_status_log),
                          " | Current file: %u bytes", currentSize);
                          status = true;
              }
          } else {
                  strncat(storage_status_log, " | Cannot open file for size check",
                          sizeof(storage_status_log) - strlen(storage_status_log) - 1);
                  storage_initialized = false;
                  status = false;
              }

        }

    }
    
    return status;
    LOG(storage_status_log);
}
*/


char meta_data[1000];


uint64_t screen_refresh_interval = (1ULL*60ULL*1000ULL); // refreshing entire screen once 5 mins daytime, else 30mins at night 180 seconds, b4 hibernating
const uint64_t upload_fequency = (10ULL*60ULL*1000ULL); // every 10 minutes
boolean uploaded = false;

const uint64_t wifi_checK_interval = (2ULL * 1000ULL);
uint64_t    last_wifi_check = 0; 


uint64_t data_update_interval = 60ULL * 1000ULL; // 30 seconds if power is good, 60 seconds when not
bool json_data_logged = false;
bool csv_data_logged = false;

bool fans_are_togglable = false; // when power is low, totawaana

uint32_t loop_count = 0; // tracker for whenever loop or a RTOS task yeesibye


char activity_log[1024];
bool is_night_time = false;
bool is_at_peak_sunshine = false;

void loop() {
    now_now_ms = esp_timer_get_time() / 1000ULL; // us to ms
     update_display(); // this returns 99.9% of the time and only updates screen ever 5, or 30mins

    if(packet_received) { buzzer.beep(1,50,0);  extract_readings(); packet_received = false; }
    
    // Sensor readings
    if ((now_now_ms - last_read_time_ms) >= data_update_interval) { // 60 seconds if power is good for 24 hours:: 1,440 saved packets/day
        last_read_time_ms = now_now_ms;  // Update FIRST to prevent race conditions
        
          //most recent parameters
          
          query_rtc();      
          is_night_time = (hr >= 20 || hr < 7);  // 8pm to 7am... time for idling
          is_at_peak_sunshine = (hr > 9 && hr < 18); // 9am to 3pm...[and if solar radiation>1000W/m2]...time for peak peformance
        
          if(is_night_time) screen_refresh_interval = (60ULL*60ULL*1000ULL); //  refresh once every hour...maybe between light sleeps
          else {
            if(is_at_peak_sunshine) screen_refresh_interval = (60ULL*1000ULL); // refresh once every minute
            else screen_refresh_interval = (10ULL*60ULL*1000ULL); // refresh once every 10 minutes
          }

          MonitorBattery(); // battery voltage
          
          monitor_box_conditions(); // internal temp and fan state
          monitor_dryer_readings(); 
          
          //check_meta_data(); // column headers for CSV
           json_bound_successfully = bind_dynamic_data_into_json();
           csv_bound_successfully = bind_dynamic_data_into_csv();

          json_data_logged = save_json_to_sd_card(); delay(500);
          csv_data_logged  = save_csv_to_sd_card(); delay(500);

        if(json_data_logged && csv_data_logged)  buzzer.beep(2, 50, 0);

          snprintf(activity_log, sizeof(activity_log), 
              "\nLoopCount:%lu\nUptime: %llu, Reading complete! Power mode: %d, Voltage: %.1fV\njson_data_logged: %s",     
              loop_count, last_read_time_ms/1000, current_power, voltage, json_data_logged?"YES":"NO"
              );

          LOG(activity_log);
    }
    

  /*
    if((now_now_ms - last_upload_time_ms) > upload_frequency_ms){ // either every 10mins or at the tenth minute
      if(can_send){ // when using WiFi to upload
       switch_radio_to_wifi(); 
       UploadStatus st Uploader.upload_to_web_of_iot(sendable_to_cloud_db);
        Serial.printf("[Upload Task] upload status=%d; report=%s\n", (int)st, Uploader.get_upload_report());
        snprintf(last_upload_time, sizeof(last_upload_time), "%s", SystemTime);
         snprintf(upload_error_code, sizeof(upload_error_code), 
                        "Uploaded at %s on %s", ShortTime_am_pm, SystemDate);
        LOG(upload_error_code);
        can_send = false;

        data_sent = true;
        last_upload_time_ms = now_now_ms;


      }
    }
 
 */

    bool wifi_triggered = read_button(wifi_toggler, now_now_ms);

    bool otaTrigger = read_button(ota_button, now_now_ms);

 if (otaTrigger && !otaModeActive){

    Serial.println("\n=== OTA Triggered ===");

                buzzer.beep(3, 500, 200);

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

    otaModeActive = true;
    otaStartTime = now_now_ms;

    Serial.printf("OTA Mode Active for %llu minutes\n",
                  (OTA_TIMEOUT / 60000));
  
            currentScreen = 10;
            special_call = true;
            update_display();
            
  }

 // when activated successfully
  
  if (otaModeActive) { // KEEP CHECKING OTA HANDLE EVERY LOOP
        ArduinoOTA.handle();
        flash(now_now_ms, blinker, 50, 50, 50, 50); // uploading
       
     if((now_now_ms - last_wifi_check) >= wifi_checK_interval) { // CHECK WIFI STATUS ONCE EVERY 2 SECONDS, ... 
            wifi_connected = wifi_obj.ensure_wifi(); 
            last_wifi_check = now_now_ms;
        }

        if (otaFinished) { // let it be seen and heard
                strcpy(ota_log, "Update complete!"); 
                snprintf(LastOTAUpdate, sizeof(LastOTAUpdate), "On Date: %s, at %s", SystemDate, ShortTime);
                buzzer.beep(2, 100, 50);
                delay(50);
                flash(now_now_ms, blinker, 1000, 100, 0, 0);
                otaFinished = false;
        }

     if(otaStarted){
            strcpy(ota_log, "Start updating...");
            buzzer.beep(1, 50, 0); 
            delay(50);
            otaStarted = false;
      }
     if(otaError){
            buzzer.beep(1, 500, 500);
            delay(500);
            otaError = false;
     }
     if(otaProgress){

     }

    // Timeout after 5 minutes
    if ((now_now_ms - otaStartTime) >= OTA_TIMEOUT) {
            Serial.println();
            Serial.printf("OTA timeout reached!!! Started at: %llu, Ended at: %llu ", otaStartTime/1000, now_now_ms/1000);
            Serial.println("Returning to ESP-NOW mode...");
        
            otaModeActive = false;
            switch_radio_to_espnow();
        }
   } // !otaModeActive         
 
  else  {   delay(10); // make the loop less when not polling ota
           flash(now_now_ms, blinker, 50, 75, 50, 1500); // normal
        }


  /*
    // OTA handling (only in excellent power mode)
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
        loop_count++;

}
/// loop()

/*
ACTIVE LOW
bool read_button(uint8_t pin, uint64_t time_now){
  static uint64_t pressStart = 0;
  static bool lastState = HIGH;
  static bool longPressHandled = false;
  bool pressed = false;

  bool state = digitalRead(pin);

  if (state == LOW && lastState == HIGH) {
    pressStart = time_now;
    longPressHandled = false;
  }

  if (state == LOW && !longPressHandled && (time_now - pressStart >= 3000ULL)) {
    pressed = true;
    longPressHandled = true;
    buzzer.beep(1, 50, 0);
  }

  if (state == HIGH && lastState == LOW) {
    longPressHandled = false;
  }

  lastState = state;
  return pressed;
}
*/

bool read_button(uint8_t pin, uint64_t time_now)
{
    static uint64_t pressStart = 0;
    static bool lastState = HIGH;    // idle is HIGH with INPUT_PULLUP
    static bool longPressHandled = false;

    bool pressed = false;
    bool state = digitalRead(pin);   // LOW = pressed (active LOW)

    // Button pressed (HIGH → LOW transition)
    if (state == LOW && lastState == HIGH) {
        pressStart = time_now;
        longPressHandled = false;
    }

    // Long press detected (3 seconds)
    if (state == LOW &&
        !longPressHandled &&
        (time_now - pressStart >= 3000ULL))
    {
        pressed = true;
        longPressHandled = true;
        buzzer.beep(1, 50, 0);
    }

    // Button released (LOW → HIGH transition)
    if (state == HIGH && lastState == LOW) {
        longPressHandled = false;
    }

    lastState = state;
    return pressed;
}

// replace switch_radio_to_espnow() with the version from the "Fix B" snippet above
// and replace OnSensorData_received with the safer copy version above



#define ESPNOW_MAX_RETRIES 3

bool switch_radio_to_espnow() {
  Serial.println("Switching radio to ESP-NOW...");

  /*
      // Proper cleanup
    if (esp_now_deinit() != ESP_OK) {
        Serial.println("Warning: ESP-NOW deinit had issues");
    }
  */  
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    WiFi.mode(WIFI_STA); Serial.printf("WiFi channel: %d\n", WiFi.channel());

  for (uint8_t attempt = 1; attempt <= ESPNOW_MAX_RETRIES; attempt++) {
      if (esp_now_init() == ESP_OK) { 
            Serial.printf("✓ ESP-NOW initialized successfully (attempt %d/%d)\n", attempt, ESPNOW_MAX_RETRIES);
      
   // Register callback function
          esp_err_t result = esp_now_register_recv_cb(esp_now_recv_cb_t(OnSensorData_received)); // Serial.print("CALLBACK: ");Serial.println(result);
          if(result == ESP_OK) Serial.println("Call Back of Call Back successfully set!"); 
           
        } else {
           // print that Wireless Radio is faulty
            //return;
            delay(100 * attempt); // Exponential backoff
        }
        
    }
    
    
    Serial.println("!!! ESP-NOW initialization failed after retries");
    return false;
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

void extract_readings(){
      if(!packet_received) { LOG("No packet received!"); return; }  // this never runs, but just in case 
      digitalWrite(sensor_indicator, HIGH); delay(50); digitalWrite(sensor_indicator, LOW); 
       DeserializationError err = deserializeJson(JSON_data, dataPack);

        if (err) {
            Serial.print("JSON Deserialization Error: ");
            Serial.println(err.c_str());
            return;
        }
       // LOG(dataPack); // SEE WHAT HAS BEEN RECEIVED!

          
    const char * sensor_ID = JSON_data["S_ID"] | "unknown";

     //fastest approach... as switch is Extremely fast for consecutive integers

  /*
  📊 Direct Performance Rankings (Fastest to Slowest)
      switch with consecutive integers ⚡ Fastest

      else if chain with integer comparisons 🚀 Fast

      if statements with integer comparisons 🏃 Medium

      else if chain with string comparisons 🐌 Slow

    if statements with string comparisons 🐌🐌 Slowest
  */


 int sensor_index_num = extract_sensor_number(sensor_ID); 
  
          switch (sensor_index_num) {
              case 1: {
                sensor_1_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                sensor_1_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                sensor_1_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                sensor_1_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                sensor_1_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                sensor_1_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                sensor_1_packet_size = JSON_data["PKT"] | 0;
            
                strncpy(sensor_1_transmissions,  JSON_data["Sends"], sizeof(sensor_1_transmissions)); // up to 11 bytes: e.g., "1234567890"
                strncpy(sensor_1_CPU_freq,  JSON_data["CPU"], sizeof(sensor_1_CPU_freq)); // 11 bytes: "80MHz CPU"
                
                temperature_readings[0] = sensor_1_temp; humidity_readings[0] = sensor_1_humidity;  // arrays or indexes
                strncpy(sensor_1_last_seen, SystemTime, sizeof(sensor_1_last_seen));      sensor_1_last_seen[sizeof(sensor_1_last_seen) - 1] = '\0'; 
   
              }
                 break;
              case 2:  {
                    sensor_2_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                    sensor_2_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                    sensor_2_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                    sensor_2_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                    sensor_2_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                    sensor_2_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                    sensor_2_packet_size = JSON_data["PKT"] | 0;
                
                    strncpy(sensor_2_transmissions,  JSON_data["Sends"], sizeof(sensor_2_transmissions)); // up to 11 bytes: e.g., "1234567890"
                    strncpy(sensor_2_CPU_freq,  JSON_data["CPU"], sizeof(sensor_2_CPU_freq)); // 11 bytes: "80MHz CPU"
                    
                    temperature_readings[0] = sensor_2_temp; humidity_readings[0] = sensor_2_humidity;  // arrays or indexes
                    strncpy(sensor_2_last_seen, SystemTime, sizeof(sensor_2_last_seen));      sensor_2_last_seen[sizeof(sensor_2_last_seen) - 1] = '\0'; 
   
              }
              break;

                  case 3: {
                      sensor_3_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_3_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_3_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_3_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_3_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_3_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_3_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_3_transmissions, JSON_data["Sends"], sizeof(sensor_3_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_3_CPU_freq, JSON_data["CPU"], sizeof(sensor_3_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[2] = sensor_3_temp; humidity_readings[2] = sensor_3_humidity;  // arrays or indexes
                      strncpy(sensor_3_last_seen, SystemTime, sizeof(sensor_3_last_seen)); sensor_3_last_seen[sizeof(sensor_3_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 4: {
                      sensor_4_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_4_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_4_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_4_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_4_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_4_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_4_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_4_transmissions, JSON_data["Sends"], sizeof(sensor_4_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_4_CPU_freq, JSON_data["CPU"], sizeof(sensor_4_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[3] = sensor_4_temp; humidity_readings[3] = sensor_4_humidity;  // arrays or indexes
                      strncpy(sensor_4_last_seen, SystemTime, sizeof(sensor_4_last_seen)); sensor_4_last_seen[sizeof(sensor_4_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 5: {
                      sensor_5_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_5_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_5_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_5_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_5_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_5_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_5_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_5_transmissions, JSON_data["Sends"], sizeof(sensor_5_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_5_CPU_freq, JSON_data["CPU"], sizeof(sensor_5_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[4] = sensor_5_temp; humidity_readings[4] = sensor_5_humidity;  // arrays or indexes
                      strncpy(sensor_5_last_seen, SystemTime, sizeof(sensor_5_last_seen)); sensor_5_last_seen[sizeof(sensor_5_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 6: {
                      sensor_6_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_6_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_6_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_6_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_6_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_6_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_6_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_6_transmissions, JSON_data["Sends"], sizeof(sensor_6_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_6_CPU_freq, JSON_data["CPU"], sizeof(sensor_6_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[5] = sensor_6_temp; humidity_readings[5] = sensor_6_humidity;  // arrays or indexes
                      strncpy(sensor_6_last_seen, SystemTime, sizeof(sensor_6_last_seen)); sensor_6_last_seen[sizeof(sensor_6_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 7: {
                      sensor_7_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_7_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_7_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_7_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_7_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_7_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_7_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_7_transmissions, JSON_data["Sends"], sizeof(sensor_7_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_7_CPU_freq, JSON_data["CPU"], sizeof(sensor_7_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[6] = sensor_7_temp; humidity_readings[6] = sensor_7_humidity;  // arrays or indexes
                      strncpy(sensor_7_last_seen, SystemTime, sizeof(sensor_7_last_seen)); sensor_7_last_seen[sizeof(sensor_7_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 8: {
                      sensor_8_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_8_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_8_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_8_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_8_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_8_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_8_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_8_transmissions, JSON_data["Sends"], sizeof(sensor_8_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_8_CPU_freq, JSON_data["CPU"], sizeof(sensor_8_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[7] = sensor_8_temp; humidity_readings[7] = sensor_8_humidity;  // arrays or indexes
                      strncpy(sensor_8_last_seen, SystemTime, sizeof(sensor_8_last_seen)); sensor_8_last_seen[sizeof(sensor_8_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 9: {
                      sensor_9_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_9_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_9_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_9_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_9_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_9_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_9_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_9_transmissions, JSON_data["Sends"], sizeof(sensor_9_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_9_CPU_freq, JSON_data["CPU"], sizeof(sensor_9_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[8] = sensor_9_temp; humidity_readings[8] = sensor_9_humidity;  // arrays or indexes
                      strncpy(sensor_9_last_seen, SystemTime, sizeof(sensor_9_last_seen)); sensor_9_last_seen[sizeof(sensor_9_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 10: {
                      sensor_10_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_10_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_10_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_10_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_10_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_10_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_10_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_10_transmissions, JSON_data["Sends"], sizeof(sensor_10_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_10_CPU_freq, JSON_data["CPU"], sizeof(sensor_10_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[9] = sensor_10_temp; humidity_readings[9] = sensor_10_humidity;  // arrays or indexes
                      strncpy(sensor_10_last_seen, SystemTime, sizeof(sensor_10_last_seen)); sensor_10_last_seen[sizeof(sensor_10_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 11: {
                      sensor_11_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_11_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_11_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_11_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_11_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_11_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_11_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_11_transmissions, JSON_data["Sends"], sizeof(sensor_11_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_11_CPU_freq, JSON_data["CPU"], sizeof(sensor_11_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[10] = sensor_11_temp; humidity_readings[10] = sensor_11_humidity;  // arrays or indexes
                      strncpy(sensor_11_last_seen, SystemTime, sizeof(sensor_11_last_seen)); sensor_11_last_seen[sizeof(sensor_11_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  case 12: {
                      sensor_12_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                      sensor_12_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                      sensor_12_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                      sensor_12_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                      sensor_12_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                      sensor_12_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                      sensor_12_packet_size = JSON_data["PKT"] | 0;
                  
                      strncpy(sensor_12_transmissions, JSON_data["Sends"], sizeof(sensor_12_transmissions)); // up to 11 bytes: e.g., "1234567890"
                      strncpy(sensor_12_CPU_freq, JSON_data["CPU"], sizeof(sensor_12_CPU_freq)); // 11 bytes: "80MHz CPU"
                      
                      temperature_readings[11] = sensor_12_temp; humidity_readings[11] = sensor_12_humidity;  // arrays or indexes
                      strncpy(sensor_12_last_seen, SystemTime, sizeof(sensor_12_last_seen)); sensor_12_last_seen[sizeof(sensor_12_last_seen) - 1] = '\0'; 
                  }
                  break;
                  
                  default: {
                      Serial.printf("Unknown sensor: %s\n", sensor_ID);
                  }
                  break;
             
           } // end of switch
   
} // end of extractor
 


      





/*const char * sensor_2_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_3_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_4_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_5_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_6_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_7_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_8_ID = JSON_data["S_ID"] | "unknown";
    
    const char * sensor_9_ID = JSON_data["S_ID"] | "unknown";  const char * sensor_10_ID = JSON_data["S_ID"] | "unknown";
   
    const char * sensor_11_ID = JSON_data["S_ID"] | "unknown"; const char * sensor_12_ID = JSON_data["S_ID"] | "unknown";
  */

/*
//deserialize to assign char[] and floats accordingly
void extract_readings(){
  if(!packet_received) { LOG("No packet received!"); return; }  // this never runs, but just in case 
      
       DeserializationError err = deserializeJson(JSON_data, dataPack);

        if (err) {
            Serial.print("JSON Deserialization Error: ");
            Serial.println(err.c_str());
            return;
        }
       // LOG(dataPack); // SEE WHAT HAS BEEN RECEIVED!

          
    const char * sensor_ID = JSON_data["S_ID"] | "unknown"; 

        if(strcmp(sensor_ID, "T_1") == 0){
            sensor_1_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
            sensor_1_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
            sensor_1_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
            sensor_1_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
            sensor_1_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
            sensor_1_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
            sensor_1_packet_size = JSON_data["PKT"] | 0;
        
            strncpy(sensor_1_transmissions,  JSON_data["Sends"], sizeof(sensor_1_transmissions)); // up to 11 bytes: e.g., "1234567890"
            strncpy(sensor_1_CPU_freq,  JSON_data["CPU"], sizeof(sensor_1_CPU_freq)); // 11 bytes: "80MHz CPU"
            
            temperature_readings[0] = sensor_1_temp; humidity_readings[0] = sensor_1_humidity;  // arrays or indexes
            strncpy(sensor_1_last_seen, ShortTime, sizeof(sensor_1_last_seen));      sensor_1_last_seen[sizeof(sensor_1_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_2") == 0){
                  sensor_2_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_2_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_2_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_2_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_2_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_2_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_2_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_2_transmissions,  JSON_data["Sends"], sizeof(sensor_2_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_2_CPU_freq,  JSON_data["CPU"], sizeof(sensor_2_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[0] = sensor_2_temp; humidity_readings[0] = sensor_2_humidity;  // arrays or indexes
                  strncpy(sensor_2_last_seen, ShortTime, sizeof(sensor_2_last_seen));      sensor_2_last_seen[sizeof(sensor_2_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_3") == 0){
                  sensor_3_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_3_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_3_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_3_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_3_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_3_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_3_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_3_transmissions,  JSON_data["Sends"], sizeof(sensor_3_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_3_CPU_freq,  JSON_data["CPU"], sizeof(sensor_3_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[2] = sensor_3_temp; humidity_readings[2] = sensor_3_humidity;  // arrays or indexes
                  strncpy(sensor_3_last_seen, ShortTime, sizeof(sensor_3_last_seen));      sensor_3_last_seen[sizeof(sensor_3_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_4") == 0){
                  sensor_4_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_4_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_4_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_4_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_4_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_4_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_4_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_4_transmissions,  JSON_data["Sends"], sizeof(sensor_4_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_4_CPU_freq,  JSON_data["CPU"], sizeof(sensor_4_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[3] = sensor_4_temp; humidity_readings[3] = sensor_4_humidity;  // arrays or indexes
                  strncpy(sensor_4_last_seen, ShortTime, sizeof(sensor_4_last_seen));      sensor_4_last_seen[sizeof(sensor_4_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_5") == 0){
                  sensor_5_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_5_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_5_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_5_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_5_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_5_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_5_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_5_transmissions,  JSON_data["Sends"], sizeof(sensor_5_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_5_CPU_freq,  JSON_data["CPU"], sizeof(sensor_5_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[4] = sensor_5_temp; humidity_readings[4] = sensor_5_humidity;  // arrays or indexes
                  strncpy(sensor_5_last_seen, ShortTime, sizeof(sensor_5_last_seen));      sensor_5_last_seen[sizeof(sensor_5_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_6") == 0){
                  sensor_6_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_6_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_6_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_6_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_6_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_6_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_6_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_6_transmissions,  JSON_data["Sends"], sizeof(sensor_6_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_6_CPU_freq,  JSON_data["CPU"], sizeof(sensor_6_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[5] = sensor_6_temp; humidity_readings[5] = sensor_6_humidity;  // arrays or indexes
                  strncpy(sensor_6_last_seen, ShortTime, sizeof(sensor_6_last_seen));      sensor_6_last_seen[sizeof(sensor_6_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_7") == 0){
                  sensor_7_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_7_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_7_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_7_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_7_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_7_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_7_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_7_transmissions,  JSON_data["Sends"], sizeof(sensor_7_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_7_CPU_freq,  JSON_data["CPU"], sizeof(sensor_7_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[6] = sensor_7_temp; humidity_readings[6] = sensor_7_humidity;  // arrays or indexes
                  strncpy(sensor_7_last_seen, ShortTime, sizeof(sensor_7_last_seen));      sensor_7_last_seen[sizeof(sensor_7_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_8") == 0){
                  sensor_8_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_8_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_8_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_8_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_8_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_8_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_8_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_8_transmissions,  JSON_data["Sends"], sizeof(sensor_8_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_8_CPU_freq,  JSON_data["CPU"], sizeof(sensor_8_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[7] = sensor_8_temp; humidity_readings[7] = sensor_8_humidity;  // arrays or indexes
                  strncpy(sensor_8_last_seen, ShortTime, sizeof(sensor_8_last_seen));      sensor_8_last_seen[sizeof(sensor_8_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_9") == 0){
                  sensor_9_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_9_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_9_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_9_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_9_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_9_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_9_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_9_transmissions,  JSON_data["Sends"], sizeof(sensor_9_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_9_CPU_freq,  JSON_data["CPU"], sizeof(sensor_9_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[8] = sensor_9_temp; humidity_readings[8] = sensor_9_humidity;  // arrays or indexes
                  strncpy(sensor_9_last_seen, ShortTime, sizeof(sensor_9_last_seen));      sensor_9_last_seen[sizeof(sensor_9_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_10") == 0){
                  sensor_10_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_10_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_10_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_10_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_10_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_10_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_10_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_10_transmissions,  JSON_data["Sends"], sizeof(sensor_10_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_10_CPU_freq,  JSON_data["CPU"], sizeof(sensor_10_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[9] = sensor_10_temp; humidity_readings[9] = sensor_10_humidity;  // arrays or indexes
                  strncpy(sensor_10_last_seen, ShortTime, sizeof(sensor_10_last_seen));      sensor_10_last_seen[sizeof(sensor_10_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_11") == 0){
                  sensor_11_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_11_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_11_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_11_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_11_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_11_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_11_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_11_transmissions,  JSON_data["Sends"], sizeof(sensor_11_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_11_CPU_freq,  JSON_data["CPU"], sizeof(sensor_11_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[10] = sensor_11_temp; humidity_readings[10] = sensor_11_humidity;  // arrays or indexes
                  strncpy(sensor_11_last_seen, ShortTime, sizeof(sensor_11_last_seen));      sensor_11_last_seen[sizeof(sensor_11_last_seen) - 1] = '\0'; 
      }

      else if(strcmp(sensor_ID, "T_12") == 0){
                  sensor_12_temp = JSON_data["Temp"] | NAN; // float e.g. 26.6
                  sensor_12_humidity = JSON_data["Humi"] | NAN; // up to 6 bytes: e.g., 65.9
                  sensor_12_pressure = JSON_data["Pre"] | NAN; // up to 7 bytes: e.g., 101.325
                  sensor_12_elevation = JSON_data["Elv"] | NAN; // up to 7 bytes: e.g., 1200.5
                  sensor_12_h_index = JSON_data["Heat_ind"] | NAN; // // up to 9 bytes: e.g., 27.94374
                  sensor_12_running_time_ms = JSON_data["UpTime"] | 0; // up to 11 bytes: e.g., 86400 
                  sensor_12_packet_size = JSON_data["PKT"] | 0;
              
                  strncpy(sensor_12_transmissions,  JSON_data["Sends"], sizeof(sensor_12_transmissions)); // up to 11 bytes: e.g., "1234567890"
                  strncpy(sensor_12_CPU_freq,  JSON_data["CPU"], sizeof(sensor_12_CPU_freq)); // 11 bytes: "80MHz CPU"
                  
                  temperature_readings[11] = sensor_12_temp; humidity_readings[11] = sensor_12_humidity;  // arrays or indexes
                  strncpy(sensor_12_last_seen, ShortTime, sizeof(sensor_12_last_seen));      sensor_12_last_seen[sizeof(sensor_12_last_seen) - 1] = '\0'; 
      }

        else {
               Serial.printf("Unknown sensor: %s\n", sensor_ID);
             }

}

*/


int extract_sensor_number(const char *identity){
    int index = 0;

     if (identity[0] == 'T' && identity[1] == '_') { // T_1
          index = atoi(&identity[2]);  // extract the index from the ID name: "T_7" → 7
        
          }
    return index;
}

uint8_t reading_count = 0;

// now_now_ms
void monitor_dryer_readings(){

  average_temp = 0.00;
  average_humi = 0.00;
  active_sensors = 0;
  average_press = 0.0;
  average_elev = 0.0;

  average_press = sensor_1_pressure + sensor_2_pressure + sensor_3_pressure + sensor_4_pressure + sensor_5_pressure + sensor_6_pressure
                + sensor_7_pressure + sensor_8_pressure + sensor_9_pressure + sensor_10_pressure + sensor_11_pressure + sensor_12_pressure;

  average_elev = sensor_1_elevation + sensor_2_elevation + sensor_3_elevation + sensor_4_elevation + sensor_5_elevation + sensor_6_elevation
               + sensor_7_elevation + sensor_8_elevation + sensor_9_elevation + sensor_10_elevation + sensor_11_elevation + sensor_12_elevation;

  for(int i=0; i<(sizeof(temperature_readings)/sizeof(temperature_readings[0])); i++){
    if(temperature_readings[i] >= 2.00){
      active_sensors++;
      average_temp += temperature_readings[i];
      average_humi += humidity_readings[i];
    }
  }

  Serial.print("Active Sensors: "); Serial.println(active_sensors);

  if(active_sensors <= 0){
      average_temp = 0.00;
      average_humi = 0.00;
      average_press = 0.00;
      average_elev = 0.00;
  }
  else{
      average_temp /= active_sensors;
      average_humi /= active_sensors;
      average_press /= active_sensors;
      average_elev /= active_sensors;
  }

  // ==============================
  // STORE HISTORY (Refined Block)
  // ==============================

  if(new_average_temp != average_temp){

      // shift arrays backward (oldest dropped)
      for(int i = 9; i > 0; i--){
          historical_temperature_readings[i] = historical_temperature_readings[i-1];
          historical_outside_temperatures[i] = historical_outside_temperatures[i-1];
          historical_humidity_readings[i]    = historical_humidity_readings[i-1];
          historical_pressure_readings[i]    = historical_pressure_readings[i-1];
          strcpy(time_stamps[i], time_stamps[i-1]);
      }

      // insert newest at index 0
      historical_temperature_readings[0] = average_temp;
      historical_outside_temperatures[0] = sensor_12_temp;
      historical_humidity_readings[0]    = average_humi;
      historical_pressure_readings[0]    = average_press;
      strcpy(time_stamps[0], ShortTime); // HH:MM

      if(reading_count < 10) reading_count++;

      new_average_temp = average_temp;
  }

  // ==============================

  dtostrf(average_temp, 3, 1, average_temp_str);
  dtostrf(average_humi, 3, 1, average_humi_str);

  Serial.print("Average Temp: "); Serial.println(average_temp);
  Serial.print("Average Humi: "); Serial.println(average_humi);
}




// Helper function to write static metadata to a file
void write_json_metadata(File &file) {
    if (!file) return;
    
    // Create a JSON document for metadata
    StaticJsonDocument<1024> metaDoc;
    
    // SYSTEM PARAMETERS (static)
    JsonObject System = metaDoc["System"].to<JsonObject>();
    System["FW_Version"] = FW_VERSION;
    System["Device_ID"] = devicename; // Assuming you have a device ID
    System["Created_Date"] = SystemDate;
    System["Created_Time"] = SystemTime;
    System["Max_File_Size_MB"] = MAX_FILE_SIZE / (1024 * 1024);
    System["Max_Total_Size_MB"] = MAX_TOTAL_SIZE / (1024 * 1024);
    
    // Sensor positions (static configuration)
    JsonObject Sensor_Config = metaDoc["Sensor_Config"].to<JsonObject>();
    
    JsonObject Sensor_1_Config = Sensor_Config["1"].to<JsonObject>();
    Sensor_1_Config["Position"] = "AIR INLET 1";
    
    JsonObject Sensor_2_Config = Sensor_Config["2"].to<JsonObject>();
    Sensor_2_Config["Position"] = "AIR INLET 2";
    
    JsonObject Sensor_3_Config = Sensor_Config["3"].to<JsonObject>();
    Sensor_3_Config["Position"] = "AIR OUTLET 1";
    
    JsonObject Sensor_4_Config = Sensor_Config["4"].to<JsonObject>();
    Sensor_4_Config["Position"] = "AIR OUTLET 2";
    
    JsonObject Sensor_5_Config = Sensor_Config["5"].to<JsonObject>();
    Sensor_5_Config["Position"] = "DRYING BED LOWEST";
    
    JsonObject Sensor_6_Config = Sensor_Config["6"].to<JsonObject>();
    Sensor_6_Config["Position"] = "DRYING BED MIDDLE";
    
    JsonObject Sensor_7_Config = Sensor_Config["7"].to<JsonObject>();
    Sensor_7_Config["Position"] = "DRYING BED TOP";
    
    JsonObject Sensor_8_Config = Sensor_Config["8"].to<JsonObject>();
    Sensor_8_Config["Position"] = "DRYING BED MIDDLE ZONE";
    
    JsonObject Sensor_9_Config = Sensor_Config["9"].to<JsonObject>();
    Sensor_9_Config["Position"] = "DRYING BED ENTRY ZONE";
    
    JsonObject Sensor_10_Config = Sensor_Config["10"].to<JsonObject>();
    Sensor_10_Config["Position"] = "DRYING BED EXIT ZONE";
    
    JsonObject Sensor_11_Config = Sensor_Config["11"].to<JsonObject>();
    Sensor_11_Config["Position"] = "DRYING BED RANDOM POSITION";
    
    JsonObject Sensor_12_Config = Sensor_Config["12"].to<JsonObject>();
    Sensor_12_Config["Position"] = "DRYING BED AMBIENT ZONE";
    
    // Start JSON array for data entries
    file.print("{\"metadata\":");
    serializeJson(metaDoc, file);
    file.print(",\"data\":[\n");
    
    // Write metadata to global variable for reference
    serializeJson(metaDoc, meta_data);
    meta_data[sizeof(meta_data) - 1] = '\0';
    
    LOG("[JSON] Metadata written to new file");
}

char serialization_log[512] = "JSON Serialization Not Done!";
char csv_bind_log[512] = "CSV Serialization Not Done!";


bool bind_dynamic_data_into_json() {

    bool bound_successfully = false;

    JSON_sendable.clear();
   // save_counter++; // save this into EEPROM or SPIFFS

    // ================= SYSTEM RUNTIME DATA =================
    JsonObject System = JSON_sendable["System"].to<JsonObject>();
    System["Uptime"] = (now_now_ms / 1000);
    System["Date"] = SystemDate;
    System["Time"] = SystemTime;
    System["Datalog_Count"] = save_counter;
    System["Voltage"] = voltage;
    System["Internal_Temp"] = cabin_temperature;
    System["Internal_Fan"] = inner_fan_status;
    System["OTA_Status"] = ota_log;
    System["LastOTA_Update"] = LastOTAUpdate;
    System["SD_TOTAL_SIZE"] = sd_storage_size;
    System["SD_USED_SPACE"] = usedSpace;
    System["SD_REMAINING_SPACE"] = freeSpace;

    // ================= DRYER SNAPSHOT =================
    JsonObject Dryer_Conditions = JSON_sendable["Dryer_Conditions"].to<JsonObject>();
    Dryer_Conditions["Active_Sensors"] = active_sensors;
    Dryer_Conditions["Average_Temperature"] = average_temp;
    Dryer_Conditions["Average_Humidity"] = average_humi;
    Dryer_Conditions["Average_Pressure"] = average_press;

    // ================= HISTORICAL SNAPSHOT ARRAYS =================
    JsonObject All_Sensors = JSON_sendable["All_Sensors"].to<JsonObject>();

    for (int i = 0; i < active_sensors; i++) {
        All_Sensors["Temperature_Readings"][i] = historical_temperature_readings[i];
        All_Sensors["External Temperatures"][i] = historical_outside_temperatures[i];
        All_Sensors["Humidity_Readings"][i] = historical_humidity_readings[i];
        All_Sensors["Pressure_Readings"][i] = historical_pressure_readings[i];
        All_Sensors["Time"][i] = time_stamps[i];
    }

    // ================= INDIVIDUAL SENSOR RUNTIME VALUES =================

    JsonObject Sensor_1 = JSON_sendable["Sensor_1"].to<JsonObject>();
    Sensor_1["Temp"] = sensor_1_temp;
    Sensor_1["Humi"] = sensor_1_humidity;
    Sensor_1["Pressure"] = sensor_1_pressure;
    Sensor_1["Last_Seen"] = sensor_1_last_seen;
    Sensor_1["Sends"] = sensor_1_transmissions;

    JsonObject Sensor_2 = JSON_sendable["Sensor_2"].to<JsonObject>();
    Sensor_2["Temp"] = sensor_2_temp;
    Sensor_2["Humi"] = sensor_2_humidity;
    Sensor_2["Pressure"] = sensor_2_pressure;
    Sensor_2["Last_Seen"] = sensor_2_last_seen;
    Sensor_2["Sends"] = sensor_2_transmissions;

    JsonObject Sensor_3 = JSON_sendable["Sensor_3"].to<JsonObject>();
    Sensor_3["Temp"] = sensor_3_temp;
    Sensor_3["Humi"] = sensor_3_humidity;
    Sensor_3["Pressure"] = sensor_3_pressure;
    Sensor_3["Last_Seen"] = sensor_3_last_seen;
    Sensor_3["Sends"] = sensor_3_transmissions;

    JsonObject Sensor_4 = JSON_sendable["Sensor_4"].to<JsonObject>();
    Sensor_4["Temp"] = sensor_4_temp;
    Sensor_4["Humi"] = sensor_4_humidity;
    Sensor_4["Pressure"] = sensor_4_pressure;
    Sensor_4["Last_Seen"] = sensor_4_last_seen;
    Sensor_4["Sends"] = sensor_4_transmissions;

    JsonObject Sensor_5 = JSON_sendable["Sensor_5"].to<JsonObject>();
    Sensor_5["Temp"] = sensor_5_temp;
    Sensor_5["Humi"] = sensor_5_humidity;
    Sensor_5["Pressure"] = sensor_5_pressure;
    Sensor_5["Last_Seen"] = sensor_5_last_seen;
    Sensor_5["Sends"] = sensor_5_transmissions;

    JsonObject Sensor_6 = JSON_sendable["Sensor_6"].to<JsonObject>();
    Sensor_6["Temp"] = sensor_6_temp;
    Sensor_6["Humi"] = sensor_6_humidity;
    Sensor_6["Pressure"] = sensor_6_pressure;
    Sensor_6["Last_Seen"] = sensor_6_last_seen;
    Sensor_6["Sends"] = sensor_6_transmissions;

    JsonObject Sensor_7 = JSON_sendable["Sensor_7"].to<JsonObject>();
    Sensor_7["Temp"] = sensor_7_temp;
    Sensor_7["Humi"] = sensor_7_humidity;
    Sensor_7["Pressure"] = sensor_7_pressure;
    Sensor_7["Last_Seen"] = sensor_7_last_seen;
    Sensor_7["Sends"] = sensor_7_transmissions;

    JsonObject Sensor_8 = JSON_sendable["Sensor_8"].to<JsonObject>();
    Sensor_8["Temp"] = sensor_8_temp;
    Sensor_8["Humi"] = sensor_8_humidity;
    Sensor_8["Pressure"] = sensor_8_pressure;
    Sensor_8["Last_Seen"] = sensor_8_last_seen;
    Sensor_8["Sends"] = sensor_8_transmissions;

    JsonObject Sensor_9 = JSON_sendable["Sensor_9"].to<JsonObject>();
    Sensor_9["Temp"] = sensor_9_temp;
    Sensor_9["Humi"] = sensor_9_humidity;
    Sensor_9["Pressure"] = sensor_9_pressure;
    Sensor_9["Last_Seen"] = sensor_9_last_seen;
    Sensor_9["Sends"] = sensor_9_transmissions;

    JsonObject Sensor_10 = JSON_sendable["Sensor_10"].to<JsonObject>();
    Sensor_10["Temp"] = sensor_10_temp;
    Sensor_10["Humi"] = sensor_10_humidity;
    Sensor_10["Pressure"] = sensor_10_pressure;
    Sensor_10["Last_Seen"] = sensor_10_last_seen;
    Sensor_10["Sends"] = sensor_10_transmissions;

    JsonObject Sensor_11 = JSON_sendable["Sensor_11"].to<JsonObject>();
    Sensor_11["Temp"] = sensor_11_temp;
    Sensor_11["Humi"] = sensor_11_humidity;
    Sensor_11["Pressure"] = sensor_11_pressure;
    Sensor_11["Last_Seen"] = sensor_11_last_seen;
    Sensor_11["Sends"] = sensor_11_transmissions;

    JsonObject Sensor_12 = JSON_sendable["Sensor_12"].to<JsonObject>();
    Sensor_12["Temp"] = sensor_12_temp;
    Sensor_12["Humi"] = sensor_12_humidity;
    Sensor_12["Pressure"] = sensor_12_pressure;
    Sensor_12["Last_Seen"] = sensor_12_last_seen;
    Sensor_12["Sends"] = sensor_12_transmissions;

    // ================= ENVIRONMENT =================
    JsonObject Environment = JSON_sendable["Environment"].to<JsonObject>();
    Environment["Solar_Radiation"] = average_solar_radiation;
    Environment["Wind_Speed"] = average_wind_speed;

    // ================= SERIALIZATION =================

                // ================= SERIALIZATION =================

            size_t len_sd   = serializeJsonPretty(JSON_sendable, sendable_to_sd_card); // serializeJsonPretty
            size_t len_cloud = serializeJson(JSON_sendable, sendable_to_cloud_db);

            if (len_sd == 0 || len_cloud == 0) {
                snprintf(serialization_log,
                        sizeof(serialization_log),
                        "[JSON] Serialization failed!");
                LOG(serialization_log);
                return false;
            }

            // Optional: detect truncation (ArduinoJson does NOT null-terminate if truncated)
            if (len_sd >= sizeof(sendable_to_sd_card) ||
                len_cloud >= sizeof(sendable_to_cloud_db))
            {
                snprintf(serialization_log,
                        sizeof(serialization_log),
                        "[JSON] WARNING: Buffer may be too small (truncation detected)");
                LOG(serialization_log);
            }

            bound_successfully = true;

            snprintf(serialization_log,
                    sizeof(serialization_log),
                    "[JSON] Serialized OK | SD: %u bytes | Cloud: %u bytes",
                    (unsigned int)len_sd,
                    (unsigned int)len_cloud);

            LOG(serialization_log);


    return bound_successfully;
}




bool bind_dynamic_data_into_csv(){

    bool bound_successfully = false;

    // Clear buffer
    memset(sendable_to_sd_card_csv, 0, sizeof(sendable_to_sd_card_csv));

    // Build single CSV row
    int len = snprintf(
        sendable_to_sd_card_csv,
        sizeof(sendable_to_sd_card_csv),

        // ================= HEADER ORDER =================
        "%s,%s," // System + Dryer snapshot [Date, Time]
     //   "%.2f,%.2f,%.2f,"   
        "%.2f,%.2f,%.2f,"                  // Sensor 1
        "%.2f,%.2f,%.2f,"                  // Sensor 2
        "%.2f,%.2f,%.2f,"                  // Sensor 3
        "%.2f,%.2f,%.2f,"                  // Sensor 4
        "%.2f,%.2f,%.2f,"                  // Sensor 5
        "%.2f,%.2f,%.2f,"                  // Sensor 6
        "%.2f,%.2f,%.2f,"                  // Sensor 7
        "%.2f,%.2f,%.2f,"                  // Sensor 8
        "%.2f,%.2f,%.2f,"                  // Sensor 9
        "%.2f,%.2f,%.2f,"                  // Sensor 10
        "%.2f,%.2f,%.2f,"                  // Sensor 11
        "%.2f,%.2f,%.2f,"                  // Sensor 12
        "%.2f,%.2f,%llu,%.2f\n",                     // Environment [solar, wind], uptime, voltage

        // ================= VALUES =================
        SystemDate,
        ShortTime_am_pm, //ShortTime, //SystemTime, 
        // average_temp,
       // average_humi,
       // average_press,

        sensor_1_temp, sensor_1_humidity, sensor_1_pressure,
        sensor_2_temp, sensor_2_humidity, sensor_2_pressure,
        sensor_3_temp, sensor_3_humidity, sensor_3_pressure,
        sensor_4_temp, sensor_4_humidity, sensor_4_pressure,
        sensor_5_temp, sensor_5_humidity, sensor_5_pressure,
        sensor_6_temp, sensor_6_humidity, sensor_6_pressure,
        sensor_7_temp, sensor_7_humidity, sensor_7_pressure,
        sensor_8_temp, sensor_8_humidity, sensor_8_pressure,
        sensor_9_temp, sensor_9_humidity, sensor_9_pressure,
        sensor_10_temp, sensor_10_humidity, sensor_10_pressure,
        sensor_11_temp, sensor_11_humidity, sensor_11_pressure,
        sensor_12_temp, sensor_12_humidity, sensor_12_pressure,

        average_solar_radiation,
        average_wind_speed,
        (unsigned long long)(now_now_ms / 1000), // converted to secs
         voltage
       
    );

    // ================= VALIDATION =================
    if (len <= 0) {
        snprintf(csv_bind_log,
                 sizeof(csv_bind_log),
                 "[CSV] Serialization failed!");
        LOG(csv_bind_log);
        return false;
    }

    if (len >= sizeof(sendable_to_sd_card_csv)) {
        snprintf(csv_bind_log,
                 sizeof(csv_bind_log),
                 "[CSV] WARNING: Buffer too small (truncated)");
        LOG(csv_bind_log);
    }

    bound_successfully = true;

    snprintf(csv_bind_log,
             sizeof(csv_bind_log),
             "[CSV] Serialized OK | %u bytes",
             (unsigned int)len);

    LOG(csv_bind_log);

    return bound_successfully;
}



/*
bool bind_dynamic_data_into_json(){
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
        System["OTA_Status"] = ota_log;
        System["LastOTA_Update"] = LastOTAUpdate;

        System["SD_REMAINING_SPACE"] = storage;

        // SNAPSHOTS OF READINGS AT A CERTAIN TIME, NOT CUMMULATED MEDIANS/MEANS
        JsonObject Dryer_Conditions      = JSON_sendable["Dryer_Conditions"].to<JsonObject>();
        Dryer_Conditions["Active_Sensors"] = active_sensors;
        Dryer_Conditions["Average_Temperature"] = average_temp;
        Dryer_Conditions["Average_Humidity"] = average_humi;
        Dryer_Conditions["Average_Pressure"] = average_press;

        
        //ALL SENSOR COMBINED SENSOR PARAMETERS
        JsonObject All_Sensors      = JSON_sendable["All_Sensors"].to<JsonObject>();
    for(int i = 0; i < active_sensors; i++){ 
        All_Sensors["Temperature_Readings"][i] = historical_temperature_readings[i];
        All_Sensors["Humidity_Readings"][i] = historical_humidity_readings[i];
        All_Sensors["Pressure_Readings"][i] = historical_pressure_readings[i];
        All_Sensors["Time"][i] = time_stamps[i];
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
          Sensor_12["Temp"]      =   sensor_12_temp;
          Sensor_12["Humi"]      =   sensor_12_humidity;
          Sensor_12["Pressure"]  =   sensor_12_pressure;
          Sensor_12["Last_Seen"] =   sensor_12_last_seen;
          Sensor_12["Sends"]     =   sensor_12_transmissions;

          // SOLAR RADIATION DATA // WIND SPEED DATA
      JsonObject Environment    = JSON_sendable["Environment"].to<JsonObject>();
          Environment["Solar_Radiation"]     = average_solar_radiation;
          Environment["Wind_Speed"]     = average_wind_speed;
          
          // --- Serialize into a buffer or string ---
           size_t len = serializeJsonPretty(JSON_sendable, sendable_to_sd_card); /// to save to data.json, then as dada.csv
           serializeJson(JSON_sendable, sendable_to_cloud_db);  // to send via the 4G SIM to upload_to_web_of_iot(sendable_to_cloud_db)

      
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
            LOG(sendable_to_sd_card); // optional: can be commented out if too large//
          } 
          
  return bound_successfully;

 
}
*/

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
            data_update_interval = 120ULL * 60ULL * 1000ULL; // 120 minutes
            break;
        case POWER_LOW:
            data_update_interval = 30ULL * 60ULL * 1000ULL;  // 30 minutes
            break;
        case POWER_MODERATE:
            data_update_interval = 10ULL * 60ULL * 1000ULL;  // 10 minutes
            break;
        case POWER_EXCELLENT:
            data_update_interval = 10ULL * 1000ULL;          // 10 seconds
            break;
    }
    
    Serial.printf("Power mode %d: Read frequency = %.1f minutes\n", 
                 current_power, (float)data_update_interval / 60000.0f);
}

//  Serial.print("Inside Temp: "); Serial.print(cabin_temperature);  Serial.print("\tFAN: ");


void monitor_box_conditions() {

    // --- Read internal temperature ---
    cabin_temperature = real_time.getTemperature();

    // ---------- FAN CONTROL (with hysteresis) ----------
    
    if (cabin_temperature >= 29.5f && !innerFan_ON) { // turn fan ON
        digitalWrite(innerFAN, HIGH);
        innerFan_ON = true;
        buzzer.beep(1, 50, 0);
    }
    else if (cabin_temperature <= 27.0f && innerFan_ON) { // turn fan OFF
        digitalWrite(innerFAN, LOW);
        innerFan_ON = false;
        buzzer.beep(2, 300, 200);
    }

    // ---------- NIGHT LIGHT CONTROL ----------
    // ON from 8:00 → 07:00
    /*
    if (is_night_time && !night_Light_ON) { // turn light ON
        digitalWrite(night_Light, HIGH);
        night_Light_ON = true;
        buzzer.beep(1, 100, 0);
    }
    else if (!is_night_time && night_Light_ON) { // turn light OFF
        digitalWrite(night_Light, LOW);
        night_Light_ON = false;
        buzzer.beep(2, 100, 100);
    }
    */

    // ---------- STATUS LOG ----------
    snprintf(
                internals_log,
                sizeof(internals_log),
                "SD: %s | System Temp: %.1fC | Fan: %s",
                sd_initialized?"Initialized":"Failed!",
                cabin_temperature,
                innerFan_ON ? "ON" : "OFF"
                
           );
    
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

// ====== Wi-Fi Connect Cycle ======
bool connectKnownWiFi() {
  bool connected_state = false;
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  int n = WiFi.scanNetworks();
  if (n <= 0) return false;

  int bestIndex = -1, bestRSSI = -999;
  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    for (int j = 0; j < knownCount; j++) {
      if (foundSSID == knownNetworks[j].SECRET_SSID) {
        if (rssi > bestRSSI) { bestRSSI = rssi; bestIndex = j; }
      }
    }
  }

  if (bestIndex == -1) return false;

  WiFi.begin(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(100);
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    //err = 14;
    //error_log(err);
    connected_state = true;
   // return true;
  } else { // WiFi connection failed
    //err = 15;
    //error_log(err);
    connected_state = false;
   // return false;
  }

  return connected_state;
}

// Add dynamic sleep based on conditions
void optimize_power_consumption() {
    // Reduce update frequency during night/low activity
    bool is_night = (hr > 20 || hr < 6);
    bool low_activity = (average_temp < 18.0 && average_humi > 60.0); // go to idle at night
    
    if (is_night && low_activity) {
      data_update_interval = 30ULL * 60ULL * 1000ULL;
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
        .onStart([]() { otaStarted = true; otaProgress = false;}) 
        .onEnd([]() { otaFinished = true; otaProgress = false; }) 
        .onProgress([](unsigned int progress, unsigned int total) { otaProgress = true;
           // snprintf(ota_log, sizeof(ota_log), "Progress: %u%%", (progress * 100) / total);
           // flash(now_now_ms, blinker, 50, 50, 0, 0);
        })
        .onError([](ota_error_t error) { otaError = true; otaProgress = false;
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

  Serial.printf("\tPOST JSON => %s\n\tTo: %s\n", httpsData, server_address);

  WiFiClientSecure client;   // no heap leak
  client.setInsecure();      // accept all certs (consider proper CA in production)

  HTTPClient https;
  if (https.begin(client, server_address)) {
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

/*
void upload_to_web_of_iot(){
   //  Serial.print("\tData to send via HTTP POST => ");    Serial.println(dataPack);    Serial.print("\tTo: "); Serial.println(server_address); 
   
    
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
      
    //  bool initialized = https.begin(server_address); 
        bool initialized = https.begin(*client, server_address); // Your Domain name with URL path or IP address with path

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

}

*/



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




void query_rtc(){

    char root[5] = "th";

    char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
    char Moonth[12][12] = {"January", "February", "March", "April", "May", "June",
                           "July", "August", "September", "October", "November", "December"};
    char short_Month[12][10] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};

    char hr_str[5] = "";
    char min_str[5] = "";
    char sec_str[5] = "";
    char day_str[15] = "";
    char date_str[10] = "";
    char mth_str[15] = "";
    char short_mth_str[12] = "";
    char yr_str[7] = "";

    uint8_t datey = 0;
    uint8_t reminder = 0;

    if(clock_is_working){

        DateTime time_now = real_time.now();

        hr = time_now.hour();
        mint = time_now.minute();
        sec = time_now.second();

        datey = time_now.day();
        mth   = time_now.month();
        mwaka = time_now.year();
        day_  = time_now.dayOfTheWeek();

        cabin_temperature = real_time.getTemperature();

        if(hr > 24){
            hr = 24; mint = 59; sec = 59;
            Serial.println("Time Chip Failed!");
        }

        // ---- 12 hour correction ----
        reminder = (hr % 12);
        uint8_t display_hr = (hr <= 12) ? hr : reminder;
        if(display_hr == 0) display_hr = 12;  // midnight case

        // ---- Safe numeric conversions with zero padding ----
        snprintf(hr_str,  sizeof(hr_str),  "%02u", display_hr);
        snprintf(min_str, sizeof(min_str), "%02u", mint);
        snprintf(sec_str, sizeof(sec_str), "%02u", sec);
        snprintf(date_str,sizeof(date_str),"%02u", datey);
        snprintf(yr_str,  sizeof(yr_str),  "%u", mwaka);

        // ---- Copy text safely ----
        snprintf(day_str,        sizeof(day_str),        "%s", daysOfTheWeek[day_]);
        snprintf(mth_str,        sizeof(mth_str),        "%s", Moonth[mth-1]);
        snprintf(short_mth_str,  sizeof(short_mth_str),  "%s", short_Month[mth-1]);

        // ===============================
        // Construct Time Strings
        // ===============================

        snprintf(ShortTime, sizeof(ShortTime), "%s:%s", hr_str, min_str);
        snprintf(SystemTime, sizeof(SystemTime), "%s:%s", ShortTime, sec_str);
        snprintf(ShortTime_am_pm, sizeof(ShortTime_am_pm),
                 "%s %s", ShortTime, (hr < 12) ? "am" : "pm");

        // ===============================
        // Date suffix logic (fixed bug)
        // ===============================
        if(datey == 1 || datey == 21 || datey == 31)
            snprintf(root, sizeof(root), "st");
        else if(datey == 2 || datey == 22)
            snprintf(root, sizeof(root), "nd");
        else if(datey == 3 || datey == 23)
            snprintf(root, sizeof(root), "rd");
        else
            snprintf(root, sizeof(root), "th");

        // ===============================
        // Construct Date Strings
        // ===============================

        snprintf(SystemDate, sizeof(SystemDate),
                 "%s %s%s %s %s",
                 day_str, date_str, root, mth_str, yr_str);

        snprintf(ShortDate, sizeof(ShortDate),
                 "%s-%s-%s",
                 date_str, short_mth_str, yr_str);

        // ===============================
        // Debug Output
        // ===============================
        Serial.println();
        Serial.print("Internal Temperature: "); Serial.println(cabin_temperature);
        Serial.print("Short Time: "); Serial.println(ShortTime);
        Serial.print("Short Time AM/PM: "); Serial.println(ShortTime_am_pm);
        Serial.print("Full System Time: "); Serial.print(SystemTime);
        Serial.print("\tSystem Date: "); Serial.println(SystemDate);
        Serial.println();

        // ===============================
        // 10-Minute Trigger Logic
        // ===============================
        if(mint % 10 == 0){
            if(!data_sent) can_send = true;
            else can_send = false;
        }
        else{
            data_sent = false;
        }
    }

    // ===============================
    // Fallback Timing Mode
    // ===============================
    else{
        if(now_now_ms - last_upload_time_ms >= upload_fequency){
            can_send = true;
            last_upload_time_ms = now_now_ms;
        }
    }
}


uint16_t box_width  = 150;
uint16_t box_height = 95;

bool is_home = false;

// ---- Layout constants ----

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

uint32_t entry_count = 0;

// ---- Data table layout ----
// Table dimensions
static const uint16_t TABLE_X = 20;
static const uint16_t TABLE_Y = 40;
static const uint16_t TABLE_W = 360;
static const uint16_t TABLE_H = 230;

static const uint16_t ROW_H = 24;
static const uint8_t  MAX_ROWS = 8;
static const uint8_t  VISIBLE_ROWS = (TABLE_H / ROW_H) - 1; // Account for header

// Column widths - ensure they sum to TABLE_W
static const uint16_t COL_ID_W    = 40;   // Increased for better readability
static const uint16_t COL_TEMP_W  = 80;   // Adjusted for better distribution
static const uint16_t COL_HUMI_W  = 80;   // Adjusted for better distribution
static const uint16_t COL_PRE_W   = 85;   // Adjusted for better distribution
static const uint16_t COL_TIME_W  = 75;   // Remaining width

// Verify column widths sum correctly
static_assert(COL_ID_W + COL_TEMP_W + COL_HUMI_W + COL_PRE_W + COL_TIME_W == TABLE_W, 
              "Column widths must sum to TABLE_W");

// Text positioning constants
static const uint8_t TEXT_Y_OFFSET_HEADER = 17;  // Vertical position for header text
static const uint8_t TEXT_Y_OFFSET_ROWS = 20;    // Vertical position for row data
static const uint8_t TEXT_X_PADDING = 10;         // Left padding for text

void drawTableFrame() {
    uint16_t headerColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    
    // Outer border
    LCD.drawRect(TABLE_X, TABLE_Y, TABLE_W, TABLE_H, GxEPD_BLACK);
    
    // Header ribbon
    LCD.fillRect(TABLE_X, TABLE_Y, TABLE_W, ROW_H, headerColor);
    
    // Calculate column positions for cleaner code
    uint16_t colPos[5]; // X positions for column boundaries
    colPos[0] = TABLE_X;                    // Start
    colPos[1] = colPos[0] + COL_ID_W;       // After ID column
    colPos[2] = colPos[1] + COL_TEMP_W;     // After Temp column
    colPos[3] = colPos[2] + COL_HUMI_W;     // After Humi column
    colPos[4] = colPos[3] + COL_PRE_W;      // After Pressure column
    
    // Draw vertical lines
    for (int i = 1; i < 5; i++) { // Skip first (start) and last (end)
        LCD.drawLine(colPos[i], TABLE_Y, colPos[i], TABLE_Y + TABLE_H, GxEPD_BLACK);
    }
    
    // Draw horizontal lines between rows
    for (uint16_t y = TABLE_Y + ROW_H; y < TABLE_Y + TABLE_H; y += ROW_H) {
        LCD.drawLine(TABLE_X, y, TABLE_X + TABLE_W, y, GxEPD_BLACK);
    }
    
    // ---- Column titles ----
    LCD.setFont(&FreeSans9pt7b);
    LCD.setTextColor(GxEPD_WHITE);
    
    // Column headers with better alignment
    LCD.setCursor(colPos[0] + TEXT_X_PADDING, TABLE_Y + TEXT_Y_OFFSET_HEADER);
    LCD.print("ID");
    
    LCD.setCursor(colPos[1] + TEXT_X_PADDING, TABLE_Y + TEXT_Y_OFFSET_HEADER);
    LCD.print("Temp");
    
    LCD.setCursor(colPos[2] + TEXT_X_PADDING, TABLE_Y + TEXT_Y_OFFSET_HEADER);
    LCD.print("Humi");
    
    LCD.setCursor(colPos[3] + TEXT_X_PADDING, TABLE_Y + TEXT_Y_OFFSET_HEADER);
    LCD.print("Pressure");
    
    LCD.setCursor(colPos[4] + TEXT_X_PADDING, TABLE_Y + TEXT_Y_OFFSET_HEADER);
    LCD.print("Time");
    
    // Optional: Add units in smaller font
    
    LCD.setFont();
    LCD.setTextColor(GxEPD_WHITE);
    LCD.setCursor(colPos[1] + COL_TEMP_W - 30, TABLE_Y + TEXT_Y_OFFSET_HEADER - 2);
    LCD.print("degC");
    LCD.setCursor(colPos[2] + COL_HUMI_W - 20, TABLE_Y + TEXT_Y_OFFSET_HEADER - 2);
    LCD.print("%");
   // LCD.setCursor(colPos[3] + COL_PRE_W - 25, TABLE_Y + TEXT_Y_OFFSET_HEADER - 2);
   // LCD.print("hPa");
    
}

char table_log[32] = "...";

// Helper function to populate a row with data
void drawTableRow(uint8_t rowIndex, const char* id, float temp, float humi, float pressure, const char* time) {

    if (rowIndex >= MAX_ROWS) {
        strncat(table_log, "No Data to Tabulate\n",
                sizeof(table_log) - strlen(table_log) - 1);
        return;
    }
    else if(rowIndex < 0){
       strncat(table_log, "Invalid Entry\n",
                sizeof(table_log) - strlen(table_log) - 1);
        return;
    }

    else if(rowIndex == 0){ // make sure to not overwrite the header
       uint16_t yPos = TABLE_Y + (2 * ROW_H); // 
        
        LCD.setFont(&FreeSans9pt7b);
        LCD.setTextColor(GxEPD_BLACK);
        
        // Calculate column positions
        uint16_t colPos[5];
        colPos[0] = TABLE_X;
        colPos[1] = colPos[0] + COL_ID_W;
        colPos[2] = colPos[1] + COL_TEMP_W;
        colPos[3] = colPos[2] + COL_HUMI_W;
        colPos[4] = colPos[3] + COL_PRE_W;
        
        // Draw row data
        LCD.setCursor(colPos[0] + TEXT_X_PADDING-5, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_1"); 
        
        // Format temperature with 1 decimal
        char tempStr[8];
        snprintf(tempStr, sizeof(tempStr), "%.1f", temp);
        LCD.setCursor(colPos[1] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(tempStr);
        
        // Format humidity with 1 decimal
        char humiStr[8];
        snprintf(humiStr, sizeof(humiStr), "%.1f", humi);
        LCD.setCursor(colPos[2] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(humiStr);
        
        // Format pressure with 1 decimal
        char preStr[8];
        snprintf(preStr, sizeof(preStr), "%.1f", pressure);
        LCD.setCursor(colPos[3] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(preStr);
        
        LCD.setFont();
        LCD.setCursor(colPos[4] + TEXT_X_PADDING, yPos +  TEXT_Y_OFFSET_ROWS - ROW_H - 10);
        LCD.print(time);

        return;
    }

    else { // for all d remaining entries
        uint16_t yPos = TABLE_Y + ((rowIndex + 2) * ROW_H); // +1 to skip header
        
        LCD.setFont(&FreeSans9pt7b);
        LCD.setTextColor(GxEPD_BLACK);
        
        // Calculate column positions
        uint16_t colPos[5];
        colPos[0] = TABLE_X;
        colPos[1] = colPos[0] + COL_ID_W;
        colPos[2] = colPos[1] + COL_TEMP_W;
        colPos[3] = colPos[2] + COL_HUMI_W;
        colPos[4] = colPos[3] + COL_PRE_W;
        
        // Draw row data
        LCD.setCursor(colPos[0] + TEXT_X_PADDING-5, yPos + TEXT_Y_OFFSET_ROWS - ROW_H-2);
        LCD.print(id); // stop here if no data
        /*
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_2"); 
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_3"); 
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_4"); 
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_5"); 
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_6"); 
        LCD.setCursor(colPos[0] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print("T_7"); 
        */
                       
        // do not print zero'z

            // Format temperature with 1 decimal
        char tempStr[8];
        snprintf(tempStr, sizeof(tempStr), "%.1f", temp);
        LCD.setCursor(colPos[1] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(tempStr);
        
        // Format humidity with 1 decimal
        char humiStr[8];
        snprintf(humiStr, sizeof(humiStr), "%.1f", humi);
        LCD.setCursor(colPos[2] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(humiStr);
        
        // Format pressure with 1 decimal
        char preStr[8];
        snprintf(preStr, sizeof(preStr), "%.1f", pressure);
        LCD.setCursor(colPos[3] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H);
        LCD.print(preStr);
        LCD.setFont();
        LCD.setCursor(colPos[4] + TEXT_X_PADDING, yPos + TEXT_Y_OFFSET_ROWS - ROW_H-10);
        LCD.print(time);
    }
}

// Helper function to clear a specific row
void clearTableRow(uint8_t rowIndex) {
    if (rowIndex >= MAX_ROWS) return;
    
    uint16_t yPos = TABLE_Y + ((rowIndex + 1) * ROW_H);
    LCD.fillRect(TABLE_X + 1, yPos - ROW_H + 1, TABLE_W - 2, ROW_H - 2, GxEPD_WHITE);
}

// Helper function to clear entire table content (keeps frame)
void clearTableContent() {
    LCD.fillRect(TABLE_X + 1, TABLE_Y + ROW_H + 1, TABLE_W - 2, TABLE_H - ROW_H - 2, GxEPD_WHITE);
}



#define LINEGRAPH 1
#define BARGRAPH 2
#define PIECHART 3

uint8_t graph_mode = 1; // show line graph by default

#define FULLTABLE 1
#define ONLY_TEMP 2
#define ONLY_HUMI 3

uint8_t which_data_table = 1; // show all as default

/*
    homepage() → Tiles only
    tablePage() → Pure data
    graphPage() → Pure visualization
    filesPage() → Directory view
    logsPage() → Diagnostics
    otaPage() → OTA mode
*/

bool shouldRotateScreen(uint64_t screen_time_ms = 0) {
        // Display update (often in daytime, less frequent at night) 5mins vs 30mins
   
    if ((screen_time_ms - last_refresh_time_ms) >= screen_refresh_interval) {
        last_refresh_time_ms = screen_time_ms;
          if(currentScreen > 5) currentScreen = 1;
          else currentScreen++;
        return true;
    }

    return false;
}

void update_display(){

    if((!shouldRotateScreen(now_now_ms)) && (!special_call)) // Most of the time: subtraction, compasison, branch, return...Calling a function every loop that usually returns immediately...microseconds
        return;
    //  proceed either only after timer expiring or after receiving a special call 

    switch(currentScreen){
        case 0: Boot();
            break;

        case 1: homepage(); //  with tiles of temp, humi, wind, sun
            break;

        case 2: tablePage(); // having the table of results
            break;

        case 3: graphPage(); // having graphs of inside vs outside temp
            break;
        
        case 4: filesPage(); // showing the folders in main directory
        break;

        case 5: logsPage(); // files, saves, cloud sends, offline sensors, etc
            break;

        case 6: cloudPage();
            break;

        case 10: otaPage(); // showing OTA mode, WiFi, and IP
            break;

        default: errorPage();
        break;
    }

    LCD.hibernate();
    if(special_call)   special_call = false;  // reset after handling

}


void cloudPage(){

    uint16_t accentColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

    uint64_t now = now_now_ms;
    uint64_t elapsed = now - last_upload_time_ms;

    uint64_t remaining = 0;
    if (elapsed < upload_frequency_ms)
        remaining = upload_frequency_ms - elapsed;

    uint32_t rem_sec = remaining / 1000;
    uint16_t rem_min = rem_sec / 60;
    uint16_t rem_s   = rem_sec % 60;

    LCD.firstPage();
    do {
        LCD.fillScreen(GxEPD_WHITE);

        // ================= HEADER =================
        LCD.fillRect(1, 1, SCREEN_W - 2, 40, accentColor);

        LCD.setFont(&FreeSans12pt7b);
        LCD.setTextColor(GxEPD_WHITE);
        LCD.setCursor(20, 28);
        LCD.print("Cloud Connectivity");

        LCD.setTextColor(GxEPD_BLACK);

        int y = 60;

        // ================= WIFI STATUS =================
        LCD.setFont(&FreeSans9pt7b);

        LCD.setCursor(10, y);
        LCD.print("WiFi Status:");

        if (wifi_connected) {
            LCD.setCursor(160, y);
            LCD.print("CONNECTED");
        } else {
            LCD.setCursor(160, y);
            LCD.print("DISCONNECTED");
        }

        y += 25;

        // ================= LAST UPLOAD =================
        LCD.drawLine(0, y, SCREEN_W, y, GxEPD_BLACK);
        y += 20;

        LCD.setCursor(10, y);
        LCD.printf("Last Upload: %s", last_upload_time);

        y += 20;

        LCD.setCursor(10, y);
        LCD.printf("Next Upload In: %02u:%02u", rem_min, rem_s);

        y += 20;

        LCD.setCursor(10, y);
        LCD.printf("Interval: %llu min",
                   (unsigned long long)(upload_frequency_ms / 60000ULL));

        y += 25;

        // ================= DATA STATUS =================
        LCD.drawLine(0, y, SCREEN_W, y, GxEPD_BLACK);
        y += 20;

        LCD.setCursor(10, y);
        LCD.printf("Data Sent: %s", data_sent ? "YES" : "NO");

        y += 20;

        LCD.setCursor(10, y);
        LCD.printf("Payload Size: %d bytes",
                   strlen(sendable_to_cloud_db));

        y += 25;

        // ================= HTTP REPORT =================
        LCD.drawLine(0, y, SCREEN_W, y, GxEPD_BLACK);
        y += 20;

        LCD.setCursor(10, y);
        LCD.print("Last HTTP Report:");
        y += 15;

        LCD.setFont(&FreeMono9pt7b);
        LCD.setCursor(10, y);
        LCD.print(Uploader.get_upload_report());

        y += 20;

        LCD.setCursor(10, y);
        LCD.print("Error Code:");
        y += 15;

        LCD.setCursor(10, y);
        LCD.print(Uploader.get_error_code());

        footer();

    } while (LCD.nextPage());
}

/*
void update_display(){

  if(currentScreen == 1) homepage();
  if(currentScreen == 2) dataPage();
  if(currentScreen == 3) otaPage();
  if(currentScreen == 4) logsPage();

  LCD.hibernate();
}
*/

void graphPage(){  //GRAPHS, CHARTS etc ----
      LCD.firstPage();
  do {

    if(graph_mode == LINEGRAPH) { // ---- line graph for inside vs outside temps, temporally varied 

        uint16_t accentColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

        // ---- Header ----
        LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, accentColor);
        LCD.fillRoundRect(260, 2, 120, 30, 5, GxEPD_WHITE);

        LCD.setFont(&FreeSans12pt7b);
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(270, 22);
        LCD.print("Graphs");

        LCD.setFont(&FreeSans9pt7b);
        LCD.setTextColor(GxEPD_WHITE);
        LCD.setCursor(20, 19);
        LCD.print("Tiles");

        LCD.setCursor(140, 19);
        LCD.print("Data Table");

        LCD.setFont();
        LCD.setTextColor(GxEPD_RED);
        LCD.setCursor(0, 285);
        LCD.print(storage_status_log);

        // ===============================
        // GRAPH AREA
        // ===============================

        int graph_left   = 40;
        int graph_right  = 360;
        int graph_top    = 60;
        int graph_bottom = 250;

        int graph_width  = graph_right - graph_left;
        int graph_height = graph_bottom - graph_top;

        // ---- Draw Axes ----
        LCD.drawLine(graph_left, graph_bottom, graph_right, graph_bottom, GxEPD_BLACK); // LCD.fillTriangle(); // X-axis // pointed forward
        LCD.drawLine(graph_left, graph_bottom+1, graph_right, graph_bottom+1, GxEPD_BLACK); // LCD.fillTriangle(); // X-axis // pointed forward
        LCD.drawLine(graph_left, graph_top, graph_left, graph_bottom, GxEPD_BLACK);    //  LCD.fillTriangle();  // Y-axis // pointed upward
        LCD.drawLine(graph_left-1, graph_top, graph_left-1, graph_bottom, GxEPD_BLACK);    //  LCD.fillTriangle();  // Y-axis // pointed upward
        
                    // ---- Axis Arrows ----
            // X-axis arrow (pointing right)
            LCD.fillTriangle(
                graph_right, graph_bottom,
                graph_right - 8, graph_bottom - 5,
                graph_right - 8, graph_bottom + 5,
                GxEPD_BLACK
            );

            // Y-axis arrow (pointing up)
            LCD.fillTriangle(
                graph_left, graph_top,
                graph_left - 5, graph_top + 8,
                graph_left + 5, graph_top + 8,
                GxEPD_BLACK
            );

        // ---- Only draw if we have data ----
        if(reading_count > 1){
            
            //FIXED SCALING
            float minTemp = 15.0f;
            float maxTemp = 55.0f;
            float range   = maxTemp - minTemp;
            /*
            //DYNAMIC SCALING

            // Find min & max for scaling
            float minTemp = historical_temperature_readings[0];
            float maxTemp = historical_temperature_readings[0];

            for(int i = 0; i < reading_count; i++){
                if(historical_temperature_readings[i] < minTemp)
                    minTemp = historical_temperature_readings[i];
                if(historical_temperature_readings[i] > maxTemp)
                    maxTemp = historical_temperature_readings[i];
            }

            // Prevent flat-line divide-by-zero
            if(maxTemp - minTemp < 0.5f){
                maxTemp += 0.5f;
                minTemp -= 0.5f;
            }

            float range = maxTemp - minTemp;
            */

            int prev_x1 = 0, prev_y1 = 0;  // inside
            int prev_x2 = 0, prev_y2 = 0;  // outside


                for(int i = 0; i < reading_count; i++){

                    //int x = graph_left + (graph_width * i) / 9;
                    int x = graph_left + (graph_width * i) / (reading_count - 1);
                    float inside  = historical_temperature_readings[i];
                    float outside = historical_outside_temperatures[i];

                    // Clamp both
                    if(inside < minTemp) inside = minTemp;
                    if(inside > maxTemp) inside = maxTemp;

                    if(outside < minTemp) outside = minTemp;
                    if(outside > maxTemp) outside = maxTemp;

                    int y_inside  = graph_bottom - ((inside  - minTemp) / range) * graph_height;
                    int y_outside = graph_bottom - ((outside - minTemp) / range) * graph_height;

                    // ---- DRAW OUTSIDE FIRST (background line) ----
                    LCD.fillCircle(x, y_outside, 2, GxEPD_BLACK);

                    if(i > 0){
                        LCD.drawLine(prev_x2, prev_y2, x, y_outside, GxEPD_BLACK);
                    }

                    // ---- DRAW INSIDE (accent color on top) ----
                    LCD.fillCircle(x, y_inside, 3, accentColor);

                    if(i > 0){
                        LCD.drawLine(prev_x1, prev_y1, x, y_inside, accentColor);
                    }

                    // Store previous positions
                    prev_x1 = x;
                    prev_y1 = y_inside;

                    prev_x2 = x;
                    prev_y2 = y_outside;
                }
             // ---- Y-axis Labels ----
                LCD.setFont(&FreeSans9pt7b);
                LCD.setTextColor(GxEPD_BLACK);

                LCD.setCursor(5, graph_top + 10);
                LCD.print("55C");

                LCD.setCursor(5, graph_bottom);
                LCD.print("15C");

             //Add faint horizontal lines every 5°C:
             /*
            for(int t = 20; t <= 50; t += 5){
                int y = graph_bottom - ((t - minTemp) / range) * graph_height;
                LCD.drawLine(graph_left, y, graph_right, y, GxEPD_RED);
            }
            */

           // ---- Time Labels (X-axis) ----
                LCD.setFont(&FreeSans9pt7b);
               // LCD.setFont();
                LCD.setTextColor(GxEPD_BLACK);

                for(int i = 0; i < reading_count; i++){

                    int x = graph_left + (graph_width * i) / 9;
                    // int x = graph_left + (graph_width * i) / (reading_count - 1);

                    // Only draw label if we have a valid timestamp
                    if(strlen(time_stamps[i]) > 0){

                        int label_x = x - 18;  // center under dot (HH:MM ≈ 5 chars)
                        int label_y = graph_bottom + 18;

                        // Prevent left overflow
                        if(label_x < 2)
                            label_x = 2;

                        // Prevent right overflow
                        if(label_x > SCREEN_W - 60)
                            label_x = SCREEN_W - 60;

                        LCD.setCursor(label_x, label_y);
                     // LCD.print(time_stamps[i], 90);  // HH:MM // rotate to 90deg to prevent the horizontal overlaps
                      //  if(i % 2 == 0) LCD.print(time_stamps[i]);  // HH:MM // Now labels appear every other point.
                            int step = reading_count > 6 ? 2 : 1; // ADAPTIVE label density
                            if(i % step == 0) LCD.print(time_stamps[i]); 
                    }
                }
                

            }
    }

     LCD.setFont(&FreeMono9pt7b); LCD.setTextColor(GxEPD_RED); //FreeMono9pt7b
     LCD.setCursor(70, 70); LCD.print(active_sensors); LCD.print(" sensors active");


  } while (LCD.nextPage());
    
}


void filesPage(){

  LCD.firstPage();
  do {

    uint16_t accentColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

    // ================= HEADER =================
    LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, accentColor);
    LCD.fillRoundRect(260, 2, 120, 30, 5, GxEPD_WHITE);

    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setCursor(270, 22);
    LCD.print("Data Files");

    LCD.setFont(&FreeSans9pt7b);
    LCD.setTextColor(GxEPD_WHITE);
    LCD.setCursor(20, 19);
    LCD.print("Tiles");

    LCD.setCursor(140, 19);
    LCD.print("Data");

    LCD.setCursor(200, 19);
    LCD.print("Graphs");

    LCD.setFont();
    LCD.setTextColor(GxEPD_RED);
    LCD.setCursor(0, 285);
    LCD.print(storage_status_log);

    // ================= FILE AREA =================

    int startY = 70;
    int boxW = 150;
    int boxH = 90;
    int gapX = 30;

    // ================= GET FILE SIZES =================
    


    // ===================================================
    // DRAW FILE ICON FUNCTION (inline style)
    // ===================================================

    auto drawFileIcon = [&](int x, int y, const char* name,
                            size_t sizeBytes,
                            const char* lastSave,
                            uint16_t color){

        // Main rectangle
        LCD.drawRect(x, y, boxW, boxH, color);

        // Folded corner
        LCD.fillTriangle(
            x + boxW - 20, y,
            x + boxW, y,
            x + boxW, y + 20,
            color
        );

        // File name
        LCD.setFont(&FreeSans9pt7b);
        LCD.setTextColor(color);
        LCD.setCursor(x + 10, y + 25);
        LCD.print(name);

        // Size
        LCD.setFont();
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(x + 10, y + 45);

        float sizeKB = sizeBytes / 1024.0f;
        LCD.print(sizeKB, 2);
        LCD.print(" KB");

        // Last save
        LCD.setCursor(x + 10, y + 60);
        LCD.print("Last Saved:");
        LCD.setCursor(x + 10, y + 75);
        LCD.print(lastSave);
    };

    // ================= DRAW FILES =================

    // JSON (left)
    drawFileIcon(40, startY,
                 "JSON LOG",
                 jsonSize,
                 last_json_save_time,
                 accentColor);

    // CSV (right)
    drawFileIcon(40 + boxW + gapX, startY,
                 "CSV LOG",
                 csvSize,
                 last_csv_save_time,
                 GxEPD_BLACK);

    // ================= FOOTER =================
    footer();

  } while (LCD.nextPage());
}


void logsPage(){

    uint16_t accentColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

    LCD.firstPage();
    do {

        LCD.fillScreen(GxEPD_WHITE);

        // ================= HEADER =================
        LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, GxEPD_RED);

        LCD.setFont(&FreeSans12pt7b);
        LCD.setTextColor(GxEPD_WHITE);
        LCD.setCursor(20, 22);
        LCD.print("System Diagnostics");

        LCD.setTextColor(GxEPD_BLACK);

        int y = 55;

        // =================================================
        // ICON STATUS ROW
        // =================================================

        int x = 20;
        int spacing = 100;

        // SD
         //LCD.setFont(&FreeSans12pt7b); 
         LCD.setFont(&FreeSans9pt7b);

         LCD.setCursor(x, 50); LCD.print("SD");
        drawFolderIcon(x, y);
        LCD.setCursor(x, y + 40); 
        LCD.print(storage_initialized ? "OK" : "FAIL");
        x += spacing;

        // JSON
         LCD.setCursor(x-10, 50); LCD.print("JSON");
         drawFileIcon(x, y);
        LCD.setCursor(x-20, y + 40);  
        LCD.print(json_bound_successfully ? "Saved" : "Failed"); // come here bind_json
        x += spacing;
        

        // CSV
         LCD.setCursor(x-10, 50); LCD.print("CSV");
        drawFileIcon(x, y);
        LCD.setCursor(x-10, y + 40); 
        LCD.print(csv_bound_successfully ? "Saved" : "Failed");
        x += spacing;

        // Cloud
        LCD.setCursor(x-20, 50); LCD.print("CLOUD");
         drawCloudIcon(x, y);
        LCD.setCursor(x-20, y + 40); 
        //LCD.print(strstr(upload_log, "Success") ? "OK" : "FAIL");
        LCD.setFont();
        LCD.print(data_sent ? "Uploaded" : "Not Uploaded!");

        y += 65;

        LCD.drawLine(0, y, SCREEN_W, y, GxEPD_BLACK);
        y += 20;

        // =================================================
        // DETAILED LOG TEXT (Full Width)
        // =================================================

        LCD.setFont(&FreeMono9pt7b);

        LCD.setCursor(20, y);
        LCD.printf("JSON : %.50s", "20 array elements......");
        y += 18;

        LCD.setCursor(20, y);
        LCD.printf("CSV  : %.50s", "20 entries ......");
        y += 18;

        LCD.setCursor(20, y);
        LCD.printf("Cloud: %.50s", "upload_log...");
        y += 22;

        LCD.drawLine(0, y, SCREEN_W, y, GxEPD_BLACK);
        y += 15;

        // =================================================
        // PAYLOAD CONSOLE (Your existing one)
        // =================================================

        int consoleHeight = 90;
        LCD.drawRect(15, y, SCREEN_W - 30, consoleHeight, GxEPD_BLACK);

        LCD.setCursor(25, y + 15);
        LCD.print("Last Payload:");

      //  LCD.setFont(&FreeMono9pt7b);
       LCD.setFont();


        int textY = y + 25;
        int maxChars = 200;
        int charsPerLine = 45;
        int count = 0;

        LCD.setCursor(25, textY);

        for(int i = 0; i < maxChars && sendable_to_sd_card[i] != '\0'; i++){
            LCD.write(sendable_to_sd_card[i]);
            count++;

            if(count >= charsPerLine){
                count = 0;
                textY += 15;
                LCD.setCursor(25, textY);
            }
        }

        //footer();

    } while (LCD.nextPage());
}

void drawFolderIcon(int x, int y) {
    // Folder body
    LCD.fillRect(x, y + 8, 28, 18, GxEPD_BLACK);

    // Folder tab
    LCD.fillRect(x + 3, y, 14, 10, GxEPD_BLACK);

    // Inner highlight
    LCD.fillRect(x + 2, y + 10, 24, 14, GxEPD_WHITE);
}

void drawFileIcon(int x, int y) {
    LCD.drawRect(x, y, 22, 28, GxEPD_BLACK);

    // Folded corner
    LCD.drawLine(x + 15, y, x + 22, y + 7, GxEPD_BLACK);
    LCD.drawLine(x + 22, y + 7, x + 15, y + 7, GxEPD_BLACK);
}

void drawCloudIcon(int x, int y) {
    LCD.fillCircle(x + 10, y + 10, 6, GxEPD_BLACK);
    LCD.fillCircle(x + 18, y + 10, 8, GxEPD_BLACK);
    LCD.fillCircle(x + 26, y + 12, 6, GxEPD_BLACK);
    LCD.fillRect(x + 10, y + 12, 20, 10, GxEPD_BLACK);
}




void tablePage(){

     LCD.firstPage();
  do {

        
    if(which_data_table == FULLTABLE){ // the data table page
        
     // ---- Header ----
      uint16_t headerColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
      LCD.fillRect(1, 1, SCREEN_W - 3, HEADER_H, headerColor);
      LCD.fillRoundRect(120, 2, 130, 33, 5, GxEPD_WHITE);

      LCD.setFont(&FreeSans12pt7b);
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setCursor(130, 22);
      LCD.print("Data Table");

      LCD.setFont(&FreeSans9pt7b);
      LCD.setTextColor(GxEPD_WHITE);
      LCD.setCursor(20, 19);
      LCD.print("Tiles");

      //LCD.fillRect(250, 5, 2, 20, GxEPD_WHITE);

      LCD.setCursor(290, 19);
      LCD.print("Files");

      LCD.setCursor(40, 40); LCD.setFont(); LCD.print(upload_log);

  
      // ---- Table ----
      drawTableFrame();
      // In your setup or initialization:
      //  initializeTable();  // This shows T1-T8 with "--" placeholders immediately


     // uint8_t visible_rows = min(entry_count, MAX_ROWS);
      uint8_t visible_rows = min((uint8_t)entry_count, MAX_ROWS);

      
                  // To populate a single row (automatically uses T_1, T_2, etc.)
              if(sensor_1_temp > 0.00) drawTableRow(0, "T_1", sensor_1_temp, sensor_1_humidity, sensor_1_pressure, sensor_1_last_seen);  // Row 0 = T_1
              if(sensor_2_temp > 0.00) drawTableRow(1, "T_2",sensor_2_temp, sensor_2_humidity, sensor_2_pressure, sensor_2_last_seen);  // Row 1 = T_2
              if(sensor_3_temp > 0.00) drawTableRow(2, "T_3",sensor_3_temp, sensor_3_humidity, sensor_3_pressure, sensor_3_last_seen);  // Row 2 = T_3
              if(sensor_4_temp > 0.00) drawTableRow(3, "T_4",sensor_4_temp, sensor_4_humidity, sensor_4_pressure, sensor_4_last_seen);  // Row 3 = T_4
              if(sensor_5_temp > 0.00) drawTableRow(4, "T_5",sensor_5_temp, sensor_5_humidity, sensor_5_pressure, sensor_5_last_seen);  // Row 4 = T_5
              if(sensor_6_temp > 0.00) drawTableRow(5, "T_6",sensor_6_temp, sensor_6_humidity, sensor_6_pressure, sensor_6_last_seen);  // Row 5 = T_6
              if(sensor_7_temp > 0.00) drawTableRow(6, "T_7",sensor_7_temp, sensor_7_humidity, sensor_7_pressure, sensor_7_last_seen);  // Row 6 = T_7
              if(sensor_8_temp > 0.00) drawTableRow(7, "T_8",sensor_8_temp, sensor_8_humidity, sensor_8_pressure, sensor_8_last_seen);  // Row 7 = T_8
            /*
              drawTableRow(8, sensor_5_temp, 67.8, 1012.9, "14:31");  // Row 1 = T_2
              drawTableRow(9, sensor_5_temp, 67.8, 1012.9, "14:31");  // Row 1 = T_2
              drawTableRow(10, sensor_5_temp, 67.8, 1012.9, "14:31");  // Row 1 = T_2
              drawTableRow(11, sensor_5_temp, 67.8, 1012.9, "14:31");  // Row 1 = T_2
              drawTableRow(12, sensor_5_temp, 67.8, 1012.9, "14:31");  // Row 1 = T_2
              */

      /*
              // To populate all rows at once
              float temps[12] = {25.3, 26.1, 25.8, 25.9, 26.2, 25.7, 25.4, 25.6, 25.1, 25.5, 25.8, 25.2};
              float humis[12] = {65.5, 64.8, 65.2, 65.0, 64.5, 65.3, 65.1, 64.9, 65.4, 65.2, 65.0, 64.7};
              float pressures[12] = {1013.2, 1012.9, 1013.1, 1013.0, 1012.8, 1013.3, 1012.7, 1013.2, 1013.0, 1012.9, 1013.1, 1012.8};
              const char* times[12] = {"14:30", "14:31", "14:32", "14:33", "14:34", "14:35", 
                            "14:36", "14:37", "14:38", "14:39", "14:40", "14:41"};

          drawAllTableRows(temps, humis, pressures, times);
          */
        /*
        // Later, when data arrives for specific sensors:
        updateTableRow(0, 25.3, 65.5, 1013.2, "14:30");  // Updates T1 with real data
        updateTableRow(1, 26.1, 64.8, 1012.9, "14:31");  // Updates T2 with real data
        // T3-T8 remain visible but show "--" until updated

        // When T9 data arrives (optional):
        updateTableRow(7, 24.7, 66.2, 1013.5, "14:32");  // Updates T9 (will now appear)
      */

                  footer(); // universal footing

    }

          footer(); // semi universal footing

   } while (LCD.nextPage());

}


void errorPage(){
         LCD.firstPage();
  do {
               
                footer(); // semi universal footing

    } while (LCD.nextPage());
}

void otaPage(){
      LCD.firstPage();
  do {

    LCD.fillScreen(GxEPD_WHITE);

    uint16_t accent = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);

    // ===== Header =====
    LCD.setFont(&FreeSans24pt7b);
    LCD.setTextColor(accent);
    LCD.setCursor(50, 50);
    LCD.print("OTA MODE");

    // ===== WiFi Icon (Simple Bars) =====
    int baseX = 40;
    int baseY = 120;

    for (int i = 0; i < 4; i++) {
        LCD.fillRect(baseX + i * 15,
                     baseY - (i * 15),
                     10,
                     i * 15,
                     accent);
    }

    // ===== Device Name =====
    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setCursor(40, 170);
    LCD.print("Device:");
    LCD.setCursor(160, 170);
    LCD.print(devicename);

    // ===== Connected SSID =====
    LCD.setCursor(40, 200);
    LCD.print("WiFi:");
    LCD.setCursor(160, 200);
   // LCD.print(WiFi.SSID());
   // LCD.print(wifi_obj.WiFi.SSID());

    // ===== IP Address =====
    LCD.setCursor(40, 230);
    LCD.print("IP:");
    LCD.setCursor(160, 230);
    LCD.print(WiFi.localIP());

    // ===== OTA Status =====
    LCD.setFont(&FreeSans9pt7b);
    LCD.setCursor(40, 260);
    LCD.print(ota_log);

   // LCD.display();
     } while (LCD.nextPage());
}








const char Manufacturer[15] = "IntelliSys UG";
const char DeviceID[32] = "Dryer Monitoring System";
const char DeviceClient[36] = "Susan M PhD Project, MUARIK";
uint8_t SystemAddress[] = {0x68, 0xFE, 0x71, 0x88, 0x1A, 0x6C}; 
char macAddressStr[32]; // Format: XX:XX:XX:XX:XX:XX + null terminator

void Boot(){
  LCD.setRotation(2);
  LCD.setFullWindow();


    // Variables for text bounds
  int16_t tbx, tby; uint16_t tbw, tbh; 
  int16_t tbx2, tby2; uint16_t tbw2, tbh2;
  int16_t tbx3, tby3; uint16_t tbw3, tbh3;
  int16_t tbx4, tby4; uint16_t tbw4, tbh4;  // For fourth line
  int16_t tbx5, tby5; uint16_t tbw5, tbh5;  // For fourth line

   // Format MAC address as string
    snprintf(macAddressStr, sizeof(macAddressStr), 
             "Address=>[%02X:%02x:%02x:%02x:%02x:%02x]",
             SystemAddress[0], SystemAddress[1], SystemAddress[2],
             SystemAddress[3], SystemAddress[4], SystemAddress[5]);

    // Getting text bounds for each line with appropriate fonts
  LCD.setFont(&FreeSansBold24pt7b);
  LCD.getTextBounds(Manufacturer, 0, 0, &tbx, &tby, &tbw, &tbh);
  
  LCD.setFont(&FreeSans12pt7b);
  LCD.getTextBounds(DeviceID, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);

  LCD.setFont(&FreeMonoBold9pt7b); // 
  LCD.getTextBounds(DeviceClient, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);

   LCD.setFont(&FreeSans9pt7b);  // Smaller font for MAC address
   LCD.getTextBounds(macAddressStr, 0, 0, &tbx4, &tby4, &tbw4, &tbh4);

   LCD.getTextBounds(storage_status_log, 0, 0, &tbx5, &tby5, &tbw5, &tbh5);

    // Calculate X positions for centering
    uint16_t x1 = ((LCD.width() - tbw) / 2) - tbx; 
    uint16_t x2 = ((LCD.width() - tbw2) / 2) - tbx2; 
    uint16_t x3 = ((LCD.width() - tbw3) / 2) - tbx3;
    uint16_t x4 = ((LCD.width() - tbw4) / 2) - tbx4;  // For MAC address
    uint16_t x5 = ((LCD.width() - tbw5) / 2) - tbx5;
   // Fixed Y positions (can be adjusted based on screen layout needs)
    const uint16_t y1 = 40;   // Manufacturer position
    const uint16_t y2 = 100;  // Device ID position
    const uint16_t y3 = 220;  // Client info position
    const uint16_t y4 = 250;  // MAC address position
    const uint16_t y5 = 290;  // MAC address position

  //uint16_t y = ((LCD.height() - tbh) / 2) - tby;

      // Add a small delay before starting display update
    delay(10);

  LCD.firstPage(); //FULLY CENTERED TEXTS
  do{
        LCD.fillScreen(GxEPD_WHITE);
     
        // Manufacturer - Red if color display available
        LCD.setFont(&FreeSansBold24pt7b);
        LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
        LCD.setCursor(x1, y1);
        LCD.print(Manufacturer);
        
        // Device ID - Black
        LCD.setFont(&FreeSans12pt7b);
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(x2, y2);
        LCD.print(DeviceID);
        
        LCD.setFont();
        LCD.setcursor(1, 150); LCD.println(ESP_IDF_VER); 
         LCD.setcursor(1, 150); LCD.print(ARD_CORE_VER);
        
        // Client Info - Black with monospace font for clean look
        LCD.setFont(&FreeMonoBold9pt7b);
        LCD.setCursor(x3, y3);
        LCD.print(DeviceClient);
        
        // Optional: A separator
         LCD.drawLine(20, y3 + 50, LCD.width() - 40, y3 + 50, GxEPD_BLACK);
        
                 // Client Info - Black with monospace font for clean look
        LCD.setFont(&FreeSans9pt7b);         LCD.setTextColor(GxEPD_RED);

        
        LCD.setCursor(x4, y4);
        LCD.print(macAddressStr);

        LCD.setFont();
        LCD.setCursor(0, y5);
        LCD.print(storage_status_log);
        

     //   delay(1000);

    }  while (LCD.nextPage());
}



void homepage() {

  LCD.firstPage();
  do {

      LCD.fillScreen(GxEPD_WHITE);
      LCD.drawRect(0, 0, SCREEN_W, SCREEN_H, GxEPD_BLACK);
      LCD.drawRect(1, 1, SCREEN_W - 2, SCREEN_H - 2, GxEPD_BLACK);

  //  if (side_scroll == 1) { // 


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
      LCD.print("Tiles");

      LCD.setFont(&FreeSans9pt7b);
      LCD.setTextColor(GxEPD_WHITE);
      LCD.setCursor(140, 19);
      LCD.print("Data Table");

      LCD.fillRect(250, 5, 2, 20, GxEPD_WHITE);

      LCD.setCursor(290, 19);
      LCD.print("Graphs");


      // ---- Average Temperature ----
      drawInfoBox(LEFT_X, TOP_ROW_Y, "Av. Temperature", GxEPD_BLACK, GxEPD_BLACK);

      
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setFont(&FreeMono12pt7b); LCD.setCursor(142, 80); LCD.print("o");
      LCD.setFont(&FreeSans12pt7b); LCD.setCursor(155, 90); LCD.print("C");

       LCD.setFont(&FreeSansBold24pt7b); LCD.setTextColor(GxEPD_BLACK);   
       LCD.setCursor(50, 110);      LCD.print(average_temp_str);
    
        LCD.setFont(&FreeMono9pt7b); LCD.setTextColor(GxEPD_RED); //FreeMono9pt7b
        LCD.setCursor(60, 128); LCD.print(active_sensors);
      


      // ---- Humidity Box ----
      drawInfoBox(RIGHT_X, TOP_ROW_Y, "Dryer Humidity", GxEPD_BLACK, GxEPD_BLACK);

      LCD.setFont(&FreeMono12pt7b);
      LCD.setTextColor(GxEPD_BLACK);
      LCD.setCursor(RIGHT_X + 120, 100);      LCD.print("%");

      LCD.setTextColor(GxEPD_WHITE);
       //  LCD.setCursor(((10+box_width)+(1.2*box_width)), 37);          LCD.print(average_humi);

         LCD.setTextColor(GxEPD_BLACK);      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(50+((1.2*box_width)), 110);    LCD.print(average_humi_str); 



      // ---- Solar Radiation ----
      uint16_t fanColor = (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
      drawInfoBox(LEFT_X, BOTTOM_ROW_Y, "Solar Radiation", fanColor, fanColor);
    // draw an icon of the sun in this box



     //   LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b);       LCD.setTextColor(fanColor);

         LCD.setCursor(60, (215)); LCD.print(solar_rad_str); 
         LCD.setFont();  // FreeMono9pt7b
         LCD.setCursor(50, (232)); LCD.print("Watts / sq metres");

      // ---- Wind Speed ----
      drawInfoBox(RIGHT_X, BOTTOM_ROW_Y, "Wind Speed", fanColor, fanColor);

      LCD.setTextColor(fanColor);
      LCD.setCursor(RIGHT_X + 60, BOTTOM_ROW_Y + 45);
      

      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(20+(60+box_width), (215));    LCD.print(wind_speed_str);

         LCD.setFont(); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(90+(box_width), (232));  LCD.print("metres / second");
        
               footer(); // semi universal footing

    //} // side_scroll == 1
      
      

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

        // LOGS FOR NETWORK
            LCD.setFont();
            LCD.setTextColor(GxEPD_RED);
            LCD.setCursor(40, 260); LCD.print(internals_log);

            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_WHITE);
    
            //DATE
            LCD.setCursor(2, 290);  LCD.print(SystemDate); 

            //NETWORK BARS  // WIFI & UPLOADS
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






