/*
    XODDOCODE 2026 - 9TH MAR 2026
    SENSOR DATA-RECEIVES BY ESPNOW
    FAN CTRL BY PWM
    RTC
    PACKET ASSY
    SEND DATA BY UART
    RECEIVE COMMAND BY UART
*/
const char* devicename = "Fans_Controller";
const char *OTA_PASS = "43!21";

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

#include "Wire.h"
#include "RTClib.h"


#include <WiFi.h>
#include <esp_now.h>

#include "ArduinoOTA.h"
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include "time_keeper_2.h"
#include "smart_buzzer.h"
#include "packet_handler.h"
#include "uart_handler.h"

TimerKeeper sawa;
Buzzer buzzer(14);
PacketHandler packetHandler;
JsonDocument JSON_sendable; // Adjust size as needed

char sendable_to_cloud_db[8096]; // Buffer for cloud upload

// Global variables
uint32_t now_now_ms = 0;
uint64_t packet_loss_counter = 0;  // Changed to 0 instead of 1
uint64_t total_packets_received = 0;

double voltage = 0.0;

void check_time();
void regulate_fans();
void initialize_fans();
bool bind_dynamic_data_into_json();
void uart_data_handler(void *pvParameters);
void run_fan(const int channel, const int pwm_channel, const float duty_cycle);
void fan_speed_test(int how_many_fans, uint64_t rising_for, uint64_t falling_for);
void ramp_up_fan(const int any_channel, uint32_t duty_limit, uint64_t total_time_ms);
void ramp_down_fan(const int any_channel, uint32_t duty_limit, uint32_t final_stop_duty, uint64_t total_time_ms);


bool isNightTime();
bool check_weather_for_rain();
float aggregate_sensor_temperatures(); // get average temperature from all sensors for fan control
float aggregate_sensor_humidities(); // get average humidity from all sensors for fan control

char serialization_log[512] = "JSON Serialization Not Done!";
char csv_bind_log[512] = "CSV Serialization Not Done!";

// 2 tasks on core 0: PacketHandler and UART sender
// PACKET DETAILS parameters
const int PACKET_TASK_PRIORITY = 3; // Higher than normal tasks
const int PACKET_TASK_CORE = 0; 
const int PACKET_STACK_SIZE = 12288;

//UART DETAILS
const int UART_TASK_PRIORITY = 2; // Lower than packet handler
const int UART_TASK_CORE = 0;
const int UART_STACK_SIZE = 16384; // Large stack for JSON formatting and UART operations
TaskHandle_t uartTaskHandle = NULL; // Handle for UART task



float temp_when_cooling_fans_turned_on = 0.0f;
float humi_when_extract_fans_turned_on = 0.0f;

float temperature_gradient_due_to_cooling_fans = 0.0f; // to track how much the temperature has dropped since we turned on the cooling fans, so that we can decide when to turn them off based on the gradient, rather than just absolute thresholds
float temperature_gradient_due_to_extract_fans = 0.0f; // to track how much the temperature has dropped since we turned on the cooling fans, so that we can decide when to turn them off based on the gradient, rather than just absolute thresholds
float humidity_gradient_due_to_extract_fans = 0.0f; // to track how much the humidity has dropped since we turned on the extract fans, so that we can decide when to turn them off based on the gradient, rather than just absolute thresholds
float humidity_gradient_due_to_cooling_fans = 0.0f; // to track how much the humidity has dropped since we turned on the extract fans, so that we can decide when to turn them off based on the gradient, rather than just absolute thresholds


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

uint64_t sensor_1_last_seen_ms = 0; uint64_t sensor_2_last_seen_ms = 0; uint64_t sensor_3_last_seen_ms = 0; uint64_t sensor_4_last_seen_ms = 0;
uint64_t sensor_5_last_seen_ms = 0; uint64_t sensor_6_last_seen_ms = 0; uint64_t sensor_7_last_seen_ms = 0; uint64_t sensor_8_last_seen_ms = 0;
uint64_t sensor_9_last_seen_ms = 0; uint64_t sensor_10_last_seen_ms = 0; uint64_t sensor_11_last_seen_ms = 0; uint64_t sensor_12_last_seen_ms = 0;

bool cooling_fan_1_on = false, cooling_fan_2_on = false;
bool extract_fan_1_on = false, extract_fan_2_on = false, extract_fan_3_on = false, extract_fan_4_on = false;    

//want to measure how long the dryer temp/humi is responding to fans
uint64_t cooling_fan_1_on_time_ms = 0, cooling_fan_2_on_time_ms = 0;
uint64_t extract_fan_1_on_time_ms = 0, extract_fan_2_on_time_ms = 0, extract_fan_3_on_time_ms = 0, extract_fan_4_on_time_ms = 0;    

uint64_t cooling_fan_1_duration = 0, cooling_fan_2_duration = 0;
uint64_t extract_fan_1_duration = 0, extract_fan_2_duration = 0, extract_fan_3_duration = 0, extract_fan_4_duration = 0;

//printable on screen
char cooling_fan_1_status[60], cooling_fan_2_status[60]; 
char extract_fan_1_status[60], extract_fan_2_status[60], extract_fan_3_status[60], extract_fan_4_status[60];

// time fan went on
char cooling_fan_1_time_on_str[20] = "--:--:--", cooling_fan_2_time_on_str[20] = "--:--:--";
char extract_fan_1_time_on_str[20] = "--:--:--", extract_fan_2_time_on_str[20] = "--:--:--", extract_fan_3_time_on_str[20] = "--:--:--", extract_fan_4_time_on_str[20] = "--:--:--";

char cooling_fan_1_time_off_str[20] = "--:--:--", cooling_fan_2_time_off_str[20] = "--:--:--";
char extract_fan_1_time_off_str[20] = "--:--:--", extract_fan_2_time_off_str[20] = "--:--:--", extract_fan_3_time_off_str[20] = "--:--:--", extract_fan_4_time_off_str[20] = "--:--:--";

//use the SPI pins for the fans, as they are PWM capable and not used by other peripherals in this project
const uint8_t cooling_fan_1 = 4, cooling_fan_2 = 5;
const uint8_t extract_fan_1 = 13, extract_fan_2 = 15, extract_fan_3 = 18, extract_fan_4 = 23;
/*
    The original ESP32 (DevKit) features 16 hardware PWM channels (8 high-speed + 8 low-speed) via the LEDC peripheral. 
    The ESP32-S3 has 8 hardware PWM channels (LEDC), although it also includes specialized motor control (MCPWM) peripherals for extra functionality. 
*/
const int PWM_FREQ = 25000; // 25kHz is a recommended frequency for quiet fan operation.
const int PWM_CHANNEL_1 = 0;
const int PWM_CHANNEL_2 = 1;
const int PWM_CHANNEL_3 = 2;
const int PWM_CHANNEL_4 = 3;
const int PWM_CHANNEL_5 = 4;
const int PWM_CHANNEL_6 = 5;
const uint8_t all_fans = 6; // Assuming we have 6 fans to control (2 cooling + 4 extract)

const int resolution = 10; // 10 bit = 0-1023 or 12bit: 0-4096
uint64_t rise_time_secs = 0, fall_time_secs = 0;
uint32_t hold_time = 10 * 1000;
uint32_t max_duty;

// For debug logging
#define LOG(msg)  Serial.println(msg)


const uint32_t com_baud = 921600;
uint32_t uart_send_interval_ms = 5000; // Send data every 5 seconds

bool esp_now_initialized = false; char esp_now_status_msg[200];
bool serial_intialized = false;   char serial_init_log[200];

bool clock_initialized = false; bool clock_working = false; char clock_status_msg[200];

void setup() { delay(50);
    //INITIALIZING the buzzer task once - it will start suspended
      buzzer.createTask(4, 2048, 1); // Core 1 Task created in class to auto-Check only_if_buzzing_started
      buzzer.beep(1,50,0);
             
  
      Serial.begin(115200); // with serial monitor
      Serial2.begin(com_baud, SERIAL_8N1, 16, 17); // or 1Mbps with chip 2 to send 4kB in < 50ms
      Serial2.setRxBufferSize(1024); // for receiving commands from chip 2
      Serial2.setTimeout(500);

      snprintf(serial_init_log, sizeof(serial_init_log), "Serial 2 Initialized with BAUD: %lu", com_baud);
      delay(1000); // wait for serial to initialize

      
      clock_initialized = sawa.initialize_RTC();
      snprintf(clock_status_msg, sizeof(clock_status_msg), "RTC Clock: %s", clock_initialized ? "Initialized Successfully" : "Initialization Failed!");

       /*
      esp_now_initialized = initialize_espnow();
      snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "ESP-NOW : %s", esp_now_initialized ? "Initialized Successfully" : "Initialization Failed!");
      Serial.println(esp_now_status_msg);
      delay(1000);
      */

      // Initialize packet handler
      esp_now_initialized = packetHandler.begin(); // pinned to core 0: has espnow, callbacks, and queue creation inside
    if (!esp_now_initialized) snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "Failed to initialize packet handler!");
        
    else     snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "ESP-NOW Initialization Successful!");
    
    //show the stack before starting the UART task
    LOG("[Setup] Starting UART Task for sending data to receiver...");
    xTaskCreatePinnedToCore(
        uart_data_handler,   // Task function
        "UARTDataHandler",             // Name of the task
        UART_STACK_SIZE,           // Stack size in words
        NULL,                        // Task input parameter
        UART_TASK_PRIORITY,        // Priority of the task
        NULL,                        // Task handle
        UART_TASK_CORE             // Core where the task should run
    );

    // show the heap and the stack after starting the UART task
    LOG("[Setup] UART Task Started.");
    delay(1000);

        
    initialize_fans(); // all in PWM mode
    max_duty = (1 << resolution) - 1;  // auto-calc max duty
    // 255 for 8-bit, 1023 for 10-bit, 4095 for 12-bit resolution
    rise_time_secs = (10ULL * 1000ULL); // ACCELERATE FOR 10 SECONDS
    fall_time_secs = (5ULL * 1000ULL); // DECELERATE FOR 5 SECONDS

    fan_speed_test(all_fans, rise_time_secs, fall_time_secs);

    
    Serial.println(esp_now_status_msg);
    Serial.println(clock_status_msg);
    Serial.println(serial_init_log);
    
       
    
    delay(1000);

    




}



void initialize_fans(){
        ledcSetup(PWM_CHANNEL_1, PWM_FREQ, resolution); // channel 0, 25kHz, 10-bit resolution
        ledcSetup(PWM_CHANNEL_2, PWM_FREQ, resolution); // channel 1, 25kHz, 10-bit resolution
        ledcSetup(PWM_CHANNEL_3, PWM_FREQ, resolution); // channel 2, 25kHz, 10-bit resolution
        ledcSetup(PWM_CHANNEL_4, PWM_FREQ, resolution); // channel 2, 25kHz, 10-bit resolution
        ledcSetup(PWM_CHANNEL_5, PWM_FREQ, resolution); // channel 2, 25kHz, 10-bit resolution
        ledcSetup(PWM_CHANNEL_6, PWM_FREQ, resolution); // channel 2, 25kHz, 10-bit resolution
        // start all at 0

        //attach the cooling fan pin to the pwm channel and init OFF
        ledcAttachPin(cooling_fan_1, PWM_CHANNEL_1); ledcWrite(PWM_CHANNEL_1, 0);
        ledcAttachPin(cooling_fan_2, PWM_CHANNEL_2); ledcWrite(PWM_CHANNEL_2, 0);

        //attach the extracting fans to the pwm channel and init OFF
        ledcAttachPin(extract_fan_1, PWM_CHANNEL_3); ledcWrite(PWM_CHANNEL_3, 0);
        ledcAttachPin(extract_fan_2, PWM_CHANNEL_4); ledcWrite(PWM_CHANNEL_4, 0);
        ledcAttachPin(extract_fan_3, PWM_CHANNEL_5); ledcWrite(PWM_CHANNEL_5, 0);
        ledcAttachPin(extract_fan_4, PWM_CHANNEL_6); ledcWrite(PWM_CHANNEL_6, 0);

}


// // run them successively: spool up to max, then spool up next, after the 6th spool down successively

void fan_speed_test(int how_many_fans, uint64_t rising_for, uint64_t falling_for) {
  if (how_many_fans != 6) {
    // Single random fan test
    uint32_t which_fan = random(1, 7);  // random(1,7) gives 1-6, 
    uint32_t that_pwm_channel = which_fan - 1;  // Map fan 1-6 to channel 0-5
    
    Serial.printf("Testing single fan %d on channel %d\n", which_fan, that_pwm_channel);
    
    ramp_up_fan(that_pwm_channel, (0.1 * max_duty), rising_for);
    delay(hold_time);
    ramp_down_fan(that_pwm_channel, (0.1 * max_duty), 0, falling_for);
    
  } else {
    // All 6 fans, successively
    int channels[6] = {PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, 
                       PWM_CHANNEL_4, PWM_CHANNEL_5, PWM_CHANNEL_6};
    
    // Ramp UP each fan sequentially
    for (int i = 0; i < 6; i++) {
      Serial.printf("Ramping up fan %d (channel %d)\n", i + 1, channels[i]);
      ramp_up_fan(channels[i], (0.1 * max_duty), rising_for);
      delay(hold_time);
    }
    
    // Now ramp DOWN each fan sequentially
    for (int i = 0; i < 6; i++) {
      Serial.printf("Ramping down fan %d (channel %d)\n", i + 1, channels[i]);
      ramp_down_fan(channels[i], (0.1 * max_duty), 0, falling_for);
      delay(hold_time);
    }
  }
}

void ramp_up_fan(const int any_channel, uint32_t duty_limit, uint64_t total_time_ms){
    const uint16_t steps = 200;
    uint32_t step_delay = total_time_ms / steps;
    
    for (uint16_t i = 0; i <= steps; i++) {
      float progress = (float)i / (float)steps;  // 0.0 to 1.0
      float exponential = pow(progress, 3.0);   // Slow start, fast end
      uint32_t duty = (uint32_t)(exponential * duty_limit);
      ledcWrite(any_channel, duty);
      delay(step_delay);
    }
}

void ramp_down_fan(const int any_channel, uint32_t start_duty, uint32_t final_stop_duty, uint64_t total_time_ms){
    const uint16_t steps = 200;
    uint32_t step_delay = total_time_ms / steps;

    for (uint16_t i = 0; i <= steps; i++) {
        float progress = (float)i / (float)steps;
        float exponential = pow(progress, 3.0);

        uint32_t duty = start_duty - (uint32_t)((start_duty - final_stop_duty) * exponential);

        ledcWrite(any_channel, duty);
        delay(step_delay);
    }
}
bool json_bound_successfully = false;
char serial_send_log[512] = "JSON Not Bound Yet!";
//sender
void uart_data_handler(void *pvParameters) {
    while (true) {
        // Serialize all data into a JSON string
        // bearer for sensor data, fan states, buzzer, time, events etc.
        //send this JSON string over Serial2 to the receiver
        // get a delivery confirmation or response from the receiver (optional)
        // Process the response if needed (optional)
         json_bound_successfully = bind_dynamic_data_into_json();
         if(!json_bound_successfully){
                snprintf(serial_send_log, sizeof(serial_send_log), "[UART] JSON Binding Failed! Skipping this transmission.");
            }
            else{
                snprintf(serial_send_log, sizeof(serial_send_log), "[UART] JSON Binding Successful. Sending data to receiver...");
                Serial2.println(sendable_to_cloud_db); // Send the JSON string to the receiver
         }

         LOG(serial_send_log);

        vTaskDelay(pdMS_TO_TICKS(uart_send_interval_ms)); // Adjust delay as needed
    }
}

//to the receiver
/*

void uart_data_handler(void *pvParameters) {
    while (true) {
        // Check for incoming data from Serial2
        if (Serial2.available() > 0) {
            String command = Serial2.readStringUntil('\n');
            command.trim();
            Serial.print("Received command: ");
            Serial.println(command);
            // Process the command as needed
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}
*/


bool bind_dynamic_data_into_json() {

    bool bound_successfully = false;

    JSON_sendable.clear();
   // save_counter++; // save this into EEPROM or SPIFFS

    // ================= SYSTEM RUNTIME DATA =================
    JsonObject System = JSON_sendable["System"].to<JsonObject>();
    System["Uptime"] = (now_now_ms / 1000);
    System["Date"] = sawa.SystemDate;
    System["Time"] = sawa.SystemTime;
    System["Voltage"] = voltage;

     // ================= DRYER METRICS =================
    JsonObject Dryer = JSON_sendable["Dryer"].to<JsonObject>();
    Dryer["Temperature"] = aggregate_sensor_temperatures();
    Dryer["Humidity"] = aggregate_sensor_humidities();
    Dryer["Temperature_Range"] = highest_temperature - lowest_temperature;
    Dryer["Humidity_Range"] = highest_humidity - lowest_humidity;
    Dryer["Active_Sensors"] = active_sensors;
    Dryer["Fan_control_Mode"] = isNightTime() ? "Night Mode" : "Day Mode";
    Dryer["Rain_Detected"] = check_weather_for_rain() ? "Yes" : "No";
    Dryer["Dryer_to_env_Temp_Diff"] = aggregate_sensor_temperatures() - sensor_12_temp; // sensor 12 is the outdoor sensor, so this is the temp difference between dryer and outdoor, which can be a useful metric for controlling fans and understanding drying conditions
    Dryer["Dryer_to_env_Humi_Diff"] = aggregate_sensor_humidities() - sensor_12_humidity; // same for humidity
    Dryer["Gradient_Temp_cooling"] = temperature_gradient_due_to_cooling_fans; // rate of change of temperature, which can indicate how quickly the dryer is drying and whether fans need to be adjusted
    Dryer["Gradient_Temp_extract"] = temperature_gradient_due_to_extract_fans; // rate of change of temperature, which can indicate how quickly the dryer is drying and whether fans need to be adjusted
    Dryer["Gradient_Humi_extract"] = humidity_gradient_due_to_extract_fans; // same for humidity
    Dryer["Gradient_Humi_cooling"] = humidity_gradient_due_to_cooling_fans; // how much the humidity has dropped since we turned on the cooling fans, which can indicate whether they are helping with drying or if they are making it worse by adding humidity

    // ================= FAN STATES =================
    JsonObject Fans = JSON_sendable["Fans"].to<JsonObject>();
    Fans["Fan_1"]   = cooling_fan_1_on? "ON" : "OFF";
    Fans["Fan_2"]   = cooling_fan_2_on? "ON" : "OFF";
    Fans["Fan_3"]   = extract_fan_1_on? "ON" : "OFF";
    Fans["Fan_4"]   = extract_fan_2_on? "ON" : "OFF";
    Fans["Fan_5"]   = extract_fan_3_on? "ON" : "OFF";
    Fans["Fan_6"]   = extract_fan_4_on? "ON" : "OFF";

    if(cooling_fan_1_on){
        Fans["Fan_1_Turn_On_Time"] = cooling_fan_1_time_on_str;
        Fans["Fan_1_Turn_Off_Time"] = cooling_fan_1_time_off_str;
      //  Fans["Fan_1_Turn_On_MS"] = cooling_fan_1_on_time_ms;
        Fans["Fan_1_Total_Run_Time"] = cooling_fan_1_duration/1000ULL; // in seconds
    }
    if(cooling_fan_2_on){
        Fans["Fan_2_Turn_On_Time"] = cooling_fan_2_time_on_str;
        Fans["Fan_2_Turn_Off_Time"] = cooling_fan_2_time_off_str;
      //  Fans["Fan_2_Turn_On_MS"] = cooling_fan_2_on_time_ms;
        Fans["Fan_2_Total_Run_Time"] = cooling_fan_2_duration/1000ULL; // in seconds
    }
    if(extract_fan_1_on){
        Fans["Fan_3_Turn_On_Time"] = extract_fan_1_time_on_str;
        Fans["Fan_3_Turn_Off_Time"] = extract_fan_1_time_off_str;
        //Fans["Fan_3_Turn_On_MS"] = extract_fan_1_on_time_ms;
        Fans["Fan_3_Total_Run_Time_secs"] = extract_fan_1_duration/1000ULL; // in seconds
        
            
    }
    if(extract_fan_2_on){
        Fans["Fan_4_Turn_On_Time"] = extract_fan_2_time_on_str;
        Fans["Fan_4_Turn_Off_Time"] = extract_fan_2_time_off_str;
        //Fans["Fan_4_Turn_On_MS"] = extract_fan_2_on_time_ms;
        Fans["Fan_4_Total_Run_Time"] = extract_fan_2_duration/1000ULL; // in seconds
    }
    if(extract_fan_3_on){
        Fans["Fan_5_Turn_On_Time"] = extract_fan_3_time_on_str;
        Fans["Fan_5_Turn_Off_Time"] = extract_fan_3_time_off_str;
        //Fans["Fan_5_Turn_On_MS"] = extract_fan_3_on_time_ms;
        Fans["Fan_5_Total_Run_Time"] = extract_fan_3_duration/1000ULL; // in seconds
    }
    if(extract_fan_4_on){
        Fans["Fan_6_Turn_On_Time"] = extract_fan_4_time_on_str;
        Fans["Fan_6_Turn_Off_Time"] = extract_fan_4_time_off_str;
        //Fans["Fan_6_Turn_On_MS"] = extract_fan_4_on_time_ms;
        Fans["Fan_6_Total_Run_Time"] = extract_fan_4_duration/1000ULL; // in seconds
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

                // ================= SERIALIZATION =================

            size_t len_cloud = serializeJson(JSON_sendable, sendable_to_cloud_db);

            if (len_cloud == 0) {
                snprintf(serialization_log,
                        sizeof(serialization_log),
                        "[JSON] Serialization failed!");
                LOG(serialization_log);
                return false;
            }

            // Optional: detect truncation (ArduinoJson does NOT null-terminate if truncated)
            if (len_cloud >= sizeof(sendable_to_cloud_db)){
            
                snprintf(serialization_log,   sizeof(serialization_log),  "[JSON] WARNING: Buffer may be too small (truncation detected)");
                LOG(serialization_log);
            }

            bound_successfully = true;

            snprintf(serialization_log, sizeof(serialization_log),     "[JSON] Serialized OK | Cloud: %u bytes",   (unsigned int)len_cloud);

            LOG(serialization_log);


    return bound_successfully;
}



bool is_buzzing = false;

// Monitor task for checking buzzer status
void MonitorTask(void *pvParams) {
  is_buzzing = buzzer.isBuzzing();
  while (true) {
    if (is_buzzing) {
      Serial.println("Buzzer is active");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}



uint64_t lastStats = 0;

void loop() {
  // put your main code here, to run repeatedly:
  now_now_ms = esp_timer_get_time() / 1000ULL; // current time in ms since boot, for timestamping packets and calculating "last seen" times

  // Optional: Print statistics periodically
    static unsigned long lastStats = 0;
    if (now_now_ms - lastStats > 10000) {  // Every 10 seconds
        check_time();
        regulate_fans();
        packetHandler.printStatistics();
        lastStats = now_now_ms;
    }
}


void check_time(){
  if (sawa.isClockWorking()) {
     sawa.update(); // Update time
    
    // Check for 5-minute markers
    if (sawa.is5MinuteMarker()) {
        Serial.print("5-minute marker at ");
        Serial.println(sawa.getTime24Hour());
        // Do something every 5 minutes
    }
    
    // Check for 10-minute markers
    if (sawa.is10MinuteMarker()) {
        Serial.print("10-minute marker at ");
        Serial.println(sawa.getTime24Hour());
        // Do something every 10 minutes
    }
    
    // Check for hourly markers
    if (sawa.isHourlyMarker()) {
        Serial.println("Hourly task running");
        // Do something every hour
    }
    
    // Original new minute check still works
    if (sawa.isNewMinute()) {
        Serial.print("New minute: ");
        Serial.println(sawa.getTime12Hour());
    }
  }
}


uint8_t active_sensors = 0;
float highest_temperature = -100.0f; // initialize to a very low value to ensure any valid reading will be higher
float lowest_temperature = 200.0f; // initialize to a very high value to ensure

float lowest_humidity = 200.0f; // initialize to a very high value to ensure any valid reading will be lower
float highest_humidity = -100.0f; // initialize to a very low value to ensure any valid reading will be higher

float temperature_range = 0.0f; // to track the range of temperatures across the sensors, to detect if there is a large discrepancy that might indicate a malfunctioning sensor
float humidity_range = 0.0f; // same for humidity

float temperature_deviation = 0.0f; // to track how much the current temperature deviates from the average of the sensors, to detect if one sensor is giving a wildly different reading that might indicate a malfunction
float humidity_deviation = 0.0f; // same for humidity

float average_temperature = 0.0f; // to track the average temperature across all sensors, for comparison against individual sensor readings
float average_humidity = 0.0f; // same for humidity

float drying_temperature = 0.0f;
float drying_humidity = 0.0f;
bool is_night_time = false; // This can be determined based on time or very low temperature
bool is_raining = false; // This can be determined based on very high humidity or external weather data

const int rapid_ramp = 2; // 2 seconds for a rapid ramp
const int slow_ramp = 5; // 5 seconds for a slow ramp

float very_high_temp_threshold = 60.0f; // either over drying, or too hot a day
float mild_high_temp_threshold = 50.0f; 
float low_temp_threshold = 35.0f;  // Example threshold for low temperature
const float very_low_temp_threshold = 15.0f; // Example threshold for very low temperature (e.g., night time)

const float very_high_humidity_threshold = 70.0f; // either it is raining, or air over crops too wet
const float mild_high_humidity_threshold = 60.0f;  // 
const float high_humidity_threshold = 50.0f; // Example threshold for high humidity
const float low_humidity_threshold = 30.0f;  // Example threshold for low humidity
const float very_low_humidity_threshold = 15.0f; // 

bool extract_fan_1_engaged = false, extract_fan_2_engaged = false, extract_fan_3_engaged = false, extract_fan_4_engaged = false; // to track if we have already turned on the extract fans based on humidity thresholds, so that we can turn them off when humidity drops
bool cooling_fans_running = false; // to track if cooling fans are already running based on temperature thresholds, so that we can turn them off when temperature drops 
bool extract_fans_running = false; // to track if extract fans are already running based on humidity thresholds, so that we can turn them off when humidity drops
uint32_t current_cooling_fan_speed = 0; // to track current speed of fans for smooth ramping
uint32_t current_extract_fan_speed = 0;


char all_fans_log[200]; // to log the actions taken on fans for debugging and monitoring purposes

// control fans based on temp and humi, we shall handle a 3degree and 5% hysteresis later
void regulate_fans(){  // Fan control logic based on temperature and humidity thresholds
    drying_temperature = aggregate_sensor_temperatures(); // from the 11 internal sensors
    drying_humidity = average_humidity; // same, from the 11 internal sensors
    // drying_humidity = aggregate_sensor_humidities(); 


    // assuming fans are already running that this section runs every time regulate_fans is called...
    if(cooling_fans_running){ //  cooling_fan_1_on
        cooling_fan_1_duration = (now_now_ms - cooling_fan_1_on_time_ms);
        cooling_fan_2_duration = (now_now_ms - cooling_fan_2_on_time_ms);
        temperature_gradient_due_to_cooling_fans = temp_when_cooling_fans_turned_on - drying_temperature;
    }

    if(extract_fans_running){
          extract_fan_1_duration  = (now_now_ms - extract_fan_1_on_time_ms);
          extract_fan_2_duration  = (now_now_ms - extract_fan_2_on_time_ms);
          extract_fan_3_duration  = (now_now_ms - extract_fan_3_on_time_ms);
          extract_fan_4_duration  = (now_now_ms - extract_fan_4_on_time_ms);
        humidity_gradient_due_to_extract_fans = humi_when_extract_fans_turned_on - drying_humidity;
    }

   

    

    if(drying_temperature <= 0.0 || drying_temperature >= 100.0 || drying_humidity <= 0.0f || isnan(drying_temperature) || isnan(drying_humidity)){
        drying_temperature = 0.0f; // casr NAN to 0, JSON.send(NaN) or LCD.print(NAN) might be weird
        drying_humidity = 0.0f;
        // if we don't have valid readings, play it safe and turn off all fans
        for(int i = 1; i <= 6; i++){ // turn off all 6 fans
            if(current_cooling_fan_speed == current_extract_fan_speed){
            ramp_down_fan(i-1, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
            delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
            }
            else {
                if(i <= 2){ // cooling fans
                    ramp_down_fan(i-1, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
                    delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
                }
                else{ // extract fans
                    ramp_down_fan(i-1, current_extract_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
                    delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
                }
            }
        }
         cooling_fan_1_on = false; cooling_fan_2_on = false;
         extract_fan_1_on = false; extract_fan_2_on = false; extract_fan_3_on = false; extract_fan_4_on = false;
         cooling_fan_1_duration = (now_now_ms - cooling_fan_1_on_time_ms); cooling_fan_2_duration = (now_now_ms - cooling_fan_2_on_time_ms);
         extract_fan_1_duration = (now_now_ms - extract_fan_1_on_time_ms); extract_fan_2_duration = (now_now_ms - extract_fan_2_on_time_ms); 
         extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms); extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);    
        
         snprintf(cooling_fan_1_time_off_str, sizeof(cooling_fan_1_time_off_str), "%s", sawa.SystemTime);
         snprintf(cooling_fan_2_time_off_str, sizeof(cooling_fan_2_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_1_time_off_str, sizeof(extract_fan_1_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_2_time_off_str, sizeof(extract_fan_2_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);

         cooling_fan_1_on_time_ms = 0; cooling_fan_2_on_time_ms = 0;
         extract_fan_1_on_time_ms = 0; extract_fan_2_on_time_ms = 0; extract_fan_3_on_time_ms = 0; extract_fan_4_on_time_ms = 0;
         snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF all Fans due to invalid sensor readings (Temp: %.2f C, Humidity: %.2f %)", drying_temperature, drying_humidity);
         Serial.println(all_fans_log);
         return;
    }



    // < 15deg and > 100% humidity: turn off all fans, it is probably night time or raining, we can check the time from RTC to confirm if it's night time, but let's just use the temp and humi for now for simplicity
    if((drying_temperature < very_low_temp_threshold) && (drying_humidity > very_high_humidity_threshold))  { // either it is night or raining
        // turn off all fans, it is probably night time or raining
        for(int i = 1; i <= 6; i++){ // turn off all 6 fans
           if(current_cooling_fan_speed == current_extract_fan_speed){
            ramp_down_fan(i-1, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
            delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
            }
            else {
                if(i <= 2){ // cooling fans
                    ramp_down_fan(i-1, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
                    delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
                }
                else{ // extract fans
                    ramp_down_fan(i-1, current_extract_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
                    delay(500); // stagger the shutdowns by 0.5 seconds to avoid power surges
                }
            }
        }
         cooling_fan_1_on = false; cooling_fan_2_on = false;
         extract_fan_1_on = false; extract_fan_2_on = false; extract_fan_3_on = false; extract_fan_4_on = false;
        
         cooling_fan_1_duration = (now_now_ms - cooling_fan_1_on_time_ms); cooling_fan_2_duration = (now_now_ms - cooling_fan_2_on_time_ms);
         extract_fan_1_duration = (now_now_ms - extract_fan_1_on_time_ms); extract_fan_2_duration = (now_now_ms - extract_fan_2_on_time_ms); 
         extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms); extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);    
        
         snprintf(cooling_fan_1_time_off_str, sizeof(cooling_fan_1_time_off_str), "%s", sawa.SystemTime);
         snprintf(cooling_fan_2_time_off_str, sizeof(cooling_fan_2_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_1_time_off_str, sizeof(extract_fan_1_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_2_time_off_str, sizeof(extract_fan_2_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
         snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);
         
         cooling_fan_1_on_time_ms = 0; cooling_fan_2_on_time_ms = 0;
         extract_fan_1_on_time_ms = 0; extract_fan_2_on_time_ms = 0; extract_fan_3_on_time_ms = 0; extract_fan_4_on_time_ms = 0;
         snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF all Fans due to very low temperature (%.2f C) and very high humidity (%.2f %)", drying_temperature, drying_humidity);

         Serial.println(all_fans_log);
    return;
        }

    // > 60 degrees: full blast, 50-60: medium, < 35: off, 35-50: cooling off 
    if(drying_temperature > very_high_temp_threshold){

    uint32_t target_speed = max_duty;

    if(!cooling_fans_running){
        ramp_up_fan(PWM_CHANNEL_1, target_speed, rapid_ramp);
        ramp_up_fan(PWM_CHANNEL_2, target_speed, rapid_ramp);

        cooling_fans_running = true;

        temp_when_cooling_fans_turned_on = drying_temperature; // set the baseline temperature when we turned on the fans, to compare against for deciding when to turn them off based on gradient
        cooling_fan_1_on_time_ms = now_now_ms; snprintf(cooling_fan_1_time_on_str, sizeof(cooling_fan_1_time_on_str), "%s", sawa.SystemTime);
        cooling_fan_2_on_time_ms = now_now_ms; snprintf(cooling_fan_2_time_on_str, sizeof(cooling_fan_2_time_on_str), "%s", sawa.SystemTime);

    }
    else if(current_cooling_fan_speed != target_speed){
        ramp_up_fan(PWM_CHANNEL_1, target_speed, rapid_ramp);
        ramp_up_fan(PWM_CHANNEL_2, target_speed, rapid_ramp);
    }

    current_cooling_fan_speed = target_speed;

    cooling_fan_1_on = true;   cooling_fan_2_on = true;

    snprintf(all_fans_log, sizeof(all_fans_log), "Turning ON Cooling Fans at FULL speed due to very high temperature (%.2f C)", drying_temperature);
    Serial.println(all_fans_log);
    
}
    
    // 50 - 60 is moderately hot, so run fans at medium speed
    else if((drying_temperature <= very_high_temp_threshold) && (drying_temperature >= mild_high_temp_threshold)){

    uint32_t target_speed = (0.5 * max_duty);

    if(!cooling_fans_running){
        ramp_up_fan(PWM_CHANNEL_1, target_speed, slow_ramp);
        ramp_up_fan(PWM_CHANNEL_2, target_speed, slow_ramp);

        cooling_fans_running = true;

        temp_when_cooling_fans_turned_on = drying_temperature; // set the baseline temperature when we turned on the fans, to compare against for deciding when to turn them off based on gradient
        cooling_fan_1_on_time_ms = now_now_ms; snprintf(cooling_fan_1_time_on_str, sizeof(cooling_fan_1_time_on_str), "%s", sawa.SystemTime);
        cooling_fan_2_on_time_ms = now_now_ms; snprintf(cooling_fan_2_time_on_str, sizeof(cooling_fan_2_time_on_str), "%s", sawa.SystemTime);

    }
    else if(current_cooling_fan_speed != target_speed){

        if(current_cooling_fan_speed > target_speed){
            ramp_down_fan(PWM_CHANNEL_1, current_cooling_fan_speed, target_speed, slow_ramp);
            ramp_down_fan(PWM_CHANNEL_2, current_cooling_fan_speed, target_speed, slow_ramp);
        }
        else{
            ramp_up_fan(PWM_CHANNEL_1, target_speed, slow_ramp);
            ramp_up_fan(PWM_CHANNEL_2, target_speed, slow_ramp);
        }
    }

    current_cooling_fan_speed = target_speed;

    cooling_fan_1_on = true;
    cooling_fan_2_on = true;
    snprintf(all_fans_log, sizeof(all_fans_log), "Turning ON Cooling Fans at MEDIUM speed due to moderately high temperature (%.2f C)", drying_temperature);
    Serial.println(all_fans_log);
}
   
   // 35 - 50 is moderately cool, slowly turn off cooling fans if they are on
    else if((drying_temperature < mild_high_temp_threshold) && (drying_temperature >= low_temp_threshold)) { // moderately cool - turn off cooling fans if they are on
        if(cooling_fan_1_on){
            ramp_down_fan(PWM_CHANNEL_1, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
            cooling_fan_1_on = false;
           
            cooling_fan_1_duration = (now_now_ms - cooling_fan_1_on_time_ms); 
             cooling_fan_1_on_time_ms = 0;
           
            snprintf(cooling_fan_1_time_off_str, sizeof(cooling_fan_1_time_off_str), "%s", sawa.SystemTime);
            snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Cooling Fan 1 due to moderately cool temperature (%.2f C)", drying_temperature);  
            Serial.println(all_fans_log);
        }

        if(cooling_fan_2_on){
            ramp_down_fan(PWM_CHANNEL_2, current_cooling_fan_speed, 0, slow_ramp); // slowly ramp down in 5 seconds
            cooling_fan_2_on = false;
            
            cooling_fan_2_duration = (now_now_ms - cooling_fan_2_on_time_ms);
            cooling_fan_2_on_time_ms = 0;
            snprintf(cooling_fan_2_time_off_str, sizeof(cooling_fan_2_time_off_str), "%s", sawa.SystemTime);
            snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Cooling Fan 2 due to moderately cool temperature (%.2f C)", drying_temperature);  
            Serial.println(all_fans_log);
        }
 
        current_cooling_fan_speed = 0; // we will ramp down to 0
        cooling_fans_running  = false;
    }
    
    // < 35 degrees: turn off all fans... it is probably night time
   else{

    if(cooling_fans_running){

        ramp_down_fan(PWM_CHANNEL_1, current_cooling_fan_speed, 0, slow_ramp);
        ramp_down_fan(PWM_CHANNEL_2, current_cooling_fan_speed, 0, slow_ramp);

        current_cooling_fan_speed = 0;
        cooling_fans_running = false;

        cooling_fan_1_on = false;     cooling_fan_2_on = false;

        cooling_fan_1_duration = (now_now_ms - cooling_fan_1_on_time_ms);
        cooling_fan_2_duration = (now_now_ms - cooling_fan_2_on_time_ms);
        cooling_fan_1_on_time_ms = 0;     cooling_fan_2_on_time_ms = 0;


        snprintf(cooling_fan_1_time_off_str, sizeof(cooling_fan_1_time_off_str), "%s", sawa.SystemTime);
        snprintf(cooling_fan_2_time_off_str, sizeof(cooling_fan_2_time_off_str), "%s", sawa.SystemTime);
        snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Cooling Fans due to low temperature (%.2f C)", drying_temperature);
        Serial.println(all_fans_log);
    }
}
    // AND NOW FOR THE EXTRACT FANS BASED ON HUMIDITY - we want to keep humidity around 40-50% for optimal drying, but if it goes above 70%, we want to rapidly extract the humid air, and if it goes below 30%, we want to slow down extraction to avoid over-drying the crops. So we will have 4 extract fans that can be turned on at different thresholds of humidity, and they will ramp up and down smoothly based on the current humidity level.
    // these ones turn on one at a time, based on the thresholds, because rapid extraction also cools down the dryer
    //  > 70% humidity: increase extraction, < 30% humidity: decrease extraction
    // < 30%
   if(drying_humidity < low_humidity_threshold){

    if(extract_fans_running){

        for(int i = 3; i <= 6; i++){
            ramp_down_fan(i-1, current_extract_fan_speed, 0, slow_ramp);
            delay(500);
        }

        current_extract_fan_speed = 0;
        extract_fans_running = false;

                extract_fan_1_duration = (now_now_ms - extract_fan_1_on_time_ms);
                extract_fan_2_duration = (now_now_ms - extract_fan_2_on_time_ms);
                extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms);
                extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);

            
                extract_fan_1_on_time_ms = 0;               extract_fan_2_on_time_ms = 0;
                extract_fan_3_on_time_ms = 0;               extract_fan_4_on_time_ms = 0;

                snprintf(extract_fan_1_time_off_str, sizeof(extract_fan_1_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_2_time_off_str, sizeof(extract_fan_2_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);
                snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF all Extract Fans due to low humidity (%.2f %)", drying_humidity);
                Serial.println(all_fans_log);

    }

        extract_fan_1_on = false;
        extract_fan_2_on = false;
        extract_fan_3_on = false;
        extract_fan_4_on = false;

        Serial.println("Turning OFF all Extract Fans");
}
  
    // 30 - 50% is moderately humid,  we are somehow within desired range
    //if all off, turn on 2 at medium speed, and 2 at full,
    // if all alredy on,  slow them down to medium speed, and keep the rest at max throttle
  else if((drying_humidity >= low_humidity_threshold) && (drying_humidity < high_humidity_threshold)){

    uint32_t target_speed = (0.5 * max_duty);

    if(!extract_fans_running){

        ramp_up_fan(PWM_CHANNEL_3, target_speed, slow_ramp);
        ramp_up_fan(PWM_CHANNEL_4, target_speed, slow_ramp);

        extract_fan_1_on = true;
        extract_fan_2_on = true;

        extract_fans_running = true;
        extract_fan_1_on_time_ms = now_now_ms; snprintf(extract_fan_1_time_on_str, sizeof(extract_fan_1_time_on_str), "%s", sawa.SystemTime);
        extract_fan_2_on_time_ms = now_now_ms; snprintf(extract_fan_2_time_on_str, sizeof(extract_fan_2_time_on_str), "%s", sawa.SystemTime);


        snprintf(all_fans_log, sizeof(all_fans_log), "Turning ON Extract Fans 1 and 2 at MEDIUM speed due to moderately low humidity (%.2f %)", drying_humidity);
        Serial.println(all_fans_log);

    }
    else if(extract_fans_running){

         if(current_extract_fan_speed != target_speed){ // ON BUT WERE RUNNING AT FULL BLAST, SLOW THEM DOWN TO MEDIUM

            ramp_down_fan(PWM_CHANNEL_3, current_extract_fan_speed, target_speed, slow_ramp);
            ramp_down_fan(PWM_CHANNEL_4, current_extract_fan_speed, target_speed, slow_ramp);
            snprintf(all_fans_log, sizeof(all_fans_log), "Slowing down Extract Fans 1 and 2 to MEDIUM speed due to moderately low humidity (%.2f %)", drying_humidity);
            Serial.println(all_fans_log);
         }

         if(extract_fan_3_on){ // if fan 3 was on, slow it down to zero
            ramp_down_fan(PWM_CHANNEL_5, current_extract_fan_speed, 0, slow_ramp);
            extract_fan_3_on = false;
            extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms);
            extract_fan_3_on_time_ms = 0;
            snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
            snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Extract Fan 3 due to moderately low humidity (%.2f %)", drying_humidity);
            Serial.println(all_fans_log);
         }

         if(extract_fan_4_on){ // if fan 4 was on, slow it down to zero
            ramp_down_fan(PWM_CHANNEL_6, current_extract_fan_speed, 0, slow_ramp);
            extract_fan_4_on = false;
            extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);
            extract_fan_4_on_time_ms = 0;
            snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);
            snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Extract Fan 4 due to moderately low humidity (%.2f %)", drying_humidity);
            Serial.println(all_fans_log);
         }
    }

    current_extract_fan_speed = target_speed; // for the 2 fans ON
    

}
    
// if it is not night time and it is not raining
    
    // 60 - 70% is very humid, turn on all extract fans at full blast
   // 60–70% humidity (very humid but manageable)
    else if((drying_humidity >= high_humidity_threshold) &&    (drying_humidity < very_high_humidity_threshold)){

            is_night_time = isNightTime();
            is_raining = check_weather_for_rain();

            // If night or rain → shut extraction down
            if(is_night_time || is_raining){

                if(extract_fans_running){

                    for(int i = 3; i <= 6; i++){
                        ramp_down_fan(i-1, current_extract_fan_speed, 0, slow_ramp);
                        delay(500);
                    }

                    current_extract_fan_speed = 0;
                    extract_fans_running = false;
                }

                extract_fan_1_engaged = false;
                extract_fan_2_engaged = false;
                extract_fan_3_engaged = false;
                extract_fan_4_engaged = false;

                extract_fan_1_on = false;
                extract_fan_2_on = false;
                extract_fan_3_on = false;
                extract_fan_4_on = false;

                extract_fan_1_duration = (now_now_ms - extract_fan_1_on_time_ms);
                extract_fan_2_duration = (now_now_ms - extract_fan_2_on_time_ms);
                extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms);
                extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);

                extract_fan_1_on_time_ms = 0;
                extract_fan_2_on_time_ms = 0;
                extract_fan_3_on_time_ms = 0;
                extract_fan_4_on_time_ms = 0;

                snprintf(extract_fan_1_time_off_str, sizeof(extract_fan_1_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_2_time_off_str, sizeof(extract_fan_2_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);
                snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Extract, Humidity is High (%.2f %), but it is night time or raining", drying_humidity);
                Serial.println(all_fans_log);

                return;
            }

            // Daytime high humidity → full extraction
            uint32_t target_speed = max_duty;

            if(!extract_fans_running){

                for(int i = 3; i <= 6; i++){
                    ramp_up_fan(i-1, target_speed, slow_ramp);
                    delay(300);
                }

                extract_fans_running = true;
                
                extract_fan_1_on_time_ms = now_now_ms; snprintf(extract_fan_1_time_on_str, sizeof(extract_fan_1_time_on_str), "%s", sawa.SystemTime);
                extract_fan_2_on_time_ms = now_now_ms; snprintf(extract_fan_2_time_on_str, sizeof(extract_fan_2_time_on_str), "%s", sawa.SystemTime);
                extract_fan_3_on_time_ms = now_now_ms; snprintf(extract_fan_3_time_on_str, sizeof(extract_fan_3_time_on_str), "%s", sawa.SystemTime);
                extract_fan_4_on_time_ms = now_now_ms; snprintf(extract_fan_4_time_on_str, sizeof(extract_fan_4_time_on_str), "%s", sawa.SystemTime);

            }
            else if((extract_fans_running) && current_extract_fan_speed != target_speed){

                for(int i = 3; i <= 6; i++){
                    ramp_up_fan(i-1, target_speed, slow_ramp);
                    delay(200);
                }
            }

            current_extract_fan_speed = target_speed;

            extract_fan_1_on = true;
            extract_fan_2_on = true;
            extract_fan_3_on = true;
            extract_fan_4_on = true;

            
                snprintf(all_fans_log, sizeof(all_fans_log), "Turning ON Extract Fans at FULL speed due to high humidity (%.2f %)", drying_humidity);
                Serial.println(all_fans_log);
        }


        // > 70% very_high_humidity_threshold (extreme humidity)
    else{ // check that the high humidity is not for night time or rain. 

            is_night_time = isNightTime();
            is_raining = check_weather_for_rain();

            // Extreme humidity but bad outside conditions
            if(is_night_time || is_raining){

                if(extract_fans_running){

                    for(int i = 3; i <= 6; i++){
                        ramp_down_fan(i-1, current_extract_fan_speed, 0, slow_ramp);
                        delay(500);
                    }

                    current_extract_fan_speed = 0;
                    extract_fans_running = false;
                }

                extract_fan_1_engaged = false;
                extract_fan_2_engaged = false;
                extract_fan_3_engaged = false;
                extract_fan_4_engaged = false;

                extract_fan_1_on = false;
                extract_fan_2_on = false;
                extract_fan_3_on = false;
                extract_fan_4_on = false;

                extract_fan_1_duration = (now_now_ms - extract_fan_1_on_time_ms);
                extract_fan_2_duration = (now_now_ms - extract_fan_2_on_time_ms);
                extract_fan_3_duration = (now_now_ms - extract_fan_3_on_time_ms);
                extract_fan_4_duration = (now_now_ms - extract_fan_4_on_time_ms);

            
                extract_fan_1_on_time_ms = 0;               extract_fan_2_on_time_ms = 0;
                extract_fan_3_on_time_ms = 0;               extract_fan_4_on_time_ms = 0;

                snprintf(extract_fan_1_time_off_str, sizeof(extract_fan_1_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_2_time_off_str, sizeof(extract_fan_2_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_3_time_off_str, sizeof(extract_fan_3_time_off_str), "%s", sawa.SystemTime);
                snprintf(extract_fan_4_time_off_str, sizeof(extract_fan_4_time_off_str), "%s", sawa.SystemTime);
                snprintf(all_fans_log, sizeof(all_fans_log), "Turning OFF Extract Fans, Humidity is Extreme (%.2f %), but it is night time or raining", drying_humidity);

                Serial.println(all_fans_log);

                return;
            }

            // Daytime extreme humidity → aggressive extraction
            uint32_t target_speed = max_duty;

            if(!extract_fans_running){

                for(int i = 3; i <= 6; i++){
                    ramp_up_fan(i-1, target_speed, rapid_ramp);
                    delay(300);
                }

                extract_fans_running = true;

                extract_fan_1_on_time_ms = now_now_ms; snprintf(extract_fan_1_time_on_str, sizeof(extract_fan_1_time_on_str), "%s", sawa.SystemTime);
                extract_fan_2_on_time_ms = now_now_ms; snprintf(extract_fan_2_time_on_str, sizeof(extract_fan_2_time_on_str), "%s", sawa.SystemTime);
                extract_fan_3_on_time_ms = now_now_ms; snprintf(extract_fan_3_time_on_str, sizeof(extract_fan_3_time_on_str), "%s", sawa.SystemTime);
                extract_fan_4_on_time_ms = now_now_ms; snprintf(extract_fan_4_time_on_str, sizeof(extract_fan_4_time_on_str), "%s", sawa.SystemTime);

                snprintf(all_fans_log, sizeof(all_fans_log), "Turning ON Extract Fans at FULL speed due to extreme humidity (%.2f %)", drying_humidity);
                Serial.println(all_fans_log);
            }
            else if(current_extract_fan_speed != target_speed){

                for(int i = 3; i <= 6; i++){
                    ramp_up_fan(i-1, target_speed, rapid_ramp);
                    delay(200);
                }

                extract_fan_1_on_time_ms = now_now_ms; snprintf(extract_fan_1_time_on_str, sizeof(extract_fan_1_time_on_str), "%s", sawa.SystemTime);
                extract_fan_2_on_time_ms = now_now_ms; snprintf(extract_fan_2_time_on_str, sizeof(extract_fan_2_time_on_str), "%s", sawa.SystemTime);
                extract_fan_3_on_time_ms = now_now_ms; snprintf(extract_fan_3_time_on_str, sizeof(extract_fan_3_time_on_str), "%s", sawa.SystemTime);
                extract_fan_4_on_time_ms = now_now_ms; snprintf(extract_fan_4_time_on_str, sizeof(extract_fan_4_time_on_str), "%s", sawa.SystemTime);

                snprintf(all_fans_log, sizeof(all_fans_log), "Setting Extract Fans to FULL speed due to extreme humidity (%.2f %)", drying_humidity);
                Serial.println(all_fans_log);
            
            }

            current_extract_fan_speed = target_speed;

            extract_fan_1_on = true;
            extract_fan_2_on = true;
            extract_fan_3_on = true;
            extract_fan_4_on = true;

            Serial.println("Extreme humidity daytime → Aggressive extraction");
        }

}


bool isNightTime(){ // sets to night mode if it is between 8 PM and 6 AM, otherwise day mode
    int current_hour = sawa.getCurrentHour();
    return (current_hour >= 20 || current_hour < 6);
}

bool check_time_of_day(){ 
    // Placeholder: Replace with actual logic to determine if it's night time based on RTC time
    int current_hour = sawa.getCurrentHour(); // Assuming this returns the current hour in 24-hour format
    return (current_hour >= 20 || current_hour < 6); // Example: Night time from 8 PM to 6 AM
}

/*
High humidity AND strong solar gain → aggressive extraction
High humidity but weak solar gain → moderate extraction
High humidity + no solar gain → extraction off
*/

float estimate_solar_gain(){

    float outside_temp = (!isnan(sensor_12_temp) && sensor_12_temp > 1.0f);
    float inside_temp  = aggregate_sensor_temperatures();
    float solar_gain = inside_temp - outside_temp; // assumption is indryer temp is always higher than outside temp when sun is out, and the bigger the difference, the stronger the sun. This is a simplification, but it can give us a rough estimate of solar gain without needing a dedicated solar sensor. We can then use this estimate to adjust our fan speeds accordingly, for example by increasing extraction when solar gain is high to prevent overheating and reduce drying times.

    /*
        15°C | Strong sun heating dryer | High extraction |
        8–15°C | Moderate sun | Moderate extraction |
        3–8°C | Weak heating | Reduce extraction |
        <3°C | No sun / clouds | Minimal extraction |
    */
    return solar_gain;

}

bool check_weather_for_rain(){
 // replace with the actual logic from the solar radiation sensor or an external API to determine if it is currently raining
    return false; // Placeholder: Assume it's not raining for now
}

void run_fan(const int channel, const int pwm_channel, const float duty_cycle){
    ramp_up_fan(pwm_channel, duty_cycle * max_duty, rise_time_secs);
    uint32_t duty = (uint32_t)(duty_cycle * max_duty);
    ledcWrite(pwm_channel, duty);
}



float aggregate_sensor_temperatures() {

    active_sensors = 0;

    highest_temperature = -100.0f;
    lowest_temperature  = 200.0f;

    highest_humidity = -100.0f;
    lowest_humidity  = 200.0f;

    float total_temp = 0.0f;
    float total_humi = 0.0f;

    float temperature_readings[11] = {
        sensor_1_temp, sensor_2_temp, sensor_3_temp, sensor_4_temp,
        sensor_5_temp, sensor_6_temp, sensor_7_temp, sensor_8_temp,
        sensor_9_temp, sensor_10_temp, sensor_11_temp
    };

    float humidity_readings[11] = {
        sensor_1_humidity, sensor_2_humidity, sensor_3_humidity, sensor_4_humidity,
        sensor_5_humidity, sensor_6_humidity, sensor_7_humidity, sensor_8_humidity,
        sensor_9_humidity, sensor_10_humidity, sensor_11_humidity
    };

    bool valid_sensor[11] = {false};

    // PASS 1 — collect valid sensors
    for(int i = 0; i < 11; i++){

        float t = temperature_readings[i];
        float h = humidity_readings[i];

        if(!isnan(t) && !isnan(h) && t > 1.0f && h > 1.0f){

            valid_sensor[i] = true;

            total_temp += t;
            total_humi += h;

            active_sensors++;

            if(t > highest_temperature) highest_temperature = t;
            if(t < lowest_temperature)  lowest_temperature  = t;

            if(h > highest_humidity) highest_humidity = h;
            if(h < lowest_humidity)  lowest_humidity  = h;
        }
    }

    if(active_sensors == 0){
        return NAN;
    }

    average_temperature = total_temp / active_sensors;
    average_humidity    = total_humi / active_sensors;

    temperature_range = highest_temperature - lowest_temperature;
    humidity_range    = highest_humidity - lowest_humidity;

    // PASS 2 — detect deviations and dead zones
    for(int i = 0; i < 11; i++){

        if(!valid_sensor[i]) continue;

        float t = temperature_readings[i];
        float h = humidity_readings[i];

        temperature_deviation = t - average_temperature;
        humidity_deviation    = h - average_humidity;

        // Dead-zone detection
        if(temperature_deviation < -2.0f && humidity_deviation > 5.0f){

            Serial.print("Dead zone suspected near sensor ");
            Serial.println(i + 1);
        }

        // Sensor malfunction detection
        if(abs(temperature_deviation) > 8.0f){

            Serial.print("Temperature sensor deviation too large at sensor ");
            Serial.println(i + 1);
        }

        if(abs(humidity_deviation) > 15.0f){

            Serial.print("Humidity sensor deviation too large at sensor ");
            Serial.println(i + 1);
        }
    }

    return average_temperature;
}



// ==================== HUMIDITY AGGREGATION ====================
float aggregate_sensor_humidities() {
    float total_humidity = 0.0f;
    uint8_t active_sensors = 0;
    
    // FIXED: Properly count and sum only valid readings
    if (!isnan(sensor_1_humidity) && sensor_1_humidity >= 0.0f && sensor_1_humidity <= 100.0f) { 
        total_humidity += sensor_1_humidity; active_sensors++; 
    }
    if (!isnan(sensor_2_humidity) && sensor_2_humidity >= 0.0f && sensor_2_humidity <= 100.0f) { 
        total_humidity += sensor_2_humidity; active_sensors++; 
    }
    if (!isnan(sensor_3_humidity) && sensor_3_humidity >= 0.0f && sensor_3_humidity <= 100.0f) { 
        total_humidity += sensor_3_humidity; active_sensors++; 
    }
    if (!isnan(sensor_4_humidity) && sensor_4_humidity >= 0.0f && sensor_4_humidity <= 100.0f) { 
        total_humidity += sensor_4_humidity; active_sensors++; 
    }
    if (!isnan(sensor_5_humidity) && sensor_5_humidity >= 0.0f && sensor_5_humidity <= 100.0f) { 
        total_humidity += sensor_5_humidity; active_sensors++; 
    }
    if (!isnan(sensor_6_humidity) && sensor_6_humidity >= 0.0f && sensor_6_humidity <= 100.0f) { 
        total_humidity += sensor_6_humidity; active_sensors++; 
    }
    if (!isnan(sensor_7_humidity) && sensor_7_humidity >= 0.0f && sensor_7_humidity <= 100.0f) { 
        total_humidity += sensor_7_humidity; active_sensors++; 
    }
    if (!isnan(sensor_8_humidity) && sensor_8_humidity >= 0.0f && sensor_8_humidity <= 100.0f) { 
        total_humidity += sensor_8_humidity; active_sensors++; 
    }
    if (!isnan(sensor_9_humidity) && sensor_9_humidity >= 0.0f && sensor_9_humidity <= 100.0f) { 
        total_humidity += sensor_9_humidity; active_sensors++; 
    }
    if (!isnan(sensor_10_humidity) && sensor_10_humidity >= 0.0f && sensor_10_humidity <= 100.0f) { 
        total_humidity += sensor_10_humidity; active_sensors++; 
    }
    if (!isnan(sensor_11_humidity) && sensor_11_humidity >= 0.0f && sensor_11_humidity <= 100.0f) { 
        total_humidity += sensor_11_humidity; active_sensors++; 
    }
    
    
    if (active_sensors == 0) {
        return NAN; // No valid readings
    }
    
    return total_humidity / active_sensors;
}
