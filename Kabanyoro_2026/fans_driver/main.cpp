/*
    XODDOCODE 2026 - 30TH MAY 2026
    SENSOR DATA-RECEIVES BY ESPNOW
    FAN CTRL BY PWM
    RTC
    PACKET ASSY
    SEND DATA BY UART
    RECEIVE COMMAND BY UART
*/
const char* devicename = "Fans_Controller_CPU2";
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
#include "fans.h"

#define buzzingpin 14
Fans Fan;
TimerKeeper sawa;
Buzzer buzzer(buzzingpin);
PacketHandler packetHandler;
JsonDocument JSON_sendable; // Adjust size as needed

char sendable_to_cloud_db[8096]; // Buffer for cloud upload
char pretty_data[12000];


const uint32_t com_baud = 250000; // down from 921600;
uint32_t uart_send_interval_ms = 30UL*1000UL; // Send data every 30 seconds
const uint8_t tx = 19; // GPIO pin for UART TX (to chip 2)

// Global variables
uint64_t now_now_ms = 0;
uint64_t packet_loss_counter = 0;  // Changed to 0 instead of 1
uint64_t total_packets_received = 0;

double voltage = 0.0;


SemaphoreHandle_t i2c_mutex   = nullptr;  // guards sawa.update() and sawa.* reads
SemaphoreHandle_t serial_mutex = nullptr;  // guards multi-line Serial output sequences



void check_time();
bool bind_dynamic_data_into_json();
// check for memory usage, dangling pointers etc
void printSystemSnapshot();

float aggregate_sensor_humidities();
float aggregate_sensor_temperatures();

char serialization_log[512] = "JSON Serialization Not Done!";
char csv_bind_log[512] = "CSV Serialization Not Done!";

// 2 tasks on core 0: PacketHandler and UART sender
// PACKET DETAILS parameters
const int PACKET_TASK_CORE = 0; 
const int PACKET_TASK_PRIORITY = 3; // Higher than normal tasks
const int PACKET_STACK_SIZE = 12288; // 12kB
TaskHandle_t packetTaskHandle = NULL;

//UART DETAILS
const int UART_TASK_CORE = 0;
const int UART_TASK_PRIORITY = 2; // Lower than packet handler
const int UART_STACK_SIZE = 16384; // 16kB Large stack for JSON formatting and UART operations
TaskHandle_t uartTaskHandle = NULL; // Handle for UART task


// 2 tasks on core 1: Buzzing and Fanning
const int FAN_TASK_CORE = 1;
const int FAN_TASK_PRIORITY = 1;
const int FAN_STACK_SIZE = 16384; // large stack for multiple floats, times, snprintf's etc
TaskHandle_t fanTaskHandle = NULL;

const int BUZZER_TASK_CORE = 1; 
const int BUZZER_TASK_PRIORITY = 5;
const int BUZZER_TASK_STACK = 2048; 
TaskHandle_t MonitorBuzzHandle = nullptr; // handle for MonitorBuzz

const int MEMORY_TASK_CORE = 1;
const int MEMORY_TASK_PRIORITY = 1;
const int MEMORY_STACK_SIZE = 8192; // 8kB stack for memory monitoring task
TaskHandle_t memoryTaskHandle = nullptr; // handle for MemoryMonitorTask


void MonitorBuzz(void *pvParams);
void check_on_fans_task(void *pvParams);
void uart_data_handler(void *pvParameters);
void MemoryMonitorTask(void *pvParameters);


bool ten_min = false; // alarm for 10 minute event (UPLOADS)
bool five_min = false; // alert for 5 minute event (SD SAVES)
bool hourly = false; // alert for hourly events ()
bool every_new_minute = true; // alert for somewhat slow events like check fans when idle...


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
/*
    The original ESP32 (DevKit) features 16 hardware PWM channels (8 high-speed + 8 low-speed) via the LEDC peripheral. 
    The ESP32-S3 has 8 hardware PWM channels (LEDC), although it also includes specialized motor control (MCPWM) peripherals for extra functionality. 
*/
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

// For debug logging
#define LOG(msg)  Serial.println(msg)


bool esp_now_initialized = false; char esp_now_status_msg[200];
bool serial_intialized = false;   char serial_init_log[200];

bool clock_initialized = false; bool clock_working = false; char clock_status_msg[200];
uint64_t boot_duration = 0;

char heap_stats[300]; 
uint32_t used_heap = 0;
uint32_t free_heap = 0;

// ── Per-step heap snapshot helper ────────────────────────────────────────────
// Shows free heap, bytes consumed by THIS step, and largest contiguous block.
// Largest block (maxBlk) is the fragmentation sensor:
//   If maxBlk << free  → heap is fragmented → large mallocs will fail silently.
static void heap_snap(const char *label, uint32_t heap_before) {
    uint32_t free_now  = ESP.getFreeHeap();
    uint32_t used_step = (heap_before > free_now) ? (heap_before - free_now) : 0;
    uint32_t max_blk   = ESP.getMaxAllocHeap();
    snprintf(heap_stats, sizeof(heap_stats),
             "%-20s free:%6uB used:%5uB maxBlk:%6uB",
             label, free_now, used_step, max_blk);
    Serial.println(heap_stats);
}

uint16_t mutex_retries = 0;

void setup() { delay(50);
    
    Serial.begin(9600); // with serial monitor
    Serial.print(devicename); Serial.println(" booting...");

    now_now_ms = esp_timer_get_time() / 1000ULL; // current time in ms since boot, for timestamping packets and calculating "last seen" times
    uint64_t bootstart = esp_timer_get_time();

   free_heap = (uint32_t)ESP.getFreeHeap() + 1;

    // Create mutexes before any task is started
    i2c_mutex    = xSemaphoreCreateMutex();
    serial_mutex = xSemaphoreCreateMutex();

    if (!i2c_mutex || !serial_mutex) {
        Serial.println("[BOOT] FATAL: mutex creation failed");
        while(true) { delay(1000); if(mutex_retries >= 100) break; mutex_retries++; } // or halt — cannot continue safely
    }

    // ── Baseline ─────────────────────────────────────────────────────────────
    heap_snap("BOOT baseline", ESP.getFreeHeap() + 1);
    uint32_t h = 0;
    // ---- Heap snapshot 1 (before any tasks) ---------------------------------
    snprintf(heap_stats, sizeof(heap_stats), "%lu", free_heap);
    
    // TASK 1: BUZZER------------------------------------- INITIALLY SUSPENDED
        h = ESP.getFreeHeap();
        // Core 1 Task created in class to auto-Check only_if_buzzing_started
      buzzer.begin();
      buzzer.createTask(BUZZER_TASK_PRIORITY, BUZZER_TASK_STACK, BUZZER_TASK_CORE); 
      buzzer.beep(1,50,0); // starts then auto suspends
      heap_snap("buzzer task created, suspended initially until beep is called", h);    

      ////INITIALIZING the buzzer task once - it will start suspended
    // IF THE TASK DOES NOT CORRECTLY START
    // pinMode(buzzingpin, OUTPUT);
    // digitalWrite(buzzingpin, 1); delay(50); digitalWrite(buzzingpin, 0);
   
      
     snprintf(heap_stats, sizeof(heap_stats), "%lu",    used_heap);
     Serial.println(heap_stats);

  
    /*
      esp_now_initialized = initialize_espnow();
      snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "ESP-NOW : %s", esp_now_initialized ? "Initialized Successfully" : "Initialization Failed!");
      Serial.println(esp_now_status_msg);
      delay(1000);
    */
   /*
   if(wifi_connected) {
               
                // Don't re-initialize! Just use existing connection
                wifi_obj.sync_ntp(10800, 0, NTP_TIMEOUT);  // SECONDS
                if(wifi_obj.ntp_synced) {
                    sawa.apply_ntp_time(wifi_obj.ntp_timeinfo);
                    Serial.println("✅ Using NTP Time to set RTC Time");
                } else {
                    Serial.println("⚠️ NTP sync failed - keeping existing RTC time");
                }
            } else {
                Serial.println("⚠️ WiFi not connected - skipping NTP sync");
            }

            snprintf(clock_status_msg, sizeof(clock_status_msg),
                "RTC: %s | NTP: %s | Time: %s | Date: %s",
                clock_initialized        ? "OK"      : "FAILED",
                wifi_obj.ntp_synced     ? "Synced"  : "Not synced",
                sawa.SystemTime,
                sawa.SystemDate);

        } else snprintf(clock_status_msg, sizeof(clock_status_msg), "RTC initialization failed!");
        

                LOG(clock_status_msg);

        */

   

        
      clock_initialized = sawa.initialize_RTC();
      snprintf(clock_status_msg, sizeof(clock_status_msg), "RTC Clock: %s", clock_initialized ? "Initialized Successfully" : "Initialization Failed!");
      Serial.println(clock_status_msg);


 // TASK 3 PACKETS --------------------- QUEUE TYPE
      // Initialize packet handler
        h = ESP.getFreeHeap();

    //     static const int PACKET_TASK_CORE = 0;  static const int PACKET_STACK_SIZE = 16384;

      esp_now_initialized = packetHandler.begin(); // pinned to core 0: has espnow, callbacks, and queue creation inside
    if (!esp_now_initialized) snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "Failed to initialize packet handler!");
        
    else     snprintf(esp_now_status_msg, sizeof(esp_now_status_msg), "ESP-NOW Initialization Successful!");
    // HANDLES onDataReceived
    
     snprintf(heap_stats, sizeof(heap_stats), "%lu",    used_heap);
     Serial.println(heap_stats);

     



 // TASK 4 FANS ------------------------------- ALWAYS RUNNING
    Fan.begin(sawa.SystemTime, aggregate_sensor_temperatures, &average_humidity, &sensor_12_temp, &now_now_ms);
    Fan.initialize_fans(); // all in PWM mode
    Fan.fan_speed_test(Fan.all_fans, Fan.rise_time_ms, Fan.fall_time_ms);

     h = ESP.getFreeHeap();
    // show the heap and the stack after starting the FAN task
    
    
      
     average_temperature = aggregate_sensor_temperatures();
     // to get average_humidity and temperatures for the first time before the task starts, so that we have some data to work with in the fans task right from the start (for example to decide if we need to turn on the fans immediately)

     
    xTaskCreatePinnedToCore(
        check_on_fans_task,
        "FAN_HANDLER",
        FAN_STACK_SIZE, 
        NULL,
        FAN_TASK_PRIORITY,
        &fanTaskHandle,
        FAN_TASK_CORE
    );
     heap_snap("buzzer uart created", h);    
     
    snprintf(heap_stats, sizeof(heap_stats), "%lu",    used_heap);
    Serial.println(heap_stats);


     LOG("[Setup] FAN Task Started.");
     delay(1000);

    
     // Serial2.setRxBufferSize(1024); // for receiving commands from chip 2
      Serial2.begin(com_baud, SERIAL_8N1, 16, 17); // or 1Mbps with chip 2 to send 4kB in < 50ms
      Serial2.setTimeout(900);

      if(Serial2.available() > 0){
        Serial.println(Serial2.read()); // if there is any data already in the buffer, read and print it (for debug)
      }

      snprintf(serial_init_log, sizeof(serial_init_log), "Serial 2 Initialized with BAUD: %lu", com_baud);
      delay(1000); // wait for serial to initialize

      
      // TASK 2 UART ------------------------ ALWAYS RUNNING
    
    //show the stack before starting the UART task
     h = ESP.getFreeHeap();
    LOG("[Setup] Starting UART Task for sending data to receiver...");
    xTaskCreatePinnedToCore(
        uart_data_handler,   // Task function
        "UARTDataHandler",             // Name of the task
        UART_STACK_SIZE,           // Stack size in words
        NULL,                        // Task input parameter
        UART_TASK_PRIORITY,        // Priority of the task
        &uartTaskHandle,                       // Task handle
        UART_TASK_CORE             // Core where the task should run
    );

    heap_snap("buzzer uart created", h);    
          
    snprintf(heap_stats, sizeof(heap_stats), "%lu",    used_heap);
    Serial.println(heap_stats);

    delay(1000); // give the UART task a moment to start before we start the memory monitor task, to get a more accurate heap snapshot of the memory monitor task's impact
    
    
     h = ESP.getFreeHeap();
    xTaskCreatePinnedToCore(
        MemoryMonitorTask,
        "MEMORY_MONITOR",
        MEMORY_STACK_SIZE, 
        NULL,
        MEMORY_TASK_PRIORITY,
        &memoryTaskHandle,
        MEMORY_TASK_CORE
    );

    Serial.println("Memory Monitor Task Started.");
    heap_snap("memory monitor task created", h);


    boot_duration = (esp_timer_get_time() - bootstart)/1000000ULL;

    Serial.println(esp_now_status_msg);
    Serial.println(clock_status_msg);
    Serial.println(serial_init_log);
    Serial.print("Boot Duration (s): " ); Serial.println(boot_duration);
    
       
    
    delay(1000);

    


}





// Globals (keep these, used for transmission)
float heap_fragmentation = 0.0f;
float largest_contiguous = 0.0;
float min_free_heap      = 0.0;
float free_heap_f          = 0.0;

// Watermark globals — written by each task, read by transmit logic
volatile uint32_t wm_buzzing_bytes  = 0;
volatile uint32_t wm_fanning_bytes  = 0;
volatile uint32_t wm_sensing_bytes  = 0; 


char sensing_watermark_char[128] = "SENSING Stack not yet allocated";  // slightly wider than others — longer label
char fanning_watermark_char[128] = "FANNING Stack not yet allocated";
char uart_watermark_char[128] = "UART Stack not yet allocated";
char buzzing_watermark_char[128] = "BUZZING Stack not yet allocated";
char heap_report[1024] = "..";
char tasks_memory_pressure[2048] = "..";

const uint32_t memory_check_frequency = (5UL * 60UL * 1000UL); // 5 minute intervals

// CREATTE A MEMORY MONITORING TASK THAT RUNS EVERY 30 SECONDS AND CHECKS THE FREE HEAP, THE LARGEST CONTIGUOUS BLOCK, AND THE STACK WATERMARKS OF THE FAN HANDLER AND UART HANDLER TASKS.
void MemoryMonitorTask(void *pv){
    uint32_t minHeapSeen = UINT32_MAX;
    const TickType_t memDelay = pdMS_TO_TICKS(memory_check_frequency); // 5 minutes

    while(true){
        // ========== HEAP METRICS ==========
        free_heap_f = (float)esp_get_free_heap_size();
        min_free_heap = (float)esp_get_minimum_free_heap_size();
        largest_contiguous = (float)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        heap_fragmentation = (free_heap_f > 0) ? (100.0f - ((largest_contiguous * 100.0f) / free_heap_f)) : 0.0f;

        bool newMin = (min_free_heap < minHeapSeen);
        if(newMin) minHeapSeen = min_free_heap;

        // ========== STACK WATERMARKS FOR ALL TASKS ==========
        
        // 1. UART TASK
        UBaseType_t uart_wm = uxTaskGetStackHighWaterMark(uartTaskHandle);
        uint32_t uart_wm_bytes = uart_wm * sizeof(StackType_t);
        uint32_t uart_used = UART_STACK_SIZE - uart_wm_bytes;
        uint32_t uart_percent = (uart_used * 100) / UART_STACK_SIZE;

        snprintf(uart_watermark_char, sizeof(uart_watermark_char),
            "[UART] used %u/%u bytes (%u%%) | %.1fkB free",
            uart_used, UART_STACK_SIZE, uart_percent,
            uart_wm_bytes / 1024.0f);

        // 2. FANNING TASK
        UBaseType_t fan_wm = uxTaskGetStackHighWaterMark(fanTaskHandle);
        uint32_t fan_wm_bytes = fan_wm * sizeof(StackType_t);
        uint32_t fan_used = FAN_STACK_SIZE - fan_wm_bytes;
        uint32_t fan_percent = (fan_used * 100) / FAN_STACK_SIZE;

        snprintf(fanning_watermark_char, sizeof(fanning_watermark_char),
            "[FAN] used %u/%u bytes (%u%%) | %.1fkB free",
            fan_used, FAN_STACK_SIZE, fan_percent,
            fan_wm_bytes / 1024.0f);

        // ========== FORMAT HEAP REPORT ==========
        if (newMin) {
            snprintf(heap_report, sizeof(heap_report), 
                "\n[MEMORY - HEAPS] (!) NEW MIN! \r\nRuntime Free Heap=%.1fkB, \r\nRuntime Min Free Heap=%.1fkB, \nLargest Contiguous Block=%.1fkB, \r\nRuntime Frag=%.1%%\n",
                free_heap / 1024.0f,
                min_free_heap / 1024.0f,
                largest_contiguous / 1024.0f,
                heap_fragmentation);
        } else {
            snprintf(heap_report, sizeof(heap_report), 
                "[MEMORY - HEAPS] \r\nRuntime Free Heap=%.1fkB, \r\nRuntime Min=%.1fkB, \r\nLargest Contiguous Block=%.1fkB, \r\nRuntime Frag=%.2f%%\n",
                free_heap / 1024.0f,
                min_free_heap / 1024.0f,
                largest_contiguous / 1024.0f,
                heap_fragmentation);
        }

        // ========== FORMAT TASKS MEMORY REPORT ==========
        snprintf(tasks_memory_pressure, sizeof(tasks_memory_pressure),
                "[MEMORY - STACKS]\n%s\n%s\n",
                uart_watermark_char,  fanning_watermark_char
            );

        // ========== PRINT REPORTS ==========
        Serial.println(heap_report);
        Serial.println(tasks_memory_pressure);
       // printSystemSnapshot(); // with cores, tasks, etc

        vTaskDelay(memDelay);
    }
}


bool json_bound_successfully = false;
char serial_send_log[512] = "JSON Not Bound Yet!";
size_t size_of_serialized_data = 0;

//sender
void uart_data_handler(void *pvParameters) {
     UBaseType_t uxHighWaterMark;
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
                snprintf(serial_send_log, sizeof(serial_send_log), "[UART] JSON Bind Successful. Sending data to receiver...");
               /*
                    Serial2.write('\n');              // flush any stale line first
                    Serial2.print(sendable_to_cloud_db);
                    Serial2.write('\n');              // end-of-frame marker
                */

                uint64_t write_start_time = esp_timer_get_time();
                Serial2.write('\x02');            // STX - start of text
                Serial2.print(sendable_to_cloud_db);
                Serial2.write('\x03');            // ETX - end of text  
              //  Serial2.print("end!");              // to signal end
                uint64_t write_end_time = esp_timer_get_time();
                Serial.println(); // Blank line for better readability in the serial monitor
                Serial.print("[UART] Data sent to receiver. Transmission time (ms): ");
                Serial.println((write_end_time - write_start_time)/1000.0f);

                // Serial2.write(sendable_to_cloud_db, size_of_serialized_data); // Send the JSON string to the receiver
                
              //  Serial.println(sendable_to_cloud_db); // Blank line for better readability in the serial monitor
                Serial.println(pretty_data); // Also print the pretty version for debugging

                /*
                    serializeJson() writes only the real JSON into sendable_to_cloud_db and null-terminates it. So even though the buffer is 8096 bytes, 
                    Serial2.println(sendable_to_cloud_db) only transmits up to that null terminator — meaning you're sending ~490 bytes, not 8096. 
                    The empty space in the buffer is never sent.
                */

         }
        // Check stack high water mark periodically

           uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        Serial.printf("[STACK] Free stack: %u bytes\n", 
                      uxHighWaterMark * sizeof(StackType_t));

         LOG(serial_send_log);

        vTaskDelay(pdMS_TO_TICKS(uart_send_interval_ms)); // Adjust delay as needed
    }
}


// Helper function to determine if a sensor is valid/active
bool is_sensor_active(float temp, float humidity, uint64_t last_seen, unsigned long now) {
    // Consider sensor active if:
    // - It has valid readings (temp > 1.0 and < 85, humidity between 0-100)
    // - AND it was seen recently (within last 5 minutes)
    bool has_valid_readings = (temp > 1.0f && temp < 85.0f) || (humidity >= 1.0f && humidity <= 100.0f);
    bool recently_seen = (now - last_seen) < 300000; // 5 minutes in ms
    bool sensor_stale = false;

    uint64_t staleness_threshold_ms = 5ULL * 60ULL * 1000ULL; // 2 minutes
    if((now_now_ms - last_seen) < staleness_threshold_ms){
        sensor_stale = true;
    }
    return has_valid_readings; // sensor_stale // && recently_seen;
}

// https://open.spotify.com/track/5zSZE82L2LZOaYzglp11bH?si=169e23e3e6e34aa9

// Helper to determine if a fan should be included
bool should_include_fan(bool is_on, bool was_running, unsigned long duration) {
    // Include if:
    // - Fan is currently on, OR
    // - It was recently running (was_running flag), OR
    // - It has non-zero duration (ran at some point this session)
    return is_on || was_running || duration > 0;
}


bool bind_dynamic_data_into_json() {
    Serial.println(); // Blank line for readability
    Serial.println("Binding dynamic data into JSON...");

    
        average_temperature = aggregate_sensor_temperatures();
    /*
        Serial.print("Average Temp: "); Serial.println(average_temperature);
        Serial.print("Average Humi: "); Serial.println(average_humidity);
     */

    bool bound_successfully = false;

    JSON_sendable.clear();
   // save_counter++; // save this into EEPROM or SPIFFS

    // ================= SYSTEM RUNTIME DATA =================
    JsonObject System = JSON_sendable["System"].to<JsonObject>();
    System["Uptime"] = (now_now_ms / 1000);
    System["Date"] = sawa.SystemDate;
    System["Time"] = sawa.ShortTime; //sawa.SystemTime;
    System["Voltage"] = voltage;
    System["HEAP_REPORT"] = heap_report;
    System["STACK_WATERMARKS"] = tasks_memory_pressure; 

        // ================= INDIVIDUAL SENSOR RUNTIME VALUES =================

    if(sensor_1_temp > 1.0 || sensor_1_humidity > 1.0){ // if sensor 1 exists
        JsonObject Sensor_1 = JSON_sendable["Sensor_1"].to<JsonObject>();
        Sensor_1["Temp"] = sensor_1_temp;
        Sensor_1["Humi"] = sensor_1_humidity;
        Sensor_1["Pressure"] = sensor_1_pressure;
        Sensor_1["Last_Seen"] = sensor_1_last_seen;
        Sensor_1["Sends"] = sensor_1_transmissions;
    }

    if(sensor_2_temp > 1.0 || sensor_2_humidity > 1.0){ // if sensor 2 exists
        JsonObject Sensor_2 = JSON_sendable["Sensor_2"].to<JsonObject>();
        Sensor_2["Temp"] = sensor_2_temp;
        Sensor_2["Humi"] = sensor_2_humidity;
        Sensor_2["Pressure"] = sensor_2_pressure;
        Sensor_2["Last_Seen"] = sensor_2_last_seen;
        Sensor_2["Sends"] = sensor_2_transmissions;
    }

    if(sensor_3_temp > 1.0 || sensor_3_humidity > 1.0){ // if sensor 9=3 exists
        JsonObject Sensor_3 = JSON_sendable["Sensor_3"].to<JsonObject>();
        Sensor_3["Temp"] = sensor_3_temp;
        Sensor_3["Humi"] = sensor_3_humidity;
        Sensor_3["Pressure"] = sensor_3_pressure;
        Sensor_3["Last_Seen"] = sensor_3_last_seen;
        Sensor_3["Sends"] = sensor_3_transmissions;
    }

    if(sensor_4_temp > 1.0 || sensor_4_humidity > 1.0){ // if sensor 4 exists
        JsonObject Sensor_4 = JSON_sendable["Sensor_4"].to<JsonObject>();
        Sensor_4["Temp"] = sensor_4_temp;
        Sensor_4["Humi"] = sensor_4_humidity;
        Sensor_4["Pressure"] = sensor_4_pressure;
        Sensor_4["Last_Seen"] = sensor_4_last_seen;
        Sensor_4["Sends"] = sensor_4_transmissions;
    }

    if(sensor_5_temp > 1.0 || sensor_5_humidity > 1.0){ // if sensor 5 exists
        JsonObject Sensor_5 = JSON_sendable["Sensor_5"].to<JsonObject>();
        Sensor_5["Temp"] = sensor_5_temp;
        Sensor_5["Humi"] = sensor_5_humidity;
        Sensor_5["Pressure"] = sensor_5_pressure;
        Sensor_5["Last_Seen"] = sensor_5_last_seen;
        Sensor_5["Sends"] = sensor_5_transmissions;
    }

    if(sensor_6_temp > 1.0 || sensor_6_humidity > 1.0){ // if sensor 6 exists
        JsonObject Sensor_6 = JSON_sendable["Sensor_6"].to<JsonObject>();
        Sensor_6["Temp"] = sensor_6_temp;
        Sensor_6["Humi"] = sensor_6_humidity;
        Sensor_6["Pressure"] = sensor_6_pressure;
        Sensor_6["Last_Seen"] = sensor_6_last_seen;
        Sensor_6["Sends"] = sensor_6_transmissions;
    }

    if(sensor_7_temp > 1.0 || sensor_7_humidity > 1.0){ // if sensor 7 exists
        JsonObject Sensor_7 = JSON_sendable["Sensor_7"].to<JsonObject>();
        Sensor_7["Temp"] = sensor_7_temp;
        Sensor_7["Humi"] = sensor_7_humidity;
        Sensor_7["Pressure"] = sensor_7_pressure;
        Sensor_7["Last_Seen"] = sensor_7_last_seen;
        Sensor_7["Sends"] = sensor_7_transmissions;
    }

    if(sensor_8_temp > 1.0 || sensor_8_humidity > 1.0){ // if sensor 8 exists
        JsonObject Sensor_8 = JSON_sendable["Sensor_8"].to<JsonObject>();
        Sensor_8["Temp"] = sensor_8_temp;
        Sensor_8["Humi"] = sensor_8_humidity;
        Sensor_8["Pressure"] = sensor_8_pressure;
        Sensor_8["Last_Seen"] = sensor_8_last_seen;
        Sensor_8["Sends"] = sensor_8_transmissions;
    }

     if(sensor_9_temp > 1.0 || sensor_9_humidity > 1.0){ // if sensor 9 exists
        JsonObject Sensor_9 = JSON_sendable["Sensor_9"].to<JsonObject>();
        Sensor_9["Temp"] = sensor_9_temp;
        Sensor_9["Humi"] = sensor_9_humidity;
        Sensor_9["Pressure"] = sensor_9_pressure;
        Sensor_9["Last_Seen"] = sensor_9_last_seen;
        Sensor_9["Sends"] = sensor_9_transmissions;
     }

     if(sensor_10_temp > 1.0 || sensor_10_humidity > 1.0){ // if sensor 10 exists
        JsonObject Sensor_10 = JSON_sendable["Sensor_10"].to<JsonObject>();
        Sensor_10["Temp"] = sensor_10_temp;
        Sensor_10["Humi"] = sensor_10_humidity;
        Sensor_10["Pressure"] = sensor_10_pressure;
        Sensor_10["Last_Seen"] = sensor_10_last_seen;
        Sensor_10["Sends"] = sensor_10_transmissions;
     }

     if(sensor_11_temp > 1.0 || sensor_11_humidity > 1.0){ // if sensor 11 exists
        JsonObject Sensor_11 = JSON_sendable["Sensor_11"].to<JsonObject>();
        Sensor_11["Temp"] = sensor_11_temp;
        Sensor_11["Humi"] = sensor_11_humidity;
        Sensor_11["Pressure"] = sensor_11_pressure;
        Sensor_11["Last_Seen"] = sensor_11_last_seen;
        Sensor_11["Sends"] = sensor_11_transmissions;
     }

    if(sensor_12_temp > 1.0 || sensor_12_humidity > 1.0){ // if sensor 12 exists
        JsonObject Sensor_12 = JSON_sendable["Sensor_12"].to<JsonObject>();
        Sensor_12["Temp"] = sensor_12_temp;
        Sensor_12["Humi"] = sensor_12_humidity;
        Sensor_12["Pressure"] = sensor_12_pressure;
        Sensor_12["Last_Seen"] = sensor_12_last_seen;
        Sensor_12["Sends"] = sensor_12_transmissions;
    }


     // ================= DRYER METRICS =================
    JsonObject Dryer = JSON_sendable["Dryer"].to<JsonObject>();
    Dryer["Drying_Temp"] = average_temperature;
    Dryer["Drying_Humi"] = average_humidity;
    Dryer["Ambient_Temp"] = sensor_12_temp; // outdoor temp
    Dryer["Ambient_Humi"] = sensor_12_humidity; // outdoor humidity
    Dryer["Temp_Diff"] = average_temperature - sensor_12_temp; // sensor 12 is the outdoor sensor, so this is the temp difference between dryer and outdoor, which can be a useful metric for controlling fans and understanding drying conditions
    Dryer["Humi_Diff"] = average_humidity - sensor_12_humidity; // same for humidity

    Dryer["InDryer_Tempe_Range"] = highest_temperature - lowest_temperature;
    Dryer["InDryer_Humi_Range"] = highest_humidity - lowest_humidity;
    Dryer["Active_Sensors"] = active_sensors;
    Dryer["Fan_control_Mode"] = Fan.is_night_time ? "Night Mode" : "Day Mode";
    Dryer["Rain_Detected"] = Fan.is_raining ? "Raining" : "Not Raining";
   
    // ================= FAN STATES =================
    JsonObject Fans = JSON_sendable["Fans"].to<JsonObject>();
    Fans["Fan_1"]   = Fan.cooling_fan_1_on? "ON" : "OFF";
    Fans["Fan_2"]   = Fan.cooling_fan_2_on? "ON" : "OFF";
    Fans["Fan_3"]   = Fan.extract_fan_1_on? "ON" : "OFF";
    Fans["Fan_4"]   = Fan.extract_fan_2_on? "ON" : "OFF";
    Fans["Fan_5"]   = Fan.extract_fan_3_on? "ON" : "OFF";
    Fans["Fan_6"]   = Fan.extract_fan_4_on? "ON" : "OFF";

    //EFFECTS OF FANNINGS
    if(Fan.cooling_fan_1_on || Fan.cooling_fan_2_on ||  Fan.cooling_1_fan_was_running ||  Fan.cooling_2_fan_was_running){
        Fans["Cooling_fan_temp_Gradient"] = Fan.temperature_gradient_due_to_cooling_fans; // rate of change of temperature, which can indicate how quickly the dryer is drying and whether fans need to be adjusted
        Fans["Cooling_fan_Humi_Gradient"] = Fan.humidity_gradient_due_to_cooling_fans; // how much the humidity has dropped since we turned on the cooling fans, which can indicate whether they are helping with drying or if they are making it worse by adding humidity

    }

    
    if(Fan.extract_fan_1_on || Fan.extract_fan_2_on || Fan.extract_fan_3_on || Fan.extract_fan_4_on){
         Fans["Extract_fan_Temp_Gradient"] = Fan.temperature_gradient_due_to_extract_fans; // rate of change of temperature, which can indicate how quickly the dryer is drying and whether fans need to be adjusted
         Fans["Extract_fan_Humi_Gradient"] = Fan.humidity_gradient_due_to_extract_fans; // change in humidity because of extraction

    }


    // nesting independent fan into object fans
    

    if(Fan.cooling_fan_1_on || Fan.cooling_1_fan_was_running){ // if is on or was recently on
         JsonObject Fan1 = JSON_sendable["Fan1"].to<JsonObject>();
            Fan1["Turn_On_Time"] = Fan.cooling_fan_1_time_on_str;
            Fan1["Turn_Off_Time"] = Fan.cooling_fan_1_time_off_str;
            Fan1["Total_Run_Time"] = Fan.cooling_fan_1_duration/1000ULL; // in seconds
    }

    if(Fan.cooling_fan_2_on || Fan.cooling_2_fan_was_running){
        JsonObject Fan2 = JSON_sendable["Fan2"].to<JsonObject>();
            Fan2["Turn_On_Time"] = Fan.cooling_fan_2_time_on_str;
            Fan2["Turn_Off_Time"] = Fan.cooling_fan_2_time_off_str;
            Fan2["Total_Run_Time"] = Fan.cooling_fan_2_duration/1000ULL; // in seconds
    
    }


    if(Fan.extract_fan_1_on || Fan.extract_fan_1_was_running){
        JsonObject Fan3 = JSON_sendable["Fan3"].to<JsonObject>();
            Fan3["Turn_On_Time"] = Fan.extract_fan_1_time_on_str;
            Fan3["Turn_Off_Time"] = Fan.extract_fan_1_time_off_str;
            Fan3["Total_Run_Time_secs"] = Fan.extract_fan_1_duration/1000ULL; // in seconds            
    }

    if(Fan.extract_fan_2_on || Fan.extract_fan_2_was_running){
         JsonObject Fan4 = JSON_sendable["Fan4"].to<JsonObject>();
            Fan4["Turn_On_Time"] = Fan.extract_fan_2_time_on_str;
            Fan4["Turn_Off_Time"] = Fan.extract_fan_2_time_off_str;
            Fan4["Total_Run_Time"] = Fan.extract_fan_2_duration/1000ULL; // in seconds
    }
    if(Fan.extract_fan_3_on || Fan.extract_fan_3_was_running){
         JsonObject Fan5 = JSON_sendable["Fan5"].to<JsonObject>();
            Fan5["Turn_On_Time"] = Fan.extract_fan_3_time_on_str;
            Fan5["Turn_Off_Time"] = Fan.extract_fan_3_time_off_str;
            //Fans["Fan_5_Turn_On_MS"] = extract_fan_3_on_time_ms;
            Fan5["Total_Run_Time"] = Fan.extract_fan_3_duration/1000ULL; // in seconds
    }
    if(Fan.extract_fan_4_on || Fan.extract_fan_4_was_running){
         JsonObject Fan6 = JSON_sendable["Fan6"].to<JsonObject>();
            Fan6["Turn_On_Time"] = Fan.extract_fan_4_time_on_str;
            Fan6["Turn_Off_Time"] = Fan.extract_fan_4_time_off_str;
            //Fans["Fan_6_Turn_On_MS"] = extract_fan_4_on_time_ms;
            Fan6["Total_Run_Time"] = Fan.extract_fan_4_duration/1000ULL; // in seconds
    }
    



                // ================= SERIALIZATION =================

            size_t len_cloud = serializeJson(JSON_sendable, sendable_to_cloud_db);
            size_t len_test = serializeJsonPretty(JSON_sendable, pretty_data); // to be deleted afterwards 

            if (len_cloud == 0) {
                snprintf(serialization_log,   sizeof(serialization_log),   "[JSON] Serialization failed!");
                LOG(serialization_log);
                return false;
            }

            // Optional: detect truncation (ArduinoJson does NOT null-terminate if truncated)
            if (len_cloud >= sizeof(sendable_to_cloud_db)){
            
                snprintf(serialization_log,   sizeof(serialization_log),  "[JSON] WARNING: Buffer may be too small (truncation detected)");
                LOG(serialization_log);
            }

            bound_successfully = true;

            size_of_serialized_data = len_cloud; 


            snprintf(serialization_log, sizeof(serialization_log),     "[JSON] Serialized OK | Cloud: %u bytes",   (unsigned int)len_cloud);

            LOG(serialization_log);

            //Serial.println(pretty_data);


    return bound_successfully;
}



bool is_buzzing = false;

// Monitor task for checking buzzer status
void MonitorBuzz(void *pvParams) {
  is_buzzing = buzzer.isBuzzing();
  while (true) {
    if (is_buzzing) {
      Serial.println("Buzzer is active");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


uint8_t mint_counter = 0; // counts how many minutes the system has been running, for tracking when to print stats and do hourly tasksmint_counter
uint64_t lastStats = 0;
uint64_t lastfancheck = 0;

void loop() {
  // put your main code here, to run repeatedly:

     now_now_ms = esp_timer_get_time() / 1000ULL; // current time in ms since boot, for timestamping packets and calculating "last seen" times


    // Print statistics periodically optionally log them to the cloud
    if((now_now_ms - lastStats) >= (60ULL*60ULL*1000)){ // once every 60 minutes
            check_time();
            printSystemSnapshot();
            packetHandler.printStatistics();
        
            lastStats = now_now_ms;
            mint_counter++; // increment the counter for how many minutes the system has been running, which can be useful for tracking long-term performance and trends    
    }

    delay(10000); // 10 seconds main loop delay, to keep the CPU unoccupied mostly
   
}








bool fan_state_previous = false;
bool any_fan_on = false;
// if a fan changes state from ON to OFF or OFF to ON, 
//buzzer.beep(1,50,0);
 uint64_t fan_check_interval = 60ULL * 1000ULL; // DEFAULT IS EVERY 60 SECONDS, unless active
void check_on_fans_task(void *pvParams){
    while(true){
        check_time();
        Fan.regulate_fans(now_now_ms);

        if(fan_state_previous != any_fan_on) buzzer.beep(1,50,0);

        any_fan_on = Fan.cooling_fan_1_on || Fan.cooling_fan_2_on ||
                      Fan.extract_fan_1_on || Fan.extract_fan_2_on ||
                      Fan.extract_fan_3_on || Fan.extract_fan_4_on;
        fan_state_previous = any_fan_on;
        fan_check_interval = any_fan_on ? 1000ULL : 60ULL * 1000ULL;
        vTaskDelay(pdMS_TO_TICKS(fan_check_interval));
    }
}



char _1minute_marked[128] = "New minute: "; //   HH:MM:SS
char _5minute_marked[128] = "5-min marker at "; //   HH:MM:SS
char _10minute_marked[128] = "10-min marker at "; //   HH:MM:SS

char _1hour_marked[128] = "Hourly task running"; //   HH:MM:SS

void check_time() {
    if (!sawa.isClockWorking()) return;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    sawa.update();

    five_min         = sawa.is5MinuteMarker();
    ten_min          = sawa.is10MinuteMarker();
    hourly           = sawa.isHourlyMarker();
    every_new_minute = sawa.isNewMinute();

    // Copy the time string while still holding the mutex,
    // so the snprintfs below read a stable value after release.
    char time_snap[20];
    strncpy(time_snap, sawa.SystemTime, sizeof(time_snap));

    xSemaphoreGive(i2c_mutex);

    if (five_min)        { snprintf(_5minute_marked,  sizeof(_5minute_marked),  "5-min marker at %s",  time_snap); Serial.println(_5minute_marked);  }
    if (ten_min)         { snprintf(_10minute_marked, sizeof(_10minute_marked), "10-min marker at %s", time_snap); Serial.println(_10minute_marked); }
    if (hourly)          { snprintf(_1hour_marked,    sizeof(_1hour_marked),    "Hourly marker at %s", time_snap); Serial.println(_1hour_marked);    }
    if (every_new_minute){ snprintf(_1minute_marked,  sizeof(_1minute_marked),  "Minutely marker at %s",time_snap); Serial.println(_1minute_marked); }
}



bool check_time_of_day(){ 
    // Placeholder: Replace with actual logic to determine if it's night time based on RTC time
    int current_hour = sawa.getCurrentHour(); // Assuming this returns the current hour in 24-hour format
    return (current_hour >= 20 || current_hour < 6); // Example: Night time from 8 PM to 6 AM
}

// REPLACE with the version below.
//
// Key changes:
//   - system_health is now 1024 bytes — enough for one complete snapshot
//   - The buffer is rebuilt fresh on every call (not appended forever)
//   - Serial output and buffer build happen together in one pass
//   - append_snapshot_to_log() writes system_health to Serial2 (or SD when ready)
//   - serial_mutex still wraps the Serial output sequence
// ───────────────────────────────────────────────────────────────────────────

// One snapshot fits comfortably in 1024 bytes:
//   header + 3 stat lines ≈ 200 chars
//   5 task rows × 65 chars = 325 chars
//   footer ≈ 55 chars
//   timestamp line ≈ 40 chars
//   Total ≈ 620 chars — well within 1024
char system_health[1024];

// Appends the current snapshot to whatever output channel you have.
// Right now this sends over Serial2 so the receiver can log it to SD.
// When you add SPIFFS/SD directly, replace Serial2.println with file.println.
static void append_snapshot_to_log() {
    
    //Serial2.println(system_health); // receiver saves this to SystemHealthLog.txt

    /*
    upload(system_health);
    or 
    save_to_sd_card(system_health)
    */
}

void printSystemSnapshot() {

    // ── Build the snapshot string into system_health ──────────────────────
    // We build into the buffer first so both Serial and the log file
    // receive identical content from the same source.

    char row[80];   // scratch buffer for one task row
    uint32_t pos = 0;

    // Helper: appends a formatted string to system_health safely
    auto append = [&](const char* s) {
        size_t remaining = sizeof(system_health) - pos - 1;
        size_t len = strnlen(s, remaining);
        memcpy(system_health + pos, s, len);
        pos += len;
        system_health[pos] = '\0';
    };

    // Timestamp header
    snprintf(row, sizeof(row), "\n[%s] SYSTEM SNAPSHOT\n", Fan._system_time ? Fan._system_time : "--:--:--");
    append(row);

    // Heap stats
    snprintf(row, sizeof(row),
             "Uptime:%lus Free:%uB MaxBlk:%uB MinEver:%uB\n",
             (unsigned long)(esp_timer_get_time() / 1000000ULL),
             ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    append(row);

    float frag = 100.0f -
        (100.0f * (float)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)
                / (float)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    snprintf(row, sizeof(row), "Fragmentation:%.1f%%\n", frag);
    append(row);

    // Table header
    append("╔══════════╦═══════╦══════╦══════════════════╗\n");
    append("║ Task     ║ Core  ║ St   ║ Stack HWM (bytes)║\n");
    append("╠══════════╬═══════╬══════╬══════════════════╣\n");

    // Per-task row builder — writes into system_health and returns the string
    auto buildTaskRow = [&](const char* name, TaskHandle_t h) {
        if (!h) {
            snprintf(row, sizeof(row),
                     "║ %-8s ║  ---  ║  --  ║       ---       ║\n", name);
        } else {
            char core = (xTaskGetAffinity(h) == 0) ? '0' :
                        (xTaskGetAffinity(h) == 1) ? '1' : 'A';
            char state = '?';
            switch (eTaskGetState(h)) {
                case eRunning: case eReady: state = 'R'; break;
                case eBlocked:             state = 'B'; break;
                case eSuspended:           state = 'S'; break;
                case eDeleted:             state = 'D'; break;
                default: break;
            }
            uint32_t hwm_bytes = uxTaskGetStackHighWaterMark(h) * 4;
            const char* warn = (hwm_bytes < 1024) ? " <1kB!" : "      ";
            snprintf(row, sizeof(row),
                     "║ %-8s ║   C%c  ║   %c  ║  %6u B%s ║\n",
                     name, core, state, hwm_bytes, warn);
        }
        append(row);
    };

    buildTaskRow("loop",    xTaskGetCurrentTaskHandle());
    buildTaskRow("buzzer",  MonitorBuzzHandle);
    buildTaskRow("uart",    uartTaskHandle);
    buildTaskRow("fans",    fanTaskHandle);
    buildTaskRow("packets", packetTaskHandle);

    append("╚══════════╩═══════╩══════╩══════════════════╝\n");

    // ── Print to Serial under mutex ───────────────────────────────────────
    if (xSemaphoreTake(serial_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        Serial.print(system_health);   // single call — entire table as one string
        xSemaphoreGive(serial_mutex);
    }

    // ── Append to log file ────────────────────────────────────────────────
    append_snapshot_to_log();
}


char temps_log[256];
char humis_log[256];

float aggregate_sensor_temperatures() {

    float avg_temp = 0.0f;

    active_sensors = 0;

    highest_temperature = -100.0f;
    lowest_temperature  = 200.0f;

    highest_humidity = -100.0f;
    lowest_humidity  = 200.0f;

    float total_temp = 0.0f;
    float total_humi = 0.0f;
    int active_humi_sensors = 0;  // NEW: track humidity sensors separately
    
    /*
    Serial.print("Sensor 2 Temp: "); Serial.println(sensor_2_temp);
    Serial.print("Sensor 8 Temp: "); Serial.println(sensor_8_temp);
    Serial.print("Sensor 12 Temp: "); Serial.println(sensor_12_temp);
    */

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

        // Check if temperature is valid (always true for your sensors)
        bool temp_valid = (!isnan(t) && t > 1.0f);
        
        // Check if humidity is valid separately (must be >1 and not NaN, and not -1 error code)
        bool humi_valid = (!isnan(h) && h > 1.0f && h != -1.0f);

        if(temp_valid || humi_valid){ 
            valid_sensor[i] = true;

            // Handle temperature (always present)
            if(temp_valid) {
                total_temp += t;
                active_sensors++;  // Count for temperature-based active sensors
                
                if(t > highest_temperature) highest_temperature = t;
                if(t < lowest_temperature)  lowest_temperature  = t;
            }

            // Handle humidity independently
            if(humi_valid) {
                total_humi += h;
                active_humi_sensors++;
                
                if(h > highest_humidity) highest_humidity = h;
                if(h < lowest_humidity)  lowest_humidity  = h;
            }
        }
    }

    if(active_sensors == 0){
        return NAN;
    }

    avg_temp = total_temp / active_sensors;
    
    // Only calculate average humidity if we have at least one valid humidity reading
    if(active_humi_sensors > 0) {
        average_humidity = total_humi / active_humi_sensors;
    } else {
        average_humidity = NAN;  // Will become null in JSON
    }

    temperature_range = highest_temperature - lowest_temperature;
    humidity_range    = highest_humidity - lowest_humidity;

    // PASS 2 — detect deviations and dead zones
    for(int i = 0; i < 11; i++){

        if(!valid_sensor[i]) continue;

        float t = temperature_readings[i];
        float h = humidity_readings[i];
        
        bool temp_valid = (!isnan(t) && t > 1.0f);
        bool humi_valid = (!isnan(h) && h > 1.0f && h != -1.0f);

        // Only calculate temperature deviation if temperature is valid
        if(temp_valid) {
            temperature_deviation = t - average_temperature;
            
            // Dead-zone detection (requires both temp and humidity to be valid)
            if(humi_valid && temperature_deviation < -2.0f && (h - average_humidity) > 5.0f){
                snprintf(temps_log, sizeof(temps_log), "Dead zone suspected near sensor %u", i+1);
                Serial.println(temps_log);
            }
            
            // Sensor malfunction detection - temperature only
            if(abs(temperature_deviation) > 8.0f){
                snprintf(temps_log, sizeof(temps_log), "Temperature sensor deviation too large at sensor %u", i+1);
                Serial.println(temps_log);
            }
        }
        
        // Humidity deviation check - only if humidity is valid
        if(humi_valid) {
            humidity_deviation = h - average_humidity;
            
            if(abs(humidity_deviation) > 15.0f){
                snprintf(humis_log, sizeof(humis_log), "Humidity sensor deviation too large at sensor %u", i+1);
                Serial.println(humis_log);
            }
        }
    }

    return avg_temp;
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
    float avg_humi = total_humidity / active_sensors;
    return avg_humi;
}



