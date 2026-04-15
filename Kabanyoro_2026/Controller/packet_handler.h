#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include "buzzer.h"
#include "time_keeper_2.h"

// External global variables (defined in main.cpp)
// ~1kB in RAM for sensor data

extern float sensor_1_temp, sensor_2_temp, sensor_3_temp, sensor_4_temp, sensor_5_temp, sensor_6_temp;
extern float sensor_7_temp, sensor_8_temp, sensor_9_temp, sensor_10_temp, sensor_11_temp, sensor_12_temp;
extern float sensor_1_humidity, sensor_2_humidity, sensor_3_humidity, sensor_4_humidity, sensor_5_humidity, sensor_6_humidity;
extern float sensor_7_humidity, sensor_8_humidity, sensor_9_humidity, sensor_10_humidity, sensor_11_humidity, sensor_12_humidity;
extern float sensor_1_pressure, sensor_2_pressure, sensor_3_pressure, sensor_4_pressure, sensor_5_pressure, sensor_6_pressure;
extern float sensor_7_pressure, sensor_8_pressure, sensor_9_pressure, sensor_10_pressure, sensor_11_pressure, sensor_12_pressure;

extern float sensor_1_elevation, sensor_2_elevation, sensor_3_elevation, sensor_4_elevation, sensor_5_elevation, sensor_6_elevation;
extern float sensor_7_elevation, sensor_8_elevation, sensor_9_elevation, sensor_10_elevation, sensor_11_elevation, sensor_12_elevation;
//extern float sensor_1_h_index, sensor_2_h_index, sensor_3_h_index, sensor_4_h_index, sensor_5_h_index, sensor_6_h_index;
//extern float sensor_7_h_index, sensor_8_h_index, sensor_9_h_index, sensor_10_h_index, sensor_11_h_index, sensor_12_h_index;

extern uint64_t sensor_1_running_time_ms, sensor_2_running_time_ms, sensor_3_running_time_ms, sensor_4_running_time_ms;
extern uint64_t sensor_5_running_time_ms, sensor_6_running_time_ms, sensor_7_running_time_ms, sensor_8_running_time_ms;
extern uint64_t sensor_9_running_time_ms, sensor_10_running_time_ms, sensor_11_running_time_ms, sensor_12_running_time_ms;
extern uint64_t sensor_1_last_seen_ms, sensor_2_last_seen_ms, sensor_3_last_seen_ms, sensor_4_last_seen_ms;
extern uint64_t sensor_5_last_seen_ms, sensor_6_last_seen_ms, sensor_7_last_seen_ms, sensor_8_last_seen_ms;
extern uint64_t sensor_9_last_seen_ms, sensor_10_last_seen_ms, sensor_11_last_seen_ms, sensor_12_last_seen_ms;

extern char sensor_1_last_seen[12], sensor_2_last_seen[12], sensor_3_last_seen[12], sensor_4_last_seen[12];
extern char sensor_5_last_seen[12], sensor_6_last_seen[12], sensor_7_last_seen[12], sensor_8_last_seen[12];
extern char sensor_9_last_seen[12], sensor_10_last_seen[12], sensor_11_last_seen[12], sensor_12_last_seen[12];

/*
extern char sensor_1_CPU_freq[12], sensor_2_CPU_freq[12], sensor_3_CPU_freq[12], sensor_4_CPU_freq[12];
extern char sensor_5_CPU_freq[12], sensor_6_CPU_freq[12], sensor_7_CPU_freq[12], sensor_8_CPU_freq[12];
extern char sensor_9_CPU_freq[12], sensor_10_CPU_freq[12], sensor_11_CPU_freq[12], sensor_12_CPU_freq[12];
*/

// External RTC time string (from your main code)
extern char SystemTime[20];  // Format "HH:MM:SS"

// Packet structure that matches your sender ESPs
typedef struct __attribute__((packed)) {
    uint8_t sensor_id;           // 1-12 from S_ID of T_1 to T_12
    float temperature;
    float humidity;
    float pressure;
    float elevation;
    float heat_index;
    char transmissions[9]; 
    char CPU_Freq[12];
    char last_seen[12];          // Will be set on receiver
    uint64_t last_seen_ms;        // Will be set on receiver
    uint64_t time_on_in_ms;       // Time the sender has been on
    uint8_t rssi;                 // Signal strength
} ReceivedPacket;

// Structure for queuing raw JSON data
typedef struct {
    uint8_t data[256];            // Raw JSON data
    int length;                    // Length of data
} QueuedJsonPacket;

class PacketHandler {
private:
    // Queue handles
    QueueHandle_t _jsonPacketQueue;
    TaskHandle_t _packetTaskHandle;

    
    // Task parameters
    static const int QUEUE_SIZE = 20;
    static const int PACKET_STACK_SIZE = 4096; //8192;//16384; // 16KB,  it only uses up to 1,816
    static const int PACKET_TASK_PRIORITY = 3;
    static const int PACKET_TASK_CORE = 0;
    
    // Statistics
    uint64_t _packet_loss_counter;
    uint64_t _total_packets_received;
    
    // Private methods
    static void packetHandlerTask(void *pvParams);
    static void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);
    int extractSensorNumber(const char *identity);
    void parseJsonPacket(const uint8_t *data, int len, ReceivedPacket *packet);
    void extractReadingsFromPacket(ReceivedPacket *packet);
    
    // Static instance pointer for callback
    static PacketHandler* _instance;
    
public:

// instantiations
//esp_now_peer_info_t peerInfo;
    
    PacketHandler();
    bool begin();
    void shutdown();  // to enable a proper task suspension when needed
    void printStatistics();
    uint64_t getPacketLossCount() { return _packet_loss_counter; }
    uint64_t getTotalPacketsReceived() { return _total_packets_received; }
};

#endif