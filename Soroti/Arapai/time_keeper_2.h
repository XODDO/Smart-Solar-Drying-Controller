#ifndef TIME_KEEPER_2_H
#define TIME_KEEPER_2_H

#include <Arduino.h>
#include "Wire.h"
#include <RTClib.h>
#include "WiFi_Manager.h"


#include "time.h"

class TimerKeeper {
private:
    // Private member variables
    uint8_t hr, mint, sec, day_, mth;
    uint16_t mwaka;
    bool time_set;
    bool clock_is_working;
    float rtc_temperature;
    uint8_t last_hourly_marker = 255;  // 255 = never fired


     uint32_t last_reinit_check = 0;
    uint32_t last_successful_read = 0;
    uint8_t consecutive_failures = 0;

    bool rtc_available;          // true if RTC is present and working
    bool internal_time_valid;    // true if software time has been set
    time_t base_epoch;           // epoch seconds at base_micros
    uint64_t base_micros;        // esp_timer_get_time() at base_epoch
    struct tm internal_tm;       // current time as tm (for software mode)
    
    
    // RTC object
    RTC_DS3231 real_time;
    
    // Constants for date formatting
    static const char * DAYS_OF_WEEK[];
    static const char * MONTHS_FULL[];
    static const char * MONTHS_SHORT[];
    
    // New: Minute tracking for interval flags
    uint8_t lastCheckedMinute;
    uint8_t last5MinuteMarker;
    uint8_t last10MinuteMarker;
    uint8_t last30MinuteMarker;
    
    // Private helper methods
    const char* getOrdinalSuffix(uint8_t day);
    void formatTime12Hour(uint8_t hour, uint8_t minute, uint8_t second);
    void formatLongDate(uint8_t dayOfWeek, uint8_t day, uint8_t month, uint16_t year);
    void formatShortDate(uint8_t day, uint8_t month, uint16_t year);
    bool validateRTCTime(uint8_t hour, uint8_t minute, uint8_t second);
    void handleRTCFailure();
    bool validateTemperature(float temp);

public:
    bool apply_ntp_time(const struct tm& timeinfo);
    uint64_t last_ntp_sync_us;   // for periodic NTP sync in software mode

    char SystemTime[12];   // HH:MM:SS
        // Date/Time string buffers
    char ShortTime[10] = "12:45";    // HH:MM am/pm
    char SystemDate[30] = "Sunday 24th May 2026";   // Mon 5th July, 2026
    char ShortDate[16] = "24 May 2026";    // 15 Sept, 2026

    float temp_reading = 0.0f;
    // Constructor
    TimerKeeper();
    
    // Public methods
    bool initialize_RTC(const uint8_t i2c_dio, const uint8_t i2c_clock);
    void query_rtc();
    void update();  // Alternative name for query_rtc
    void periodic_rtc_health_check();

    void setInternalTime(const struct tm& timeinfo);
    void updateInternalTime();   // recompute from esp_timer
    bool isRtcAvailable() const { return rtc_available; }
    bool isInternalTimeValid() const { return internal_time_valid; }
    void periodicNtpSync(WiFi_Manager& wifi, long gmtOffset, int daylightOffset);
    
    // Getters for time components
    uint8_t getCurrentHour()   const { return hr; }
    uint8_t getCurrentMinute() const { return mint; }
    uint8_t getCurrentSecond() const { return sec; }
    uint8_t getCurrentDay()    const { return day_; }
    uint8_t getCurrentMonth()  const { return mth; }
    uint16_t getCurrentYear()  const { return mwaka; }
    
    // Getters for formatted strings
    const char* getTime12Hour() const { return ShortTime; }
    const char* getTime24Hour() const { return SystemTime; }
    const char* getLongDate() const { return SystemDate; }
    const char* getShortDate() const { return ShortDate; }
    
    // Temperature methods
    float getRTCTemperature() const { return rtc_temperature; }
    bool isTemperatureValid() const { return !isnan(rtc_temperature) && rtc_temperature > -50.0f && rtc_temperature < 80.0f; }
    
    // Utility methods
    void getTimestamp(char* buffer, size_t bufferSize) const;
    bool isNewMinute();
    bool isClockWorking() const { return clock_is_working; }
    
    // NEW: Time setting methods
    bool setTime(uint16_t year, uint8_t month, uint8_t day, 
                 uint8_t hour, uint8_t minute, uint8_t second);
    bool setTimeFromCompileTime();
    bool setTimeFromUnixTimestamp(uint32_t timestamp);
    
    // NEW: Minute interval flags
    bool is5MinuteMarker();   // Returns true at 5,10,15,20...55 minutes
    bool is10MinuteMarker();  // Returns true at 10,20,30,40,50 minutes
    bool is30MinuteMarker();  // Returns true at 30,00 minutes
    
    bool isHourlyMarker();    // Returns true at 00 minutes
    bool isIntervalMinute(uint8_t interval); // Generic interval checker
    
    // NEW: Check if current minute is a multiple of given interval
    bool isMinuteMultiple(uint8_t interval) const;
    
    // NEW: Time until next marker (in seconds)
    uint16_t secondsToNext5Minute() const;
    uint16_t secondsToNext10Minute() const;
    uint16_t secondsToNextInterval(uint8_t interval) const;

    void setLastNtpSyncTime(uint64_t us) { last_ntp_sync_us = us; }
};

#endif