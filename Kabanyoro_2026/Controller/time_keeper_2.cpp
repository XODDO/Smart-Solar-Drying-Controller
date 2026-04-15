#include "time_keeper_2.h"

// Initialize static constants
const char* TimerKeeper::DAYS_OF_WEEK[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char* TimerKeeper::MONTHS_FULL[] = {"January", "February", "March", "April", "May", "June", 
                                        "July", "August", "September", "October", "November", "December"};
const char* TimerKeeper::MONTHS_SHORT[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                                         "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// Constructor
TimerKeeper::TimerKeeper() {
    hr = 0; mint = 0; sec = 0; day_ = 0; mth = 0;
    mwaka = 2000;
    time_set = false;
    clock_is_working = false;
    rtc_temperature = NAN;
    
    // Initialize minute tracking
    lastCheckedMinute = 0;
    last5MinuteMarker = 0;
    last10MinuteMarker = 0;
    last30MinuteMarker = 0;

    
    // Initialize string buffers
    ShortTime[0] = '\0';
    SystemTime[0] = '\0';
    SystemDate[0] = '\0';
    ShortDate[0] = '\0';
}

const char* TimerKeeper::getOrdinalSuffix(uint8_t day) {
    if (day == 1 || day == 21 || day == 31) return "st";
    if (day == 2 || day == 22) return "nd";
    if (day == 3 || day == 23) return "rd";
    return "th";
}

bool TimerKeeper::validateTemperature(float temp) {
    if (isnan(temp) || isinf(temp)) {
        return false;
    }
    if (temp < -50.0f || temp > 80.0f) {
        return false;
    }
    return true;
}

void TimerKeeper::formatTime12Hour(uint8_t hour, uint8_t minute, uint8_t second) {
    char hourStr[3], minuteStr[3], secondStr[3];
    
    uint8_t displayHour = (hour == 0 || hour == 12) ? 12 : hour % 12;
    
    snprintf(hourStr, sizeof(hourStr), "%02d", displayHour);
    snprintf(minuteStr, sizeof(minuteStr), "%02d", minute);
    snprintf(secondStr, sizeof(secondStr), "%02d", second);
    
    snprintf(ShortTime, sizeof(ShortTime), "%s:%s %s", 
             hourStr, minuteStr, (hour < 12) ? "am" : "pm");
    snprintf(SystemTime, sizeof(SystemTime), "%s:%s:%s", 
             hourStr, minuteStr, secondStr);
}

void TimerKeeper::formatLongDate(uint8_t dayOfWeek, uint8_t day, uint8_t month, uint16_t year) {
    const char* ordinal = getOrdinalSuffix(day);
    snprintf(SystemDate, sizeof(SystemDate), "%s %d%s %s %d", 
             DAYS_OF_WEEK[dayOfWeek], day, ordinal, MONTHS_FULL[month-1], year);
}

void TimerKeeper::formatShortDate(uint8_t day, uint8_t month, uint16_t year) {
    snprintf(ShortDate, sizeof(ShortDate), "%d %s %d", day, MONTHS_SHORT[month-1], year);
}

bool TimerKeeper::validateRTCTime(uint8_t hour, uint8_t minute, uint8_t second) {
    return (hour <= 23 && minute <= 59 && second <= 59);
}

void TimerKeeper::handleRTCFailure() {
    Serial.println("RTC Chip Failed! Using fallback values.");
    hr = 23; mint = 59; sec = 59;
    rtc_temperature = NAN;
}

bool TimerKeeper::initialize_RTC() {
    uint8_t trial = 0;
    while (!real_time.begin()) {
        Serial.println("Couldn't find RTC");
        if (++trial >= 3) break;
        delay(100);
        clock_is_working = false;
    }
    
    if (real_time.begin()) {
        clock_is_working = true;
        Serial.println("RTC initialized successfully");
        
        if (real_time.lostPower()) {
            Serial.println("RTC lost power, setting compile time...");
            setTimeFromCompileTime();
        }
        
        float testTemp = real_time.getTemperature();
        if (validateTemperature(testTemp)) {
            Serial.printf("RTC Temperature sensor working: %.2f°C\n", testTemp);
        } else {
            Serial.println("RTC Temperature sensor reading invalid");
        }
        
    } else {
        Serial.println("RTC initialization failed");
        clock_is_working = false;
    }

    return clock_is_working;
}

void TimerKeeper::query_rtc() {
    if (!clock_is_working) {
        Serial.println("RTC not working!");
        rtc_temperature = NAN;
        return;
    }
    
    DateTime time_now = real_time.now();
    
    // Extract time components
    hr = time_now.hour();
    mint = time_now.minute();
    sec = time_now.second();
    uint8_t date_day = time_now.day();
    mth = time_now.month();
    mwaka = time_now.year();
    day_ = time_now.dayOfTheWeek();
    
    // Read temperature
     temp_reading = real_time.getTemperature();
    if (validateTemperature(temp_reading)) {
        rtc_temperature = temp_reading;
    } else {
        rtc_temperature = NAN;
        Serial.println("RTC temperature reading invalid");
    }
    
    if (!validateRTCTime(hr, mint, sec)) {
        handleRTCFailure();
        return;
    }
    
    // Format strings
    formatTime12Hour(hr, mint, sec);
    formatLongDate(day_, date_day, mth, mwaka);
    formatShortDate(date_day, mth, mwaka);
}

void TimerKeeper::update() {
    query_rtc();
}

void TimerKeeper::getTimestamp(char* buffer, size_t bufferSize) const {
    snprintf(buffer, bufferSize, "[%s %s]", ShortDate, SystemTime);
}

bool TimerKeeper::isNewMinute() {
    static uint8_t lastMinute = 0;
    bool newMinute = (mint != lastMinute);
    lastMinute = mint;
    return newMinute;
}

// NEW: Time setting methods
bool TimerKeeper::setTime(uint16_t year, uint8_t month, uint8_t day, 
                         uint8_t hour, uint8_t minute, uint8_t second) {
    if (!clock_is_working) {
        Serial.println("Cannot set time - RTC not working");
        return false;
    }
    
    // Validate input
    if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour > 23 || minute > 59 || second > 59) {
        Serial.println("Invalid time parameters");
        return false;
    }
    
    real_time.adjust(DateTime(year, month, day, hour, minute, second));
    Serial.println("Time set manually");
    
    // Update current values
    query_rtc();
    return true;
}

bool TimerKeeper::setTimeFromCompileTime() {
    if (!clock_is_working) return false;
    
    real_time.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Time set from compile time");
    
    query_rtc();
    return true;
}

bool TimerKeeper::setTimeFromUnixTimestamp(uint32_t timestamp) {
    if (!clock_is_working) return false;
    
    real_time.adjust(DateTime(timestamp));
    Serial.println("Time set from Unix timestamp");
    
    query_rtc();
    return true;
}

// NEW: Minute interval flag methods
bool TimerKeeper::isMinuteMultiple(uint8_t interval) const {
    if (interval == 0) return false;
    return (mint % interval == 0);
}

bool TimerKeeper::is5MinuteMarker() {
    bool isMarker = (mint % 5 == 0);
    
    // Only trigger once per marker
    if (isMarker && last5MinuteMarker != mint) {
        last5MinuteMarker = mint;
        return true;
    }
    return false;
}

bool TimerKeeper::is10MinuteMarker() {
    bool isMarker = (mint % 10 == 0);
    
    // Only trigger once per marker
    if (isMarker && last10MinuteMarker != mint) {
        last10MinuteMarker = mint;
        return true;
    }
    return false;
}


bool TimerKeeper::is30MinuteMarker() {
    // Returns true for 30 seconds after each 30-minute mark
   // bool isMarker = (mint == 0 || mint == 30);
    bool isMarker = (mint%30 == 0);
    uint8_t sec_threshold = 59;  // true for 59-seconds

    if (isMarker && last30MinuteMarker != mint) {
        last30MinuteMarker = mint;
        return true;
    }
    /*
    bool isMarker = (mint % 30 == 0);
    
    // Only trigger once per marker
    if (isMarker && last30MinuteMarker != mint) {
        last30MinuteMarker = mint;
        return true;
    }
    */

    return false;
}

// also correctly handles the edge case at midnight (hr rolls from 23 → 0)
bool TimerKeeper::isHourlyMarker() {
    if(mint != 0) return false; // only turns at the zeroth minute
    
    if (last_hourly_marker != hr) {
        last_hourly_marker = hr;
        return true;
    }
    return false;
}



bool TimerKeeper::isIntervalMinute(uint8_t interval) {
    static uint8_t lastMarker[11] = {0}; // Support intervals up to 10
    if (interval < 1 || interval > 10) return false;
    
    bool isMarker = (mint % interval == 0);
    if (isMarker && lastMarker[interval] != mint) {
        lastMarker[interval] = mint;
        return true;
    }
    return false;
}

uint16_t TimerKeeper::secondsToNext5Minute() const {
    return secondsToNextInterval(5);
}

uint16_t TimerKeeper::secondsToNext10Minute() const {
    return secondsToNextInterval(10);
}

uint16_t TimerKeeper::secondsToNextInterval(uint8_t interval) const {
    uint8_t minutesToNext = interval - (mint % interval);
    if (minutesToNext == interval) minutesToNext = 0;
    
    return (minutesToNext * 60) - sec;
}

bool TimerKeeper::apply_ntp_time(const struct tm& timeinfo) {
    if (!clock_is_working) {
        Serial.println("[RTC] Cannot apply NTP - RTC not ready.");
        return false;
    }

    real_time.adjust(DateTime(
        timeinfo.tm_year + 1900,
        timeinfo.tm_mon  + 1,
        timeinfo.tm_mday,
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec
    ));

    query_rtc();  // Refresh sawa's internal fields
    Serial.println("[RTC] Time applied from NTP successfully.");
    return true;
}

void TimerKeeper::periodic_rtc_health_check() {
        uint32_t now = millis();
        
        // Check RTC health every hour
        if (now - last_reinit_check >= 3600000) {
            last_reinit_check = now;
            
            // Attempt to read RTC
            DateTime test_time = real_time.now();
            
            // Validate the read
            if (test_time.year() < 2020 || test_time.year() > 2100) {
                consecutive_failures++;
                Serial.printf("[RTC] Invalid read (year=%d), failure %d\n", 
                             test_time.year(), consecutive_failures);
                
                if (consecutive_failures >= 3) {
                    Serial.println("[RTC] Multiple failures, reinitializing...");
                    clock_is_working = false;
                    initialize_RTC();
                    consecutive_failures = 0;
                }
            } else {
                consecutive_failures = 0;
                if (!clock_is_working) {
                    Serial.println("[RTC] RTC recovered!");
                    clock_is_working = true;
                }
            }
        }
    }
