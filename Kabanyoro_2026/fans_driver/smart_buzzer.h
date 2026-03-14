#ifndef SMART_BUZZER_H
#define SMART_BUZZER_H

#include "Arduino.h"

class Buzzer {
  private:
    int pin;
    bool buzzerOn = false;
    uint32_t lastToggle = 0;
    uint8_t beepsLeft = 0;
    uint16_t beepDuration = 100; // default ms
    uint16_t beepGap = 150;      // default ms
    
    // RTOS integration
    TaskHandle_t taskHandle = NULL;
    bool taskCreated = false;

  public:
    Buzzer(int pin = 14); // default pin 14 for buzzer unless specified otherwise
    void begin();
    void beep(uint8_t times, uint16_t duration = 100, uint16_t gap = 150);
    void update();
    
    // RTOS methods
    void createTask(UBaseType_t priority = 4, uint16_t stackSize = 2048, BaseType_t core = 1);
    void stopTask();  // Suspend the task
    void resumeTask(); // Resume the task if needed
    
    // Status
    bool isBuzzing() const { return beepsLeft > 0 || buzzerOn; }
    TaskHandle_t getTaskHandle() const { return taskHandle; }
};

#endif