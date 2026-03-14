#include "smart_buzzer.h"

Buzzer::Buzzer(int p) {
  pin = p;
  taskHandle = NULL;
}

void Buzzer::begin() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Buzzer::beep(uint8_t times, uint16_t duration, uint16_t gap) {
  if (times == 0) return;
  
  // Store the beep pattern
  beepsLeft = times;
  beepDuration = duration;
  beepGap = gap;
  
  // Start the first beep
  buzzerOn = true;
  lastToggle = millis();
  digitalWrite(pin, HIGH);
  
  // Notify the task to wake up if it exists
  if (taskHandle != NULL) {
    xTaskNotifyGive(taskHandle);
  }
}

void Buzzer::update() {
  if (beepsLeft == 0 && !buzzerOn) return;

  uint32_t now = millis();
  uint32_t elapsed = now - lastToggle;

  if (buzzerOn && elapsed >= beepDuration) {
    digitalWrite(pin, LOW);
    buzzerOn = false;
    lastToggle = now;
    
    // If this was the last beep and no more left, we're done
    if (beepsLeft == 0) {
      // Task will suspend itself in the next iteration
    }
  }
  else if (!buzzerOn && elapsed >= beepGap && beepsLeft > 0) {
    digitalWrite(pin, HIGH);
    buzzerOn = true;
    lastToggle = now;
    beepsLeft--;
  }
}

// Static task function
static void buzzerTaskFunction(void* parameter) {
  Buzzer* buzzer = static_cast<Buzzer*>(parameter);
  uint32_t notificationValue;
  
  while (true) {
    // Wait for notification with timeout
    // If we get a notification, we start buzzing
    // If not, we'll timeout and check if we need to suspend
    if (xTaskNotifyWait(0, 0, &notificationValue, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Got notification - we're buzzing
    }
    
    // Update the buzzer
    buzzer->update();
    
    // If we're done buzzing, suspend the task
    if (!buzzer->isBuzzing()) {
      // Small delay to ensure final state is set
      vTaskDelay(pdMS_TO_TICKS(10));
      
      // Suspend this task
      vTaskSuspend(NULL);
    }
  }
}

void Buzzer::createTask(UBaseType_t priority, uint16_t stackSize, BaseType_t core) {
  if (taskCreated) return;
  
  BaseType_t result = xTaskCreatePinnedToCore(
    buzzerTaskFunction,
    "BuzzerTask",
    stackSize,
    this,
    priority,
    &taskHandle,
    core
  );
  
  if (result == pdPASS) {
    taskCreated = true;
    // Start with task suspended
    vTaskSuspend(taskHandle);
  }
}

void Buzzer::stopTask() {
  if (taskHandle != NULL) {
    vTaskSuspend(taskHandle);
  }
}

void Buzzer::resumeTask() {
  if (taskHandle != NULL && isBuzzing()) {
    vTaskResume(taskHandle);
  }
}