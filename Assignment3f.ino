#include <Arduino.h>

extern "C" {
  #include "workkernel.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ============================================================
// Pin mapping - based on your current wiring
// ============================================================
static const int PIN_SYNC    = 25;   // button A: SYNC
static const int PIN_IN_S    = 33;   // button B: sporadic trigger
static const int PIN_IN_MODE = 18;   // button C: mode toggle

// ACK outputs
static const int PIN_ACK_A   = 14;
static const int PIN_ACK_B   = 27;
static const int PIN_ACK_AGG = 26;
static const int PIN_ACK_C   = 13;
static const int PIN_ACK_D   = 12;
static const int PIN_ACK_S   = 15;

static const int PIN_ERR     = 2;    // onboard LED or optional error indicator

// ============================================================
// Timing / budgets
// ============================================================
static const uint32_t PERIOD_A_MS   = 10;
static const uint32_t PERIOD_B_MS   = 20;
static const uint32_t PERIOD_AGG_MS = 20;
static const uint32_t PERIOD_C_MS   = 50;
static const uint32_t PERIOD_D_MS   = 50;
static const uint32_t DEADLINE_S_MS = 30;

static const uint32_t BUDGET_A_CYCLES   = 672000u;
static const uint32_t BUDGET_B_CYCLES   = 960000u;
static const uint32_t BUDGET_AGG_CYCLES = 480000u;
static const uint32_t BUDGET_C_CYCLES   = 1680000u;
static const uint32_t BUDGET_D_CYCLES   = 960000u;
static const uint32_t BUDGET_S_CYCLES   = 600000u;

static const uint32_t FINAL_REPORT_AFTER_MS = 10000u;  // 10 seconds

// ============================================================
// Global state
// ============================================================
static volatile bool g_started = false;
static uint32_t g_t0_us = 0;
static TickType_t g_t0_tick = 0;

// Latest tokens published by A and B
static uint32_t g_tokenA = 0;
static uint32_t g_tokenB = 0;
static bool g_tokenA_valid = false;
static bool g_tokenB_valid = false;

// Shared mode state: 0 or 1
static volatile uint32_t g_modeState = 0;

// Protect shared token state
static portMUX_TYPE g_tokenMux = portMUX_INITIALIZER_UNLOCKED;

// Protect serial output
static SemaphoreHandle_t g_serialMutex = nullptr;

// Queue for sporadic release timestamps
static QueueHandle_t g_sQueue = nullptr;

// ISR debounce / min inter-arrival
static volatile uint32_t g_lastSReleaseUs = 0;
static volatile uint32_t g_lastModeToggleUs = 0;

// Task handles
static TaskHandle_t g_taskAHandle        = nullptr;
static TaskHandle_t g_taskBHandle        = nullptr;
static TaskHandle_t g_taskAGGHandle      = nullptr;
static TaskHandle_t g_taskCHandle        = nullptr;
static TaskHandle_t g_taskDHandle        = nullptr;
static TaskHandle_t g_taskSHandle        = nullptr;
static TaskHandle_t g_taskModeDbgHandle  = nullptr;
static TaskHandle_t g_taskReportHandle   = nullptr;

// Use one core only
static const BaseType_t APP_CORE = 1;

// ============================================================
// Simple summary counters
// ============================================================
static volatile uint32_t g_jobsA   = 0;
static volatile uint32_t g_jobsB   = 0;
static volatile uint32_t g_jobsAGG = 0;
static volatile uint32_t g_jobsC   = 0;
static volatile uint32_t g_jobsD   = 0;
static volatile uint32_t g_jobsS   = 0;

static volatile uint32_t g_missA   = 0;
static volatile uint32_t g_missB   = 0;
static volatile uint32_t g_missAGG = 0;
static volatile uint32_t g_missC   = 0;
static volatile uint32_t g_missD   = 0;
static volatile uint32_t g_missS   = 0;

// ============================================================
// Helpers
// ============================================================
static inline uint32_t now_us() {
  return (uint32_t)micros();
}

static inline void ack_high(int pin) {
  digitalWrite(pin, HIGH);
}

static inline void ack_low(int pin) {
  digitalWrite(pin, LOW);
}

static void safe_println(const String &msg) {
  if (g_serialMutex != nullptr) {
    xSemaphoreTake(g_serialMutex, portMAX_DELAY);
    Serial.println(msg);
    xSemaphoreGive(g_serialMutex);
  } else {
    Serial.println(msg);
  }
}

static void safe_print_triplet(const char *name, uint32_t id, uint32_t x, uint32_t tok) {
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.print(name);
  Serial.print(",");
  Serial.print(id);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.println(tok);
  xSemaphoreGive(g_serialMutex);
}

static void safe_print_pair(const char *name, uint32_t id, uint32_t tok) {
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.print(name);
  Serial.print(",");
  Serial.print(id);
  Serial.print(",");
  Serial.println(tok);
  xSemaphoreGive(g_serialMutex);
}

static void safe_print_mode_pair(const char *name, uint32_t id, uint32_t modeNow, uint32_t tok) {
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.print(name);
  Serial.print(",");
  Serial.print(id);
  Serial.print(",mode=");
  Serial.print(modeNow);
  Serial.print(",");
  Serial.println(tok);
  xSemaphoreGive(g_serialMutex);
}

static void safe_print_mode_skip(const char *name, uint32_t id) {
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.print(name);
  Serial.print(",");
  Serial.print(id);
  Serial.println(",skipped_mode=0");
  xSemaphoreGive(g_serialMutex);
}

static void set_error_and_log(const char *taskName, uint32_t id, int32_t lateness_us) {
  digitalWrite(PIN_ERR, HIGH);
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.print("MISS,");
  Serial.print(taskName);
  Serial.print(",");
  Serial.print(id);
  Serial.print(",");
  Serial.println(lateness_us);
  xSemaphoreGive(g_serialMutex);
}

// ============================================================
// ISR for sporadic task S
// ============================================================
void IRAM_ATTR on_sporadic_rise() {
  if (!g_started) return;

  const uint32_t t = (uint32_t)micros();

  if ((int32_t)(t - g_lastSReleaseUs) < 30000) {
    return;
  }

  g_lastSReleaseUs = t;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (g_sQueue != nullptr) {
    xQueueSendFromISR(g_sQueue, &t, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================
// ISR for MODE toggle
// ============================================================
void IRAM_ATTR on_mode_rise() {
  if (!g_started) return;

  const uint32_t t = (uint32_t)micros();

  // debounce ~150ms
  if ((int32_t)(t - g_lastModeToggleUs) < 150000) {
    return;
  }

  g_lastModeToggleUs = t;
  g_modeState ^= 1u;
}

// ============================================================
// Job bodies
// ============================================================
static void runTaskA(uint32_t id) {
  const uint32_t release_us = g_t0_us + id * PERIOD_A_MS * 1000u;
  const uint32_t countA = 0;   // no tester in demo setup
  const uint32_t seed = (id << 16) ^ countA ^ 0xA1u;

  ack_high(PIN_ACK_A);
  const uint32_t token = WorkKernel(BUDGET_A_CYCLES, seed);
  ack_low(PIN_ACK_A);

  portENTER_CRITICAL(&g_tokenMux);
  g_tokenA = token;
  g_tokenA_valid = true;
  portEXIT_CRITICAL(&g_tokenMux);

  g_jobsA++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + PERIOD_A_MS * 1000u);
  if (lateness > 0) {
    g_missA++;
    set_error_and_log("A", id, lateness);
  }

  safe_print_triplet("A", id, countA, token);
}

static void runTaskB(uint32_t id) {
  const uint32_t release_us = g_t0_us + id * PERIOD_B_MS * 1000u;
  const uint32_t countB = 0;   // no tester in demo setup
  const uint32_t seed = (id << 16) ^ countB ^ 0xB2u;

  ack_high(PIN_ACK_B);
  const uint32_t token = WorkKernel(BUDGET_B_CYCLES, seed);
  ack_low(PIN_ACK_B);

  portENTER_CRITICAL(&g_tokenMux);
  g_tokenB = token;
  g_tokenB_valid = true;
  portEXIT_CRITICAL(&g_tokenMux);

  g_jobsB++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + PERIOD_B_MS * 1000u);
  if (lateness > 0) {
    g_missB++;
    set_error_and_log("B", id, lateness);
  }

  safe_print_triplet("B", id, countB, token);
}

static void runTaskAGG(uint32_t id) {
  const uint32_t release_us = g_t0_us + id * PERIOD_AGG_MS * 1000u;

  uint32_t tokA = 0;
  uint32_t tokB = 0;
  bool validA = false;
  bool validB = false;

  portENTER_CRITICAL(&g_tokenMux);
  tokA = g_tokenA;
  tokB = g_tokenB;
  validA = g_tokenA_valid;
  validB = g_tokenB_valid;
  portEXIT_CRITICAL(&g_tokenMux);

  const uint32_t agg = (validA && validB) ? (tokA ^ tokB) : 0xDEADBEEFu;
  const uint32_t seed = (id << 16) ^ agg ^ 0xD4u;

  ack_high(PIN_ACK_AGG);
  const uint32_t tok = WorkKernel(BUDGET_AGG_CYCLES, seed);
  ack_low(PIN_ACK_AGG);

  g_jobsAGG++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + PERIOD_AGG_MS * 1000u);
  if (lateness > 0) {
    g_missAGG++;
    set_error_and_log("AGG", id, lateness);
  }

  safe_print_triplet("AGG", id, agg, tok);
}

static void runTaskC(uint32_t id) {
  const uint32_t release_us = g_t0_us + id * PERIOD_C_MS * 1000u;
  const uint32_t seed = (id << 16) ^ 0xC3u;
  const uint32_t modeNow = g_modeState;

  ack_high(PIN_ACK_C);
  const uint32_t tok = WorkKernel(BUDGET_C_CYCLES, seed);
  ack_low(PIN_ACK_C);

  g_jobsC++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + PERIOD_C_MS * 1000u);
  if (lateness > 0) {
    g_missC++;
    set_error_and_log("C", id, lateness);
  }

  safe_print_mode_pair("C", id, modeNow, tok);
}

static void runTaskD(uint32_t id) {
  const uint32_t release_us = g_t0_us + id * PERIOD_D_MS * 1000u;
  const uint32_t seed = (id << 16) ^ 0xD5u;
  const uint32_t modeNow = g_modeState;

  ack_high(PIN_ACK_D);
  const uint32_t tok = WorkKernel(BUDGET_D_CYCLES, seed);
  ack_low(PIN_ACK_D);

  g_jobsD++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + PERIOD_D_MS * 1000u);
  if (lateness > 0) {
    g_missD++;
    set_error_and_log("D", id, lateness);
  }

  safe_print_mode_pair("D", id, modeNow, tok);
}

static void runTaskS(uint32_t id, uint32_t release_us) {
  const uint32_t seed = (id << 16) ^ 0x55u;

  ack_high(PIN_ACK_S);
  const uint32_t tok = WorkKernel(BUDGET_S_CYCLES, seed);
  ack_low(PIN_ACK_S);

  g_jobsS++;

  const int32_t lateness = (int32_t)now_us() - (int32_t)(release_us + DEADLINE_S_MS * 1000u);
  if (lateness > 0) {
    g_missS++;
    set_error_and_log("S", id, lateness);
  }

  safe_print_pair("S", id, tok);
}

// ============================================================
// FreeRTOS task wrappers
// ============================================================
void taskA_RTOS(void *pv) {
  TickType_t lastWake = g_t0_tick;
  uint32_t id = 0;

  for (;;) {
    runTaskA(id);
    id++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_A_MS));
  }
}

void taskB_RTOS(void *pv) {
  TickType_t lastWake = g_t0_tick;
  uint32_t id = 0;

  for (;;) {
    runTaskB(id);
    id++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_B_MS));
  }
}

void taskAGG_RTOS(void *pv) {
  TickType_t lastWake = g_t0_tick;
  uint32_t id = 0;

  for (;;) {
    runTaskAGG(id);
    id++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_AGG_MS));
  }
}

void taskC_RTOS(void *pv) {
  TickType_t lastWake = g_t0_tick;
  uint32_t id = 0;

  for (;;) {
    if (g_modeState == 1u) {
      runTaskC(id);
    } else {
      safe_print_mode_skip("C", id);
    }
    id++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_C_MS));
  }
}

void taskD_RTOS(void *pv) {
  TickType_t lastWake = g_t0_tick;
  uint32_t id = 0;

  for (;;) {
    if (g_modeState == 1u) {
      runTaskD(id);
    } else {
      safe_print_mode_skip("D", id);
    }
    id++;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_D_MS));
  }
}

void taskS_RTOS(void *pv) {
  uint32_t id = 0;
  uint32_t release_us = 0;

  for (;;) {
    if (xQueueReceive(g_sQueue, &release_us, portMAX_DELAY) == pdTRUE) {
      runTaskS(id, release_us);
      id++;
    }
  }
}

void taskModeDebug_RTOS(void *pv) {
  int lastMode = -1;

  for (;;) {
    int modeNow = (int)g_modeState;
    if (modeNow != lastMode) {
      xSemaphoreTake(g_serialMutex, portMAX_DELAY);
      Serial.print("MODE toggled -> ");
      Serial.println(modeNow);
      xSemaphoreGive(g_serialMutex);
      lastMode = modeNow;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskReport_RTOS(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(FINAL_REPORT_AFTER_MS));

  if (g_taskAHandle)       vTaskSuspend(g_taskAHandle);
  if (g_taskBHandle)       vTaskSuspend(g_taskBHandle);
  if (g_taskAGGHandle)     vTaskSuspend(g_taskAGGHandle);
  if (g_taskCHandle)       vTaskSuspend(g_taskCHandle);
  if (g_taskDHandle)       vTaskSuspend(g_taskDHandle);
  if (g_taskSHandle)       vTaskSuspend(g_taskSHandle);
  if (g_taskModeDbgHandle) vTaskSuspend(g_taskModeDbgHandle);

  detachInterrupt(digitalPinToInterrupt(PIN_IN_S));
  detachInterrupt(digitalPinToInterrupt(PIN_IN_MODE));

  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
  Serial.println("FINAL_REPORT_BEGIN");
  Serial.print("A jobs=");   Serial.print(g_jobsA);   Serial.print(" misses="); Serial.println(g_missA);
  Serial.print("B jobs=");   Serial.print(g_jobsB);   Serial.print(" misses="); Serial.println(g_missB);
  Serial.print("AGG jobs="); Serial.print(g_jobsAGG); Serial.print(" misses="); Serial.println(g_missAGG);
  Serial.print("C jobs=");   Serial.print(g_jobsC);   Serial.print(" misses="); Serial.println(g_missC);
  Serial.print("D jobs=");   Serial.print(g_jobsD);   Serial.print(" misses="); Serial.println(g_missD);
  Serial.print("S jobs=");   Serial.print(g_jobsS);   Serial.print(" misses="); Serial.println(g_missS);
  Serial.println("FINAL_REPORT_END");
  xSemaphoreGive(g_serialMutex);

  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SYNC, INPUT_PULLDOWN);
  pinMode(PIN_IN_S, INPUT_PULLDOWN);
  pinMode(PIN_IN_MODE, INPUT_PULLDOWN);

  pinMode(PIN_ACK_A, OUTPUT);
  pinMode(PIN_ACK_B, OUTPUT);
  pinMode(PIN_ACK_AGG, OUTPUT);
  pinMode(PIN_ACK_C, OUTPUT);
  pinMode(PIN_ACK_D, OUTPUT);
  pinMode(PIN_ACK_S, OUTPUT);
  pinMode(PIN_ERR, OUTPUT);

  ack_low(PIN_ACK_A);
  ack_low(PIN_ACK_B);
  ack_low(PIN_ACK_AGG);
  ack_low(PIN_ACK_C);
  ack_low(PIN_ACK_D);
  ack_low(PIN_ACK_S);
  digitalWrite(PIN_ERR, LOW);

  g_serialMutex = xSemaphoreCreateMutex();
  g_sQueue = xQueueCreate(16, sizeof(uint32_t));

  safe_println("Waiting for SYNC rising edge...");

  while (digitalRead(PIN_SYNC) == HIGH) {}
  while (digitalRead(PIN_SYNC) == LOW) {}

  g_t0_us = now_us();
  g_t0_tick = xTaskGetTickCount();
  g_started = true;
  g_modeState = 0;

  safe_println(String("SYNC captured. T0_us=") + String(g_t0_us));
  safe_println("MODE initial -> 0");

  attachInterrupt(digitalPinToInterrupt(PIN_IN_S), on_sporadic_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_MODE), on_mode_rise, RISING);

  // priorities
  xTaskCreatePinnedToCore(taskA_RTOS,        "TaskA",   3072, nullptr, 5, &g_taskAHandle,       APP_CORE);
  xTaskCreatePinnedToCore(taskB_RTOS,        "TaskB",   3072, nullptr, 4, &g_taskBHandle,       APP_CORE);
  xTaskCreatePinnedToCore(taskAGG_RTOS,      "TaskAGG", 3072, nullptr, 3, &g_taskAGGHandle,     APP_CORE);
  xTaskCreatePinnedToCore(taskC_RTOS,        "TaskC",   3072, nullptr, 2, &g_taskCHandle,       APP_CORE);
  xTaskCreatePinnedToCore(taskD_RTOS,        "TaskD",   3072, nullptr, 2, &g_taskDHandle,       APP_CORE);
  xTaskCreatePinnedToCore(taskS_RTOS,        "TaskS",   3072, nullptr, 3, &g_taskSHandle,       APP_CORE);
  xTaskCreatePinnedToCore(taskModeDebug_RTOS,"ModeDbg", 2048, nullptr, 1, &g_taskModeDbgHandle, APP_CORE);
  xTaskCreatePinnedToCore(taskReport_RTOS,   "Report",  2048, nullptr, 6, &g_taskReportHandle,  APP_CORE);
}

void loop() {
  vTaskDelete(nullptr);
}
