#include "blackbox.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "blackbox";

// Ring buffer storage
static blackbox_entry_t buffer[BLACKBOX_MAX_ENTRIES];
static volatile uint16_t write_index = 0;
static volatile uint16_t entry_count = 0;
static volatile bool recording = false; // Start OFF, enable on ARM

// Mutex for buffer access during reads
static SemaphoreHandle_t buffer_mutex __attribute__((unused)) = NULL;

// FreeRTOS queue for passing entries from control loop to blackbox task
#define BLACKBOX_QUEUE_SIZE 16
static QueueHandle_t log_queue = NULL;

// Blackbox task handle
static TaskHandle_t blackbox_task_handle = NULL;

/**
 * @brief Blackbox task - runs on Core 0
 *        Consumes entries from queue and writes to ring buffer.
 */
static void blackbox_task(void *arg) {
  (void)arg;
  blackbox_entry_t entry;

  ESP_LOGI(TAG, "Blackbox task started on Core %d", xPortGetCoreID());

  while (1) {
    // Wait for entry from queue (blocking, but on Core 0 so doesn't affect
    // flight)
    if (xQueueReceive(log_queue, &entry, portMAX_DELAY) == pdTRUE) {
      if (recording) {
        // Write to ring buffer
        memcpy(&buffer[write_index], &entry, sizeof(blackbox_entry_t));

        // Update indices
        write_index = (write_index + 1) % BLACKBOX_MAX_ENTRIES;
        if (entry_count < BLACKBOX_MAX_ENTRIES) {
          entry_count++;
        }
      }
    }
  }
}

void blackbox_init(void) {
  // Create queue
  log_queue = xQueueCreate(BLACKBOX_QUEUE_SIZE, sizeof(blackbox_entry_t));
  if (log_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create queue!");
    return;
  }

  // Clear buffer
  memset(buffer, 0, sizeof(buffer));
  write_index = 0;
  entry_count = 0;
  recording = false;

  // Create task pinned to Core 0
  // Stack size: 2KB is enough for this simple task
  xTaskCreatePinnedToCore(blackbox_task, "blackbox", 2048, NULL,
                          5, // Priority (lower than control loop)
                          &blackbox_task_handle, 0 // Core 0
  );

  ESP_LOGI(TAG, "Blackbox initialized. Buffer: %d entries, %d bytes",
           BLACKBOX_MAX_ENTRIES,
           (int)(BLACKBOX_MAX_ENTRIES * sizeof(blackbox_entry_t)));
}

void blackbox_log(const blackbox_entry_t *entry) {
  if (log_queue == NULL || entry == NULL || !recording) {
    return;
  }

  // Non-blocking send - if queue is full, drop the entry
  // This ensures we NEVER block the control loop
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Check if we're in ISR context
  if (xPortInIsrContext()) {
    xQueueSendFromISR(log_queue, entry, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  } else {
    // Not in ISR, use regular send with 0 timeout
    xQueueSend(log_queue, entry, 0);
  }
}

void blackbox_clear(void) {
  // Temporarily stop recording
  bool was_recording = recording;
  recording = false;

  // Wait a bit for any pending writes
  vTaskDelay(pdMS_TO_TICKS(10));

  // Clear queue
  if (log_queue != NULL) {
    xQueueReset(log_queue);
  }

  // Reset indices
  write_index = 0;
  entry_count = 0;

  // Resume recording if it was active
  recording = was_recording;

  ESP_LOGI(TAG, "Blackbox cleared");
}

uint16_t blackbox_get_count(void) { return entry_count; }

const blackbox_entry_t *blackbox_get_entry(uint16_t index) {
  if (index >= entry_count) {
    return NULL;
  }

  // Calculate actual index in ring buffer
  // If buffer is full, oldest entry is at write_index
  // If not full, oldest entry is at 0
  uint16_t actual_index;
  if (entry_count >= BLACKBOX_MAX_ENTRIES) {
    // Buffer is full, wrap around
    actual_index = (write_index + index) % BLACKBOX_MAX_ENTRIES;
  } else {
    // Buffer not full, start from 0
    actual_index = index;
  }

  return &buffer[actual_index];
}

void blackbox_start(void) {
  recording = true;
  ESP_LOGI(TAG, "Recording started");
}

void blackbox_stop(void) {
  recording = false;
  ESP_LOGI(TAG, "Recording stopped");
}
