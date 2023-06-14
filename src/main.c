
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <u8g2.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"

#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"

// SDA - GPIO21
#define PIN_SDA 22

// SCL - GPIO22
#define PIN_SCL 23

static const char *TAG = "DualPWM ";

int freq = 1000;

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
  BaseType_t high_task_wakeup;
  QueueHandle_t queue = (QueueHandle_t)user_ctx;
  // send event data to queue, from this interrupt callback
  xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
  return (high_task_wakeup == pdTRUE);
}

void app_main(void *ignore)
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
  u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  u8g2_t u8g2; // a structure which will contain all the data for one display
               // u8g2_Setup_ssd1306_i2c_128x64_noname_f(
  u8g2_Setup_sh1106_i2c_128x64_noname_f(
      &u8g2, U8G2_R0,
      // u8x8_byte_sw_i2c,
      u8g2_esp32_i2c_byte_cb,
      u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
                           // sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  //  ESP_LOGI(TAG, "u8g2_DrawBox");
  //  u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
  //  u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

  u8g2_SetFont(&u8g2, u8g2_font_8x13_tf);

  u8g2_SendBuffer(&u8g2);

  ESP_LOGI(TAG, "All done!");

  ESP_LOGI(TAG, "install pcnt unit");
  pcnt_unit_config_t unit_config = {
      .high_limit = 80,
      .low_limit = -80,
  };
  pcnt_unit_handle_t pcnt_unit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

  ESP_LOGI(TAG, "set glitch filter");
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

  ESP_LOGI(TAG, "install pcnt channels");
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = 19,
      .level_gpio_num = 18,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = 18,
      .level_gpio_num = 19,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

  ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  /*
    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {0, 100};

    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++)
    {
      ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }

    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };

    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));
  */
  ESP_LOGI(TAG, "enable pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_LOGI(TAG, "clear pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_LOGI(TAG, "start pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

  // Report counter value
  int pulse_count = 0;
  // int event_count = 0;
  char buf[32];
  while (1)
  {
    /*    if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000)))
        {
          ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        }
        else
    */
    {
      u8g2_ClearBuffer(&u8g2);
      //  ESP_LOGI(TAG, "u8g2_DrawBox");
      //  u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
      //  u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

      ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

      if (abs(pulse_count) >= 4)
        ESP_LOGI(TAG, "Pulse count: %d", pulse_count);

      if (abs(pulse_count) >= 4)
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

      if (abs(pulse_count) > 26)
        // freq = freq + pulse_count / 4 * 100;
        freq = freq * 2;
      else if (abs(pulse_count) > 16)
        freq = freq + (freq / 100) * (pulse_count / 4);
      else
        freq = freq + pulse_count / 4;

      snprintf(buf, sizeof(buf), "FREQ: %d", freq);
      u8g2_DrawStr(&u8g2, 2, 17, buf);

      u8g2_SendBuffer(&u8g2);

      // ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}
