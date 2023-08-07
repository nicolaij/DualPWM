
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

#include "esp_timer.h"

#include "driver/ledc.h"

#include "button.h"

// SDA - GPIO21
#define PIN_SDA 22

// SCL - GPIO22
#define PIN_SCL 23

#define PIN_ENCODER_A 18
#define PIN_ENCODER_B 19
#define PIN_ENCODER_KEY 21

#define PIN_KEY1 34
#define PIN_KEY2 35

#define PIN_OUT_PWM1 17
#define PIN_OUT_PWM2 16

static const char *TAG = "DualPWM ";

int freq = 10; // Hz
bool hi_mode = false;

double duty1 = 50.0;   // %
double duty2 = 50.0;   // %
double offset2 = 25.0; // %

// real
int duty1r = 0;
int duty2r = 0;
int offset2r = 0;

bool start = false;

int top = 0;

char buf[32];

const int lines = 5;

#include <math.h>

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES_LO_FREQ LEDC_TIMER_10_BIT // Set duty resolution bits
#define LEDC_DUTY_MAX_LO_FREQ 1023
#define LEDC_DUTY_RES_HI_FREQ LEDC_TIMER_9_BIT // Set duty resolution bits
#define LEDC_DUTY_MAX_HI_FREQ 511

static QueueHandle_t gpio_evt_queue = NULL;

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
  BaseType_t high_task_wakeup;
  QueueHandle_t queue = (QueueHandle_t)user_ctx;
  // send event data to queue, from this interrupt callback
  xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
  return (high_task_wakeup == pdTRUE);
}

static void ledc_timer_init()
{
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = (hi_mode == true) ? LEDC_DUTY_RES_HI_FREQ : LEDC_DUTY_RES_LO_FREQ,
      .freq_hz = freq,
      .clk_cfg = (hi_mode == true) ? LEDC_USE_APB_CLK : LEDC_USE_REF_TICK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

static void ledc_channel_init()
{
  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel1 = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = PIN_OUT_PWM1,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ledc_channel_config_t ledc_channel2 = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_1,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = PIN_OUT_PWM2,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
}

/*
selector - 1..99 - Выбор пункта меню, 100..199 - редактирование пункта
*/
void drawMenu(u8g2_t *ptr_u8g2, int selector)
{
  int ypos = 0;
  int yheight = u8g2_GetDisplayHeight(ptr_u8g2) / lines;
  int w = 0;
  u8g2_ClearBuffer(ptr_u8g2);

  // 1
  if (selector == 1)
    u8g2_DrawBox(ptr_u8g2, 0, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);

  u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "FREQ: ");

  if (hi_mode)
    snprintf(buf, sizeof(buf), " %d kHz", freq / 1000);
  else
    snprintf(buf, sizeof(buf), " %d Hz", freq);

  w = u8g2_GetStrWidth(ptr_u8g2, buf);

  if (selector == 101)
  {
    u8g2_DrawBox(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 2, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);
  }
  u8g2_DrawStr(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 1, ypos + yheight, buf);

  ypos += yheight;
  // 2
  if (selector == 2)
    u8g2_DrawBox(ptr_u8g2, 0, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);

  u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "1 Duty: ");

  snprintf(buf, sizeof(buf), "%3.1f%%", duty1);
  w = u8g2_GetStrWidth(ptr_u8g2, buf);

  if (selector == 102)
  {
    u8g2_DrawBox(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 2, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);
  }
  u8g2_DrawStr(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 1, ypos + yheight, buf);

  ypos += yheight;
  // 3
  if (selector == 3)
    u8g2_DrawBox(ptr_u8g2, 0, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);

  u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "2 Duty: ");

  snprintf(buf, sizeof(buf), "%3.1f%%", duty2);

  w = u8g2_GetStrWidth(ptr_u8g2, buf);

  if (selector == 103)
  {
    u8g2_DrawBox(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 2, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);
  }
  u8g2_DrawStr(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 1, ypos + yheight, buf);

  ypos += yheight;
  // 4
  if (selector == 4)
    u8g2_DrawBox(ptr_u8g2, 0, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);

  u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "2 Offset: ");

  snprintf(buf, sizeof(buf), "%3.1f%%", offset2);

  w = u8g2_GetStrWidth(ptr_u8g2, buf);

  if (selector == 104)
  {
    u8g2_DrawBox(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 2, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);
  }
  u8g2_DrawStr(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 1, ypos + yheight, buf);

  ypos += yheight;
  // 5
  if (selector == 5)
    u8g2_DrawBox(ptr_u8g2, 0, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);

  if (start == false)
  {
    u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "START");
    snprintf(buf, sizeof(buf), "%s", "Off");
  }
  else
  {
    u8g2_DrawStr(ptr_u8g2, 2, ypos + yheight, "STOP");
    snprintf(buf, sizeof(buf), "%s", "ON");
  }

  w = u8g2_GetStrWidth(ptr_u8g2, buf);

  if (selector != 5 && start == true)
  {
    u8g2_DrawBox(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 2, ypos + 1, u8g2_GetDisplayWidth(ptr_u8g2), yheight + 1);
  }
  u8g2_DrawStr(ptr_u8g2, u8g2_GetDisplayWidth(ptr_u8g2) - w - 1, ypos + yheight, buf);

  u8g2_SendBuffer(ptr_u8g2);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t)arg;
  static int64_t key_millis = 0;
  if (esp_timer_get_time() - key_millis > 150000)
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
  key_millis = esp_timer_get_time();
}

void stopPWM()
{
  ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, LEDC_CHANNEL_0, 0));
  ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, LEDC_CHANNEL_1, 0));
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
  u8g2_SetFontMode(&u8g2, 1);  /* activate transparent font mode */
  u8g2_SetDrawColor(&u8g2, 2); /* XOR */

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
      .edge_gpio_num = PIN_ENCODER_B,
      .level_gpio_num = PIN_ENCODER_A,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = PIN_ENCODER_A,
      .level_gpio_num = PIN_ENCODER_B,
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

  // Set the LEDC peripheral configuration
  ledc_timer_init();
  ledc_channel_init();
  // Set duty to 50%
  ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_0, 0, 0));
  // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1, 0, 0));

  stopPWM();

  // Report counter value
  // int event_count = 0;
  bool display_update = true;
  int64_t millis = 0;
  int64_t deltamillis = 0;
  int remainder_pcnt = 0;

  int selector = 5;
  int cur_value = 0;

  int res = LEDC_DUTY_MAX_LO_FREQ;

  button_event_t ev;
  QueueHandle_t button_events = button_init(PIN_BIT(PIN_KEY1) | PIN_BIT(PIN_KEY2) | PIN_BIT(PIN_ENCODER_KEY));

  while (1)
  {
    // Key press
    if (xQueueReceive(button_events, &ev, 200 / portTICK_PERIOD_MS))
    {
      if ((ev.pin == PIN_ENCODER_KEY) && (ev.event == BUTTON_DOWN))
      {
        if (selector < 99)
        {
          selector += 100;
          switch (selector)
          {
          case 101:
            cur_value = freq;
            break;
          case 102:
            cur_value = duty1 * 10.0;
            break;
          case 103:
            cur_value = duty2 * 10.0;
            break;
          case 104:
            cur_value = offset2 * 10.0;
            break;
          case 105:
            selector = 5;
            if (start)
            {
              stopPWM();
              start = false;
            }
            else
            {
              if (hi_mode)
                res = LEDC_DUTY_MAX_HI_FREQ;
              else
                res = LEDC_DUTY_MAX_LO_FREQ;

              ledc_timer_init();
              duty1r = res * duty1;
              duty2r = res * duty2;
              offset2r = res * offset2;
              ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_0, duty1r, 0));
              ESP_ERROR_CHECK(ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1, duty2r, offset2r));
              ESP_LOGI(TAG, "Duty1: %d", duty1r);
              ESP_LOGI(TAG, "Duty2: %d", duty2r);
              ESP_LOGI(TAG, "Offset2: %d", offset2r);

              ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
              ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
              start = true;

              duty1r = (ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_0));
              duty2r = (ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_1));
              offset2r = (ledc_get_hpoint(LEDC_MODE, LEDC_CHANNEL_1));
              ESP_LOGI(TAG, "Get Duty1: %d", duty1r);
              ESP_LOGI(TAG, "Get Duty2: %d", duty2r);
              ESP_LOGI(TAG, "Get Offset2: %d", offset2r);
            }
            break;

          default:
            break;
          }
        }
        else if (selector < 199)
        {
          selector -= 100;
          ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
          ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
        }
      }
      display_update = true;
    }

    int pulse_count = 0;
    int pulse_count_norm = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

    deltamillis = esp_timer_get_time() - millis;
    millis = esp_timer_get_time();

    pulse_count_norm = (pulse_count + remainder_pcnt) / 4;
    remainder_pcnt = (pulse_count + remainder_pcnt) % 4;

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    if (ev.event)
      pulse_count_norm = 0;

    // Encoder rotate
    if (pulse_count_norm != 0)
    {
      int rotate_speed = (abs(pulse_count_norm) * 1000) / (deltamillis / 1000); // clicks per second

      ESP_LOGI(TAG, "Pulse count: %d speed %d", pulse_count_norm, rotate_speed);

      int d = pulse_count_norm;

      // Меню
      if (selector < 99)
      {
        selector += d;
        if (selector > lines)
          selector = lines;
        if (selector < 1)
          selector = 1;
      }
      else if (selector > 99)
      {
        // ускорение 5%
        //        if (rotate_speed > 20)
        //        {
        //          d = (cur_value / 20) * pulse_count_norm;
        //          if (d != 0)
        //            rotate_speed = 0;
        //        }

        // ускорение x5
        if (rotate_speed > 10)
        {
          d = pulse_count_norm * 10;
          if (d == 0)
            d = pulse_count_norm;
        }
      }

      if (hi_mode == true && selector == 101)
      {
        cur_value = (cur_value / 1000) * 1000;
        d = d * 1000;
      }

      if (selector >= 102 && selector <= 104)
      {
        d = d * 2;
      }

      ESP_LOGI(TAG, "cur_value: %d add %d", cur_value, d);
      cur_value = cur_value + d;

      // меняем настройки
      switch (selector)
      {
      case 101:
        if (cur_value < 1)
          cur_value = 1;

        if (hi_mode == true && cur_value <= 1000)
        {
          cur_value = 900;
          freq = 900;
        }

        if (hi_mode == false && cur_value > 900)
        {
          cur_value = 1000;
          freq = 1000;
        }

        if ((hi_mode == true && cur_value <= 1000) || (hi_mode == false && cur_value > 900))
        {
          printf("stop\n");
          stopPWM();
          start = false;
          hi_mode = !hi_mode;
          ledc_timer_init();
        }
        else
        {
          if (ledc_set_freq(LEDC_MODE, LEDC_TIMER, cur_value) == ESP_OK)
          {
            freq = cur_value;
          }
          else
          {
            cur_value = ledc_get_freq(LEDC_MODE, LEDC_TIMER);
          }
        }

        break;
      case 102:
        if (cur_value < 0)
          cur_value = 0;
        if (cur_value > 1000)
          cur_value = 1000;

        if (ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_0, res * (double)(cur_value / 10.0), 0) == ESP_OK)
        {
          // v = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_0);
          duty1 = cur_value / 10.0;
        }
        break;
      case 103:
        if (cur_value < 0)
          cur_value = 0;
        if (cur_value > 1000)
          cur_value = 1000;

        if (ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1, res * (double)(cur_value / 10.0), res * offset2) == ESP_OK)
        {
          // v = ledc_get_duty(LEDC_MODE, LEDC_CHANNEL_1);
          duty2 = cur_value / 10.0;
        }
        break;
      case 104:
        if (cur_value < 0)
          cur_value = 0;
        if (cur_value > 1000)
          cur_value = 1000;

        if (ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CHANNEL_1, res * duty2, res * (double)(cur_value / 10.0)) == ESP_OK)
        {
          // v = ledc_get_hpoint(LEDC_MODE, LEDC_CHANNEL_1);
          offset2 = cur_value / 10.0;
        }
        break;

      default:
        break;
      }
      display_update = true;
    }

    if (display_update)
    {
      drawMenu(&u8g2, selector);
      display_update = false;
    }

    ev.event = 0;
  }
}
