#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "driver/isp_ccm.h"
#include "driver/isp_demosaic.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define CURRENT_SCREEN SCREEN_10_1_DSI_TOUCH_A
#include "displays_config.h"
#include "i2c.h"
#include "ov5647_regs.h"

static constexpr uint8_t OV5647_I2C_ADDR = 0x36;
static constexpr uint16_t CAM_WIDTH = 800;
static constexpr uint16_t CAM_HEIGHT = 1280;
static constexpr size_t CAM_FRAME_SIZE = static_cast<size_t>(CAM_WIDTH) * CAM_HEIGHT * 2;
static constexpr uint8_t CAM_BUFFER_COUNT = 3;
static constexpr int CSI_LANE_BIT_RATE_MBPS = 400;
static constexpr int ISP_CLOCK_HZ = 80 * 1000 * 1000;
static constexpr int MIPI_LDO_CHANNEL = 3;
static constexpr int MIPI_LDO_MV = 2500;

static DEV_I2C_Port s_i2c;
static esp_ldo_channel_handle_t s_mipi_ldo = nullptr;
static esp_cam_ctlr_handle_t s_cam_handle = nullptr;
static isp_proc_handle_t s_isp_handle = nullptr;
static QueueHandle_t s_ready_queue = nullptr;
static uint8_t *s_frame_buffers[CAM_BUFFER_COUNT] = {nullptr};
static esp_cam_ctlr_trans_t s_transfers[CAM_BUFFER_COUNT] = {};
static constexpr size_t FRAME_ALIGNMENT = 128;
static volatile uint32_t s_captured_frames = 0;
static uint32_t s_drawn_frames = 0;
static uint32_t s_last_stats_ms = 0;

Arduino_ESP32DSIPanel *bus = new Arduino_ESP32DSIPanel(
  display_cfg.hsync_pulse_width,
  display_cfg.hsync_back_porch,
  display_cfg.hsync_front_porch,
  display_cfg.vsync_pulse_width,
  display_cfg.vsync_back_porch,
  display_cfg.vsync_front_porch,
  display_cfg.prefer_speed,
  display_cfg.lane_bit_rate);

Arduino_DSI_Display *gfx = new Arduino_DSI_Display(
  display_cfg.width,
  display_cfg.height,
  bus,
  0,
  true,
  -1,
  display_cfg.init_cmds,
  display_cfg.init_cmds_size);

static void fatal_screen(const char *message)
{
  Serial.println(message);
  if (gfx != nullptr) {
    gfx->fillScreen(RGB565_RED);
    gfx->setTextColor(RGB565_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(24, 24);
    gfx->print(message);
  }
  while (true) {
    delay(1000);
  }
}

static bool ov5647_select()
{
  DEV_I2C_Set_Slave_Addr(&s_i2c.dev, OV5647_I2C_ADDR);
  return s_i2c.dev != nullptr;
}

static bool ov5647_write_reg(uint16_t reg, uint8_t value)
{
  if (s_i2c.dev == nullptr) {
    return false;
  }

  uint8_t payload[3] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF),
    value
  };
  return i2c_master_transmit(s_i2c.dev, payload, sizeof(payload), 100) == ESP_OK;
}

static bool ov5647_read_reg(uint16_t reg, uint8_t *value)
{
  if ((s_i2c.dev == nullptr) || (value == nullptr)) {
    return false;
  }

  uint8_t reg_addr[2] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF)
  };
  return i2c_master_transmit_receive(s_i2c.dev, reg_addr, sizeof(reg_addr), value, 1, 100) == ESP_OK;
}

static bool ov5647_check_id()
{
  uint8_t id_h = 0;
  uint8_t id_l = 0;
  if (!ov5647_read_reg(0x300A, &id_h) || !ov5647_read_reg(0x300B, &id_l)) {
    return false;
  }

  const uint16_t chip_id = (static_cast<uint16_t>(id_h) << 8) | id_l;
  Serial.printf("[OV5647] chip-id: 0x%04X\n", chip_id);
  return chip_id == 0x5647;
}

static bool ov5647_apply_init_sequence()
{
  if (!ov5647_select()) {
    Serial.println("[OV5647] I2C select failed");
    return false;
  }

  if (!ov5647_write_reg(0x0100, 0x00)) {
    return false;
  }
  if (!ov5647_write_reg(0x0103, 0x01)) {
    return false;
  }
  delay(20);

  if (!ov5647_check_id()) {
    Serial.println("[OV5647] detect failed");
    return false;
  }

  for (size_t i = 0; ov5647_init_regs[i].reg != OV5647_REG_END; ++i) {
    if (!ov5647_write_reg(ov5647_init_regs[i].reg, ov5647_init_regs[i].val)) {
      Serial.printf("[OV5647] write failed at 0x%04X\n", ov5647_init_regs[i].reg);
      return false;
    }
    delay(1);
  }

  return true;
}

static bool ov5647_start_stream()
{
  static const ov5647_reginfo_t stream_on_regs[] = {
    {0x4800, 0x14},
    {0x3002, 0x01},
    {0x300D, 0x01},
    {0x300E, 0x01},
    {0x4800, 0x04},
    {0x0100, 0x01},
  };

  if (!ov5647_select()) {
    return false;
  }

  for (size_t i = 0; i < (sizeof(stream_on_regs) / sizeof(stream_on_regs[0])); ++i) {
    if (!ov5647_write_reg(stream_on_regs[i].reg, stream_on_regs[i].val)) {
      Serial.printf("[OV5647] stream-on failed at 0x%04X\n", stream_on_regs[i].reg);
      return false;
    }
  }

  return true;
}

static bool allocate_frame_buffers()
{
  for (uint8_t i = 0; i < CAM_BUFFER_COUNT; ++i) {
    s_frame_buffers[i] = static_cast<uint8_t *>(
      heap_caps_aligned_calloc(
        FRAME_ALIGNMENT,
        1,
        CAM_FRAME_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    if (s_frame_buffers[i] == nullptr) {
      Serial.printf("[CAM] buffer alloc failed: %u\n", i);
      return false;
    }

    s_transfers[i].buffer = s_frame_buffers[i];
    s_transfers[i].buflen = CAM_FRAME_SIZE;
    s_transfers[i].received_size = 0;
  }

  return true;
}

static bool IRAM_ATTR camera_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
  (void)handle;
  (void)user_data;

  uint8_t ready_index = 0xFF;
  for (uint8_t i = 0; i < CAM_BUFFER_COUNT; ++i) {
    if (trans->buffer == s_frame_buffers[i]) {
      ready_index = i;
      break;
    }
  }

  if (ready_index >= CAM_BUFFER_COUNT) {
    return false;
  }

  BaseType_t high_task_woken = pdFALSE;
  xQueueSendFromISR(s_ready_queue, &ready_index, &high_task_woken);
  s_captured_frames++;
  return high_task_woken == pdTRUE;
}

static bool init_camera_pipeline()
{
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = MIPI_LDO_CHANNEL,
    .voltage_mv = MIPI_LDO_MV,
  };
  if (esp_ldo_acquire_channel(&ldo_config, &s_mipi_ldo) != ESP_OK) {
    Serial.println("[CAM] MIPI LDO acquire failed");
    return false;
  }

  if (!ov5647_apply_init_sequence()) {
    return false;
  }

  if (!allocate_frame_buffers()) {
    return false;
  }

  s_ready_queue = xQueueCreate(CAM_BUFFER_COUNT, sizeof(uint8_t));
  if (s_ready_queue == nullptr) {
    Serial.println("[CAM] ready queue alloc failed");
    return false;
  }

  esp_cam_ctlr_csi_config_t csi_config = {
    .ctlr_id = 0,
    .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
    .h_res = CAM_WIDTH,
    .v_res = CAM_HEIGHT,
    .data_lane_num = 2,
    .lane_bit_rate_mbps = CSI_LANE_BIT_RATE_MBPS,
    .input_data_color_type = CAM_CTLR_COLOR_RAW8,
    .output_data_color_type = CAM_CTLR_COLOR_RAW8,
    .queue_items = CAM_BUFFER_COUNT,
    .byte_swap_en = 0,
    .bk_buffer_dis = 0,
  };

  esp_err_t err = esp_cam_new_csi_ctlr(&csi_config, &s_cam_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] esp_cam_new_csi_ctlr failed: %s\n", esp_err_to_name(err));
    return false;
  }

  esp_cam_ctlr_evt_cbs_t callbacks = {};
  callbacks.on_trans_finished = camera_trans_finished;
  err = esp_cam_ctlr_register_event_callbacks(s_cam_handle, &callbacks, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[CAM] register callbacks failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_cam_ctlr_enable(s_cam_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] controller enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  esp_isp_processor_cfg_t isp_config = {
    .clk_hz = ISP_CLOCK_HZ,
    .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
    .input_data_color_type = ISP_COLOR_RAW8,
    .output_data_color_type = ISP_COLOR_RGB565,
    .has_line_start_packet = false,
    .has_line_end_packet = false,
    .h_res = CAM_WIDTH,
    .v_res = CAM_HEIGHT,
    .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG,
  };

  err = esp_isp_new_processor(&isp_config, &s_isp_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] esp_isp_new_processor failed: %s\n", esp_err_to_name(err));
    return false;
  }

  esp_isp_demosaic_config_t demosaic_config = {};
  demosaic_config.grad_ratio.integer = 1;
  err = esp_isp_demosaic_configure(s_isp_handle, &demosaic_config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] demosaic config failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_isp_demosaic_enable(s_isp_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] demosaic enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  esp_isp_ccm_config_t ccm_config = {};
  ccm_config.matrix[0][0] = 1.0f;
  ccm_config.matrix[1][1] = 1.0f;
  ccm_config.matrix[2][2] = 1.0f;
  ccm_config.saturation = true;
  err = esp_isp_ccm_configure(s_isp_handle, &ccm_config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] CCM config failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_isp_ccm_enable(s_isp_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] CCM enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_isp_enable(s_isp_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] ISP enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  for (uint8_t i = 0; i < CAM_BUFFER_COUNT; ++i) {
    err = esp_cam_ctlr_receive(s_cam_handle, &s_transfers[i], 100);
    if (err != ESP_OK) {
      Serial.printf("[CAM] initial queue failed on buffer %u: %s\n", i, esp_err_to_name(err));
      return false;
    }
  }

  err = esp_cam_ctlr_start(s_cam_handle);
  if (err != ESP_OK) {
    Serial.printf("[CAM] controller start failed: %s\n", esp_err_to_name(err));
    return false;
  }

  if (!ov5647_start_stream()) {
    Serial.println("[CAM] sensor stream-on failed");
    return false;
  }

  Serial.println("[CAM] preview pipeline ready");
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] ESP32-P4 OV5647 live preview");

  s_i2c = DEV_I2C_Init();
  display_init(s_i2c);
  set_display_backlight(s_i2c, 255);

  if (!gfx->begin()) {
    while (true) {
      delay(1000);
    }
  }

  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(24, 24);
  gfx->print("Initializing OV5647...");

  if (!init_camera_pipeline()) {
    fatal_screen("[FATAL] Camera init failed");
  }

  gfx->fillScreen(RGB565_BLACK);
  s_last_stats_ms = millis();
}

void loop()
{
  uint8_t ready_index = 0;
  if (xQueueReceive(s_ready_queue, &ready_index, pdMS_TO_TICKS(50)) == pdTRUE) {
    gfx->draw16bitRGBBitmap(
      0,
      0,
      reinterpret_cast<uint16_t *>(s_frame_buffers[ready_index]),
      CAM_WIDTH,
      CAM_HEIGHT);

    esp_err_t err = esp_cam_ctlr_receive(s_cam_handle, &s_transfers[ready_index], 100);
    if (err != ESP_OK) {
      Serial.printf("[CAM] requeue failed on buffer %u: %s\n", ready_index, esp_err_to_name(err));
    } else {
      s_drawn_frames++;
    }
  }

  const uint32_t now = millis();
  if (now - s_last_stats_ms >= 1000) {
    const uint32_t captured = s_captured_frames;
    s_captured_frames = 0;
    Serial.printf(
      "[STAT] captured=%lu drawn=%lu ready=%u\n",
      static_cast<unsigned long>(captured),
      static_cast<unsigned long>(s_drawn_frames),
      static_cast<unsigned>(uxQueueMessagesWaiting(s_ready_queue)));
    s_drawn_frames = 0;
    s_last_stats_ms = now;
  }
}
