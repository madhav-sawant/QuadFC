/*
 * Telemetry Transmitter
 * nRF24L01 via SPI
 *
 * Sends flight data to ground station
 */

#include "telemetry.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TELEMETRY";

// SPI pins for nRF24L01
#define NRF_MOSI 23
#define NRF_MISO 19
#define NRF_SCK 18
#define NRF_CSN 5
#define NRF_CE 17

// nRF24L01 registers
#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_TX_ADDR 0x10
#define NRF_RX_ADDR_P0 0x0A
#define NRF_FIFO_STATUS 0x17

// Commands
#define NRF_W_REGISTER 0x20
#define NRF_W_TX_PAYLOAD 0xA0
#define NRF_FLUSH_TX 0xE1

static spi_device_handle_t spi_handle;
static bool initialized = false;

// Address - must match ground station
static const uint8_t tx_address[5] = {'D', 'R', 'O', 'N', 'E'};

static void nrf_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {NRF_W_REGISTER | reg, value};
  spi_transaction_t t = {
      .length = 16,
      .tx_buffer = tx,
  };
  spi_device_transmit(spi_handle, &t);
}

static void nrf_write_address(uint8_t reg, const uint8_t *addr, uint8_t len) {
  uint8_t tx[6];
  tx[0] = NRF_W_REGISTER | reg;
  for (int i = 0; i < len; i++) {
    tx[i + 1] = addr[i];
  }
  spi_transaction_t t = {
      .length = (len + 1) * 8,
      .tx_buffer = tx,
  };
  spi_device_transmit(spi_handle, &t);
}

static void nrf_send_payload(const uint8_t *data, uint8_t len) {
  uint8_t tx[33];
  tx[0] = NRF_W_TX_PAYLOAD;
  for (int i = 0; i < len && i < 32; i++) {
    tx[i + 1] = data[i];
  }
  spi_transaction_t t = {
      .length = (len + 1) * 8,
      .tx_buffer = tx,
  };
  spi_device_transmit(spi_handle, &t);
}

static void nrf_flush_tx(void) {
  uint8_t cmd = NRF_FLUSH_TX;
  spi_transaction_t t = {
      .length = 8,
      .tx_buffer = &cmd,
  };
  spi_device_transmit(spi_handle, &t);
}

static void nrf_ce_high(void) { gpio_set_level(NRF_CE, 1); }

static void nrf_ce_low(void) { gpio_set_level(NRF_CE, 0); }

void telemetry_init(void) {
  ESP_LOGI(TAG, "Initializing nRF24L01...");

  // Configure CE pin
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << NRF_CE),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = 0,
      .pull_down_en = 0,
  };
  gpio_config(&io_conf);
  nrf_ce_low();

  // Initialize SPI
  spi_bus_config_t buscfg = {
      .mosi_io_num = NRF_MOSI,
      .miso_io_num = NRF_MISO,
      .sclk_io_num = NRF_SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 1000000, // 1 MHz
      .mode = 0,
      .spics_io_num = NRF_CSN,
      .queue_size = 1,
  };

  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI bus init failed");
    return;
  }

  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI device add failed");
    return;
  }

  // Power on delay
  vTaskDelay(pdMS_TO_TICKS(100));

  // Configure nRF24L01
  nrf_write_reg(NRF_CONFIG, 0x0E);     // Power up, TX mode, CRC enabled
  nrf_write_reg(NRF_EN_AA, 0x00);      // Disable auto-ack (simple mode)
  nrf_write_reg(NRF_EN_RXADDR, 0x00);  // Disable RX
  nrf_write_reg(NRF_SETUP_AW, 0x03);   // 5 byte address
  nrf_write_reg(NRF_SETUP_RETR, 0x00); // No retransmit
  nrf_write_reg(NRF_RF_CH, 76);        // Channel 76
  nrf_write_reg(NRF_RF_SETUP, 0x26);   // 250kbps, 0dBm

  // Set TX address
  nrf_write_address(NRF_TX_ADDR, tx_address, 5);
  nrf_write_address(NRF_RX_ADDR_P0, tx_address, 5);

  nrf_flush_tx();

  initialized = true;
  ESP_LOGI(TAG, "nRF24L01 initialized");
}

void telemetry_send(const telemetry_packet_t *packet) {
  if (!initialized)
    return;

  nrf_ce_low();
  nrf_flush_tx();
  nrf_send_payload((const uint8_t *)packet, sizeof(telemetry_packet_t));
  nrf_ce_high();

  // CE pulse for TX
  vTaskDelay(1);
  nrf_ce_low();
}

bool telemetry_ready(void) { return initialized; }
