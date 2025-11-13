#ifndef RF24_H
#define RF24_H

#include "stm32f1xx_hal.h"  // Thay đổi theo dòng STM32 của mày (f1xx, f4xx, h7xx, ...)
#include <stdint.h>
#include <stdbool.h>

/* ============ nRF24L01 Register Addresses ============ */
#define NRF_CONFIG      0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define NRF_STATUS      0x07
#define OBSERVE_TX      0x08
#define RPD             0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

/* ============ Bit Mnemonics ============ */
// CONFIG register
#define MASK_RX_DR      6
#define MASK_TX_DS      5
#define MASK_MAX_RT     4
#define EN_CRC          3
#define CRCO            2
#define PWR_UP          1
#define PRIM_RX         0

// RF_SETUP register
#define CONT_WAVE       7
#define RF_DR_LOW       5
#define RF_DR_HIGH      3
#define RF_PWR_LOW      1
#define RF_PWR_HIGH     2

// STATUS register
#define RX_DR           6
#define TX_DS           5
#define MAX_RT          4
#define RX_P_NO         1
#define TX_FULL         0

// FIFO_STATUS
#define TX_REUSE        6
#define TX_FULL_FIFO    5
#define TX_EMPTY        4
#define RX_FULL         1
#define RX_EMPTY        0

/* ============ SPI Commands ============ */
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define ACTIVATE        0x50
#define R_RX_PL_WID     0x60
#define W_ACK_PAYLOAD   0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP_CMD         0xFF

/* ============ Configuration Values ============ */
// Data rates
#define RF24_1MBPS      0
#define RF24_2MBPS      1
#define RF24_250KBPS    2

// PA levels
#define RF24_PA_MIN     0
#define RF24_PA_LOW     1
#define RF24_PA_HIGH    2
#define RF24_PA_MAX     3

// CRC
#define RF24_CRC_DISABLED   0
#define RF24_CRC_8          1
#define RF24_CRC_16         2

/* ============ Structure ============ */
typedef struct {
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;

    GPIO_TypeDef *csn_port;
    uint16_t csn_pin;

    uint8_t payload_size;
    uint8_t addr_width;
    uint8_t config_reg;

    bool dynamic_payloads;
    bool ack_payloads;

    uint8_t pipe0_reading_address[5];
} nrf24_t;

/* ============ API Functions ============ */
// Initialization
void nrf24_init(nrf24_t *dev, SPI_HandleTypeDef *hspi,
                GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool nrf24_begin(nrf24_t *dev);

// Configuration
void nrf24_set_channel(nrf24_t *dev, uint8_t channel);
void nrf24_set_retries(nrf24_t *dev, uint8_t delay, uint8_t count);
void nrf24_set_pa_level(nrf24_t *dev, uint8_t level);
void nrf24_set_data_rate(nrf24_t *dev, uint8_t speed);
void nrf24_set_crc_length(nrf24_t *dev, uint8_t length);
void nrf24_set_payload_size(nrf24_t *dev, uint8_t size);

// Address management
void nrf24_open_writing_pipe(nrf24_t *dev, const uint8_t *address);
void nrf24_open_reading_pipe(nrf24_t *dev, uint8_t pipe, const uint8_t *address);
void nrf24_close_reading_pipe(nrf24_t *dev, uint8_t pipe);

// TX/RX operations
void nrf24_start_listening(nrf24_t *dev);
void nrf24_stop_listening(nrf24_t *dev);
bool nrf24_write(nrf24_t *dev, const void *buf, uint8_t len);
bool nrf24_available(nrf24_t *dev);
bool nrf24_available_pipe(nrf24_t *dev, uint8_t *pipe_num);
void nrf24_read(nrf24_t *dev, void *buf, uint8_t len);

// Power management
void nrf24_power_up(nrf24_t *dev);
void nrf24_power_down(nrf24_t *dev);

// Status and diagnostic
uint8_t nrf24_get_status(nrf24_t *dev);
void nrf24_flush_rx(nrf24_t *dev);
void nrf24_flush_tx(nrf24_t *dev);
void nrf24_print_status(nrf24_t *dev, uint8_t status);
void nrf24_print_observe_tx(nrf24_t *dev);

// Advanced features
void nrf24_enable_dynamic_payloads(nrf24_t *dev);
void nrf24_disable_dynamic_payloads(nrf24_t *dev);
void nrf24_enable_ack_payload(nrf24_t *dev);
void nrf24_write_ack_payload(nrf24_t *dev, uint8_t pipe, const void *buf, uint8_t len);

// Test function
bool nrf24_test_carrier(nrf24_t *dev);
bool nrf24_test_rpd(nrf24_t *dev);
// Check if ACK payload is available after write
bool nrf24_is_ack_payload_available(nrf24_t *dev);

#endif // NRF24_H
