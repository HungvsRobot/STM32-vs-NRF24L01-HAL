#include "RF24.h"
#include <string.h>

/* ============ Private Functions ============ */

// Delay microseconds - Cần enable DWT counter trong main
static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while((DWT->CYCCNT - start) < cycles);
}

// CE pin control
static inline void ce_high(nrf24_t *dev) {
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_SET);
}

static inline void ce_low(nrf24_t *dev) {
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_RESET);
}

// CSN pin control
static inline void csn_high(nrf24_t *dev) {
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_SET);
}

static inline void csn_low(nrf24_t *dev) {
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_RESET);
}

// Read single register
static uint8_t read_register(nrf24_t *dev, uint8_t reg) {
    uint8_t tx[2] = {R_REGISTER | reg, 0xFF};
    uint8_t rx[2];

    csn_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    csn_high(dev);

    return rx[1];
}

// Write single register
static void write_register(nrf24_t *dev, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {W_REGISTER | reg, value};

    csn_low(dev);
    HAL_SPI_Transmit(dev->hspi, tx, 2, 100);
    csn_high(dev);
}

// Read multiple bytes from register
static void read_register_multi(nrf24_t *dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = R_REGISTER | reg;
    for(uint8_t i = 1; i <= len; i++) {
        tx[i] = 0xFF;
    }

    csn_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1, 100);
    csn_high(dev);

    memcpy(buf, &rx[1], len);
}

// Write multiple bytes to register
static void write_register_multi(nrf24_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len) {
    uint8_t tx[33];

    tx[0] = W_REGISTER | reg;
    memcpy(&tx[1], buf, len);

    csn_low(dev);
    HAL_SPI_Transmit(dev->hspi, tx, len + 1, 100);
    csn_high(dev);
}

// Send SPI command
static uint8_t send_command(nrf24_t *dev, uint8_t cmd) {
    uint8_t status;

    csn_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, &cmd, &status, 1, 100);
    csn_high(dev);

    return status;
}

/* ============ Public Functions ============ */

void nrf24_init(nrf24_t *dev, SPI_HandleTypeDef *hspi,
                GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    dev->hspi = hspi;
    dev->ce_port = ce_port;
    dev->ce_pin = ce_pin;
    dev->csn_port = csn_port;
    dev->csn_pin = csn_pin;

    dev->payload_size = 32;
    dev->addr_width = 5;
    dev->dynamic_payloads = false;
    dev->ack_payloads = false;

    ce_low(dev);
    csn_high(dev);
}

bool nrf24_begin(nrf24_t *dev) {
    // Delay for power up
    HAL_Delay(5);

    // Set default config
    dev->config_reg = 0x0C;  // EN_CRC + CRC 1 byte + PWR_UP
    write_register(dev, NRF_CONFIG, dev->config_reg);

    // Set retries: 1500us delay, 15 retries
    nrf24_set_retries(dev, 5, 15);

    // Set data rate to 1Mbps
    nrf24_set_data_rate(dev, RF24_1MBPS);

    // Set channel to 76
    nrf24_set_channel(dev, 76);

    // Set PA level to MAX
    nrf24_set_pa_level(dev, RF24_PA_MAX);

    // Enable auto-ack on all pipes
    write_register(dev, EN_AA, 0x3F);

    // Enable RX addresses on pipe 0 and 1
    write_register(dev, EN_RXADDR, 0x03);

    // Set address width to 5 bytes
    write_register(dev, SETUP_AW, 0x03);

    // Set payload size for all pipes
    for(uint8_t i = 0; i < 6; i++) {
        write_register(dev, RX_PW_P0 + i, dev->payload_size);
    }

    // Clear status flags
    write_register(dev, NRF_STATUS, 0x70);

    // Flush FIFOs
    nrf24_flush_rx(dev);
    nrf24_flush_tx(dev);

    // Power up
    nrf24_power_up(dev);

    // Check if chip is responding
    uint8_t setup = read_register(dev, RF_SETUP);
    if(setup == 0x00 || setup == 0xFF) {
        return false;  // Chip not found
    }

    return true;
}

void nrf24_set_channel(nrf24_t *dev, uint8_t channel) {
    if(channel > 125) channel = 125;
    write_register(dev, RF_CH, channel);
}

void nrf24_set_retries(nrf24_t *dev, uint8_t delay, uint8_t count) {
    write_register(dev, SETUP_RETR, (delay & 0x0F) << 4 | (count & 0x0F));
}

void nrf24_set_pa_level(nrf24_t *dev, uint8_t level) {
    uint8_t setup = read_register(dev, RF_SETUP) & 0xF8;

    switch(level) {
        case RF24_PA_MIN:
            setup |= 0x00;
            break;
        case RF24_PA_LOW:
            setup |= 0x02;
            break;
        case RF24_PA_HIGH:
            setup |= 0x04;
            break;
        case RF24_PA_MAX:
            setup |= 0x06;
            break;
    }

    write_register(dev, RF_SETUP, setup);
}

void nrf24_set_data_rate(nrf24_t *dev, uint8_t speed) {
    uint8_t setup = read_register(dev, RF_SETUP) & ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));

    switch(speed) {
        case RF24_1MBPS:
            break;
        case RF24_2MBPS:
            setup |= (1 << RF_DR_HIGH);
            break;
        case RF24_250KBPS:
            setup |= (1 << RF_DR_LOW);
            break;
    }

    write_register(dev, RF_SETUP, setup);
}

void nrf24_set_crc_length(nrf24_t *dev, uint8_t length) {
    uint8_t config = dev->config_reg & ~((1 << CRCO) | (1 << EN_CRC));

    if(length == RF24_CRC_DISABLED) {
        // Do nothing
    } else if(length == RF24_CRC_8) {
        config |= (1 << EN_CRC);
    } else {
        config |= (1 << EN_CRC) | (1 << CRCO);
    }

    dev->config_reg = config;
    write_register(dev, NRF_CONFIG, config);
}

void nrf24_set_payload_size(nrf24_t *dev, uint8_t size) {
    if(size > 32) size = 32;
    dev->payload_size = size;
}

void nrf24_open_writing_pipe(nrf24_t *dev, const uint8_t *address) {
    // Write TX_ADDR
    write_register_multi(dev, TX_ADDR, address, dev->addr_width);

    // Write RX_ADDR_P0 (for auto-ack)
    write_register_multi(dev, RX_ADDR_P0, address, dev->addr_width);
}

void nrf24_open_reading_pipe(nrf24_t *dev, uint8_t pipe, const uint8_t *address) {
    if(pipe > 5) return;

    if(pipe == 0) {
        memcpy(dev->pipe0_reading_address, address, dev->addr_width);
    }

    if(pipe < 2) {
        write_register_multi(dev, RX_ADDR_P0 + pipe, address, dev->addr_width);
    } else {
        write_register(dev, RX_ADDR_P0 + pipe, address[0]);
    }

    // Enable pipe
    uint8_t en_rxaddr = read_register(dev, EN_RXADDR);
    write_register(dev, EN_RXADDR, en_rxaddr | (1 << pipe));
}

void nrf24_close_reading_pipe(nrf24_t *dev, uint8_t pipe) {
    uint8_t en_rxaddr = read_register(dev, EN_RXADDR);
    write_register(dev, EN_RXADDR, en_rxaddr & ~(1 << pipe));
}

void nrf24_start_listening(nrf24_t *dev) {
    dev->config_reg |= (1 << PRIM_RX);
    write_register(dev, NRF_CONFIG, dev->config_reg);
    write_register(dev, NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    ce_high(dev);
    delay_us(130);
}

void nrf24_stop_listening(nrf24_t *dev) {
    ce_low(dev);
    delay_us(100);

    nrf24_flush_tx(dev);
    nrf24_flush_rx(dev);

    dev->config_reg &= ~(1 << PRIM_RX);
    write_register(dev, NRF_CONFIG, dev->config_reg);
}

bool nrf24_write(nrf24_t *dev, const void *buf, uint8_t len) {
    uint8_t tx_buf[33];

    // Prepare payload
    tx_buf[0] = W_TX_PAYLOAD;
    memcpy(&tx_buf[1], buf, len);

    // Pad if necessary
    if(len < dev->payload_size) {
        memset(&tx_buf[1 + len], 0, dev->payload_size - len);
        len = dev->payload_size;
    }

    // Write payload
    csn_low(dev);
    HAL_SPI_Transmit(dev->hspi, tx_buf, len + 1, 100);
    csn_high(dev);

    // Pulse CE to start transmission
    ce_high(dev);
    delay_us(15);
    ce_low(dev);

    // Wait for TX to complete
    uint32_t start = HAL_GetTick();
    while(1) {
        uint8_t status = nrf24_get_status(dev);

        if(status & ((1 << TX_DS) | (1 << MAX_RT))) {
            // Clear flags
            write_register(dev, NRF_STATUS, (1 << TX_DS) | (1 << MAX_RT));

            if(status & (1 << MAX_RT)) {
                nrf24_flush_tx(dev);
                return false;  // Max retries reached
            }

            return true;  // Success
        }

        if(HAL_GetTick() - start > 100) {
            return false;  // Timeout
        }
    }
}

bool nrf24_available(nrf24_t *dev) {
    return nrf24_available_pipe(dev, NULL);
}

bool nrf24_available_pipe(nrf24_t *dev, uint8_t *pipe_num) {
    uint8_t status = nrf24_get_status(dev);

    if(status & (1 << RX_DR)) {
        if(pipe_num) {
            *pipe_num = (status >> RX_P_NO) & 0x07;
        }
        return true;
    }

    // Check FIFO
    uint8_t fifo = read_register(dev, FIFO_STATUS);
    return !(fifo & (1 << RX_EMPTY));
}

void nrf24_read(nrf24_t *dev, void *buf, uint8_t len) {
    uint8_t tx_buf[33];
    uint8_t rx_buf[33];

    tx_buf[0] = R_RX_PAYLOAD;
    memset(&tx_buf[1], 0xFF, len);

    csn_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx_buf, rx_buf, len + 1, 100);
    csn_high(dev);

    memcpy(buf, &rx_buf[1], len);

    // Clear RX flag
    write_register(dev, NRF_STATUS, (1 << RX_DR));
}

void nrf24_power_up(nrf24_t *dev) {
    if(!(dev->config_reg & (1 << PWR_UP))) {
        dev->config_reg |= (1 << PWR_UP);
        write_register(dev, NRF_CONFIG, dev->config_reg);
        HAL_Delay(5);
    }
}

void nrf24_power_down(nrf24_t *dev) {
    ce_low(dev);
    dev->config_reg &= ~(1 << PWR_UP);
    write_register(dev, NRF_CONFIG, dev->config_reg);
}

uint8_t nrf24_get_status(nrf24_t *dev) {
    return send_command(dev, NOP_CMD);
}

void nrf24_flush_rx(nrf24_t *dev) {
    send_command(dev, FLUSH_RX);
}

void nrf24_flush_tx(nrf24_t *dev) {
    send_command(dev, FLUSH_TX);
}

void nrf24_enable_dynamic_payloads(nrf24_t *dev) {
    write_register(dev, FEATURE, read_register(dev, FEATURE) | (1 << 2));
    write_register(dev, DYNPD, 0x3F);
    dev->dynamic_payloads = true;
}

void nrf24_disable_dynamic_payloads(nrf24_t *dev) {
    write_register(dev, FEATURE, read_register(dev, FEATURE) & ~(1 << 2));
    write_register(dev, DYNPD, 0x00);
    dev->dynamic_payloads = false;
}

void nrf24_enable_ack_payload(nrf24_t *dev) {
    write_register(dev, FEATURE, read_register(dev, FEATURE) | 0x06);
    write_register(dev, DYNPD, 0x3F);
    dev->ack_payloads = true;
}

void nrf24_write_ack_payload(nrf24_t *dev, uint8_t pipe, const void *buf, uint8_t len) {
    uint8_t tx_buf[33];

    tx_buf[0] = W_ACK_PAYLOAD | (pipe & 0x07);
    memcpy(&tx_buf[1], buf, len);

    csn_low(dev);
    HAL_SPI_Transmit(dev->hspi, tx_buf, len + 1, 100);
    csn_high(dev);
}

bool nrf24_test_carrier(nrf24_t *dev) {
    return read_register(dev, RPD) & 1;
}

bool nrf24_test_rpd(nrf24_t *dev) {
    return read_register(dev, RPD) & 1;
}

bool nrf24_is_ack_payload_available(nrf24_t *dev) {
    // Đọc FIFO_STATUS register
    uint8_t fifo_status = read_register(dev, FIFO_STATUS);

    // Kiểm tra RX_EMPTY bit (bit 0)
    // Nếu bit = 0 nghĩa là RX FIFO có data (ACK payload)
    // Nếu bit = 1 nghĩa là RX FIFO rỗng (không có ACK payload)
    return !(fifo_status & (1 << RX_EMPTY));
}



void nrf24_print_status(nrf24_t *dev, uint8_t status) {
    printf("STATUS=0x%02x RX_DR=%d TX_DS=%d MAX_RT=%d RX_P_NO=%d TX_FULL=%d\r\n",
           status,
           (status & (1 << RX_DR)) ? 1 : 0,
           (status & (1 << TX_DS)) ? 1 : 0,
           (status & (1 << MAX_RT)) ? 1 : 0,
           (status >> RX_P_NO) & 0x07,
           (status & (1 << TX_FULL)) ? 1 : 0);
}

void nrf24_print_observe_tx(nrf24_t *dev) {
    uint8_t observe = read_register(dev, OBSERVE_TX);
    printf("OBSERVE_TX=0x%02x: PLOS_CNT=%d ARC_CNT=%d\r\n",
           observe,
           (observe >> 4) & 0x0F,
           observe & 0x0F);
}
