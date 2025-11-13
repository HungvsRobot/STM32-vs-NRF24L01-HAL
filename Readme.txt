# Thư viện nRF24L01 cho STM32 HAL

Driver điều khiển module RF nRF24L01+ cho vi điều khiển STM32 sử dụng HAL library.

## Yêu cầu

- Vi điều khiển STM32 (có SPI)
- Module nRF24L01+
- Nguồn 3.3V + tụ 10µF và 100nF

## Kết nối phần cứng

| STM32 Pin | nRF24 Pin | Chức năng |
|-----------|-----------|-----------|
| PA3       | CE        | Chip Enable |
| PA4       | CSN       | Chip Select |
| PA5       | SCK       | SPI Clock |
| PA6       | MISO      | SPI MISO |
| PA7       | MOSI      | SPI MOSI |
| 3.3V      | VCC       | Nguồn |
| GND       | GND       | Mass |



