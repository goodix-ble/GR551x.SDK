# GR551x Series SoC

## 1. Introduction

- The Goodix GR551x family is a single-mode, low-power Bluetooth 5.1 System-on-Chip (SoC). It can be configured as a Broadcaster, an Observer, a Central, a Peripheral, and supports the combination of all the above roles, making it an ideal choice for Internet of Things (IoT) and smart wearable devices.

- Based on ARM® Cortex®-M4F CPU core, the GR551x integrates Bluetooth 5.1 Protocol Stack, a 2.4 GHz RF transceiver, on-chip programmable Flash memory, RAM, and multiple peripherals.

- The [GR551x SDK](https://www.goodix.com/en/software_tool/gr551x_sdk) is the software development kit for the GR551x SoC, offering a complete solution that integrates the protocol stacks, application samples and hardware drivers.



## 2. Key Features

- Bluetooth 5.1 transceiver integrates Controller and Host layers
  - Support these data transmission rates: 1 Mbps, 2 Mbps, LR (500 kbps, 125 kbps)
  - TX power: -20 dBm ~ +7 dBm
  - -96 dBm reception sensitivity (under the 1 Mbps mode)
  - -93 dBm reception sensitivity (under the 2 Mbps mode)
  - -99 dBm reception sensitivity (under the LR 500 kbps mode)
  - -102 dBm reception sensitivity (under the LR 125 kbps mode)
  - TX current: 5.6 mA @ 0 dBm，1 Mbps
  - RX current: 4.8 mA @ 1 Mbps
- Built-in ARM® Cortex®-M4F 32-bit micro-processor, supporting floating-point operation
  - Maximum frequency: 64 MHz
  - Power consumption: 51 μA/MHz
- Memory
  - Flash Configurations for GR5515 series
    - GR5515I0NDA: no embedded Flash
    - GR5515IENDU: embedded with 512 KB Flash
    - All other SoCs: embedded with 1 MB flash
  - Flash Configurations for GR5513 series
    - GR5513 series: embedded with 512 KB Flash
- Power Management
  - On-chip DC-DC Converter
  - On-chip I/O LDO to provide I/O voltage and supply external components
  - Supply voltage: 2.2 V to 3.8 V
  - I/O voltage: 1.8 V to 3.3 V
  - OFF mode: 0.15 µA (Typical), chip in reset
  - Ultra deep sleep mode: 1.8 µA (Typical), no memory retention
  - Sleep mode: 2.7 µA (Typical), 256 KB memory retention

- Peripherals
  - 2 QSPI interfaces, up to 32 MHz
  - 2 SPI interfaces (1 SPI Master Interface with 2 Slave CS pins + 1 SPI Slave Interface), up to 32 MHz
  - 2 I2C interfaces (Supports 100 kHz, 400 kHz, 1 MHz, 2 MHz)
  - 2 I2S interfaces (1 I2S Master Interface + 1 I2S Slave Interface)
  - 2 UART interfaces (One with DMA channel)
  - 13-bit ADC, up to 1 Msps, 8 channels (5 external test channels and 3 internal signal channels), supporting both single-ended and differential inputs
  - ISO 7816 interface
  - Two PWM modules with edge alignment mode and center alignment mode, each with three channels
  - Built-in temperature and voltage sensors
  - 2 general-purpose, 32-bit timer modules
  - 1 dual timer module composed of two programmable 32-bit or 16-bit down counters
  - 1 AON hardware timer
  - 2 watchdog timers (one for the system and one is always-on)
  - 1 real-time counter (RTC), can be used as Calendar
  - Wake-up comparator
  - Supports up to 39 multiplexed GPIO pins
- Security
  - Complete secure computing engine:
    - AES 128-bit/192-bit/256-bit encryption (ECB and CBC)
    - Keyed Hash Message Authentication Code(HMAC)
    - PKC
    - TRNG
  - Comprehensive security operation mechanism:
    - Secure boot
    - Encrypted firmware runs directly from Flash
    - Fuse for encrypted key storage
    - Differentiate application data key and firmware key, supporting one data per device/product
- Packages
  - QFN56: 7 mm * 7 mm * 0.75 mm, 0.40 mm pitch
  - BGA68: 5.3 mm * 5.3 mm * 0.88 mm , 0.50 mm pitch
  - BGA55: 3.5 mm * 3.5 mm * 0.60 mm, 0.40 mm pitch
  - QFN40: 5 mm * 5 mm * 0.75 mm, 0.40 mm pitch
- Operating temperature range: -40°C to +85°C



## 3. Production Details

|                       |                    | GR5515IGND                                | GR5515I0NDA                               | GR5515IENDU                               | GR5515GGBD                                | GR5515RGBD                                | GR5513BENDU                               |
| --------------------- | ------------------ | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- | ----------------------------------------- |
| Status                |                    | Active                                    | Active                                    | Active                                    | Active                                    | Active                                    | Active                                    |
| Protocol              | Bluetooth LE [1]   | 5.1                                       | 5.1                                       | 5.1                                       | 5.1                                       | 5.1                                       | 5.1                                       |
|                       | Bluetooth Mesh     | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
| Core System           | CPU                | Cortex®-M4F                               | Cortex®-M4F                               | Cortex®-M4F                               | Cortex®-M4F                               | Cortex®-M4F                               | Cortex®-M4F                               |
|                       | Clocks             | 64 MHz / 32K   Hz                         | 64 MHz / 32   KHz                         | 64 MHz / 32   KHz                         | 64 MHz / 32   KHz                         | 64 MHz / 32   KHz                         | 64 MHz / 32   KHz                         |
|                       | Cache              | 8 KB                                      | 8 KB                                      | 8 KB                                      | 8 KB                                      | 8 KB                                      | 8 KB                                      |
|                       | RAM                | 256 KB                                    | 256 KB                                    | 256 KB                                    | 256 KB                                    | 256 KB                                    | 128 KB                                    |
|                       | OTP                |                                           |                                           |                                           |                                           |                                           |                                           |
|                       | Flash              | 1 MB                                      | External   Flash                          | 512 KB                                    | 1 MB                                      | 1 MB                                      | 512 KB                                    |
| Security              | Root of Trust      | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | Secure Key Store   | 4                                         | 4                                         | 4                                         | 4                                         | 4                                         | 4                                         |
|                       | PKC                | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | RSA                | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | AES                | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | ECC                | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | TRNG               | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
| Radio                 | Frequency          | 2.4 GHz                                   | 2.4 GHz                                   | 2.4 GHz                                   | 2.4 GHz                                   | 2.4 GHz                                   | 2.4 GHz                                   |
|                       | Maximum Tx Power   | 7 dBm                                     | 7 dBm                                     | 7 dBm                                     | 7 dBm                                     | 7 dBm                                     | 7 dBm                                     |
|                       | Rx Sensitivity     | -96   dBm(@1Mbps)                         | -96   dBm(@1Mbps)                         | -96   dBm(@1Mbps)                         | -96   dBm(@1Mbps)                         | -96   dBm(@1Mbps)                         | -96   dBm(@1Mbps)                         |
| Peripheral            | UART               | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         |
|                       | SPI                | 1 * SPIM / 1   * SPIS                     | 1 * SPIM / 1   * SPIS                     | 1 * SPIM / 1   * SPIS                     | 1 * SPIM / 1   * SPIS                     | 1 * SPIM / 1   * SPIS                     | 1 * SPIM / 1   * SPIS                     |
|                       | I2C                | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         |
|                       | QSPI               | 2                                         | 2                                         | 2                                         | 0                                         | 2                                         | 1                                         |
|                       | Timers             | 4                                         | 4                                         | 4                                         | 4                                         | 4                                         | 4                                         |
|                       | PWM                | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         | 2                                         |
|                       | RTC                | 1                                         | 1                                         | 1                                         | 1                                         | 1                                         | 1                                         |
|                       | I2S                | 1 * I2SM / 1   * I2SS                     | 1 * I2SM / 1   * I2SS                     | 1 * I2SM / 1   * I2SS                     | 1 * I2SM / 1   * I2SS                     | 1 * I2SM / 1   * I2SS                     | 1 * I2SM / 1   * I2SS                     |
|                       | ADC                | 13-bit                                    | 13-bit                                    | 13-bit                                    | 13-bit                                    | 13-bit                                    | 13-bit                                    |
|                       | Comparator         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | Temperature Sensor | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         | ●                                         |
|                       | GPIO               | 39                                        | 39                                        | 39                                        | 29                                        | 39                                        | 22                                        |
| Packages              | Type               | QFN56                                     | QFN56                                     | QFN56                                     | BGA55                                     | BGA68                                     | QFN40                                     |
|                       | Dimensions         | 7.0   * 7.0 mm                            | 7.0   * 7.0 mm                            | 7.0   * 7.0 mm                            | 3.5   *3.5 mm                             | 5.3   * 5.3 mm                            | 5.0   * 5.0 mm                            |
| Certification         |                    | PSA Level 1        SIG BQB (QDID: 119449) | PSA Level 1        SIG BQB (QDID: 119449) | PSA Level 1        SIG BQB (QDID: 119449) | PSA Level 1        SIG BQB (QDID: 119449) | PSA Level 1        SIG BQB (QDID: 119449) | PSA Level 1        SIG BQB (QDID: 119449) |
| Operating Temperature |                    | -40℃ - 85℃                                | -40℃ - 85℃                                | -40℃ - 85℃                                | -40℃ - 85℃                                | -40℃ - 85℃                                | -40℃ - 85℃                                |
| Supply Voltage Range  |                    | 2.2 V - 3.8 V                             | 2.2 V - 3.8 V                             | 2.2 V - 3.8 V                             | 2.2 V - 3.8 V                             | 2.2 V - 3.8 V                             | 2.2 V - 3.8 V                             |
| Development Kits      |                    | GR5515   Starter Kit                      | GR5515   Starter Kit                      | GR5515   Starter Kit                      | GR5515   Starter Kit                      | GR5515   Starter Kit                      | GR5515   Starter Kit                      |



## 4. Change Log

- Click to view the [change log](../../wiki/Change-Notes-for-GR551x)

