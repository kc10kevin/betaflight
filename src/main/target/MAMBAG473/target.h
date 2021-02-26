/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

//#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "MBG4"
#define USBD_PRODUCT_STRING     "MAMBAG473"

// ******* LEDs and BEEPER ********

#define LED0_PIN                PC15
#define LED1_PIN                PC14

#define USE_BEEPER
#define BEEPER_PIN              PC13
#define BEEPER_INVERTED

//#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

#define USE_DSHOT
#define USE_DSHOT_DMAR

#define USE_DMA

//#define USE_PINIO
//#define PINIO1_PIN PB0 // Bluetooth mode control, PB0 is connected to the 36 pin (P2.0) of the Bluetooth chip. Replace PB0 with the pin for your flight control and 36-pin connection

// ******* GYRO and ACC ********

#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO_1_CS_PIN           PA4
#define GYRO_1_SPI_INSTANCE     SPI1

#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_ICM20689
#define GYRO_1_ALIGN            CW180_DEG

#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_ICM20689

// ******* SERIAL ********

#define USE_VCP
#define USB_DETECT_PIN          PA8
#define USE_USB_DETECT
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PB3
#define UART2_RX_PIN            PB4

#define UART3_TX_PIN            PB10
#define UART3_RX_PIN            PB11

#define UART4_TX_PIN            PC10
#define UART4_RX_PIN            PC11

//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2

#define SERIAL_PORT_COUNT       5 //VCP, UART1-UART4

// ******* SPI ********

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PA4
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define USE_I2C_PULLUP
#define I2C1_SCL                PA13
#define I2C1_SDA                PA14

#define BARO_I2C_INSTANCE       I2C_DEVICE
#define MAG_I2C_INSTANCE        I2C_DEVICE

#define USE_MAG
#define USE_MAG_HMC5883                   //External, connect to I2C1
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL
#define MAG_HMC5883_ALIGN       CW180_DEG

#define USE_BARO
#define USE_BARO_MS5611                  //External, connect to I2C1
#define USE_BARO_BMP280                  //onboard

// ******* ADC ********

//#define USE_ADC
//#define ADC_INSTANCE            ADC3
//#define ADC3_DMA_OPT            0

//#define VBAT_ADC_PIN            PB0
//#define RSSI_ADC_PIN            PB1
//#define CURRENT_METER_ADC_PIN   PB2

// NEW ADC CODE

//#ifdef USE_DMA_SPEC
// #define UART1_TX_DMA_OPT         0
// #define UART2_TX_DMA_OPT         1
// #define UART3_TX_DMA_OPT         2
// #define UART4_TX_DMA_OPT         3
// #define UART5_TX_DMA_OPT         4
// #define UART6_TX_DMA_OPT         5
// #define UART7_TX_DMA_OPT         6
// #define UART8_TX_DMA_OPT         7
#define ADC1_DMA_OPT                8
#define ADC2_DMA_OPT                9
//#else
//#define ADC1_DMA_STREAM             DMA2_Stream0
//#define ADC2_DMA_STREAM             DMA2_Stream3
//#endif

#define USE_ADC
#define USE_ADC_INTERNAL

#define ADC1_INSTANCE               ADC1
#define ADC2_INSTANCE               ADC2
//#define ADC3_INSTANCE               ADC3

#define VBAT_ADC_PIN                PB0
#define RSSI_ADC_PIN                PB1
#define CURRENT_METER_ADC_PIN       PB2

// ******* OSD ********

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      SPI2_NSS_PIN

//******* FLASH ********

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define FLASH_CS_PIN            PC6
#define FLASH_SPI_INSTANCE      SPI2

#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// ******* FEATURES ********

#define USE_OSD

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_UART           SERIAL_PORT_USART1
#define SERIALRX_PROVIDER       SERIALRX_SBUS

#define DEFAULT_FEATURES                (FEATURE_OSD | FEATURE_TELEMETRY) // | FEATURE_SOFTSERIAL)
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC

#define VBAT_SCALE_DEFAULT            110

//#define USE_ESCSERIAL
//#define ESCSERIAL_TIMER_TX_PIN PB9

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff

#define USABLE_TIMER_CHANNEL_COUNT 5
#define USED_TIMERS             ( TIM_N(5) | TIM_N(8) )