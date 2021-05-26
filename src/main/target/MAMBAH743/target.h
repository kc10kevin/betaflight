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

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "MBH7"
#define USBD_PRODUCT_STRING  "MAMBAH743"

#define LED0_PIN                PE5  //Blue
#define LED1_PIN                PE4  //Green

#define USE_BEEPER
#define BEEPER_PIN              PE3
#define BEEPER_INVERTED
//#define BEEPER_PWM_HZ           2500 

// *************** SPI1 & SPI4, Gyro & ACC *******************

#define USE_SPI
#define USE_GYRO
#define USE_ACC
#define USE_EXTI
#define USE_GYRO_EXTI

#define USE_SPI_DEVICE_1
#define GYRO_1_SPI_INSTANCE     SPI1   //MPU6000
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define GYRO_1_CS_PIN           PA4
#define GYRO_1_EXTI_PIN         PC4

#define USE_GYRO_SPI_MPU6000
#define USE_ACC_SPI_MPU6000
#define USE_ACCGYRO_BMI270
#define GYRO_1_ALIGN            CW180_DEG

#define USE_SPI_DEVICE_4 
#define GYRO_2_SPI_INSTANCE     SPI4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define GYRO_2_CS_PIN           PE11
#define GYRO_2_EXTI_PIN         PE15

#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500
#define GYRO_2_ALIGN            CW270_DEG

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_1

// *************** SPI2 OSD ***********************

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12

// *************** SPI3 ***************************

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PA15
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PB2

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define USE_FLASH_W25M
#define USE_FLASH_W25Q128FV
#define USE_FLASH_W25N01G
#define USE_FLASH_W25N02G
#define FLASH_CS_PIN            SPI3_NSS_PIN
#define FLASH_SPI_INSTANCE      SPI3

// *************** SPI6 ***************************

//#define USE_SPI_DEVICE_6
//#define SPI6_SCK_PIN            PB3
//#define SPI6_MISO_PIN           PB4
//#define SPI6_MOSI_PIN           PB5

//#define USE_BARO
//#define USE_BARO_SPI_BMP280
//#define DEFAULT_BARO_SPI_BMP280
//#define BARO_SPI_INSTANCE     SPI6
//#define BARO_CS_PIN           PD7

// *************** I2C /Baro/Mag *********************

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1            (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define BARO_I2C_INSTANCE       (I2CDEV_1)
#define USE_BARO
//#define DEFAULT_BARO_MS5611
#define DEFAULT_BARO_DPS310
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_DPS310

#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2            (I2CDEV_2)
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define MAG_I2C_INSTANCE        (I2CDEV_2)
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

// *************** UART *****************************

#define USE_VCP
#define USB_DETECT_PIN          PE6
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6

#define USE_UART3
#define UART3_TX_PIN            PD8
#define UART3_RX_PIN            PD9

#define USE_UART4
#define UART4_TX_PIN            PD1
#define UART4_RX_PIN            PD0

#define USE_UART5
#define UART5_TX_PIN            PC12
#define UART5_RX_PIN            PD2

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7  //PPM

#define USE_UART7
#define UART7_TX_PIN            PE8
#define UART7_RX_PIN            PE7

#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            PE0

#define SERIAL_PORT_COUNT       9

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART1

// *************** ADC *****************************
#define USE_DMA
#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9

#define USE_ADC
#define USE_ADC_INTERNAL   // ADC3
#define ADC1_INSTANCE ADC1
#define ADC3_INSTANCE ADC3

#define EXTERNAL1_ADC_PIN       PC0  //ADC123 EXTERNAL ADC 
#define VBAT_ADC_PIN            PC1  //ADC123 VBAT1 
#define RSSI_ADC_PIN            PC2  //ADC123 RSSI
#define CURRENT_METER_ADC_PIN   PC3  //ADC12  CURR1 

#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
//#define VBAT_SCALE_DEFAULT            110
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
//#define CURRENT_METER_SCALE_DEFAULT   250

// *************** PINIO ***************************

#define USE_PINIO
#define PINIO1_PIN              PC5  
//#define PINIO2_PIN              PD11
#define USE_PINIOBOX

// *************** Others ***************************

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY )

//#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON

#define USE_ESCSERIAL

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 11
#define USED_TIMERS    (TIM_N(1)|TIM_N(3)|TIM_N(5)|TIM_N(8)|TIM_N(16)|TIM_N(17))