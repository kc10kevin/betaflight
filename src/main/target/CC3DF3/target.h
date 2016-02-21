/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
    
#define TARGET_BOARD_IDENTIFIER "CC3DF3"

#define LED0
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_GPIO GPIOA
#define BEEP_PIN Pin_15 // PA15 (Beeper)
#define BEEP_PERIPHERAL RCC_APBPeriph_GPIOA

#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define MPU6000_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOA
#define MPU6000_CS_GPIO                  GPIOA
#define MPU6000_CS_PIN                   GPIO_Pin_4
#define MPU6000_SPI_INSTANCE             SPI1

#define GYRO
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW270_DEG

#define ACC
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW270_DEG

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_NSS_PIN            Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_SCK_PIN            GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7

#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

/* NOT NEEDED
#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_NSS_PIN            Pin_12
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI2_SCK_PIN            Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15
*/

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           GPIO_Pin_12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 7

#define USB_IO

#define USE_VCP
#define USE_USART1
#define USE_USART3
#define SERIAL_PORT_COUNT 3

#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
#endif

#define UART3_TX_PIN        GPIO_Pin_10 // PB10
#define UART3_RX_PIN        GPIO_Pin_11 // PB11
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

#define USE_I2C
#define I2C_DEVICE (I2CDEV_2)  // Flex port - SCL/PB10, SDA/PB11

#define USE_ADC

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_0

#define CURRENT_METER_ADC_GPIO      GPIOB
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_9

#define RSSI_ADC_GPIO               GPIOB
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_0
#define RSSI_ADC_CHANNEL            ADC_Channel_8

//define GPS
//define LED_STRIP

//define LED_STRIP_TIMER TIM16

//define USE_LED_STRIP_ON_DMA1_CHANNEL3
//define WS2811_GPIO                     GPIOA
//define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
//define WS2811_GPIO_AF                  GPIO_AF_1
//define WS2811_PIN                      GPIO_Pin_6 // TIM16_CH1
//define WS2811_PIN_SOURCE               GPIO_PinSource6
//define WS2811_TIMER                    TIM16
//define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
//define WS2811_DMA_CHANNEL              DMA1_Channel3
//define WS2811_IRQ                      DMA1_Channel3_IRQn

#define BLACKBOX
//define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define USE_SERVOS
#define USE_CLI