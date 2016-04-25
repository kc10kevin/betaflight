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
    
#define TARGET_BOARD_IDENTIFIER "KBCC3DF3"

#define LED0
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEPER
#define BEEP_GPIO GPIOA
#define BEEP_PIN Pin_15 // PA15 (Beeper)
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOA

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
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define M25P16_CS_GPIO          GPIOB
#define M25P16_CS_PIN           GPIO_Pin_12
#define M25P16_SPI_INSTANCE     SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 8

#define USB_IO

#define USE_VCP
#define USE_USART1
#define USE_USART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT 4

#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 1 // PWM 2
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 2 // PWM 3

//#ifndef UART1_GPIO
#define UART1_TX_PIN        GPIO_Pin_9  // PA9
#define UART1_RX_PIN        GPIO_Pin_10 // PA10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource9
#define UART1_RX_PINSOURCE  GPIO_PinSource10
//#endif

#define UART3_TX_PIN        GPIO_Pin_10 // PB10
#define UART3_RX_PIN        GPIO_Pin_11 // PB11
#define UART3_GPIO_AF       GPIO_AF_7
#define UART3_GPIO          GPIOB
#define UART3_TX_PINSOURCE  GPIO_PinSource10
#define UART3_RX_PINSOURCE  GPIO_PinSource11

//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_2)  // SCL/PB10, SDA/PB11 - no I2C on PB10/PB11 on the STM32F303

#define USE_ADC
#define BOARD_HAS_VOLTAGE_DIVIDER

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

//#define CURRENT_METER_ADC_GPIO      GPIOB
//#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
//#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_9

#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_2

#define LED_STRIP

#define LED_STRIP_TIMER TIM16

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_GPIO                     GPIOB
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
#define WS2811_GPIO_AF                  GPIO_AF_1
#define WS2811_PIN                      GPIO_Pin_4 // TIM16_CH1
#define WS2811_PIN_SOURCE               GPIO_PinSource4
#define WS2811_TIMER                    TIM16
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define SONAR
#define GPS
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART3, PB11 (Flexport)
#define BIND_PORT  GPIOB
#define BIND_PIN   Pin_11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE