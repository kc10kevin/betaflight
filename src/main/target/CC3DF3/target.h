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


#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT
#define CONFIG_PREFER_ACC_ON

#define LED0            PB3

#define BEEPER          PA15
#define BEEPER_OPT      PA2

#define USE_EXTI
#define MPU_INT_EXTI            PA3
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU INT, SDCardDetect
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO

#define MPU6000_CS_PIN          PA4
#define MPU6000_SPI_INSTANCE    SPI1

#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN      CW270_DEG
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN       CW270_DEG

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_CS_PIN           PB12
#define M25P16_SPI_INSTANCE     SPI2

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USB_IO

#define USE_VCP
#define USE_UART1
#define USE_UART3
#define USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT 4

#define SOFTSERIAL_1_TIMER      TIM3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 1 // PWM 2
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 2 // PWM 3

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

//#define UART2_TX_PIN            PA14
//#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)


//#define USE_I2C
//#define I2C_DEVICE (I2CDEV_2)  // SCL/PB10, SDA/PB11 - no I2C on PB10/PB11 on the STM32F303

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
//#define RSSI_ADC_PIN            PA1
#define CURRENT_METER_ADC_PIN   PA1

#define LED_STRIP

#define USE_LED_STRIP_ON_DMA1_CHANNEL3
#define WS2811_PIN                      PB4
#define WS2811_TIMER                    TIM16
#define WS2811_DMA_CHANNEL              DMA1_Channel3
#define WS2811_IRQ                      DMA1_Channel3_IRQn
#define WS2811_DMA_TC_FLAG              DMA1_FLAG_TC3
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_CH3_HANDLER

#define SONAR
#define SONAR_ECHO_PIN          PB1
#define SONAR_TRIGGER_PIN       PB5

#define DEFAULT_FEATURES        FEATURE_BLACKBOX
#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

#define SPEKTRUM_BIND
// USART3,
#define BIND_PIN                PB11

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         0xffff
#define TARGET_IO_PORTF         (BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 12
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16))