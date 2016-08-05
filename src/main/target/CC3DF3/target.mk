F3_TARGETS  += $(TARGET)
FEATURES    = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/barometer_ms5611.c \
		    drivers/barometer_bmp085.c \
            drivers/barometer_bmp280.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f30x.c \
            drivers/sonar_hcsr04.c \
            drivers/serial_softserial.c
