F7X2RE_TARGETS  += $(TARGET)
FEATURES        += VCP SDCARD ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_icm20689.c \
            drivers/max7456.c
#            drivers/barometer/barometer_ms5611.c \
#            drivers/barometer/barometer_spi_ms5611.c