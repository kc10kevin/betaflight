##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=betaflight
ConfigurationName      :=Debug
WorkspacePath          := "C:\dev\KKNG\betaflight"
ProjectPath            := "C:\dev\KKNG\betaflight"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Kevin
Date                   :=08/02/2016
CodeLitePath           :="C:\Program Files\CodeLite"
LinkerName             :=C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-g++.exe
SharedObjectLinkerName :=C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-g++.exe -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="betaflight.txt"
PCHCompileFlags        :=
MakeDirCommand         :=makedir
RcCmpOptions           := 
RcCompilerName         :=windres
LinkOptions            :=  -O0
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch). $(LibraryPathSwitch)Debug 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-ar.exe rcu
CXX      := C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-g++.exe
CC       := C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-gcc.exe
CXXFLAGS :=  -g -Wall $(Preprocessors)
CFLAGS   :=   $(Preprocessors)
ASFLAGS  := 
AS       := C:/dev/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi-as.exe


##
## User defined environment variables
##
CodeLiteDir:=C:\Program Files\CodeLite
Objects0=$(IntermediateDirectory)/stmloader_loader.c$(ObjectSuffix) $(IntermediateDirectory)/stmloader_serial.c$(ObjectSuffix) $(IntermediateDirectory)/stmloader_stmbootloader.c$(ObjectSuffix) $(IntermediateDirectory)/main_build_config.c$(ObjectSuffix) $(IntermediateDirectory)/main_debug.c$(ObjectSuffix) $(IntermediateDirectory)/main_main.c$(ObjectSuffix) $(IntermediateDirectory)/main_mw.c$(ObjectSuffix) $(IntermediateDirectory)/main_scheduler.c$(ObjectSuffix) $(IntermediateDirectory)/main_version.c$(ObjectSuffix) $(IntermediateDirectory)/unit_alignsensor_unittest.cc$(ObjectSuffix) \
	$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_battery_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_encoding_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_flight_imu_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(ObjectSuffix) \
	$(IntermediateDirectory)/unit_io_serial_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_ledstrip_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_lowpass_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_maths_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_rc_controls_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_rx_rx_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/unit_ws2811_unittest.cc$(ObjectSuffix) $(IntermediateDirectory)/io_beeper.c$(ObjectSuffix) \
	$(IntermediateDirectory)/io_display.c$(ObjectSuffix) $(IntermediateDirectory)/io_flashfs.c$(ObjectSuffix) $(IntermediateDirectory)/io_gps.c$(ObjectSuffix) $(IntermediateDirectory)/io_i2c_bst.c$(ObjectSuffix) $(IntermediateDirectory)/io_ledstrip.c$(ObjectSuffix) $(IntermediateDirectory)/io_rc_controls.c$(ObjectSuffix) $(IntermediateDirectory)/io_rc_curves.c$(ObjectSuffix) $(IntermediateDirectory)/io_serial.c$(ObjectSuffix) $(IntermediateDirectory)/io_serial_1wire.c$(ObjectSuffix) $(IntermediateDirectory)/io_serial_cli.c$(ObjectSuffix) \
	$(IntermediateDirectory)/io_serial_msp.c$(ObjectSuffix) $(IntermediateDirectory)/io_statusindicator.c$(ObjectSuffix) $(IntermediateDirectory)/io_transponder_ir.c$(ObjectSuffix) $(IntermediateDirectory)/blackbox_blackbox.c$(ObjectSuffix) $(IntermediateDirectory)/blackbox_blackbox_io.c$(ObjectSuffix) $(IntermediateDirectory)/common_colorconversion.c$(ObjectSuffix) $(IntermediateDirectory)/common_encoding.c$(ObjectSuffix) $(IntermediateDirectory)/common_filter.c$(ObjectSuffix) $(IntermediateDirectory)/common_maths.c$(ObjectSuffix) $(IntermediateDirectory)/common_printf.c$(ObjectSuffix) \
	$(IntermediateDirectory)/common_typeconversion.c$(ObjectSuffix) $(IntermediateDirectory)/config_config.c$(ObjectSuffix) $(IntermediateDirectory)/config_runtime_config.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_adxl345.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_bma280.c$(ObjectSuffix) 

Objects1=$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_mma845x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_mpu.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_adc.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_adc_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_adc_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_barometer_bmp085.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_barometer_bmp280.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_barometer_ms5611.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_buf_writer.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_bus_i2c_soft.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_bus_spi.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_compass_ak8963.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_compass_ak8975.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_compass_hmc5883l.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_flash_m25p16.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_gyro_sync.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_inverter.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_light_led.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_light_ws2811strip.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_pwm_mapping.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_pwm_output.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_pwm_rx.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sdcard.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sdcard_standard.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_serial.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_serial_softserial.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_serial_uart.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_serial_usb_vcp.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sonar_hcsr04.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sound_beeper.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(ObjectSuffix) 

Objects2=$(IntermediateDirectory)/drivers_system.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_system_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_system_stm32f30x.c$(ObjectSuffix) \
	$(IntermediateDirectory)/drivers_timer.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_timer_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_timer_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_transponder_ir.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_usb_detection.c$(ObjectSuffix) $(IntermediateDirectory)/drivers_usb_io.c$(ObjectSuffix) $(IntermediateDirectory)/flight_altitudehold.c$(ObjectSuffix) $(IntermediateDirectory)/flight_failsafe.c$(ObjectSuffix) $(IntermediateDirectory)/flight_gps_conversion.c$(ObjectSuffix) \
	$(IntermediateDirectory)/flight_gtune.c$(ObjectSuffix) $(IntermediateDirectory)/flight_imu.c$(ObjectSuffix) $(IntermediateDirectory)/flight_lowpass.c$(ObjectSuffix) $(IntermediateDirectory)/flight_mixer.c$(ObjectSuffix) $(IntermediateDirectory)/flight_navigation.c$(ObjectSuffix) $(IntermediateDirectory)/flight_pid.c$(ObjectSuffix) $(IntermediateDirectory)/rx_ibus.c$(ObjectSuffix) $(IntermediateDirectory)/rx_msp.c$(ObjectSuffix) $(IntermediateDirectory)/rx_pwm.c$(ObjectSuffix) $(IntermediateDirectory)/rx_rx.c$(ObjectSuffix) \
	$(IntermediateDirectory)/rx_sbus.c$(ObjectSuffix) $(IntermediateDirectory)/rx_spektrum.c$(ObjectSuffix) $(IntermediateDirectory)/rx_sumd.c$(ObjectSuffix) $(IntermediateDirectory)/rx_sumh.c$(ObjectSuffix) $(IntermediateDirectory)/rx_xbus.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_acceleration.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_barometer.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_battery.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_boardalignment.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_compass.c$(ObjectSuffix) \
	$(IntermediateDirectory)/sensors_gyro.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_initialisation.c$(ObjectSuffix) $(IntermediateDirectory)/sensors_sonar.c$(ObjectSuffix) $(IntermediateDirectory)/telemetry_frsky.c$(ObjectSuffix) $(IntermediateDirectory)/telemetry_hott.c$(ObjectSuffix) $(IntermediateDirectory)/telemetry_ltm.c$(ObjectSuffix) $(IntermediateDirectory)/telemetry_smartport.c$(ObjectSuffix) $(IntermediateDirectory)/telemetry_telemetry.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_hw_config.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_stm32_it.c$(ObjectSuffix) \
	$(IntermediateDirectory)/vcp_usb_desc.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_usb_endp.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_usb_istr.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_usb_prop.c$(ObjectSuffix) $(IntermediateDirectory)/vcp_usb_pwr.c$(ObjectSuffix) $(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(ObjectSuffix) $(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(ObjectSuffix) 

Objects3=$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/CJMCU_hardware_revision.c$(ObjectSuffix) $(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(ObjectSuffix) \
	$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/NAZE_hardware_revision.c$(ObjectSuffix) $(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/RMDO_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(ObjectSuffix) $(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(ObjectSuffix) \
	$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(ObjectSuffix) $(IntermediateDirectory)/asyncfatfs_fat_standard.c$(ObjectSuffix) $(IntermediateDirectory)/src_gtest-all.cc$(ObjectSuffix) $(IntermediateDirectory)/src_gtest_main.cc$(ObjectSuffix) $(IntermediateDirectory)/src_usb_core.c$(ObjectSuffix) $(IntermediateDirectory)/src_usb_init.c$(ObjectSuffix) $(IntermediateDirectory)/src_usb_int.c$(ObjectSuffix) $(IntermediateDirectory)/src_usb_mem.c$(ObjectSuffix) $(IntermediateDirectory)/src_usb_regs.c$(ObjectSuffix) $(IntermediateDirectory)/src_usb_sil.c$(ObjectSuffix) \
	$(IntermediateDirectory)/src_stm32f30x_adc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_can.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_comp.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_crc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_dac.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_dma.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_exti.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_flash.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_gpio.c$(ObjectSuffix) \
	$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_i2c.c$(ObjectSuffix) 

Objects4=$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_misc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_opamp.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_pwr.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_rcc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_rtc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_spi.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_syscfg.c$(ObjectSuffix) \
	$(IntermediateDirectory)/src_stm32f30x_tim.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_usart.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f30x_wwdg.c$(ObjectSuffix) $(IntermediateDirectory)/src_misc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_adc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_bkp.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_can.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_cec.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_crc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_dac.c$(ObjectSuffix) \
	$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_dma.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_exti.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_flash.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_fsmc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_gpio.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_i2c.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_iwdg.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_pwr.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_rcc.c$(ObjectSuffix) \
	$(IntermediateDirectory)/src_stm32f10x_rtc.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_sdio.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_spi.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_tim.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_usart.c$(ObjectSuffix) $(IntermediateDirectory)/src_stm32f10x_wwdg.c$(ObjectSuffix) $(IntermediateDirectory)/CoreSupport_core_cm3.c$(ObjectSuffix) $(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(ObjectSuffix) $(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(ObjectSuffix) $(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(ObjectSuffix) \
	



Objects=$(Objects0) $(Objects1) $(Objects2) $(Objects3) $(Objects4) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	@echo $(Objects1) >> $(ObjectsFileList)
	@echo $(Objects2) >> $(ObjectsFileList)
	@echo $(Objects3) >> $(ObjectsFileList)
	@echo $(Objects4) >> $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@$(MakeDirCommand) "./Debug"


$(IntermediateDirectory)/.d:
	@$(MakeDirCommand) "./Debug"

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/stmloader_loader.c$(ObjectSuffix): support/stmloader/loader.c $(IntermediateDirectory)/stmloader_loader.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/support/stmloader/loader.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stmloader_loader.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stmloader_loader.c$(DependSuffix): support/stmloader/loader.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stmloader_loader.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stmloader_loader.c$(DependSuffix) -MM "support/stmloader/loader.c"

$(IntermediateDirectory)/stmloader_loader.c$(PreprocessSuffix): support/stmloader/loader.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stmloader_loader.c$(PreprocessSuffix) "support/stmloader/loader.c"

$(IntermediateDirectory)/stmloader_serial.c$(ObjectSuffix): support/stmloader/serial.c $(IntermediateDirectory)/stmloader_serial.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/support/stmloader/serial.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stmloader_serial.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stmloader_serial.c$(DependSuffix): support/stmloader/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stmloader_serial.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stmloader_serial.c$(DependSuffix) -MM "support/stmloader/serial.c"

$(IntermediateDirectory)/stmloader_serial.c$(PreprocessSuffix): support/stmloader/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stmloader_serial.c$(PreprocessSuffix) "support/stmloader/serial.c"

$(IntermediateDirectory)/stmloader_stmbootloader.c$(ObjectSuffix): support/stmloader/stmbootloader.c $(IntermediateDirectory)/stmloader_stmbootloader.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/support/stmloader/stmbootloader.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stmloader_stmbootloader.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stmloader_stmbootloader.c$(DependSuffix): support/stmloader/stmbootloader.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stmloader_stmbootloader.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stmloader_stmbootloader.c$(DependSuffix) -MM "support/stmloader/stmbootloader.c"

$(IntermediateDirectory)/stmloader_stmbootloader.c$(PreprocessSuffix): support/stmloader/stmbootloader.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stmloader_stmbootloader.c$(PreprocessSuffix) "support/stmloader/stmbootloader.c"

$(IntermediateDirectory)/main_build_config.c$(ObjectSuffix): src/main/build_config.c $(IntermediateDirectory)/main_build_config.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/build_config.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_build_config.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_build_config.c$(DependSuffix): src/main/build_config.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_build_config.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_build_config.c$(DependSuffix) -MM "src/main/build_config.c"

$(IntermediateDirectory)/main_build_config.c$(PreprocessSuffix): src/main/build_config.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_build_config.c$(PreprocessSuffix) "src/main/build_config.c"

$(IntermediateDirectory)/main_debug.c$(ObjectSuffix): src/main/debug.c $(IntermediateDirectory)/main_debug.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/debug.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_debug.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_debug.c$(DependSuffix): src/main/debug.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_debug.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_debug.c$(DependSuffix) -MM "src/main/debug.c"

$(IntermediateDirectory)/main_debug.c$(PreprocessSuffix): src/main/debug.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_debug.c$(PreprocessSuffix) "src/main/debug.c"

$(IntermediateDirectory)/main_main.c$(ObjectSuffix): src/main/main.c $(IntermediateDirectory)/main_main.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/main.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_main.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_main.c$(DependSuffix): src/main/main.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_main.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_main.c$(DependSuffix) -MM "src/main/main.c"

$(IntermediateDirectory)/main_main.c$(PreprocessSuffix): src/main/main.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_main.c$(PreprocessSuffix) "src/main/main.c"

$(IntermediateDirectory)/main_mw.c$(ObjectSuffix): src/main/mw.c $(IntermediateDirectory)/main_mw.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/mw.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_mw.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_mw.c$(DependSuffix): src/main/mw.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_mw.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_mw.c$(DependSuffix) -MM "src/main/mw.c"

$(IntermediateDirectory)/main_mw.c$(PreprocessSuffix): src/main/mw.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_mw.c$(PreprocessSuffix) "src/main/mw.c"

$(IntermediateDirectory)/main_scheduler.c$(ObjectSuffix): src/main/scheduler.c $(IntermediateDirectory)/main_scheduler.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/scheduler.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_scheduler.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_scheduler.c$(DependSuffix): src/main/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_scheduler.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_scheduler.c$(DependSuffix) -MM "src/main/scheduler.c"

$(IntermediateDirectory)/main_scheduler.c$(PreprocessSuffix): src/main/scheduler.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_scheduler.c$(PreprocessSuffix) "src/main/scheduler.c"

$(IntermediateDirectory)/main_version.c$(ObjectSuffix): src/main/version.c $(IntermediateDirectory)/main_version.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/version.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main_version.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main_version.c$(DependSuffix): src/main/version.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main_version.c$(ObjectSuffix) -MF$(IntermediateDirectory)/main_version.c$(DependSuffix) -MM "src/main/version.c"

$(IntermediateDirectory)/main_version.c$(PreprocessSuffix): src/main/version.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main_version.c$(PreprocessSuffix) "src/main/version.c"

$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(ObjectSuffix): src/test/unit/alignsensor_unittest.cc $(IntermediateDirectory)/unit_alignsensor_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/alignsensor_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(DependSuffix): src/test/unit/alignsensor_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(DependSuffix) -MM "src/test/unit/alignsensor_unittest.cc"

$(IntermediateDirectory)/unit_alignsensor_unittest.cc$(PreprocessSuffix): src/test/unit/alignsensor_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_alignsensor_unittest.cc$(PreprocessSuffix) "src/test/unit/alignsensor_unittest.cc"

$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(ObjectSuffix): src/test/unit/altitude_hold_unittest.cc $(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/altitude_hold_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(DependSuffix): src/test/unit/altitude_hold_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(DependSuffix) -MM "src/test/unit/altitude_hold_unittest.cc"

$(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(PreprocessSuffix): src/test/unit/altitude_hold_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_altitude_hold_unittest.cc$(PreprocessSuffix) "src/test/unit/altitude_hold_unittest.cc"

$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(ObjectSuffix): src/test/unit/baro_bmp085_unittest.cc $(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/baro_bmp085_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(DependSuffix): src/test/unit/baro_bmp085_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(DependSuffix) -MM "src/test/unit/baro_bmp085_unittest.cc"

$(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(PreprocessSuffix): src/test/unit/baro_bmp085_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_baro_bmp085_unittest.cc$(PreprocessSuffix) "src/test/unit/baro_bmp085_unittest.cc"

$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(ObjectSuffix): src/test/unit/baro_bmp280_unittest.cc $(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/baro_bmp280_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(DependSuffix): src/test/unit/baro_bmp280_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(DependSuffix) -MM "src/test/unit/baro_bmp280_unittest.cc"

$(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(PreprocessSuffix): src/test/unit/baro_bmp280_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_baro_bmp280_unittest.cc$(PreprocessSuffix) "src/test/unit/baro_bmp280_unittest.cc"

$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(ObjectSuffix): src/test/unit/baro_ms5611_unittest.cc $(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/baro_ms5611_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(DependSuffix): src/test/unit/baro_ms5611_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(DependSuffix) -MM "src/test/unit/baro_ms5611_unittest.cc"

$(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(PreprocessSuffix): src/test/unit/baro_ms5611_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_baro_ms5611_unittest.cc$(PreprocessSuffix) "src/test/unit/baro_ms5611_unittest.cc"

$(IntermediateDirectory)/unit_battery_unittest.cc$(ObjectSuffix): src/test/unit/battery_unittest.cc $(IntermediateDirectory)/unit_battery_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/battery_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_battery_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_battery_unittest.cc$(DependSuffix): src/test/unit/battery_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_battery_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_battery_unittest.cc$(DependSuffix) -MM "src/test/unit/battery_unittest.cc"

$(IntermediateDirectory)/unit_battery_unittest.cc$(PreprocessSuffix): src/test/unit/battery_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_battery_unittest.cc$(PreprocessSuffix) "src/test/unit/battery_unittest.cc"

$(IntermediateDirectory)/unit_encoding_unittest.cc$(ObjectSuffix): src/test/unit/encoding_unittest.cc $(IntermediateDirectory)/unit_encoding_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/encoding_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_encoding_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_encoding_unittest.cc$(DependSuffix): src/test/unit/encoding_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_encoding_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_encoding_unittest.cc$(DependSuffix) -MM "src/test/unit/encoding_unittest.cc"

$(IntermediateDirectory)/unit_encoding_unittest.cc$(PreprocessSuffix): src/test/unit/encoding_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_encoding_unittest.cc$(PreprocessSuffix) "src/test/unit/encoding_unittest.cc"

$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(ObjectSuffix): src/test/unit/flight_failsafe_unittest.cc $(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/flight_failsafe_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(DependSuffix): src/test/unit/flight_failsafe_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(DependSuffix) -MM "src/test/unit/flight_failsafe_unittest.cc"

$(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(PreprocessSuffix): src/test/unit/flight_failsafe_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_flight_failsafe_unittest.cc$(PreprocessSuffix) "src/test/unit/flight_failsafe_unittest.cc"

$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(ObjectSuffix): src/test/unit/flight_imu_unittest.cc $(IntermediateDirectory)/unit_flight_imu_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/flight_imu_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(DependSuffix): src/test/unit/flight_imu_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(DependSuffix) -MM "src/test/unit/flight_imu_unittest.cc"

$(IntermediateDirectory)/unit_flight_imu_unittest.cc$(PreprocessSuffix): src/test/unit/flight_imu_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_flight_imu_unittest.cc$(PreprocessSuffix) "src/test/unit/flight_imu_unittest.cc"

$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(ObjectSuffix): src/test/unit/flight_mixer_unittest.cc $(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/flight_mixer_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(DependSuffix): src/test/unit/flight_mixer_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(DependSuffix) -MM "src/test/unit/flight_mixer_unittest.cc"

$(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(PreprocessSuffix): src/test/unit/flight_mixer_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_flight_mixer_unittest.cc$(PreprocessSuffix) "src/test/unit/flight_mixer_unittest.cc"

$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(ObjectSuffix): src/test/unit/gps_conversion_unittest.cc $(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/gps_conversion_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(DependSuffix): src/test/unit/gps_conversion_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(DependSuffix) -MM "src/test/unit/gps_conversion_unittest.cc"

$(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(PreprocessSuffix): src/test/unit/gps_conversion_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_gps_conversion_unittest.cc$(PreprocessSuffix) "src/test/unit/gps_conversion_unittest.cc"

$(IntermediateDirectory)/unit_io_serial_unittest.cc$(ObjectSuffix): src/test/unit/io_serial_unittest.cc $(IntermediateDirectory)/unit_io_serial_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/io_serial_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_io_serial_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_io_serial_unittest.cc$(DependSuffix): src/test/unit/io_serial_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_io_serial_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_io_serial_unittest.cc$(DependSuffix) -MM "src/test/unit/io_serial_unittest.cc"

$(IntermediateDirectory)/unit_io_serial_unittest.cc$(PreprocessSuffix): src/test/unit/io_serial_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_io_serial_unittest.cc$(PreprocessSuffix) "src/test/unit/io_serial_unittest.cc"

$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(ObjectSuffix): src/test/unit/ledstrip_unittest.cc $(IntermediateDirectory)/unit_ledstrip_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/ledstrip_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(DependSuffix): src/test/unit/ledstrip_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(DependSuffix) -MM "src/test/unit/ledstrip_unittest.cc"

$(IntermediateDirectory)/unit_ledstrip_unittest.cc$(PreprocessSuffix): src/test/unit/ledstrip_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_ledstrip_unittest.cc$(PreprocessSuffix) "src/test/unit/ledstrip_unittest.cc"

$(IntermediateDirectory)/unit_lowpass_unittest.cc$(ObjectSuffix): src/test/unit/lowpass_unittest.cc $(IntermediateDirectory)/unit_lowpass_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/lowpass_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_lowpass_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_lowpass_unittest.cc$(DependSuffix): src/test/unit/lowpass_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_lowpass_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_lowpass_unittest.cc$(DependSuffix) -MM "src/test/unit/lowpass_unittest.cc"

$(IntermediateDirectory)/unit_lowpass_unittest.cc$(PreprocessSuffix): src/test/unit/lowpass_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_lowpass_unittest.cc$(PreprocessSuffix) "src/test/unit/lowpass_unittest.cc"

$(IntermediateDirectory)/unit_maths_unittest.cc$(ObjectSuffix): src/test/unit/maths_unittest.cc $(IntermediateDirectory)/unit_maths_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/maths_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_maths_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_maths_unittest.cc$(DependSuffix): src/test/unit/maths_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_maths_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_maths_unittest.cc$(DependSuffix) -MM "src/test/unit/maths_unittest.cc"

$(IntermediateDirectory)/unit_maths_unittest.cc$(PreprocessSuffix): src/test/unit/maths_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_maths_unittest.cc$(PreprocessSuffix) "src/test/unit/maths_unittest.cc"

$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(ObjectSuffix): src/test/unit/rc_controls_unittest.cc $(IntermediateDirectory)/unit_rc_controls_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/rc_controls_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(DependSuffix): src/test/unit/rc_controls_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(DependSuffix) -MM "src/test/unit/rc_controls_unittest.cc"

$(IntermediateDirectory)/unit_rc_controls_unittest.cc$(PreprocessSuffix): src/test/unit/rc_controls_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_rc_controls_unittest.cc$(PreprocessSuffix) "src/test/unit/rc_controls_unittest.cc"

$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(ObjectSuffix): src/test/unit/rx_ranges_unittest.cc $(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/rx_ranges_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(DependSuffix): src/test/unit/rx_ranges_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(DependSuffix) -MM "src/test/unit/rx_ranges_unittest.cc"

$(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(PreprocessSuffix): src/test/unit/rx_ranges_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_rx_ranges_unittest.cc$(PreprocessSuffix) "src/test/unit/rx_ranges_unittest.cc"

$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(ObjectSuffix): src/test/unit/rx_rx_unittest.cc $(IntermediateDirectory)/unit_rx_rx_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/rx_rx_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(DependSuffix): src/test/unit/rx_rx_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(DependSuffix) -MM "src/test/unit/rx_rx_unittest.cc"

$(IntermediateDirectory)/unit_rx_rx_unittest.cc$(PreprocessSuffix): src/test/unit/rx_rx_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_rx_rx_unittest.cc$(PreprocessSuffix) "src/test/unit/rx_rx_unittest.cc"

$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(ObjectSuffix): src/test/unit/telemetry_hott_unittest.cc $(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/telemetry_hott_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(DependSuffix): src/test/unit/telemetry_hott_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(DependSuffix) -MM "src/test/unit/telemetry_hott_unittest.cc"

$(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(PreprocessSuffix): src/test/unit/telemetry_hott_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_telemetry_hott_unittest.cc$(PreprocessSuffix) "src/test/unit/telemetry_hott_unittest.cc"

$(IntermediateDirectory)/unit_ws2811_unittest.cc$(ObjectSuffix): src/test/unit/ws2811_unittest.cc $(IntermediateDirectory)/unit_ws2811_unittest.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/test/unit/ws2811_unittest.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/unit_ws2811_unittest.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/unit_ws2811_unittest.cc$(DependSuffix): src/test/unit/ws2811_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/unit_ws2811_unittest.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/unit_ws2811_unittest.cc$(DependSuffix) -MM "src/test/unit/ws2811_unittest.cc"

$(IntermediateDirectory)/unit_ws2811_unittest.cc$(PreprocessSuffix): src/test/unit/ws2811_unittest.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/unit_ws2811_unittest.cc$(PreprocessSuffix) "src/test/unit/ws2811_unittest.cc"

$(IntermediateDirectory)/io_beeper.c$(ObjectSuffix): src/main/io/beeper.c $(IntermediateDirectory)/io_beeper.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/beeper.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_beeper.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_beeper.c$(DependSuffix): src/main/io/beeper.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_beeper.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_beeper.c$(DependSuffix) -MM "src/main/io/beeper.c"

$(IntermediateDirectory)/io_beeper.c$(PreprocessSuffix): src/main/io/beeper.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_beeper.c$(PreprocessSuffix) "src/main/io/beeper.c"

$(IntermediateDirectory)/io_display.c$(ObjectSuffix): src/main/io/display.c $(IntermediateDirectory)/io_display.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/display.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_display.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_display.c$(DependSuffix): src/main/io/display.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_display.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_display.c$(DependSuffix) -MM "src/main/io/display.c"

$(IntermediateDirectory)/io_display.c$(PreprocessSuffix): src/main/io/display.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_display.c$(PreprocessSuffix) "src/main/io/display.c"

$(IntermediateDirectory)/io_flashfs.c$(ObjectSuffix): src/main/io/flashfs.c $(IntermediateDirectory)/io_flashfs.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/flashfs.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_flashfs.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_flashfs.c$(DependSuffix): src/main/io/flashfs.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_flashfs.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_flashfs.c$(DependSuffix) -MM "src/main/io/flashfs.c"

$(IntermediateDirectory)/io_flashfs.c$(PreprocessSuffix): src/main/io/flashfs.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_flashfs.c$(PreprocessSuffix) "src/main/io/flashfs.c"

$(IntermediateDirectory)/io_gps.c$(ObjectSuffix): src/main/io/gps.c $(IntermediateDirectory)/io_gps.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/gps.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_gps.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_gps.c$(DependSuffix): src/main/io/gps.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_gps.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_gps.c$(DependSuffix) -MM "src/main/io/gps.c"

$(IntermediateDirectory)/io_gps.c$(PreprocessSuffix): src/main/io/gps.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_gps.c$(PreprocessSuffix) "src/main/io/gps.c"

$(IntermediateDirectory)/io_i2c_bst.c$(ObjectSuffix): src/main/io/i2c_bst.c $(IntermediateDirectory)/io_i2c_bst.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/i2c_bst.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_i2c_bst.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_i2c_bst.c$(DependSuffix): src/main/io/i2c_bst.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_i2c_bst.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_i2c_bst.c$(DependSuffix) -MM "src/main/io/i2c_bst.c"

$(IntermediateDirectory)/io_i2c_bst.c$(PreprocessSuffix): src/main/io/i2c_bst.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_i2c_bst.c$(PreprocessSuffix) "src/main/io/i2c_bst.c"

$(IntermediateDirectory)/io_ledstrip.c$(ObjectSuffix): src/main/io/ledstrip.c $(IntermediateDirectory)/io_ledstrip.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/ledstrip.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_ledstrip.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_ledstrip.c$(DependSuffix): src/main/io/ledstrip.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_ledstrip.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_ledstrip.c$(DependSuffix) -MM "src/main/io/ledstrip.c"

$(IntermediateDirectory)/io_ledstrip.c$(PreprocessSuffix): src/main/io/ledstrip.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_ledstrip.c$(PreprocessSuffix) "src/main/io/ledstrip.c"

$(IntermediateDirectory)/io_rc_controls.c$(ObjectSuffix): src/main/io/rc_controls.c $(IntermediateDirectory)/io_rc_controls.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/rc_controls.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_rc_controls.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_rc_controls.c$(DependSuffix): src/main/io/rc_controls.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_rc_controls.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_rc_controls.c$(DependSuffix) -MM "src/main/io/rc_controls.c"

$(IntermediateDirectory)/io_rc_controls.c$(PreprocessSuffix): src/main/io/rc_controls.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_rc_controls.c$(PreprocessSuffix) "src/main/io/rc_controls.c"

$(IntermediateDirectory)/io_rc_curves.c$(ObjectSuffix): src/main/io/rc_curves.c $(IntermediateDirectory)/io_rc_curves.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/rc_curves.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_rc_curves.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_rc_curves.c$(DependSuffix): src/main/io/rc_curves.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_rc_curves.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_rc_curves.c$(DependSuffix) -MM "src/main/io/rc_curves.c"

$(IntermediateDirectory)/io_rc_curves.c$(PreprocessSuffix): src/main/io/rc_curves.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_rc_curves.c$(PreprocessSuffix) "src/main/io/rc_curves.c"

$(IntermediateDirectory)/io_serial.c$(ObjectSuffix): src/main/io/serial.c $(IntermediateDirectory)/io_serial.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/serial.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_serial.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_serial.c$(DependSuffix): src/main/io/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_serial.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_serial.c$(DependSuffix) -MM "src/main/io/serial.c"

$(IntermediateDirectory)/io_serial.c$(PreprocessSuffix): src/main/io/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_serial.c$(PreprocessSuffix) "src/main/io/serial.c"

$(IntermediateDirectory)/io_serial_1wire.c$(ObjectSuffix): src/main/io/serial_1wire.c $(IntermediateDirectory)/io_serial_1wire.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/serial_1wire.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_serial_1wire.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_serial_1wire.c$(DependSuffix): src/main/io/serial_1wire.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_serial_1wire.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_serial_1wire.c$(DependSuffix) -MM "src/main/io/serial_1wire.c"

$(IntermediateDirectory)/io_serial_1wire.c$(PreprocessSuffix): src/main/io/serial_1wire.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_serial_1wire.c$(PreprocessSuffix) "src/main/io/serial_1wire.c"

$(IntermediateDirectory)/io_serial_cli.c$(ObjectSuffix): src/main/io/serial_cli.c $(IntermediateDirectory)/io_serial_cli.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/serial_cli.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_serial_cli.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_serial_cli.c$(DependSuffix): src/main/io/serial_cli.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_serial_cli.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_serial_cli.c$(DependSuffix) -MM "src/main/io/serial_cli.c"

$(IntermediateDirectory)/io_serial_cli.c$(PreprocessSuffix): src/main/io/serial_cli.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_serial_cli.c$(PreprocessSuffix) "src/main/io/serial_cli.c"

$(IntermediateDirectory)/io_serial_msp.c$(ObjectSuffix): src/main/io/serial_msp.c $(IntermediateDirectory)/io_serial_msp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/serial_msp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_serial_msp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_serial_msp.c$(DependSuffix): src/main/io/serial_msp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_serial_msp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_serial_msp.c$(DependSuffix) -MM "src/main/io/serial_msp.c"

$(IntermediateDirectory)/io_serial_msp.c$(PreprocessSuffix): src/main/io/serial_msp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_serial_msp.c$(PreprocessSuffix) "src/main/io/serial_msp.c"

$(IntermediateDirectory)/io_statusindicator.c$(ObjectSuffix): src/main/io/statusindicator.c $(IntermediateDirectory)/io_statusindicator.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/statusindicator.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_statusindicator.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_statusindicator.c$(DependSuffix): src/main/io/statusindicator.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_statusindicator.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_statusindicator.c$(DependSuffix) -MM "src/main/io/statusindicator.c"

$(IntermediateDirectory)/io_statusindicator.c$(PreprocessSuffix): src/main/io/statusindicator.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_statusindicator.c$(PreprocessSuffix) "src/main/io/statusindicator.c"

$(IntermediateDirectory)/io_transponder_ir.c$(ObjectSuffix): src/main/io/transponder_ir.c $(IntermediateDirectory)/io_transponder_ir.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/transponder_ir.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/io_transponder_ir.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/io_transponder_ir.c$(DependSuffix): src/main/io/transponder_ir.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/io_transponder_ir.c$(ObjectSuffix) -MF$(IntermediateDirectory)/io_transponder_ir.c$(DependSuffix) -MM "src/main/io/transponder_ir.c"

$(IntermediateDirectory)/io_transponder_ir.c$(PreprocessSuffix): src/main/io/transponder_ir.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/io_transponder_ir.c$(PreprocessSuffix) "src/main/io/transponder_ir.c"

$(IntermediateDirectory)/blackbox_blackbox.c$(ObjectSuffix): src/main/blackbox/blackbox.c $(IntermediateDirectory)/blackbox_blackbox.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/blackbox/blackbox.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/blackbox_blackbox.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/blackbox_blackbox.c$(DependSuffix): src/main/blackbox/blackbox.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/blackbox_blackbox.c$(ObjectSuffix) -MF$(IntermediateDirectory)/blackbox_blackbox.c$(DependSuffix) -MM "src/main/blackbox/blackbox.c"

$(IntermediateDirectory)/blackbox_blackbox.c$(PreprocessSuffix): src/main/blackbox/blackbox.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/blackbox_blackbox.c$(PreprocessSuffix) "src/main/blackbox/blackbox.c"

$(IntermediateDirectory)/blackbox_blackbox_io.c$(ObjectSuffix): src/main/blackbox/blackbox_io.c $(IntermediateDirectory)/blackbox_blackbox_io.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/blackbox/blackbox_io.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/blackbox_blackbox_io.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/blackbox_blackbox_io.c$(DependSuffix): src/main/blackbox/blackbox_io.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/blackbox_blackbox_io.c$(ObjectSuffix) -MF$(IntermediateDirectory)/blackbox_blackbox_io.c$(DependSuffix) -MM "src/main/blackbox/blackbox_io.c"

$(IntermediateDirectory)/blackbox_blackbox_io.c$(PreprocessSuffix): src/main/blackbox/blackbox_io.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/blackbox_blackbox_io.c$(PreprocessSuffix) "src/main/blackbox/blackbox_io.c"

$(IntermediateDirectory)/common_colorconversion.c$(ObjectSuffix): src/main/common/colorconversion.c $(IntermediateDirectory)/common_colorconversion.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/colorconversion.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_colorconversion.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_colorconversion.c$(DependSuffix): src/main/common/colorconversion.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_colorconversion.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_colorconversion.c$(DependSuffix) -MM "src/main/common/colorconversion.c"

$(IntermediateDirectory)/common_colorconversion.c$(PreprocessSuffix): src/main/common/colorconversion.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_colorconversion.c$(PreprocessSuffix) "src/main/common/colorconversion.c"

$(IntermediateDirectory)/common_encoding.c$(ObjectSuffix): src/main/common/encoding.c $(IntermediateDirectory)/common_encoding.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/encoding.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_encoding.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_encoding.c$(DependSuffix): src/main/common/encoding.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_encoding.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_encoding.c$(DependSuffix) -MM "src/main/common/encoding.c"

$(IntermediateDirectory)/common_encoding.c$(PreprocessSuffix): src/main/common/encoding.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_encoding.c$(PreprocessSuffix) "src/main/common/encoding.c"

$(IntermediateDirectory)/common_filter.c$(ObjectSuffix): src/main/common/filter.c $(IntermediateDirectory)/common_filter.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/filter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_filter.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_filter.c$(DependSuffix): src/main/common/filter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_filter.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_filter.c$(DependSuffix) -MM "src/main/common/filter.c"

$(IntermediateDirectory)/common_filter.c$(PreprocessSuffix): src/main/common/filter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_filter.c$(PreprocessSuffix) "src/main/common/filter.c"

$(IntermediateDirectory)/common_maths.c$(ObjectSuffix): src/main/common/maths.c $(IntermediateDirectory)/common_maths.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/maths.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_maths.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_maths.c$(DependSuffix): src/main/common/maths.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_maths.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_maths.c$(DependSuffix) -MM "src/main/common/maths.c"

$(IntermediateDirectory)/common_maths.c$(PreprocessSuffix): src/main/common/maths.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_maths.c$(PreprocessSuffix) "src/main/common/maths.c"

$(IntermediateDirectory)/common_printf.c$(ObjectSuffix): src/main/common/printf.c $(IntermediateDirectory)/common_printf.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/printf.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_printf.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_printf.c$(DependSuffix): src/main/common/printf.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_printf.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_printf.c$(DependSuffix) -MM "src/main/common/printf.c"

$(IntermediateDirectory)/common_printf.c$(PreprocessSuffix): src/main/common/printf.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_printf.c$(PreprocessSuffix) "src/main/common/printf.c"

$(IntermediateDirectory)/common_typeconversion.c$(ObjectSuffix): src/main/common/typeconversion.c $(IntermediateDirectory)/common_typeconversion.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/common/typeconversion.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/common_typeconversion.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/common_typeconversion.c$(DependSuffix): src/main/common/typeconversion.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/common_typeconversion.c$(ObjectSuffix) -MF$(IntermediateDirectory)/common_typeconversion.c$(DependSuffix) -MM "src/main/common/typeconversion.c"

$(IntermediateDirectory)/common_typeconversion.c$(PreprocessSuffix): src/main/common/typeconversion.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/common_typeconversion.c$(PreprocessSuffix) "src/main/common/typeconversion.c"

$(IntermediateDirectory)/config_config.c$(ObjectSuffix): src/main/config/config.c $(IntermediateDirectory)/config_config.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/config/config.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/config_config.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/config_config.c$(DependSuffix): src/main/config/config.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/config_config.c$(ObjectSuffix) -MF$(IntermediateDirectory)/config_config.c$(DependSuffix) -MM "src/main/config/config.c"

$(IntermediateDirectory)/config_config.c$(PreprocessSuffix): src/main/config/config.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/config_config.c$(PreprocessSuffix) "src/main/config/config.c"

$(IntermediateDirectory)/config_runtime_config.c$(ObjectSuffix): src/main/config/runtime_config.c $(IntermediateDirectory)/config_runtime_config.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/config/runtime_config.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/config_runtime_config.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/config_runtime_config.c$(DependSuffix): src/main/config/runtime_config.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/config_runtime_config.c$(ObjectSuffix) -MF$(IntermediateDirectory)/config_runtime_config.c$(DependSuffix) -MM "src/main/config/runtime_config.c"

$(IntermediateDirectory)/config_runtime_config.c$(PreprocessSuffix): src/main/config/runtime_config.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/config_runtime_config.c$(PreprocessSuffix) "src/main/config/runtime_config.c"

$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(ObjectSuffix): src/main/drivers/accgyro_adxl345.c $(IntermediateDirectory)/drivers_accgyro_adxl345.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_adxl345.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(DependSuffix): src/main/drivers/accgyro_adxl345.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(DependSuffix) -MM "src/main/drivers/accgyro_adxl345.c"

$(IntermediateDirectory)/drivers_accgyro_adxl345.c$(PreprocessSuffix): src/main/drivers/accgyro_adxl345.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_adxl345.c$(PreprocessSuffix) "src/main/drivers/accgyro_adxl345.c"

$(IntermediateDirectory)/drivers_accgyro_bma280.c$(ObjectSuffix): src/main/drivers/accgyro_bma280.c $(IntermediateDirectory)/drivers_accgyro_bma280.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_bma280.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_bma280.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_bma280.c$(DependSuffix): src/main/drivers/accgyro_bma280.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_bma280.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_bma280.c$(DependSuffix) -MM "src/main/drivers/accgyro_bma280.c"

$(IntermediateDirectory)/drivers_accgyro_bma280.c$(PreprocessSuffix): src/main/drivers/accgyro_bma280.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_bma280.c$(PreprocessSuffix) "src/main/drivers/accgyro_bma280.c"

$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(ObjectSuffix): src/main/drivers/accgyro_l3g4200d.c $(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_l3g4200d.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(DependSuffix): src/main/drivers/accgyro_l3g4200d.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(DependSuffix) -MM "src/main/drivers/accgyro_l3g4200d.c"

$(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(PreprocessSuffix): src/main/drivers/accgyro_l3g4200d.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_l3g4200d.c$(PreprocessSuffix) "src/main/drivers/accgyro_l3g4200d.c"

$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(ObjectSuffix): src/main/drivers/accgyro_l3gd20.c $(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_l3gd20.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(DependSuffix): src/main/drivers/accgyro_l3gd20.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(DependSuffix) -MM "src/main/drivers/accgyro_l3gd20.c"

$(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(PreprocessSuffix): src/main/drivers/accgyro_l3gd20.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_l3gd20.c$(PreprocessSuffix) "src/main/drivers/accgyro_l3gd20.c"

$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(ObjectSuffix): src/main/drivers/accgyro_lsm303dlhc.c $(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_lsm303dlhc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(DependSuffix): src/main/drivers/accgyro_lsm303dlhc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(DependSuffix) -MM "src/main/drivers/accgyro_lsm303dlhc.c"

$(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(PreprocessSuffix): src/main/drivers/accgyro_lsm303dlhc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_lsm303dlhc.c$(PreprocessSuffix) "src/main/drivers/accgyro_lsm303dlhc.c"

$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(ObjectSuffix): src/main/drivers/accgyro_mma845x.c $(IntermediateDirectory)/drivers_accgyro_mma845x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_mma845x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(DependSuffix): src/main/drivers/accgyro_mma845x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(DependSuffix) -MM "src/main/drivers/accgyro_mma845x.c"

$(IntermediateDirectory)/drivers_accgyro_mma845x.c$(PreprocessSuffix): src/main/drivers/accgyro_mma845x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_mma845x.c$(PreprocessSuffix) "src/main/drivers/accgyro_mma845x.c"

$(IntermediateDirectory)/drivers_accgyro_mpu.c$(ObjectSuffix): src/main/drivers/accgyro_mpu.c $(IntermediateDirectory)/drivers_accgyro_mpu.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_mpu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_mpu.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_mpu.c$(DependSuffix): src/main/drivers/accgyro_mpu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_mpu.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_mpu.c$(DependSuffix) -MM "src/main/drivers/accgyro_mpu.c"

$(IntermediateDirectory)/drivers_accgyro_mpu.c$(PreprocessSuffix): src/main/drivers/accgyro_mpu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_mpu.c$(PreprocessSuffix) "src/main/drivers/accgyro_mpu.c"

$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(ObjectSuffix): src/main/drivers/accgyro_mpu3050.c $(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_mpu3050.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(DependSuffix): src/main/drivers/accgyro_mpu3050.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(DependSuffix) -MM "src/main/drivers/accgyro_mpu3050.c"

$(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(PreprocessSuffix): src/main/drivers/accgyro_mpu3050.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_mpu3050.c$(PreprocessSuffix) "src/main/drivers/accgyro_mpu3050.c"

$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(ObjectSuffix): src/main/drivers/accgyro_mpu6050.c $(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_mpu6050.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(DependSuffix): src/main/drivers/accgyro_mpu6050.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(DependSuffix) -MM "src/main/drivers/accgyro_mpu6050.c"

$(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(PreprocessSuffix): src/main/drivers/accgyro_mpu6050.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_mpu6050.c$(PreprocessSuffix) "src/main/drivers/accgyro_mpu6050.c"

$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(ObjectSuffix): src/main/drivers/accgyro_mpu6500.c $(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_mpu6500.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(DependSuffix): src/main/drivers/accgyro_mpu6500.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(DependSuffix) -MM "src/main/drivers/accgyro_mpu6500.c"

$(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(PreprocessSuffix): src/main/drivers/accgyro_mpu6500.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_mpu6500.c$(PreprocessSuffix) "src/main/drivers/accgyro_mpu6500.c"

$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(ObjectSuffix): src/main/drivers/accgyro_spi_mpu6000.c $(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_spi_mpu6000.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(DependSuffix): src/main/drivers/accgyro_spi_mpu6000.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(DependSuffix) -MM "src/main/drivers/accgyro_spi_mpu6000.c"

$(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(PreprocessSuffix): src/main/drivers/accgyro_spi_mpu6000.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_spi_mpu6000.c$(PreprocessSuffix) "src/main/drivers/accgyro_spi_mpu6000.c"

$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(ObjectSuffix): src/main/drivers/accgyro_spi_mpu6500.c $(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/accgyro_spi_mpu6500.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(DependSuffix): src/main/drivers/accgyro_spi_mpu6500.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(DependSuffix) -MM "src/main/drivers/accgyro_spi_mpu6500.c"

$(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(PreprocessSuffix): src/main/drivers/accgyro_spi_mpu6500.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_accgyro_spi_mpu6500.c$(PreprocessSuffix) "src/main/drivers/accgyro_spi_mpu6500.c"

$(IntermediateDirectory)/drivers_adc.c$(ObjectSuffix): src/main/drivers/adc.c $(IntermediateDirectory)/drivers_adc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/adc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_adc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_adc.c$(DependSuffix): src/main/drivers/adc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_adc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_adc.c$(DependSuffix) -MM "src/main/drivers/adc.c"

$(IntermediateDirectory)/drivers_adc.c$(PreprocessSuffix): src/main/drivers/adc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_adc.c$(PreprocessSuffix) "src/main/drivers/adc.c"

$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(ObjectSuffix): src/main/drivers/adc_stm32f10x.c $(IntermediateDirectory)/drivers_adc_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/adc_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(DependSuffix): src/main/drivers/adc_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/adc_stm32f10x.c"

$(IntermediateDirectory)/drivers_adc_stm32f10x.c$(PreprocessSuffix): src/main/drivers/adc_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_adc_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/adc_stm32f10x.c"

$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(ObjectSuffix): src/main/drivers/adc_stm32f30x.c $(IntermediateDirectory)/drivers_adc_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/adc_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(DependSuffix): src/main/drivers/adc_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/adc_stm32f30x.c"

$(IntermediateDirectory)/drivers_adc_stm32f30x.c$(PreprocessSuffix): src/main/drivers/adc_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_adc_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/adc_stm32f30x.c"

$(IntermediateDirectory)/drivers_barometer_bmp085.c$(ObjectSuffix): src/main/drivers/barometer_bmp085.c $(IntermediateDirectory)/drivers_barometer_bmp085.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/barometer_bmp085.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_barometer_bmp085.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_barometer_bmp085.c$(DependSuffix): src/main/drivers/barometer_bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_barometer_bmp085.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_barometer_bmp085.c$(DependSuffix) -MM "src/main/drivers/barometer_bmp085.c"

$(IntermediateDirectory)/drivers_barometer_bmp085.c$(PreprocessSuffix): src/main/drivers/barometer_bmp085.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_barometer_bmp085.c$(PreprocessSuffix) "src/main/drivers/barometer_bmp085.c"

$(IntermediateDirectory)/drivers_barometer_bmp280.c$(ObjectSuffix): src/main/drivers/barometer_bmp280.c $(IntermediateDirectory)/drivers_barometer_bmp280.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/barometer_bmp280.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_barometer_bmp280.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_barometer_bmp280.c$(DependSuffix): src/main/drivers/barometer_bmp280.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_barometer_bmp280.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_barometer_bmp280.c$(DependSuffix) -MM "src/main/drivers/barometer_bmp280.c"

$(IntermediateDirectory)/drivers_barometer_bmp280.c$(PreprocessSuffix): src/main/drivers/barometer_bmp280.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_barometer_bmp280.c$(PreprocessSuffix) "src/main/drivers/barometer_bmp280.c"

$(IntermediateDirectory)/drivers_barometer_ms5611.c$(ObjectSuffix): src/main/drivers/barometer_ms5611.c $(IntermediateDirectory)/drivers_barometer_ms5611.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/barometer_ms5611.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_barometer_ms5611.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_barometer_ms5611.c$(DependSuffix): src/main/drivers/barometer_ms5611.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_barometer_ms5611.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_barometer_ms5611.c$(DependSuffix) -MM "src/main/drivers/barometer_ms5611.c"

$(IntermediateDirectory)/drivers_barometer_ms5611.c$(PreprocessSuffix): src/main/drivers/barometer_ms5611.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_barometer_ms5611.c$(PreprocessSuffix) "src/main/drivers/barometer_ms5611.c"

$(IntermediateDirectory)/drivers_buf_writer.c$(ObjectSuffix): src/main/drivers/buf_writer.c $(IntermediateDirectory)/drivers_buf_writer.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/buf_writer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_buf_writer.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_buf_writer.c$(DependSuffix): src/main/drivers/buf_writer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_buf_writer.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_buf_writer.c$(DependSuffix) -MM "src/main/drivers/buf_writer.c"

$(IntermediateDirectory)/drivers_buf_writer.c$(PreprocessSuffix): src/main/drivers/buf_writer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_buf_writer.c$(PreprocessSuffix) "src/main/drivers/buf_writer.c"

$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(ObjectSuffix): src/main/drivers/bus_bst_stm32f30x.c $(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/bus_bst_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(DependSuffix): src/main/drivers/bus_bst_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/bus_bst_stm32f30x.c"

$(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(PreprocessSuffix): src/main/drivers/bus_bst_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_bus_bst_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/bus_bst_stm32f30x.c"

$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(ObjectSuffix): src/main/drivers/bus_i2c_soft.c $(IntermediateDirectory)/drivers_bus_i2c_soft.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/bus_i2c_soft.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(DependSuffix): src/main/drivers/bus_i2c_soft.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(DependSuffix) -MM "src/main/drivers/bus_i2c_soft.c"

$(IntermediateDirectory)/drivers_bus_i2c_soft.c$(PreprocessSuffix): src/main/drivers/bus_i2c_soft.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_bus_i2c_soft.c$(PreprocessSuffix) "src/main/drivers/bus_i2c_soft.c"

$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(ObjectSuffix): src/main/drivers/bus_i2c_stm32f10x.c $(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/bus_i2c_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(DependSuffix): src/main/drivers/bus_i2c_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/bus_i2c_stm32f10x.c"

$(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(PreprocessSuffix): src/main/drivers/bus_i2c_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_bus_i2c_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/bus_i2c_stm32f10x.c"

$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(ObjectSuffix): src/main/drivers/bus_i2c_stm32f30x.c $(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/bus_i2c_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(DependSuffix): src/main/drivers/bus_i2c_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/bus_i2c_stm32f30x.c"

$(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(PreprocessSuffix): src/main/drivers/bus_i2c_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_bus_i2c_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/bus_i2c_stm32f30x.c"

$(IntermediateDirectory)/drivers_bus_spi.c$(ObjectSuffix): src/main/drivers/bus_spi.c $(IntermediateDirectory)/drivers_bus_spi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/bus_spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_bus_spi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_bus_spi.c$(DependSuffix): src/main/drivers/bus_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_bus_spi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_bus_spi.c$(DependSuffix) -MM "src/main/drivers/bus_spi.c"

$(IntermediateDirectory)/drivers_bus_spi.c$(PreprocessSuffix): src/main/drivers/bus_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_bus_spi.c$(PreprocessSuffix) "src/main/drivers/bus_spi.c"

$(IntermediateDirectory)/drivers_compass_ak8963.c$(ObjectSuffix): src/main/drivers/compass_ak8963.c $(IntermediateDirectory)/drivers_compass_ak8963.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/compass_ak8963.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_compass_ak8963.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_compass_ak8963.c$(DependSuffix): src/main/drivers/compass_ak8963.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_compass_ak8963.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_compass_ak8963.c$(DependSuffix) -MM "src/main/drivers/compass_ak8963.c"

$(IntermediateDirectory)/drivers_compass_ak8963.c$(PreprocessSuffix): src/main/drivers/compass_ak8963.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_compass_ak8963.c$(PreprocessSuffix) "src/main/drivers/compass_ak8963.c"

$(IntermediateDirectory)/drivers_compass_ak8975.c$(ObjectSuffix): src/main/drivers/compass_ak8975.c $(IntermediateDirectory)/drivers_compass_ak8975.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/compass_ak8975.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_compass_ak8975.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_compass_ak8975.c$(DependSuffix): src/main/drivers/compass_ak8975.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_compass_ak8975.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_compass_ak8975.c$(DependSuffix) -MM "src/main/drivers/compass_ak8975.c"

$(IntermediateDirectory)/drivers_compass_ak8975.c$(PreprocessSuffix): src/main/drivers/compass_ak8975.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_compass_ak8975.c$(PreprocessSuffix) "src/main/drivers/compass_ak8975.c"

$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(ObjectSuffix): src/main/drivers/compass_hmc5883l.c $(IntermediateDirectory)/drivers_compass_hmc5883l.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/compass_hmc5883l.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(DependSuffix): src/main/drivers/compass_hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(DependSuffix) -MM "src/main/drivers/compass_hmc5883l.c"

$(IntermediateDirectory)/drivers_compass_hmc5883l.c$(PreprocessSuffix): src/main/drivers/compass_hmc5883l.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_compass_hmc5883l.c$(PreprocessSuffix) "src/main/drivers/compass_hmc5883l.c"

$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(ObjectSuffix): src/main/drivers/display_ug2864hsweg01.c $(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/display_ug2864hsweg01.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(DependSuffix): src/main/drivers/display_ug2864hsweg01.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(DependSuffix) -MM "src/main/drivers/display_ug2864hsweg01.c"

$(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(PreprocessSuffix): src/main/drivers/display_ug2864hsweg01.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_display_ug2864hsweg01.c$(PreprocessSuffix) "src/main/drivers/display_ug2864hsweg01.c"

$(IntermediateDirectory)/drivers_flash_m25p16.c$(ObjectSuffix): src/main/drivers/flash_m25p16.c $(IntermediateDirectory)/drivers_flash_m25p16.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/flash_m25p16.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_flash_m25p16.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_flash_m25p16.c$(DependSuffix): src/main/drivers/flash_m25p16.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_flash_m25p16.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_flash_m25p16.c$(DependSuffix) -MM "src/main/drivers/flash_m25p16.c"

$(IntermediateDirectory)/drivers_flash_m25p16.c$(PreprocessSuffix): src/main/drivers/flash_m25p16.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_flash_m25p16.c$(PreprocessSuffix) "src/main/drivers/flash_m25p16.c"

$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(ObjectSuffix): src/main/drivers/gpio_stm32f10x.c $(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/gpio_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(DependSuffix): src/main/drivers/gpio_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/gpio_stm32f10x.c"

$(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(PreprocessSuffix): src/main/drivers/gpio_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_gpio_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/gpio_stm32f10x.c"

$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(ObjectSuffix): src/main/drivers/gpio_stm32f30x.c $(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/gpio_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(DependSuffix): src/main/drivers/gpio_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/gpio_stm32f30x.c"

$(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(PreprocessSuffix): src/main/drivers/gpio_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_gpio_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/gpio_stm32f30x.c"

$(IntermediateDirectory)/drivers_gyro_sync.c$(ObjectSuffix): src/main/drivers/gyro_sync.c $(IntermediateDirectory)/drivers_gyro_sync.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/gyro_sync.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_gyro_sync.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_gyro_sync.c$(DependSuffix): src/main/drivers/gyro_sync.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_gyro_sync.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_gyro_sync.c$(DependSuffix) -MM "src/main/drivers/gyro_sync.c"

$(IntermediateDirectory)/drivers_gyro_sync.c$(PreprocessSuffix): src/main/drivers/gyro_sync.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_gyro_sync.c$(PreprocessSuffix) "src/main/drivers/gyro_sync.c"

$(IntermediateDirectory)/drivers_inverter.c$(ObjectSuffix): src/main/drivers/inverter.c $(IntermediateDirectory)/drivers_inverter.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/inverter.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_inverter.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_inverter.c$(DependSuffix): src/main/drivers/inverter.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_inverter.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_inverter.c$(DependSuffix) -MM "src/main/drivers/inverter.c"

$(IntermediateDirectory)/drivers_inverter.c$(PreprocessSuffix): src/main/drivers/inverter.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_inverter.c$(PreprocessSuffix) "src/main/drivers/inverter.c"

$(IntermediateDirectory)/drivers_light_led.c$(ObjectSuffix): src/main/drivers/light_led.c $(IntermediateDirectory)/drivers_light_led.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_led.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_led.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_led.c$(DependSuffix): src/main/drivers/light_led.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_led.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_led.c$(DependSuffix) -MM "src/main/drivers/light_led.c"

$(IntermediateDirectory)/drivers_light_led.c$(PreprocessSuffix): src/main/drivers/light_led.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_led.c$(PreprocessSuffix) "src/main/drivers/light_led.c"

$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(ObjectSuffix): src/main/drivers/light_led_stm32f10x.c $(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_led_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(DependSuffix): src/main/drivers/light_led_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/light_led_stm32f10x.c"

$(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(PreprocessSuffix): src/main/drivers/light_led_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_led_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/light_led_stm32f10x.c"

$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(ObjectSuffix): src/main/drivers/light_led_stm32f30x.c $(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_led_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(DependSuffix): src/main/drivers/light_led_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/light_led_stm32f30x.c"

$(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(PreprocessSuffix): src/main/drivers/light_led_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_led_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/light_led_stm32f30x.c"

$(IntermediateDirectory)/drivers_light_ws2811strip.c$(ObjectSuffix): src/main/drivers/light_ws2811strip.c $(IntermediateDirectory)/drivers_light_ws2811strip.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_ws2811strip.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_ws2811strip.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_ws2811strip.c$(DependSuffix): src/main/drivers/light_ws2811strip.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_ws2811strip.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_ws2811strip.c$(DependSuffix) -MM "src/main/drivers/light_ws2811strip.c"

$(IntermediateDirectory)/drivers_light_ws2811strip.c$(PreprocessSuffix): src/main/drivers/light_ws2811strip.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_ws2811strip.c$(PreprocessSuffix) "src/main/drivers/light_ws2811strip.c"

$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(ObjectSuffix): src/main/drivers/light_ws2811strip_stm32f10x.c $(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_ws2811strip_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(DependSuffix): src/main/drivers/light_ws2811strip_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/light_ws2811strip_stm32f10x.c"

$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(PreprocessSuffix): src/main/drivers/light_ws2811strip_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_ws2811strip_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/light_ws2811strip_stm32f10x.c"

$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(ObjectSuffix): src/main/drivers/light_ws2811strip_stm32f30x.c $(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/light_ws2811strip_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(DependSuffix): src/main/drivers/light_ws2811strip_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/light_ws2811strip_stm32f30x.c"

$(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(PreprocessSuffix): src/main/drivers/light_ws2811strip_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_light_ws2811strip_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/light_ws2811strip_stm32f30x.c"

$(IntermediateDirectory)/drivers_pwm_mapping.c$(ObjectSuffix): src/main/drivers/pwm_mapping.c $(IntermediateDirectory)/drivers_pwm_mapping.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/pwm_mapping.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_pwm_mapping.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_pwm_mapping.c$(DependSuffix): src/main/drivers/pwm_mapping.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_pwm_mapping.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_pwm_mapping.c$(DependSuffix) -MM "src/main/drivers/pwm_mapping.c"

$(IntermediateDirectory)/drivers_pwm_mapping.c$(PreprocessSuffix): src/main/drivers/pwm_mapping.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_pwm_mapping.c$(PreprocessSuffix) "src/main/drivers/pwm_mapping.c"

$(IntermediateDirectory)/drivers_pwm_output.c$(ObjectSuffix): src/main/drivers/pwm_output.c $(IntermediateDirectory)/drivers_pwm_output.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/pwm_output.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_pwm_output.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_pwm_output.c$(DependSuffix): src/main/drivers/pwm_output.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_pwm_output.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_pwm_output.c$(DependSuffix) -MM "src/main/drivers/pwm_output.c"

$(IntermediateDirectory)/drivers_pwm_output.c$(PreprocessSuffix): src/main/drivers/pwm_output.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_pwm_output.c$(PreprocessSuffix) "src/main/drivers/pwm_output.c"

$(IntermediateDirectory)/drivers_pwm_rx.c$(ObjectSuffix): src/main/drivers/pwm_rx.c $(IntermediateDirectory)/drivers_pwm_rx.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/pwm_rx.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_pwm_rx.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_pwm_rx.c$(DependSuffix): src/main/drivers/pwm_rx.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_pwm_rx.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_pwm_rx.c$(DependSuffix) -MM "src/main/drivers/pwm_rx.c"

$(IntermediateDirectory)/drivers_pwm_rx.c$(PreprocessSuffix): src/main/drivers/pwm_rx.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_pwm_rx.c$(PreprocessSuffix) "src/main/drivers/pwm_rx.c"

$(IntermediateDirectory)/drivers_sdcard.c$(ObjectSuffix): src/main/drivers/sdcard.c $(IntermediateDirectory)/drivers_sdcard.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sdcard.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sdcard.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sdcard.c$(DependSuffix): src/main/drivers/sdcard.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sdcard.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sdcard.c$(DependSuffix) -MM "src/main/drivers/sdcard.c"

$(IntermediateDirectory)/drivers_sdcard.c$(PreprocessSuffix): src/main/drivers/sdcard.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sdcard.c$(PreprocessSuffix) "src/main/drivers/sdcard.c"

$(IntermediateDirectory)/drivers_sdcard_standard.c$(ObjectSuffix): src/main/drivers/sdcard_standard.c $(IntermediateDirectory)/drivers_sdcard_standard.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sdcard_standard.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sdcard_standard.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sdcard_standard.c$(DependSuffix): src/main/drivers/sdcard_standard.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sdcard_standard.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sdcard_standard.c$(DependSuffix) -MM "src/main/drivers/sdcard_standard.c"

$(IntermediateDirectory)/drivers_sdcard_standard.c$(PreprocessSuffix): src/main/drivers/sdcard_standard.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sdcard_standard.c$(PreprocessSuffix) "src/main/drivers/sdcard_standard.c"

$(IntermediateDirectory)/drivers_serial.c$(ObjectSuffix): src/main/drivers/serial.c $(IntermediateDirectory)/drivers_serial.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial.c$(DependSuffix): src/main/drivers/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial.c$(DependSuffix) -MM "src/main/drivers/serial.c"

$(IntermediateDirectory)/drivers_serial.c$(PreprocessSuffix): src/main/drivers/serial.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial.c$(PreprocessSuffix) "src/main/drivers/serial.c"

$(IntermediateDirectory)/drivers_serial_softserial.c$(ObjectSuffix): src/main/drivers/serial_softserial.c $(IntermediateDirectory)/drivers_serial_softserial.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial_softserial.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial_softserial.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial_softserial.c$(DependSuffix): src/main/drivers/serial_softserial.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial_softserial.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial_softserial.c$(DependSuffix) -MM "src/main/drivers/serial_softserial.c"

$(IntermediateDirectory)/drivers_serial_softserial.c$(PreprocessSuffix): src/main/drivers/serial_softserial.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial_softserial.c$(PreprocessSuffix) "src/main/drivers/serial_softserial.c"

$(IntermediateDirectory)/drivers_serial_uart.c$(ObjectSuffix): src/main/drivers/serial_uart.c $(IntermediateDirectory)/drivers_serial_uart.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial_uart.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial_uart.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial_uart.c$(DependSuffix): src/main/drivers/serial_uart.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial_uart.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial_uart.c$(DependSuffix) -MM "src/main/drivers/serial_uart.c"

$(IntermediateDirectory)/drivers_serial_uart.c$(PreprocessSuffix): src/main/drivers/serial_uart.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial_uart.c$(PreprocessSuffix) "src/main/drivers/serial_uart.c"

$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(ObjectSuffix): src/main/drivers/serial_uart_stm32f10x.c $(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial_uart_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(DependSuffix): src/main/drivers/serial_uart_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/serial_uart_stm32f10x.c"

$(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(PreprocessSuffix): src/main/drivers/serial_uart_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial_uart_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/serial_uart_stm32f10x.c"

$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(ObjectSuffix): src/main/drivers/serial_uart_stm32f30x.c $(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial_uart_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(DependSuffix): src/main/drivers/serial_uart_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/serial_uart_stm32f30x.c"

$(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(PreprocessSuffix): src/main/drivers/serial_uart_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial_uart_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/serial_uart_stm32f30x.c"

$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(ObjectSuffix): src/main/drivers/serial_usb_vcp.c $(IntermediateDirectory)/drivers_serial_usb_vcp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/serial_usb_vcp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(DependSuffix): src/main/drivers/serial_usb_vcp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(DependSuffix) -MM "src/main/drivers/serial_usb_vcp.c"

$(IntermediateDirectory)/drivers_serial_usb_vcp.c$(PreprocessSuffix): src/main/drivers/serial_usb_vcp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_serial_usb_vcp.c$(PreprocessSuffix) "src/main/drivers/serial_usb_vcp.c"

$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(ObjectSuffix): src/main/drivers/sonar_hcsr04.c $(IntermediateDirectory)/drivers_sonar_hcsr04.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sonar_hcsr04.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(DependSuffix): src/main/drivers/sonar_hcsr04.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(DependSuffix) -MM "src/main/drivers/sonar_hcsr04.c"

$(IntermediateDirectory)/drivers_sonar_hcsr04.c$(PreprocessSuffix): src/main/drivers/sonar_hcsr04.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sonar_hcsr04.c$(PreprocessSuffix) "src/main/drivers/sonar_hcsr04.c"

$(IntermediateDirectory)/drivers_sound_beeper.c$(ObjectSuffix): src/main/drivers/sound_beeper.c $(IntermediateDirectory)/drivers_sound_beeper.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sound_beeper.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sound_beeper.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sound_beeper.c$(DependSuffix): src/main/drivers/sound_beeper.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sound_beeper.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sound_beeper.c$(DependSuffix) -MM "src/main/drivers/sound_beeper.c"

$(IntermediateDirectory)/drivers_sound_beeper.c$(PreprocessSuffix): src/main/drivers/sound_beeper.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sound_beeper.c$(PreprocessSuffix) "src/main/drivers/sound_beeper.c"

$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(ObjectSuffix): src/main/drivers/sound_beeper_stm32f10x.c $(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sound_beeper_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(DependSuffix): src/main/drivers/sound_beeper_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/sound_beeper_stm32f10x.c"

$(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(PreprocessSuffix): src/main/drivers/sound_beeper_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sound_beeper_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/sound_beeper_stm32f10x.c"

$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(ObjectSuffix): src/main/drivers/sound_beeper_stm32f30x.c $(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/sound_beeper_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(DependSuffix): src/main/drivers/sound_beeper_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/sound_beeper_stm32f30x.c"

$(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(PreprocessSuffix): src/main/drivers/sound_beeper_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_sound_beeper_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/sound_beeper_stm32f30x.c"

$(IntermediateDirectory)/drivers_system.c$(ObjectSuffix): src/main/drivers/system.c $(IntermediateDirectory)/drivers_system.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/system.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_system.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_system.c$(DependSuffix): src/main/drivers/system.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_system.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_system.c$(DependSuffix) -MM "src/main/drivers/system.c"

$(IntermediateDirectory)/drivers_system.c$(PreprocessSuffix): src/main/drivers/system.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_system.c$(PreprocessSuffix) "src/main/drivers/system.c"

$(IntermediateDirectory)/drivers_system_stm32f10x.c$(ObjectSuffix): src/main/drivers/system_stm32f10x.c $(IntermediateDirectory)/drivers_system_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/system_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_system_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_system_stm32f10x.c$(DependSuffix): src/main/drivers/system_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_system_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_system_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/system_stm32f10x.c"

$(IntermediateDirectory)/drivers_system_stm32f10x.c$(PreprocessSuffix): src/main/drivers/system_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_system_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/system_stm32f10x.c"

$(IntermediateDirectory)/drivers_system_stm32f30x.c$(ObjectSuffix): src/main/drivers/system_stm32f30x.c $(IntermediateDirectory)/drivers_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_system_stm32f30x.c$(DependSuffix): src/main/drivers/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_system_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/system_stm32f30x.c"

$(IntermediateDirectory)/drivers_system_stm32f30x.c$(PreprocessSuffix): src/main/drivers/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_system_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/system_stm32f30x.c"

$(IntermediateDirectory)/drivers_timer.c$(ObjectSuffix): src/main/drivers/timer.c $(IntermediateDirectory)/drivers_timer.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/timer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_timer.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_timer.c$(DependSuffix): src/main/drivers/timer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_timer.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_timer.c$(DependSuffix) -MM "src/main/drivers/timer.c"

$(IntermediateDirectory)/drivers_timer.c$(PreprocessSuffix): src/main/drivers/timer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_timer.c$(PreprocessSuffix) "src/main/drivers/timer.c"

$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(ObjectSuffix): src/main/drivers/timer_stm32f10x.c $(IntermediateDirectory)/drivers_timer_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/timer_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(DependSuffix): src/main/drivers/timer_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(DependSuffix) -MM "src/main/drivers/timer_stm32f10x.c"

$(IntermediateDirectory)/drivers_timer_stm32f10x.c$(PreprocessSuffix): src/main/drivers/timer_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_timer_stm32f10x.c$(PreprocessSuffix) "src/main/drivers/timer_stm32f10x.c"

$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(ObjectSuffix): src/main/drivers/timer_stm32f30x.c $(IntermediateDirectory)/drivers_timer_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/timer_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(DependSuffix): src/main/drivers/timer_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/timer_stm32f30x.c"

$(IntermediateDirectory)/drivers_timer_stm32f30x.c$(PreprocessSuffix): src/main/drivers/timer_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_timer_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/timer_stm32f30x.c"

$(IntermediateDirectory)/drivers_transponder_ir.c$(ObjectSuffix): src/main/drivers/transponder_ir.c $(IntermediateDirectory)/drivers_transponder_ir.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/transponder_ir.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_transponder_ir.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_transponder_ir.c$(DependSuffix): src/main/drivers/transponder_ir.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_transponder_ir.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_transponder_ir.c$(DependSuffix) -MM "src/main/drivers/transponder_ir.c"

$(IntermediateDirectory)/drivers_transponder_ir.c$(PreprocessSuffix): src/main/drivers/transponder_ir.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_transponder_ir.c$(PreprocessSuffix) "src/main/drivers/transponder_ir.c"

$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(ObjectSuffix): src/main/drivers/transponder_ir_stm32f30x.c $(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/transponder_ir_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(DependSuffix): src/main/drivers/transponder_ir_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(DependSuffix) -MM "src/main/drivers/transponder_ir_stm32f30x.c"

$(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(PreprocessSuffix): src/main/drivers/transponder_ir_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_transponder_ir_stm32f30x.c$(PreprocessSuffix) "src/main/drivers/transponder_ir_stm32f30x.c"

$(IntermediateDirectory)/drivers_usb_detection.c$(ObjectSuffix): src/main/drivers/usb_detection.c $(IntermediateDirectory)/drivers_usb_detection.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/usb_detection.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_usb_detection.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_usb_detection.c$(DependSuffix): src/main/drivers/usb_detection.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_usb_detection.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_usb_detection.c$(DependSuffix) -MM "src/main/drivers/usb_detection.c"

$(IntermediateDirectory)/drivers_usb_detection.c$(PreprocessSuffix): src/main/drivers/usb_detection.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_usb_detection.c$(PreprocessSuffix) "src/main/drivers/usb_detection.c"

$(IntermediateDirectory)/drivers_usb_io.c$(ObjectSuffix): src/main/drivers/usb_io.c $(IntermediateDirectory)/drivers_usb_io.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/drivers/usb_io.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/drivers_usb_io.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/drivers_usb_io.c$(DependSuffix): src/main/drivers/usb_io.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/drivers_usb_io.c$(ObjectSuffix) -MF$(IntermediateDirectory)/drivers_usb_io.c$(DependSuffix) -MM "src/main/drivers/usb_io.c"

$(IntermediateDirectory)/drivers_usb_io.c$(PreprocessSuffix): src/main/drivers/usb_io.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/drivers_usb_io.c$(PreprocessSuffix) "src/main/drivers/usb_io.c"

$(IntermediateDirectory)/flight_altitudehold.c$(ObjectSuffix): src/main/flight/altitudehold.c $(IntermediateDirectory)/flight_altitudehold.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/altitudehold.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_altitudehold.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_altitudehold.c$(DependSuffix): src/main/flight/altitudehold.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_altitudehold.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_altitudehold.c$(DependSuffix) -MM "src/main/flight/altitudehold.c"

$(IntermediateDirectory)/flight_altitudehold.c$(PreprocessSuffix): src/main/flight/altitudehold.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_altitudehold.c$(PreprocessSuffix) "src/main/flight/altitudehold.c"

$(IntermediateDirectory)/flight_failsafe.c$(ObjectSuffix): src/main/flight/failsafe.c $(IntermediateDirectory)/flight_failsafe.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/failsafe.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_failsafe.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_failsafe.c$(DependSuffix): src/main/flight/failsafe.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_failsafe.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_failsafe.c$(DependSuffix) -MM "src/main/flight/failsafe.c"

$(IntermediateDirectory)/flight_failsafe.c$(PreprocessSuffix): src/main/flight/failsafe.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_failsafe.c$(PreprocessSuffix) "src/main/flight/failsafe.c"

$(IntermediateDirectory)/flight_gps_conversion.c$(ObjectSuffix): src/main/flight/gps_conversion.c $(IntermediateDirectory)/flight_gps_conversion.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/gps_conversion.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_gps_conversion.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_gps_conversion.c$(DependSuffix): src/main/flight/gps_conversion.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_gps_conversion.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_gps_conversion.c$(DependSuffix) -MM "src/main/flight/gps_conversion.c"

$(IntermediateDirectory)/flight_gps_conversion.c$(PreprocessSuffix): src/main/flight/gps_conversion.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_gps_conversion.c$(PreprocessSuffix) "src/main/flight/gps_conversion.c"

$(IntermediateDirectory)/flight_gtune.c$(ObjectSuffix): src/main/flight/gtune.c $(IntermediateDirectory)/flight_gtune.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/gtune.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_gtune.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_gtune.c$(DependSuffix): src/main/flight/gtune.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_gtune.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_gtune.c$(DependSuffix) -MM "src/main/flight/gtune.c"

$(IntermediateDirectory)/flight_gtune.c$(PreprocessSuffix): src/main/flight/gtune.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_gtune.c$(PreprocessSuffix) "src/main/flight/gtune.c"

$(IntermediateDirectory)/flight_imu.c$(ObjectSuffix): src/main/flight/imu.c $(IntermediateDirectory)/flight_imu.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/imu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_imu.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_imu.c$(DependSuffix): src/main/flight/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_imu.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_imu.c$(DependSuffix) -MM "src/main/flight/imu.c"

$(IntermediateDirectory)/flight_imu.c$(PreprocessSuffix): src/main/flight/imu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_imu.c$(PreprocessSuffix) "src/main/flight/imu.c"

$(IntermediateDirectory)/flight_lowpass.c$(ObjectSuffix): src/main/flight/lowpass.c $(IntermediateDirectory)/flight_lowpass.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/lowpass.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_lowpass.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_lowpass.c$(DependSuffix): src/main/flight/lowpass.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_lowpass.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_lowpass.c$(DependSuffix) -MM "src/main/flight/lowpass.c"

$(IntermediateDirectory)/flight_lowpass.c$(PreprocessSuffix): src/main/flight/lowpass.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_lowpass.c$(PreprocessSuffix) "src/main/flight/lowpass.c"

$(IntermediateDirectory)/flight_mixer.c$(ObjectSuffix): src/main/flight/mixer.c $(IntermediateDirectory)/flight_mixer.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/mixer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_mixer.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_mixer.c$(DependSuffix): src/main/flight/mixer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_mixer.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_mixer.c$(DependSuffix) -MM "src/main/flight/mixer.c"

$(IntermediateDirectory)/flight_mixer.c$(PreprocessSuffix): src/main/flight/mixer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_mixer.c$(PreprocessSuffix) "src/main/flight/mixer.c"

$(IntermediateDirectory)/flight_navigation.c$(ObjectSuffix): src/main/flight/navigation.c $(IntermediateDirectory)/flight_navigation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/navigation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_navigation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_navigation.c$(DependSuffix): src/main/flight/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_navigation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_navigation.c$(DependSuffix) -MM "src/main/flight/navigation.c"

$(IntermediateDirectory)/flight_navigation.c$(PreprocessSuffix): src/main/flight/navigation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_navigation.c$(PreprocessSuffix) "src/main/flight/navigation.c"

$(IntermediateDirectory)/flight_pid.c$(ObjectSuffix): src/main/flight/pid.c $(IntermediateDirectory)/flight_pid.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/flight/pid.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/flight_pid.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/flight_pid.c$(DependSuffix): src/main/flight/pid.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/flight_pid.c$(ObjectSuffix) -MF$(IntermediateDirectory)/flight_pid.c$(DependSuffix) -MM "src/main/flight/pid.c"

$(IntermediateDirectory)/flight_pid.c$(PreprocessSuffix): src/main/flight/pid.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/flight_pid.c$(PreprocessSuffix) "src/main/flight/pid.c"

$(IntermediateDirectory)/rx_ibus.c$(ObjectSuffix): src/main/rx/ibus.c $(IntermediateDirectory)/rx_ibus.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/ibus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_ibus.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_ibus.c$(DependSuffix): src/main/rx/ibus.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_ibus.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_ibus.c$(DependSuffix) -MM "src/main/rx/ibus.c"

$(IntermediateDirectory)/rx_ibus.c$(PreprocessSuffix): src/main/rx/ibus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_ibus.c$(PreprocessSuffix) "src/main/rx/ibus.c"

$(IntermediateDirectory)/rx_msp.c$(ObjectSuffix): src/main/rx/msp.c $(IntermediateDirectory)/rx_msp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/msp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_msp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_msp.c$(DependSuffix): src/main/rx/msp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_msp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_msp.c$(DependSuffix) -MM "src/main/rx/msp.c"

$(IntermediateDirectory)/rx_msp.c$(PreprocessSuffix): src/main/rx/msp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_msp.c$(PreprocessSuffix) "src/main/rx/msp.c"

$(IntermediateDirectory)/rx_pwm.c$(ObjectSuffix): src/main/rx/pwm.c $(IntermediateDirectory)/rx_pwm.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/pwm.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_pwm.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_pwm.c$(DependSuffix): src/main/rx/pwm.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_pwm.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_pwm.c$(DependSuffix) -MM "src/main/rx/pwm.c"

$(IntermediateDirectory)/rx_pwm.c$(PreprocessSuffix): src/main/rx/pwm.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_pwm.c$(PreprocessSuffix) "src/main/rx/pwm.c"

$(IntermediateDirectory)/rx_rx.c$(ObjectSuffix): src/main/rx/rx.c $(IntermediateDirectory)/rx_rx.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/rx.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_rx.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_rx.c$(DependSuffix): src/main/rx/rx.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_rx.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_rx.c$(DependSuffix) -MM "src/main/rx/rx.c"

$(IntermediateDirectory)/rx_rx.c$(PreprocessSuffix): src/main/rx/rx.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_rx.c$(PreprocessSuffix) "src/main/rx/rx.c"

$(IntermediateDirectory)/rx_sbus.c$(ObjectSuffix): src/main/rx/sbus.c $(IntermediateDirectory)/rx_sbus.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/sbus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_sbus.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_sbus.c$(DependSuffix): src/main/rx/sbus.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_sbus.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_sbus.c$(DependSuffix) -MM "src/main/rx/sbus.c"

$(IntermediateDirectory)/rx_sbus.c$(PreprocessSuffix): src/main/rx/sbus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_sbus.c$(PreprocessSuffix) "src/main/rx/sbus.c"

$(IntermediateDirectory)/rx_spektrum.c$(ObjectSuffix): src/main/rx/spektrum.c $(IntermediateDirectory)/rx_spektrum.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/spektrum.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_spektrum.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_spektrum.c$(DependSuffix): src/main/rx/spektrum.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_spektrum.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_spektrum.c$(DependSuffix) -MM "src/main/rx/spektrum.c"

$(IntermediateDirectory)/rx_spektrum.c$(PreprocessSuffix): src/main/rx/spektrum.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_spektrum.c$(PreprocessSuffix) "src/main/rx/spektrum.c"

$(IntermediateDirectory)/rx_sumd.c$(ObjectSuffix): src/main/rx/sumd.c $(IntermediateDirectory)/rx_sumd.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/sumd.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_sumd.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_sumd.c$(DependSuffix): src/main/rx/sumd.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_sumd.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_sumd.c$(DependSuffix) -MM "src/main/rx/sumd.c"

$(IntermediateDirectory)/rx_sumd.c$(PreprocessSuffix): src/main/rx/sumd.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_sumd.c$(PreprocessSuffix) "src/main/rx/sumd.c"

$(IntermediateDirectory)/rx_sumh.c$(ObjectSuffix): src/main/rx/sumh.c $(IntermediateDirectory)/rx_sumh.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/sumh.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_sumh.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_sumh.c$(DependSuffix): src/main/rx/sumh.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_sumh.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_sumh.c$(DependSuffix) -MM "src/main/rx/sumh.c"

$(IntermediateDirectory)/rx_sumh.c$(PreprocessSuffix): src/main/rx/sumh.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_sumh.c$(PreprocessSuffix) "src/main/rx/sumh.c"

$(IntermediateDirectory)/rx_xbus.c$(ObjectSuffix): src/main/rx/xbus.c $(IntermediateDirectory)/rx_xbus.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/rx/xbus.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/rx_xbus.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/rx_xbus.c$(DependSuffix): src/main/rx/xbus.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/rx_xbus.c$(ObjectSuffix) -MF$(IntermediateDirectory)/rx_xbus.c$(DependSuffix) -MM "src/main/rx/xbus.c"

$(IntermediateDirectory)/rx_xbus.c$(PreprocessSuffix): src/main/rx/xbus.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/rx_xbus.c$(PreprocessSuffix) "src/main/rx/xbus.c"

$(IntermediateDirectory)/sensors_acceleration.c$(ObjectSuffix): src/main/sensors/acceleration.c $(IntermediateDirectory)/sensors_acceleration.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/acceleration.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_acceleration.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_acceleration.c$(DependSuffix): src/main/sensors/acceleration.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_acceleration.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_acceleration.c$(DependSuffix) -MM "src/main/sensors/acceleration.c"

$(IntermediateDirectory)/sensors_acceleration.c$(PreprocessSuffix): src/main/sensors/acceleration.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_acceleration.c$(PreprocessSuffix) "src/main/sensors/acceleration.c"

$(IntermediateDirectory)/sensors_barometer.c$(ObjectSuffix): src/main/sensors/barometer.c $(IntermediateDirectory)/sensors_barometer.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/barometer.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_barometer.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_barometer.c$(DependSuffix): src/main/sensors/barometer.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_barometer.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_barometer.c$(DependSuffix) -MM "src/main/sensors/barometer.c"

$(IntermediateDirectory)/sensors_barometer.c$(PreprocessSuffix): src/main/sensors/barometer.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_barometer.c$(PreprocessSuffix) "src/main/sensors/barometer.c"

$(IntermediateDirectory)/sensors_battery.c$(ObjectSuffix): src/main/sensors/battery.c $(IntermediateDirectory)/sensors_battery.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/battery.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_battery.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_battery.c$(DependSuffix): src/main/sensors/battery.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_battery.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_battery.c$(DependSuffix) -MM "src/main/sensors/battery.c"

$(IntermediateDirectory)/sensors_battery.c$(PreprocessSuffix): src/main/sensors/battery.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_battery.c$(PreprocessSuffix) "src/main/sensors/battery.c"

$(IntermediateDirectory)/sensors_boardalignment.c$(ObjectSuffix): src/main/sensors/boardalignment.c $(IntermediateDirectory)/sensors_boardalignment.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/boardalignment.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_boardalignment.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_boardalignment.c$(DependSuffix): src/main/sensors/boardalignment.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_boardalignment.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_boardalignment.c$(DependSuffix) -MM "src/main/sensors/boardalignment.c"

$(IntermediateDirectory)/sensors_boardalignment.c$(PreprocessSuffix): src/main/sensors/boardalignment.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_boardalignment.c$(PreprocessSuffix) "src/main/sensors/boardalignment.c"

$(IntermediateDirectory)/sensors_compass.c$(ObjectSuffix): src/main/sensors/compass.c $(IntermediateDirectory)/sensors_compass.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/compass.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_compass.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_compass.c$(DependSuffix): src/main/sensors/compass.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_compass.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_compass.c$(DependSuffix) -MM "src/main/sensors/compass.c"

$(IntermediateDirectory)/sensors_compass.c$(PreprocessSuffix): src/main/sensors/compass.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_compass.c$(PreprocessSuffix) "src/main/sensors/compass.c"

$(IntermediateDirectory)/sensors_gyro.c$(ObjectSuffix): src/main/sensors/gyro.c $(IntermediateDirectory)/sensors_gyro.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/gyro.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_gyro.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_gyro.c$(DependSuffix): src/main/sensors/gyro.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_gyro.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_gyro.c$(DependSuffix) -MM "src/main/sensors/gyro.c"

$(IntermediateDirectory)/sensors_gyro.c$(PreprocessSuffix): src/main/sensors/gyro.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_gyro.c$(PreprocessSuffix) "src/main/sensors/gyro.c"

$(IntermediateDirectory)/sensors_initialisation.c$(ObjectSuffix): src/main/sensors/initialisation.c $(IntermediateDirectory)/sensors_initialisation.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/initialisation.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_initialisation.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_initialisation.c$(DependSuffix): src/main/sensors/initialisation.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_initialisation.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_initialisation.c$(DependSuffix) -MM "src/main/sensors/initialisation.c"

$(IntermediateDirectory)/sensors_initialisation.c$(PreprocessSuffix): src/main/sensors/initialisation.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_initialisation.c$(PreprocessSuffix) "src/main/sensors/initialisation.c"

$(IntermediateDirectory)/sensors_sonar.c$(ObjectSuffix): src/main/sensors/sonar.c $(IntermediateDirectory)/sensors_sonar.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/sensors/sonar.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sensors_sonar.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sensors_sonar.c$(DependSuffix): src/main/sensors/sonar.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sensors_sonar.c$(ObjectSuffix) -MF$(IntermediateDirectory)/sensors_sonar.c$(DependSuffix) -MM "src/main/sensors/sonar.c"

$(IntermediateDirectory)/sensors_sonar.c$(PreprocessSuffix): src/main/sensors/sonar.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sensors_sonar.c$(PreprocessSuffix) "src/main/sensors/sonar.c"

$(IntermediateDirectory)/telemetry_frsky.c$(ObjectSuffix): src/main/telemetry/frsky.c $(IntermediateDirectory)/telemetry_frsky.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/telemetry/frsky.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/telemetry_frsky.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/telemetry_frsky.c$(DependSuffix): src/main/telemetry/frsky.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/telemetry_frsky.c$(ObjectSuffix) -MF$(IntermediateDirectory)/telemetry_frsky.c$(DependSuffix) -MM "src/main/telemetry/frsky.c"

$(IntermediateDirectory)/telemetry_frsky.c$(PreprocessSuffix): src/main/telemetry/frsky.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/telemetry_frsky.c$(PreprocessSuffix) "src/main/telemetry/frsky.c"

$(IntermediateDirectory)/telemetry_hott.c$(ObjectSuffix): src/main/telemetry/hott.c $(IntermediateDirectory)/telemetry_hott.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/telemetry/hott.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/telemetry_hott.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/telemetry_hott.c$(DependSuffix): src/main/telemetry/hott.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/telemetry_hott.c$(ObjectSuffix) -MF$(IntermediateDirectory)/telemetry_hott.c$(DependSuffix) -MM "src/main/telemetry/hott.c"

$(IntermediateDirectory)/telemetry_hott.c$(PreprocessSuffix): src/main/telemetry/hott.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/telemetry_hott.c$(PreprocessSuffix) "src/main/telemetry/hott.c"

$(IntermediateDirectory)/telemetry_ltm.c$(ObjectSuffix): src/main/telemetry/ltm.c $(IntermediateDirectory)/telemetry_ltm.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/telemetry/ltm.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/telemetry_ltm.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/telemetry_ltm.c$(DependSuffix): src/main/telemetry/ltm.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/telemetry_ltm.c$(ObjectSuffix) -MF$(IntermediateDirectory)/telemetry_ltm.c$(DependSuffix) -MM "src/main/telemetry/ltm.c"

$(IntermediateDirectory)/telemetry_ltm.c$(PreprocessSuffix): src/main/telemetry/ltm.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/telemetry_ltm.c$(PreprocessSuffix) "src/main/telemetry/ltm.c"

$(IntermediateDirectory)/telemetry_smartport.c$(ObjectSuffix): src/main/telemetry/smartport.c $(IntermediateDirectory)/telemetry_smartport.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/telemetry/smartport.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/telemetry_smartport.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/telemetry_smartport.c$(DependSuffix): src/main/telemetry/smartport.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/telemetry_smartport.c$(ObjectSuffix) -MF$(IntermediateDirectory)/telemetry_smartport.c$(DependSuffix) -MM "src/main/telemetry/smartport.c"

$(IntermediateDirectory)/telemetry_smartport.c$(PreprocessSuffix): src/main/telemetry/smartport.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/telemetry_smartport.c$(PreprocessSuffix) "src/main/telemetry/smartport.c"

$(IntermediateDirectory)/telemetry_telemetry.c$(ObjectSuffix): src/main/telemetry/telemetry.c $(IntermediateDirectory)/telemetry_telemetry.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/telemetry/telemetry.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/telemetry_telemetry.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/telemetry_telemetry.c$(DependSuffix): src/main/telemetry/telemetry.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/telemetry_telemetry.c$(ObjectSuffix) -MF$(IntermediateDirectory)/telemetry_telemetry.c$(DependSuffix) -MM "src/main/telemetry/telemetry.c"

$(IntermediateDirectory)/telemetry_telemetry.c$(PreprocessSuffix): src/main/telemetry/telemetry.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/telemetry_telemetry.c$(PreprocessSuffix) "src/main/telemetry/telemetry.c"

$(IntermediateDirectory)/vcp_hw_config.c$(ObjectSuffix): src/main/vcp/hw_config.c $(IntermediateDirectory)/vcp_hw_config.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/hw_config.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_hw_config.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_hw_config.c$(DependSuffix): src/main/vcp/hw_config.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_hw_config.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_hw_config.c$(DependSuffix) -MM "src/main/vcp/hw_config.c"

$(IntermediateDirectory)/vcp_hw_config.c$(PreprocessSuffix): src/main/vcp/hw_config.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_hw_config.c$(PreprocessSuffix) "src/main/vcp/hw_config.c"

$(IntermediateDirectory)/vcp_stm32_it.c$(ObjectSuffix): src/main/vcp/stm32_it.c $(IntermediateDirectory)/vcp_stm32_it.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/stm32_it.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_stm32_it.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_stm32_it.c$(DependSuffix): src/main/vcp/stm32_it.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_stm32_it.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_stm32_it.c$(DependSuffix) -MM "src/main/vcp/stm32_it.c"

$(IntermediateDirectory)/vcp_stm32_it.c$(PreprocessSuffix): src/main/vcp/stm32_it.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_stm32_it.c$(PreprocessSuffix) "src/main/vcp/stm32_it.c"

$(IntermediateDirectory)/vcp_usb_desc.c$(ObjectSuffix): src/main/vcp/usb_desc.c $(IntermediateDirectory)/vcp_usb_desc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/usb_desc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_usb_desc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_usb_desc.c$(DependSuffix): src/main/vcp/usb_desc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_usb_desc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_usb_desc.c$(DependSuffix) -MM "src/main/vcp/usb_desc.c"

$(IntermediateDirectory)/vcp_usb_desc.c$(PreprocessSuffix): src/main/vcp/usb_desc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_usb_desc.c$(PreprocessSuffix) "src/main/vcp/usb_desc.c"

$(IntermediateDirectory)/vcp_usb_endp.c$(ObjectSuffix): src/main/vcp/usb_endp.c $(IntermediateDirectory)/vcp_usb_endp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/usb_endp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_usb_endp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_usb_endp.c$(DependSuffix): src/main/vcp/usb_endp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_usb_endp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_usb_endp.c$(DependSuffix) -MM "src/main/vcp/usb_endp.c"

$(IntermediateDirectory)/vcp_usb_endp.c$(PreprocessSuffix): src/main/vcp/usb_endp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_usb_endp.c$(PreprocessSuffix) "src/main/vcp/usb_endp.c"

$(IntermediateDirectory)/vcp_usb_istr.c$(ObjectSuffix): src/main/vcp/usb_istr.c $(IntermediateDirectory)/vcp_usb_istr.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/usb_istr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_usb_istr.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_usb_istr.c$(DependSuffix): src/main/vcp/usb_istr.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_usb_istr.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_usb_istr.c$(DependSuffix) -MM "src/main/vcp/usb_istr.c"

$(IntermediateDirectory)/vcp_usb_istr.c$(PreprocessSuffix): src/main/vcp/usb_istr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_usb_istr.c$(PreprocessSuffix) "src/main/vcp/usb_istr.c"

$(IntermediateDirectory)/vcp_usb_prop.c$(ObjectSuffix): src/main/vcp/usb_prop.c $(IntermediateDirectory)/vcp_usb_prop.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/usb_prop.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_usb_prop.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_usb_prop.c$(DependSuffix): src/main/vcp/usb_prop.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_usb_prop.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_usb_prop.c$(DependSuffix) -MM "src/main/vcp/usb_prop.c"

$(IntermediateDirectory)/vcp_usb_prop.c$(PreprocessSuffix): src/main/vcp/usb_prop.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_usb_prop.c$(PreprocessSuffix) "src/main/vcp/usb_prop.c"

$(IntermediateDirectory)/vcp_usb_pwr.c$(ObjectSuffix): src/main/vcp/usb_pwr.c $(IntermediateDirectory)/vcp_usb_pwr.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/vcp/usb_pwr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/vcp_usb_pwr.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/vcp_usb_pwr.c$(DependSuffix): src/main/vcp/usb_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/vcp_usb_pwr.c$(ObjectSuffix) -MF$(IntermediateDirectory)/vcp_usb_pwr.c$(DependSuffix) -MM "src/main/vcp/usb_pwr.c"

$(IntermediateDirectory)/vcp_usb_pwr.c$(PreprocessSuffix): src/main/vcp/usb_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/vcp_usb_pwr.c$(PreprocessSuffix) "src/main/vcp/usb_pwr.c"

$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(ObjectSuffix): src/main/target/ALIENFLIGHTF3/hardware_revision.c $(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/ALIENFLIGHTF3/hardware_revision.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(DependSuffix): src/main/target/ALIENFLIGHTF3/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(ObjectSuffix) -MF$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(DependSuffix) -MM "src/main/target/ALIENFLIGHTF3/hardware_revision.c"

$(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(PreprocessSuffix): src/main/target/ALIENFLIGHTF3/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ALIENFLIGHTF3_hardware_revision.c$(PreprocessSuffix) "src/main/target/ALIENFLIGHTF3/hardware_revision.c"

$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(ObjectSuffix): src/main/target/ALIENFLIGHTF3/system_stm32f30x.c $(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/ALIENFLIGHTF3/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(DependSuffix): src/main/target/ALIENFLIGHTF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/ALIENFLIGHTF3/system_stm32f30x.c"

$(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(PreprocessSuffix): src/main/target/ALIENFLIGHTF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/ALIENFLIGHTF3_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/ALIENFLIGHTF3/system_stm32f30x.c"

$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(ObjectSuffix): src/main/target/CHEBUZZF3/system_stm32f30x.c $(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/CHEBUZZF3/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(DependSuffix): src/main/target/CHEBUZZF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/CHEBUZZF3/system_stm32f30x.c"

$(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(PreprocessSuffix): src/main/target/CHEBUZZF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/CHEBUZZF3_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/CHEBUZZF3/system_stm32f30x.c"

$(IntermediateDirectory)/CJMCU_hardware_revision.c$(ObjectSuffix): src/main/target/CJMCU/hardware_revision.c $(IntermediateDirectory)/CJMCU_hardware_revision.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/CJMCU/hardware_revision.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/CJMCU_hardware_revision.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/CJMCU_hardware_revision.c$(DependSuffix): src/main/target/CJMCU/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/CJMCU_hardware_revision.c$(ObjectSuffix) -MF$(IntermediateDirectory)/CJMCU_hardware_revision.c$(DependSuffix) -MM "src/main/target/CJMCU/hardware_revision.c"

$(IntermediateDirectory)/CJMCU_hardware_revision.c$(PreprocessSuffix): src/main/target/CJMCU/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/CJMCU_hardware_revision.c$(PreprocessSuffix) "src/main/target/CJMCU/hardware_revision.c"

$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(ObjectSuffix): src/main/target/COLIBRI_RACE/system_stm32f30x.c $(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/COLIBRI_RACE/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(DependSuffix): src/main/target/COLIBRI_RACE/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/COLIBRI_RACE/system_stm32f30x.c"

$(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(PreprocessSuffix): src/main/target/COLIBRI_RACE/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/COLIBRI_RACE_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/COLIBRI_RACE/system_stm32f30x.c"

$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(ObjectSuffix): src/main/target/IRCFUSIONF3/system_stm32f30x.c $(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/IRCFUSIONF3/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(DependSuffix): src/main/target/IRCFUSIONF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/IRCFUSIONF3/system_stm32f30x.c"

$(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(PreprocessSuffix): src/main/target/IRCFUSIONF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/IRCFUSIONF3_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/IRCFUSIONF3/system_stm32f30x.c"

$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(ObjectSuffix): src/main/target/LUX_RACE/system_stm32f30x.c $(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/LUX_RACE/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(DependSuffix): src/main/target/LUX_RACE/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/LUX_RACE/system_stm32f30x.c"

$(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(PreprocessSuffix): src/main/target/LUX_RACE/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/LUX_RACE_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/LUX_RACE/system_stm32f30x.c"

$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(ObjectSuffix): src/main/target/MOTOLAB/system_stm32f30x.c $(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/MOTOLAB/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(DependSuffix): src/main/target/MOTOLAB/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/MOTOLAB/system_stm32f30x.c"

$(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(PreprocessSuffix): src/main/target/MOTOLAB/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/MOTOLAB_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/MOTOLAB/system_stm32f30x.c"

$(IntermediateDirectory)/NAZE_hardware_revision.c$(ObjectSuffix): src/main/target/NAZE/hardware_revision.c $(IntermediateDirectory)/NAZE_hardware_revision.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/NAZE/hardware_revision.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/NAZE_hardware_revision.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/NAZE_hardware_revision.c$(DependSuffix): src/main/target/NAZE/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/NAZE_hardware_revision.c$(ObjectSuffix) -MF$(IntermediateDirectory)/NAZE_hardware_revision.c$(DependSuffix) -MM "src/main/target/NAZE/hardware_revision.c"

$(IntermediateDirectory)/NAZE_hardware_revision.c$(PreprocessSuffix): src/main/target/NAZE/hardware_revision.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/NAZE_hardware_revision.c$(PreprocessSuffix) "src/main/target/NAZE/hardware_revision.c"

$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(ObjectSuffix): src/main/target/NAZE32PRO/system_stm32f30x.c $(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/NAZE32PRO/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(DependSuffix): src/main/target/NAZE32PRO/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/NAZE32PRO/system_stm32f30x.c"

$(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(PreprocessSuffix): src/main/target/NAZE32PRO/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/NAZE32PRO_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/NAZE32PRO/system_stm32f30x.c"

$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(ObjectSuffix): src/main/target/RMDO/system_stm32f30x.c $(IntermediateDirectory)/RMDO_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/RMDO/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(DependSuffix): src/main/target/RMDO/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/RMDO/system_stm32f30x.c"

$(IntermediateDirectory)/RMDO_system_stm32f30x.c$(PreprocessSuffix): src/main/target/RMDO/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/RMDO_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/RMDO/system_stm32f30x.c"

$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(ObjectSuffix): src/main/target/SPARKY/system_stm32f30x.c $(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/SPARKY/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(DependSuffix): src/main/target/SPARKY/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/SPARKY/system_stm32f30x.c"

$(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(PreprocessSuffix): src/main/target/SPARKY/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/SPARKY_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/SPARKY/system_stm32f30x.c"

$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(ObjectSuffix): src/main/target/SPRACINGF3/system_stm32f30x.c $(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/SPRACINGF3/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(DependSuffix): src/main/target/SPRACINGF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/SPRACINGF3/system_stm32f30x.c"

$(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(PreprocessSuffix): src/main/target/SPRACINGF3/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/SPRACINGF3_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/SPRACINGF3/system_stm32f30x.c"

$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(ObjectSuffix): src/main/target/SPRACINGF3MINI/system_stm32f30x.c $(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/SPRACINGF3MINI/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(DependSuffix): src/main/target/SPRACINGF3MINI/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/SPRACINGF3MINI/system_stm32f30x.c"

$(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(PreprocessSuffix): src/main/target/SPRACINGF3MINI/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/SPRACINGF3MINI_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/SPRACINGF3MINI/system_stm32f30x.c"

$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(ObjectSuffix): src/main/target/STM32F3DISCOVERY/system_stm32f30x.c $(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/target/STM32F3DISCOVERY/system_stm32f30x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(DependSuffix): src/main/target/STM32F3DISCOVERY/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(DependSuffix) -MM "src/main/target/STM32F3DISCOVERY/system_stm32f30x.c"

$(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(PreprocessSuffix): src/main/target/STM32F3DISCOVERY/system_stm32f30x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/STM32F3DISCOVERY_system_stm32f30x.c$(PreprocessSuffix) "src/main/target/STM32F3DISCOVERY/system_stm32f30x.c"

$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(ObjectSuffix): src/main/io/asyncfatfs/asyncfatfs.c $(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/asyncfatfs/asyncfatfs.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(DependSuffix): src/main/io/asyncfatfs/asyncfatfs.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(ObjectSuffix) -MF$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(DependSuffix) -MM "src/main/io/asyncfatfs/asyncfatfs.c"

$(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(PreprocessSuffix): src/main/io/asyncfatfs/asyncfatfs.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/asyncfatfs_asyncfatfs.c$(PreprocessSuffix) "src/main/io/asyncfatfs/asyncfatfs.c"

$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(ObjectSuffix): src/main/io/asyncfatfs/fat_standard.c $(IntermediateDirectory)/asyncfatfs_fat_standard.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/src/main/io/asyncfatfs/fat_standard.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(DependSuffix): src/main/io/asyncfatfs/fat_standard.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(ObjectSuffix) -MF$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(DependSuffix) -MM "src/main/io/asyncfatfs/fat_standard.c"

$(IntermediateDirectory)/asyncfatfs_fat_standard.c$(PreprocessSuffix): src/main/io/asyncfatfs/fat_standard.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/asyncfatfs_fat_standard.c$(PreprocessSuffix) "src/main/io/asyncfatfs/fat_standard.c"

$(IntermediateDirectory)/src_gtest-all.cc$(ObjectSuffix): lib/test/gtest/src/gtest-all.cc $(IntermediateDirectory)/src_gtest-all.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/test/gtest/src/gtest-all.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_gtest-all.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_gtest-all.cc$(DependSuffix): lib/test/gtest/src/gtest-all.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_gtest-all.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/src_gtest-all.cc$(DependSuffix) -MM "lib/test/gtest/src/gtest-all.cc"

$(IntermediateDirectory)/src_gtest-all.cc$(PreprocessSuffix): lib/test/gtest/src/gtest-all.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_gtest-all.cc$(PreprocessSuffix) "lib/test/gtest/src/gtest-all.cc"

$(IntermediateDirectory)/src_gtest_main.cc$(ObjectSuffix): lib/test/gtest/src/gtest_main.cc $(IntermediateDirectory)/src_gtest_main.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/test/gtest/src/gtest_main.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_gtest_main.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_gtest_main.cc$(DependSuffix): lib/test/gtest/src/gtest_main.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_gtest_main.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/src_gtest_main.cc$(DependSuffix) -MM "lib/test/gtest/src/gtest_main.cc"

$(IntermediateDirectory)/src_gtest_main.cc$(PreprocessSuffix): lib/test/gtest/src/gtest_main.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_gtest_main.cc$(PreprocessSuffix) "lib/test/gtest/src/gtest_main.cc"

$(IntermediateDirectory)/src_usb_core.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c $(IntermediateDirectory)/src_usb_core.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_core.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_core.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_core.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_core.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c"

$(IntermediateDirectory)/src_usb_core.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_core.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_core.c"

$(IntermediateDirectory)/src_usb_init.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c $(IntermediateDirectory)/src_usb_init.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_init.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_init.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_init.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_init.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c"

$(IntermediateDirectory)/src_usb_init.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_init.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_init.c"

$(IntermediateDirectory)/src_usb_int.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c $(IntermediateDirectory)/src_usb_int.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_int.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_int.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_int.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_int.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c"

$(IntermediateDirectory)/src_usb_int.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_int.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_int.c"

$(IntermediateDirectory)/src_usb_mem.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c $(IntermediateDirectory)/src_usb_mem.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_mem.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_mem.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_mem.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_mem.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c"

$(IntermediateDirectory)/src_usb_mem.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_mem.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_mem.c"

$(IntermediateDirectory)/src_usb_regs.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c $(IntermediateDirectory)/src_usb_regs.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_regs.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_regs.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_regs.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_regs.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c"

$(IntermediateDirectory)/src_usb_regs.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_regs.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_regs.c"

$(IntermediateDirectory)/src_usb_sil.c$(ObjectSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c $(IntermediateDirectory)/src_usb_sil.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_usb_sil.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_usb_sil.c$(DependSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_usb_sil.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_usb_sil.c$(DependSuffix) -MM "lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c"

$(IntermediateDirectory)/src_usb_sil.c$(PreprocessSuffix): lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_usb_sil.c$(PreprocessSuffix) "lib/main/STM32_USB-FS-Device_Driver/src/usb_sil.c"

$(IntermediateDirectory)/src_stm32f30x_adc.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c $(IntermediateDirectory)/src_stm32f30x_adc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_adc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_adc.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_adc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_adc.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c"

$(IntermediateDirectory)/src_stm32f30x_adc.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_adc.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_adc.c"

$(IntermediateDirectory)/src_stm32f30x_can.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c $(IntermediateDirectory)/src_stm32f30x_can.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_can.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_can.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_can.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_can.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c"

$(IntermediateDirectory)/src_stm32f30x_can.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_can.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_can.c"

$(IntermediateDirectory)/src_stm32f30x_comp.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c $(IntermediateDirectory)/src_stm32f30x_comp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_comp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_comp.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_comp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_comp.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c"

$(IntermediateDirectory)/src_stm32f30x_comp.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_comp.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_comp.c"

$(IntermediateDirectory)/src_stm32f30x_crc.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c $(IntermediateDirectory)/src_stm32f30x_crc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_crc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_crc.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_crc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_crc.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c"

$(IntermediateDirectory)/src_stm32f30x_crc.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_crc.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_crc.c"

$(IntermediateDirectory)/src_stm32f30x_dac.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c $(IntermediateDirectory)/src_stm32f30x_dac.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_dac.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_dac.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_dac.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_dac.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c"

$(IntermediateDirectory)/src_stm32f30x_dac.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_dac.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dac.c"

$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c $(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c"

$(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_dbgmcu.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dbgmcu.c"

$(IntermediateDirectory)/src_stm32f30x_dma.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c $(IntermediateDirectory)/src_stm32f30x_dma.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_dma.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_dma.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_dma.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_dma.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c"

$(IntermediateDirectory)/src_stm32f30x_dma.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_dma.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_dma.c"

$(IntermediateDirectory)/src_stm32f30x_exti.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c $(IntermediateDirectory)/src_stm32f30x_exti.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_exti.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_exti.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_exti.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_exti.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c"

$(IntermediateDirectory)/src_stm32f30x_exti.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_exti.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_exti.c"

$(IntermediateDirectory)/src_stm32f30x_flash.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c $(IntermediateDirectory)/src_stm32f30x_flash.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_flash.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_flash.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_flash.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_flash.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c"

$(IntermediateDirectory)/src_stm32f30x_flash.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_flash.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_flash.c"

$(IntermediateDirectory)/src_stm32f30x_gpio.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c $(IntermediateDirectory)/src_stm32f30x_gpio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_gpio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_gpio.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_gpio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_gpio.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c"

$(IntermediateDirectory)/src_stm32f30x_gpio.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_gpio.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_gpio.c"

$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c $(IntermediateDirectory)/src_stm32f30x_hrtim.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c"

$(IntermediateDirectory)/src_stm32f30x_hrtim.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_hrtim.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_hrtim.c"

$(IntermediateDirectory)/src_stm32f30x_i2c.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c $(IntermediateDirectory)/src_stm32f30x_i2c.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_i2c.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_i2c.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_i2c.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_i2c.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c"

$(IntermediateDirectory)/src_stm32f30x_i2c.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_i2c.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_i2c.c"

$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c $(IntermediateDirectory)/src_stm32f30x_iwdg.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c"

$(IntermediateDirectory)/src_stm32f30x_iwdg.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_iwdg.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_iwdg.c"

$(IntermediateDirectory)/src_stm32f30x_misc.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c $(IntermediateDirectory)/src_stm32f30x_misc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_misc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_misc.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_misc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_misc.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c"

$(IntermediateDirectory)/src_stm32f30x_misc.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_misc.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_misc.c"

$(IntermediateDirectory)/src_stm32f30x_opamp.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c $(IntermediateDirectory)/src_stm32f30x_opamp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_opamp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_opamp.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_opamp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_opamp.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c"

$(IntermediateDirectory)/src_stm32f30x_opamp.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_opamp.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_opamp.c"

$(IntermediateDirectory)/src_stm32f30x_pwr.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c $(IntermediateDirectory)/src_stm32f30x_pwr.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_pwr.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_pwr.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_pwr.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_pwr.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c"

$(IntermediateDirectory)/src_stm32f30x_pwr.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_pwr.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_pwr.c"

$(IntermediateDirectory)/src_stm32f30x_rcc.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c $(IntermediateDirectory)/src_stm32f30x_rcc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_rcc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_rcc.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_rcc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_rcc.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c"

$(IntermediateDirectory)/src_stm32f30x_rcc.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_rcc.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rcc.c"

$(IntermediateDirectory)/src_stm32f30x_rtc.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c $(IntermediateDirectory)/src_stm32f30x_rtc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_rtc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_rtc.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_rtc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_rtc.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c"

$(IntermediateDirectory)/src_stm32f30x_rtc.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_rtc.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_rtc.c"

$(IntermediateDirectory)/src_stm32f30x_spi.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c $(IntermediateDirectory)/src_stm32f30x_spi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_spi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_spi.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_spi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_spi.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c"

$(IntermediateDirectory)/src_stm32f30x_spi.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_spi.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_spi.c"

$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c $(IntermediateDirectory)/src_stm32f30x_syscfg.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c"

$(IntermediateDirectory)/src_stm32f30x_syscfg.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_syscfg.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_syscfg.c"

$(IntermediateDirectory)/src_stm32f30x_tim.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c $(IntermediateDirectory)/src_stm32f30x_tim.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_tim.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_tim.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_tim.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_tim.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c"

$(IntermediateDirectory)/src_stm32f30x_tim.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_tim.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_tim.c"

$(IntermediateDirectory)/src_stm32f30x_usart.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c $(IntermediateDirectory)/src_stm32f30x_usart.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_usart.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_usart.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_usart.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_usart.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c"

$(IntermediateDirectory)/src_stm32f30x_usart.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_usart.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_usart.c"

$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(ObjectSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c $(IntermediateDirectory)/src_stm32f30x_wwdg.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(DependSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(DependSuffix) -MM "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c"

$(IntermediateDirectory)/src_stm32f30x_wwdg.c$(PreprocessSuffix): lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f30x_wwdg.c$(PreprocessSuffix) "lib/main/STM32F30x_StdPeriph_Driver/src/stm32f30x_wwdg.c"

$(IntermediateDirectory)/src_misc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/misc.c $(IntermediateDirectory)/src_misc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/misc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_misc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_misc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/misc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_misc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_misc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/misc.c"

$(IntermediateDirectory)/src_misc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/misc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_misc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/misc.c"

$(IntermediateDirectory)/src_stm32f10x_adc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c $(IntermediateDirectory)/src_stm32f10x_adc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_adc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_adc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_adc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_adc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c"

$(IntermediateDirectory)/src_stm32f10x_adc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_adc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c"

$(IntermediateDirectory)/src_stm32f10x_bkp.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c $(IntermediateDirectory)/src_stm32f10x_bkp.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_bkp.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_bkp.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_bkp.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_bkp.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c"

$(IntermediateDirectory)/src_stm32f10x_bkp.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_bkp.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c"

$(IntermediateDirectory)/src_stm32f10x_can.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c $(IntermediateDirectory)/src_stm32f10x_can.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_can.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_can.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_can.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_can.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c"

$(IntermediateDirectory)/src_stm32f10x_can.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_can.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c"

$(IntermediateDirectory)/src_stm32f10x_cec.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c $(IntermediateDirectory)/src_stm32f10x_cec.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_cec.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_cec.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_cec.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_cec.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c"

$(IntermediateDirectory)/src_stm32f10x_cec.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_cec.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c"

$(IntermediateDirectory)/src_stm32f10x_crc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c $(IntermediateDirectory)/src_stm32f10x_crc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_crc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_crc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_crc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_crc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c"

$(IntermediateDirectory)/src_stm32f10x_crc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_crc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c"

$(IntermediateDirectory)/src_stm32f10x_dac.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c $(IntermediateDirectory)/src_stm32f10x_dac.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_dac.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_dac.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_dac.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_dac.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c"

$(IntermediateDirectory)/src_stm32f10x_dac.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_dac.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c"

$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c $(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c"

$(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_dbgmcu.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c"

$(IntermediateDirectory)/src_stm32f10x_dma.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c $(IntermediateDirectory)/src_stm32f10x_dma.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_dma.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_dma.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_dma.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_dma.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c"

$(IntermediateDirectory)/src_stm32f10x_dma.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_dma.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c"

$(IntermediateDirectory)/src_stm32f10x_exti.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c $(IntermediateDirectory)/src_stm32f10x_exti.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_exti.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_exti.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_exti.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_exti.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c"

$(IntermediateDirectory)/src_stm32f10x_exti.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_exti.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c"

$(IntermediateDirectory)/src_stm32f10x_flash.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c $(IntermediateDirectory)/src_stm32f10x_flash.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_flash.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_flash.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_flash.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_flash.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c"

$(IntermediateDirectory)/src_stm32f10x_flash.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_flash.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c"

$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c $(IntermediateDirectory)/src_stm32f10x_fsmc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c"

$(IntermediateDirectory)/src_stm32f10x_fsmc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_fsmc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c"

$(IntermediateDirectory)/src_stm32f10x_gpio.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c $(IntermediateDirectory)/src_stm32f10x_gpio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_gpio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_gpio.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_gpio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_gpio.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c"

$(IntermediateDirectory)/src_stm32f10x_gpio.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_gpio.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c"

$(IntermediateDirectory)/src_stm32f10x_i2c.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c $(IntermediateDirectory)/src_stm32f10x_i2c.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_i2c.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_i2c.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_i2c.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_i2c.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c"

$(IntermediateDirectory)/src_stm32f10x_i2c.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_i2c.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c"

$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c $(IntermediateDirectory)/src_stm32f10x_iwdg.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c"

$(IntermediateDirectory)/src_stm32f10x_iwdg.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_iwdg.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c"

$(IntermediateDirectory)/src_stm32f10x_pwr.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c $(IntermediateDirectory)/src_stm32f10x_pwr.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_pwr.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_pwr.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_pwr.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_pwr.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c"

$(IntermediateDirectory)/src_stm32f10x_pwr.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_pwr.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c"

$(IntermediateDirectory)/src_stm32f10x_rcc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c $(IntermediateDirectory)/src_stm32f10x_rcc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_rcc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_rcc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_rcc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_rcc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c"

$(IntermediateDirectory)/src_stm32f10x_rcc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_rcc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c"

$(IntermediateDirectory)/src_stm32f10x_rtc.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c $(IntermediateDirectory)/src_stm32f10x_rtc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_rtc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_rtc.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_rtc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_rtc.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c"

$(IntermediateDirectory)/src_stm32f10x_rtc.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_rtc.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c"

$(IntermediateDirectory)/src_stm32f10x_sdio.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c $(IntermediateDirectory)/src_stm32f10x_sdio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_sdio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_sdio.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_sdio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_sdio.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c"

$(IntermediateDirectory)/src_stm32f10x_sdio.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_sdio.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c"

$(IntermediateDirectory)/src_stm32f10x_spi.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c $(IntermediateDirectory)/src_stm32f10x_spi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_spi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_spi.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_spi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_spi.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c"

$(IntermediateDirectory)/src_stm32f10x_spi.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_spi.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c"

$(IntermediateDirectory)/src_stm32f10x_tim.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c $(IntermediateDirectory)/src_stm32f10x_tim.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_tim.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_tim.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_tim.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_tim.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c"

$(IntermediateDirectory)/src_stm32f10x_tim.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_tim.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c"

$(IntermediateDirectory)/src_stm32f10x_usart.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c $(IntermediateDirectory)/src_stm32f10x_usart.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_usart.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_usart.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_usart.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_usart.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c"

$(IntermediateDirectory)/src_stm32f10x_usart.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_usart.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c"

$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(ObjectSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c $(IntermediateDirectory)/src_stm32f10x_wwdg.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(DependSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(ObjectSuffix) -MF$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(DependSuffix) -MM "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c"

$(IntermediateDirectory)/src_stm32f10x_wwdg.c$(PreprocessSuffix): lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_stm32f10x_wwdg.c$(PreprocessSuffix) "lib/main/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c"

$(IntermediateDirectory)/CoreSupport_core_cm3.c$(ObjectSuffix): lib/main/CMSIS/CM3/CoreSupport/core_cm3.c $(IntermediateDirectory)/CoreSupport_core_cm3.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/CMSIS/CM3/CoreSupport/core_cm3.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/CoreSupport_core_cm3.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/CoreSupport_core_cm3.c$(DependSuffix): lib/main/CMSIS/CM3/CoreSupport/core_cm3.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/CoreSupport_core_cm3.c$(ObjectSuffix) -MF$(IntermediateDirectory)/CoreSupport_core_cm3.c$(DependSuffix) -MM "lib/main/CMSIS/CM3/CoreSupport/core_cm3.c"

$(IntermediateDirectory)/CoreSupport_core_cm3.c$(PreprocessSuffix): lib/main/CMSIS/CM3/CoreSupport/core_cm3.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/CoreSupport_core_cm3.c$(PreprocessSuffix) "lib/main/CMSIS/CM3/CoreSupport/core_cm3.c"

$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(ObjectSuffix): lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c $(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(DependSuffix): lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(ObjectSuffix) -MF$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(DependSuffix) -MM "lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c"

$(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(PreprocessSuffix): lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/STM32F10x_system_stm32f10x.c$(PreprocessSuffix) "lib/main/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c"

$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(ObjectSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c $(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(DependSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(ObjectSuffix) -MF$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(DependSuffix) -MM "lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c"

$(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(PreprocessSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/STM32F30x_stm32f30x_gpio.c$(PreprocessSuffix) "lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_gpio.c"

$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(ObjectSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c $(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(DependSuffix)
	$(CC) $(SourceSwitch) "C:/dev/KKNG/betaflight/lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(DependSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(ObjectSuffix) -MF$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(DependSuffix) -MM "lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c"

$(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(PreprocessSuffix): lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/STM32F30x_stm32f30x_rcc.c$(PreprocessSuffix) "lib/main/CMSIS/CM1/DeviceSupport/ST/STM32F30x/stm32f30x_rcc.c"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


