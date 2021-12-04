PROJECT_NAME     := ble_app_hrs_freertos_pca10056_s140
TARGETS          := nrf52840_xxaa

SDK_ROOT := ../..
PROJ_DIR := .

OUTPUT_DIRECTORY := _build

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := $(PROJ_DIR)/config/linkerscript.ld
  #$(PROJ_DIR)/config/ble_app_hrs_freertos_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer_freertos.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/sensorsim/sensorsim.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/external/freertos/source/croutine.c \
  $(SDK_ROOT)/external/freertos/source/event_groups.c \
  $(SDK_ROOT)/external/freertos/source/portable/MemMang/heap_4.c \
  $(SDK_ROOT)/external/freertos/source/list.c \
  $(SDK_ROOT)/external/freertos/portable/GCC/nrf52/port.c \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis.c \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c \
  $(SDK_ROOT)/external/freertos/source/queue.c \
  $(SDK_ROOT)/external/freertos/source/stream_buffer.c \
  $(SDK_ROOT)/external/freertos/source/tasks.c \
  $(SDK_ROOT)/external/freertos/source/timers.c \
  $(SDK_ROOT)/external/thedotfactory_fonts/orkney8pts.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_twi.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_ppi.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_ppi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_pwm.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_saadc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_btn_ble.c \
  $(SDK_ROOT)/components/libraries/twi_mngr/nrf_twi_mngr.c \
  $(PROJ_DIR)/main.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas/ble_bas.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs/ble_hrs.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_freertos.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/libraries/pwm/app_pwm.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/block_dev/sdc/nrf_block_dev_sdc.c \
  $(SDK_ROOT)/components/libraries/sdcard/app_sdcard.c \
  $(SDK_ROOT)/components/libraries/gfx/nrf_gfx.c \
  $(SDK_ROOT)/components/libraries/spi_mngr/nrf_spi_mngr.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/external/fatfs/port/diskio_blkdev.c \
  $(SDK_ROOT)/external/fatfs/src/ff.c \
  $(PROJ_DIR)/drivers/motor.c \
  $(PROJ_DIR)/drivers/servo.c \
  $(PROJ_DIR)/drivers/i2c.c \
  $(PROJ_DIR)/drivers/ir.c \
  $(PROJ_DIR)/drivers/encoder.c \
  $(PROJ_DIR)/drivers/encoder_with_counter.c \
  $(PROJ_DIR)/drivers/oled20.c \
  $(PROJ_DIR)/drivers/display.c \
  $(PROJ_DIR)/software/DisplayTask.c \
  $(PROJ_DIR)/drivers/functions.c \
  $(PROJ_DIR)/drivers/microsd.c \
  $(PROJ_DIR)/drivers/led.c \
  $(PROJ_DIR)/drivers/ICM_20948.c \
  $(PROJ_DIR)/ble_communication/server_communication.c \
  $(PROJ_DIR)/ble_communication/bluetooth.c \
  $(PROJ_DIR)/ble_communication/network.c \
  $(PROJ_DIR)/ble_communication/crc.c \
  $(PROJ_DIR)/ble_communication/cobs.c \
  $(PROJ_DIR)/ble_communication/buffer.c \
  $(PROJ_DIR)/ble_communication/simple_protocol.c \
  $(PROJ_DIR)/ble_communication/arq.c \
  $(PROJ_DIR)/mqtt_communication/mqtt_dongle_legacy.c \
  $(PROJ_DIR)/mqtt_communication/openTreadMQTT.c \
  $(PROJ_DIR)/software/ControllerTask.c \
  $(PROJ_DIR)/software/EstimatorTask.c \
  $(PROJ_DIR)/software/NewEstimatorTask.c \
  $(PROJ_DIR)/software/KalmanFilter.c \
  $(PROJ_DIR)/software/ExtendedKalmanFilter.c \
  $(PROJ_DIR)/software/MainComTask.c \
  $(PROJ_DIR)/software/matrix_operations.c \
  $(PROJ_DIR)/software/SensorTowerTask.c \
  $(PROJ_DIR)/software/positionEstimate.c \
  $(PROJ_DIR)/software/PID_controller.c \
  $(PROJ_DIR)/software/MotorSpeedControllerTask.c \
  $(PROJ_DIR)/test_functions/IMUTester.c \
  $(PROJ_DIR)/test_functions/EncoderTester.c \
  $(PROJ_DIR)/test_functions/EncoderWithCounterTester.c \
  $(PROJ_DIR)/test_functions/SensorTowerTester.c \
  $(PROJ_DIR)/test_functions/DebugFunctions.c \
  $(SDK_ROOT)/components/libraries/bsp/bsp_thread.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_client.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_gateway_discovery.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_fifo.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_receiver.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_sender.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_platform.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_transport_ot.c \
  $(SDK_ROOT)/components/thread/utils/thread_utils.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNConnectClient.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNConnectServer.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNDeserializePublish.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNPacket.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNSearchClient.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNSearchServer.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNSerializePublish.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNSubscribeClient.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNSubscribeServer.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNUnsubscribeClient.c \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet/MQTTSNUnsubscribeServer.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_client.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_gateway_discovery.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_fifo.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_receiver.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_packet_sender.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_platform.c \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client/mqttsn_transport_ot.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_ROOT)/components/libraries/uart/retarget.c \
  $(SDK_ROOT)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_ROOT)/components/libraries/mem_manager/mem_manager.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_nvmc.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/modules/nrfx/hal/nrf_nvmc.c \
 

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components/libraries/experimental_log \
  $(SDK_ROOT)/components/libraries/experimental_log/src \
  $(SDK_ROOT)/components/libraries/experimental_memobj \
  $(SDK_ROOT)/components/libraries/experimental_mpu \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components/nfc/ndef/generic/message \
  $(SDK_ROOT)/components/nfc/t2t_lib \
  $(SDK_ROOT)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/nfc/ndef/text \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/common \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/nfc/ndef/generic/record \
  $(SDK_ROOT)/components/nfc/t4t_parser/cc_file \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/external/freertos/portable/GCC/nrf52 \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52 \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/nfc/t4t_parser/apdu \
  $(SDK_ROOT)/external/freertos/config \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/external/freertos/portable/CMSIS/nrf52 \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/softdevice/s140/headers \
  $(SDK_ROOT)/components/nfc/t4t_parser/tlv \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/nfc/ndef/parser/record \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_ROOT)/components/nfc/t2t_parser \
  $(SDK_ROOT)/external/freertos/source/include \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/libraries/sensorsim \
  $(SDK_ROOT)/components/nfc/t4t_lib \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/nfc/ndef/parser/message \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/nfc/ndef/uri \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/twi_sensor \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/nfc/ndef/launchapp \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_ROOT)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/twi_mngr \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/block_dev \
  $(SDK_ROOT)/components/libraries/block_dev/sdc \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/external/thedotfactory_fonts \
  $(SDK_ROOT)/external/fatfs/src \
  $(SDK_ROOT)/external/fatfs/port \
  $(SDK_ROOT)/external/protothreads \
  $(SDK_ROOT)/external/protothreads/pt-1.4 \
  $(SDK_ROOT)/external/fatfs/src \
  $(SDK_ROOT)/external/thedotfactory_fonts \
  $(PROJ_DIR)/drivers \
  $(PROJ_DIR)/ble_communication \
  $(PROJ_DIR)/mqtt_communication \
  $(PROJ_DIR)/software \
  $(PROJ_DIR)/test_functions \
  $(SDK_ROOT)/components/thread/utils \
  $(SDK_ROOT)/external/openthread/include \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client \
  $(SDK_ROOT)/external/paho/mqtt-sn/mqttsn_packet \
  $(SDK_ROOT)/components/thread/mqtt_sn/mqtt_sn_client \
  $(SDK_ROOT)/components/libraries/uart \
  $(SDK_ROOT)/components/libraries/mem_manager/ \
  $(SDK_ROOT)/components/libraries/fstorage/ \

# Libraries common to all targets
LIB_FILES += \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-cli-ftd.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-diag.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-ftd.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-platform-utils.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libmbedcrypto.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-nrf52840-sdk.a \
  $(SDK_ROOT)/external/nrf_cc310/lib/cortex-m4/hard-float/libnrf_cc310_0.9.12.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libnordicsemi-nrf52840-radio-driver.a \
  $(SDK_ROOT)/external/openthread/lib/nrf52840/gcc/libopenthread-platform-utils.a \

# Optimization flags
OPT = -O3 -g3 -DDEBUG
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DFREERTOS
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DOPENTHREAD_FTD=1
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror  #Werror=threat all warnings as error
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DFREERTOS
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DOPENTHREAD_FTD=1
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs
LDFLAGS += -u_printf_float

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=1024
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=2048
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=1024
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=2048

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s140_nrf52_6.0.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := $(PROJ_DIR)/config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)


# Debug device
debug:
	#if pgrep /opt/openocd/bin/openocd; then pkill /opt/openocd/bin/openocd; fi
	mergehex -m $(SDK_ROOT)/components/softdevice/s140/hex/s140_nrf52_6.0.0_softdevice.hex _build/nrf52840_xxaa.hex -o out.hex
	JLinkGDBServerCL -device "nrf52840_xxaa" -endian little -speed 2000 -if SWD
	sleep 10
	arm-eabi-gdb -tui -iex "target extended-remote localhost:3333" _build/nrf52840_xxaa.out
	killall -s 9 /opt/openocd/bin/openocd
