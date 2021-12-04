#include "microsd.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "freeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "nrf_log.h"
#include "string.h"
#include "globals.h"
#include "freeRTOS.h"

// microSD
#define SDC_SCK_PIN     20
#define SDC_MOSI_PIN    19
#define SDC_MISO_PIN    17
#define SDC_CS_PIN      23

/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
	m_block_dev_sdc,
	NRF_BLOCK_DEV_SDC_CONFIG(
		SDC_SECTOR_SIZE,
		APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
	),
	NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

static FATFS fs;
//static DIR dir;
//static FILINFO fno;
static FIL file;
static uint8_t failed = 0;
DSTATUS disk_state = STA_NOINIT;



void microsd_write(char* filename, char* data) {
	if(failed){
        return;
        }
	uint32_t bytes_written;
    FRESULT ff_result;
     static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    //NRF_LOG_INFO("Initializing disk 0 (SDC)...");

    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        failed =1;
        NRF_LOG_INFO("Disk initialization failed. %d", disk_state);
        return;
    }
	
	//uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    //uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    //NRF_LOG_INFO("Capacity: %d MB", capacity);

    //NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {   failed = 1;
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    disk_state = STA_NOINIT;
	
	 //NRF_LOG_INFO("Writing to file %s.", filename);
    ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file %s.", filename);
        return;
    }

    ff_result = f_write(&file, data, strlen(data), (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        //NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    (void) f_close(&file);
	
	ff_result = f_mount(NULL, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }
	disk_state = disk_uninitialize(0);
}


/**@brief Task for handling microSD operations.
 *
 * @details Initializes the microSD driver, and performs write requests.
 */
void microsd_task(void *arg) {
	 
	microsd_write_operation_t write_operation;
    
	for (;;) {
		
		xQueueReceive(queue_microsd, &write_operation, portMAX_DELAY);
        xSemaphoreTake(mutex_spi, portMAX_DELAY);
        //NRF_LOG_INFO("writing %s.", write_operation.content);
        microsd_write(write_operation.filename, write_operation.content);
        xSemaphoreGive(mutex_spi);
		
    }
}

