/*
* iap_driver.h
* Demonstrate basic capability of In-Application Programming feature
*
* by: Daniel Widyanto
*/

#ifndef IAP_DRIVER_H_
#define IAP_DRIVER_H_

#include "../stdPeriphLibs/lpc_types.h"

/*
* IAP status codes
*/
typedef enum {
    CMD_SUCCESS = 0,
    INVALID_COMMAND,
    SRC_ADDR_ERROR,
    DST_ADDR_ERROR,
    SRC_ADDR_NOT_MAPPED,
    DST_ADDR_NOT_MAPPED,
    COUNT_ERROR,
    INVALID_SECTOR,
    SECTOR_NOT_BLANK,
    SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
    COMPARE_ERROR,
    BUSY,
} __e_iap_status;


extern volatile uint32_t uC_id[4];

/**
* Init IAP driver
* @return    0 for success
*/
int iap_init(void);

void iap_get_id( void );

/**
* Erase flash sector(s)
*
* @param sector_start  The start of the sector to be erased
* @param sector_end    The end of the sector to be erased
*
* @return CMD_SUCCESS, BUSY, SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
*         or INVALID_SECTOR
*/
int iap_erase_sector(unsigned int sector_start, unsigned int sector_end);

/**
* Prepare flash sector(s) for erase / writing
*
* @param sector_start  The start of the sector to be prepared
* @param sector_end    The end of the sector to be prepared
*
* @return CMD_SUCCESS, BUSY, or INVALID_SECTOR
*/
int iap_prepare_sector(unsigned int sector_start, unsigned int sector_end);

/**
* Copy RAM contents into flash
*
* @param ram_address    RAM address to be copied
*                       It should be in word boundary
* @param flash_address  Flash address where the contents are to be copied
*                       It should be within 256bytes boundary
* @param count          Number of data to be copied (in bytes)
*                       The options: 256, 512, 1024, 4096
*
* @return CMD_SUCCESS, BUSY, or INVALID_SECTOR
*/
int iap_copy_ram_to_flash(void* ram_address, void* flash_address,
        unsigned int count);

#endif /* IAP_DRIVER_H_ */
