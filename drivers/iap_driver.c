/*
* iap_driver.c
* Demonstrate basic capability of In-Application Programming feature
*
* by: Daniel Widyanto
*/

#include "LPC17xx.h"
#include "../include/iap_driver.h"
#include "../stdPeriphLibs/lpc_types.h"
/*
* The IAP funtion address in LPC11xx ROM
*/
#define IAP_ADDRESS            0x1FFF1FF1

/*
* Command codes for IAP
*/
#define PREPARE_SECTOR      50
#define COPY_RAM_TO_FLASH   51
#define ERASE_SECTOR        52
#define BLANK_CHECK_SECTOR  53
#define READ_PART_ID        54
#define READ_BOOT_CODE_REV  55
#define COMPARE             56
#define REINVOKE_ISP        57
#define READ_UID            58

typedef unsigned int (*IAP)(uint32_t[], volatile uint32_t[]);
static const IAP iap_entry = (IAP) IAP_ADDRESS;

volatile uint32_t uC_id[4];
/*---------------------------------------------------------------------------
* Public functions
*/

/**
* Init IAP driver
* @return    0 for success
*/
int iap_init(void) {
    /* Need to update 'SystemCoreClock' according to the current clock settings
     * It's needed as IAP parameter
     */
    SystemCoreClockUpdate();
    return (0);
}

void iap_get_id( void )
{
	uint32_t command[5];


	command[0] = READ_UID;
	iap_entry(command, uC_id);

}

/**
* Erase flash sector(s)
*
* @param sector_start  The start of the sector to be erased
* @param sector_end    The end of the sector to be erased
*
* @return CMD_SUCCESS, BUSY, SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
*         or INVALID_SECTOR
*/
int iap_erase_sector(unsigned int sector_start, unsigned int sector_end) {
    unsigned int command[5];
    unsigned int result[4];

    command[0] = ERASE_SECTOR;
    command[1] = (unsigned int) sector_start;
    command[2] = (unsigned int) sector_end;
    command[3] = SystemCoreClock / 1000;
    iap_entry(command, result);

    return ( (int)result[0] );
}

/**
* Prepare flash sector(s) for erase / writing
*
* @param sector_start  The start of the sector to be prepared
* @param sector_end    The end of the sector to be prepared
*
* @return CMD_SUCCESS, BUSY, or INVALID_SECTOR
*/
int iap_prepare_sector(unsigned int sector_start, unsigned int sector_end) {
    unsigned int command[5];
    unsigned int result[4];

    command[0] = PREPARE_SECTOR;
    command[1] = (unsigned int) sector_start;
    command[2] = (unsigned int) sector_end;
    iap_entry(command, result);

    return ( (int)result[0] );
}

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
        unsigned int count) {
    unsigned int command[5];
    unsigned int result[4];

    command[0] = COPY_RAM_TO_FLASH;
    command[1] = (unsigned int) flash_address;
    command[2] = (unsigned int) ram_address;
    command[3] = count;
    command[4] = SystemCoreClock / 1000;
    iap_entry(command, result);

    return ( (int)result[0]);
}
