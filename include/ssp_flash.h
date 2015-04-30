/*
 * ssp_flash.h
 *
 *  Created on: Apr 29, 2015
 *      Author: Henrique Silva
 */

#ifndef SSP_FLASH_H_
#define SSP_FLASH_H_

#define N25Q_PAGE_SIZE		256
#define SSP_BUFFER_SIZE		N25Q_PAGE_SIZE

/* N25Q128A Instruction Codes (Extended SPI) */
#define N25Q_READ_ID		(0x9F)
#define N25Q_READ_B		(0x03)
#define N25Q_FAST_READ		(0x0B)
#define N25Q_READ_OTP		(0x4B)

#define N25Q_WRITE_EN		(0x06)
#define N25Q_WRITE_DIS		(0x04)
#define N25Q_PAGE_PROG		(0x02)
#define N25Q_PROG_OTP		(0x42)
#define N25Q_PROG_OTP		(0x42)

#define N25Q_SUBSECTOR_ERASE	(0x20)
#define N25Q_SECTOR_ERASE	(0xD8)
#define N25Q_BULK_ERASE	(0xC7)

#define N25Q_PROG_ERS_RESUME	(0x7A)
#define N25Q_PROG_ERS_SUSPEND	(0x75)

#define N25Q_READ_STATUS_REG	(0x05)
#define N25Q_WRITE_STATUS_REG	(0x01)

typedef struct
{
  uint8_t mem_type;
  uint8_t mem_size;
}N25Q_Dev_ID_t;

typedef struct
{
  uint8_t uid_len;
  uint8_t extended_dev_id[2];
  uint8_t custom_fac_data[14]; //Will be all 0
}N25Q_UID_t;

typedef struct
{
  uint8_t Man_ID;
  N25Q_UID_t UID;
  N25Q_Dev_ID_t Dev_ID;
}N25Q_ReadID_t;

void ssp_init(void);
N25Q_ReadID_t n25q_readid(void);

#endif /* SSP_FLASH_H_ */
