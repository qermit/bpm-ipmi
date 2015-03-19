#include "../include/sio_usart.h"
#include "../include/ipmb_svc.h"
#include "../include/ipmi_i2c_driver.h"
#include "../include/fru.h"
#include "../include/mmc_version.h"
#include "../include/cardType.h"

//#define PCIE_GEN2

volatile uint8_t fru_buf[END_OF_FRU_AREA_OFFSET];

#ifdef AMC_CPU_COM_Express
void fru_init(void) {
  // checks the first byte of the eeprom.  If it is 0xff, chip is considered erased, and
  // a default image is written to the eeprom

  uint8_t i;

  hardware_info_area_t* 		pHWarea 	= (hardware_info_area_t*) &fru_buf[HW_HEADER_BYTE_OFFSET];
  App_Device_ID_record_t* 		pAppDevID 	= (App_Device_ID_record_t*) &fru_buf[APP_DEV_ID_BYTE_OFFSET];
  FRU_area_common_header_t*	 	pFRUhdr 	= (FRU_area_common_header_t*) &fru_buf[COMMON_HEADER_BYTE_OFFSET];
  AMC_P2P_CONN_RECORD_t*		pP2PCon 	= (AMC_P2P_CONN_RECORD_t*) &fru_buf[MULTIRECORD_AREA_BYTE_OFFSET];
//  CLK_P2P_CONN_RECORT_t*                pClkP2PCon      = (CLK_P2P_CONN_RECORT_t*) &fru_buf[CLK_P2P_CONN_REC_OFFSET];
  CLK_CFG_REC_t*                        pClkCfgCon      = (CLK_CFG_REC_t*) &fru_buf[CLK_CFG_REC_OFFSET];
  module_current_req_record_t* 	        pcurrec 	= (module_current_req_record_t*) &fru_buf[CURRENT_REQ_OFFSET];
 // MULTIRECORD_AREA_HEADER*	pmrahdr	   = (MULTIRECORD_AREA_HEADER*) &fru_buf[175];



  // init FRU
  for (i=0; i<sizeof(fru_buf); i++)
    fru_buf[i] = 0;					// initialize to zeros

  // configure hardware info area (nonzero fields)
  pHWarea->version = 1;
  pHWarea->mmc_sw_major_vers = MAJOR_MMC_VERSION_NUMBER;
  pHWarea->mmc_sw_minor_vers = MINOR_MMC_VERSION_NUMBER;
  pHWarea->checksum = calc_ipmi_xsum((uint8_t*) pHWarea, sizeof(hardware_info_area_t)-1);

  // configure Application Device ID area (nonzero fields)
  pAppDevID->devicerev  = 0x80;					// supports SDRs
  pAppDevID->fwrev1     = 0x00;
  pAppDevID->fwrev2     = 0x03;
  pAppDevID->ipmivers   = 0x02;					// IPMI v2.0 support
  pAppDevID->adddevsuppt = 0x3b;				// device supports IPMB events, has SDR, FRU info, sensors

  // configure FRU area common header (nonzero fields)
  pFRUhdr->version = 0x01;						// v1.0 of storage definition
  pFRUhdr->multirecord_area_offset = (MULTIRECORD_AREA_BYTE_OFFSET-COMMON_HEADER_BYTE_OFFSET)/8;
  pFRUhdr->checksum = calc_ipmi_xsum((uint8_t*) pFRUhdr, sizeof(FRU_area_common_header_t)-1);

  /* Point-to-point connectivity record */
  pP2PCon->record_type_id = 0xC0;	/* For all records a value of C0h (OEM) shall be used. */
  pP2PCon->eol = 0;		/* [7:7] End of list. Set to one for the last record */
  pP2PCon->reserved = 0;		/* Reserved, write as 0h.*/
  pP2PCon->version = 2;		/* record format version (2h for this definition) */
  pP2PCon->record_len = 47;//39;//sizeof(AMC_P2P_CONN_RECORD_t);	/* Record Length. */

	/* Manufacturer ID - For the AMC specification the value 12634 (00315Ah) must be used. */
  pP2PCon->manuf_id[0] = 0x5A;
  pP2PCon->manuf_id[1] = 0x31;
  pP2PCon->manuf_id[2] = 0x00;
  pP2PCon->picmg_rec_id = 0x19;	/* 0x19 for AMC Point-to-Point Connectivity record */
  pP2PCon->rec_fmt_ver = 0;	/* Record Format Version, = 0 for this specification */
  pP2PCon->oem_guid_count = 0;	/* OEM GUID Count */
  pP2PCon->record_type = 1;	/* 1 = AMC Module */
  pP2PCon->conn_dev_id = 0;	/* Connected-device ID if Record Type = 0, Reserved, otherwise. */
  pP2PCon->ch_descr_count = 3;	/* AMC Channel Descriptor Count */

  // AMC Link configuration for PCIe

  pP2PCon->amc_ch_descr0[0] = 0xA4;//0xA4;
  pP2PCon->amc_ch_descr0[1] = 0x98;//0x98;
  pP2PCon->amc_ch_descr0[2] = 0xF3;//0xF3;

  pP2PCon->amc_ch_descr1[0] = 0x28;//0xA4;
  pP2PCon->amc_ch_descr1[1] = 0xA9;//0x98;
  pP2PCon->amc_ch_descr1[2] = 0xF5;//0xF3;

  pP2PCon->amc_ch_descr2[0] = 0x00;//0x28;
  pP2PCon->amc_ch_descr2[1] = 0x00;//0xA9;
  pP2PCon->amc_ch_descr2[2] = 0x00;//0xF5;

  pP2PCon->amc_link_descr0[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr0[1] = 0x2F;
  pP2PCon->amc_link_descr0[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr0[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr0[4] = 0xFE;//0xFD;

  pP2PCon->amc_link_descr1[0] = 0x01;//0x00;
  pP2PCon->amc_link_descr1[1] = 0x2F;
  pP2PCon->amc_link_descr1[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr1[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr1[4] = 0xFE;//0xFD;

  pP2PCon->amc_link_descr2[0] = 0x00;//0x01;//0x01;
  pP2PCon->amc_link_descr2[1] = 0x00;//0x2F;//0x2F;
  pP2PCon->amc_link_descr2[2] = 0x00;//0x20;//0x00;
  pP2PCon->amc_link_descr2[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr2[4] = 0x00;//0xFE;//0xFD;

  pP2PCon->amc_link_descr3[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[1] = 0x00;//0x23;//0x2F;
  pP2PCon->amc_link_descr3[2] = 0x00;//0x10;//0x00;
  pP2PCon->amc_link_descr3[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[4] = 0x00;//0xFD;//0xFD;

  pP2PCon->amc_link_descr4[0] = 0x00;//0x01;//0x00;
  pP2PCon->amc_link_descr4[1] = 0x00;//0x21;//0x23;
  pP2PCon->amc_link_descr4[2] = 0x00;//0x00;
  pP2PCon->amc_link_descr4[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr4[4] = 0x00;//0xFD;//0xFD;

  pP2PCon->amc_link_descr5[0] = 0x00;//0x01;//0x00;
  pP2PCon->amc_link_descr5[1] = 0x00;//0x21;//0x21;
  pP2PCon->amc_link_descr5[2] = 0x00;//0x10;//0x00;
  pP2PCon->amc_link_descr5[3] = 0x00;//0x00;//0x00;
  pP2PCon->amc_link_descr5[4] = 0x00;//0xFD;

  pP2PCon->record_cksum = calc_ipmi_xsum( ( uint8_t * )&( pP2PCon->manuf_id[0] ), 47 );
  pP2PCon->header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pP2PCon->record_type_id ), 4);

//  /* Clock point-to-point connectivity record */
//  pClkP2PCon->rcrdHdr.record_type_id = 0xC0;       /* For all records a value of C0h (OEM) shall be used. */
//  pClkP2PCon->rcrdHdr.eol = 0;             /* [7:7] End of list. Set to one for the last record */
//  pClkP2PCon->rcrdHdr.reserved = 0;                /* Reserved, write as 0h.*/
//  pClkP2PCon->rcrdHdr.version = 2;         /* record format version (2h for this definition) */
//  pClkP2PCon->rcrdHdr.record_len = 11;//39;//sizeof(AMC_P2P_CONN_RECORD_t);        /* Record Length. */
//
//  /* Manufacturer ID - For the AMC specification the value 12634 (00315Ah) must be used. */
//  pClkP2PCon->rcrdHdr.manuf_id[0] = 0x5A;
//  pClkP2PCon->rcrdHdr.manuf_id[1] = 0x31;
//  pClkP2PCon->rcrdHdr.manuf_id[2] = 0x00;
//  pClkP2PCon->rcrdHdr.picmg_rec_id = 0x2C; /* 0x2C for CLK Point-to-Point Connectivity record */
//  pClkP2PCon->rcrdHdr.rec_fmt_ver  = 0;     /* Record Format Version, = 0 for this specification */
//  pClkP2PCon->clkP2PDescrCnt       = 1;
//
//  pClkP2PCon->clkDescr.clkResID      = 0x64|ipmi_i2c_state.slotid;;
//  pClkP2PCon->clkDescr.clkP2PConnCnt = 1;
//  pClkP2PCon->clkDescr.clkConnDescr.localClkID   = FCLKA;
//  pClkP2PCon->clkDescr.clkConnDescr.remoteClkID  = FCLKA;
//  pClkP2PCon->clkDescr.clkConnDescr.remoteClkRes = 0x80;
//
//  pClkP2PCon->rcrdHdr.record_cksum = calc_ipmi_xsum( ( uint8_t * )&( pClkP2PCon->rcrdHdr.manuf_id[0] ), 11);
//  pClkP2PCon->rcrdHdr.header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pClkP2PCon->rcrdHdr.record_type_id ), 4);

  /* Clock point-to-point connectivity record */
  pClkCfgCon->rcrdHdr.record_type_id = 0xC0;       /* For all records a value of C0h (OEM) shall be used. */
  pClkCfgCon->rcrdHdr.eol = 0;             /* [7:7] End of list. Set to one for the last record */
  pClkCfgCon->rcrdHdr.reserved = 0;                /* Reserved, write as 0h.*/
  pClkCfgCon->rcrdHdr.version = 2;         /* record format version (2h for this definition) */
  pClkCfgCon->rcrdHdr.record_len = 26;//39;//sizeof(AMC_P2P_CONN_RECORD_t);        /* Record Length. */

  /* Manufacturer ID - For the AMC specification the value 12634 (00315Ah) must be used. */
  pClkCfgCon->rcrdHdr.manuf_id[0] = 0x5A;
  pClkCfgCon->rcrdHdr.manuf_id[1] = 0x31;
  pClkCfgCon->rcrdHdr.manuf_id[2] = 0x00;
  pClkCfgCon->rcrdHdr.picmg_rec_id = 0x2D; /* 0x2C for CLK Point-to-Point Connectivity record */
  pClkCfgCon->rcrdHdr.rec_fmt_ver  = 0;     /* Record Format Version, = 0 for this specification */

  pClkCfgCon->clkResID = 0x40|ipmi_i2c_state.slotid;//0xff;
  pClkCfgCon->clkCfgDescCnt = 1;
  pClkCfgCon->clkCfgDescr.clockID = FCLKA;
  pClkCfgCon->clkCfgDescr.clockCtrl = 0;
  pClkCfgCon->clkCfgDescr.indirClkDescCnt = 0;
  pClkCfgCon->clkCfgDescr.dirClkDescCnt = 1;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFeatures = 0;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFamily = 0x02;
  pClkCfgCon->clkCfgDescr.dirDescr.clockAccuracy = 10;

  pClkCfgCon->clkCfgDescr.dirDescr.clockFreq[0] = 0x00;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreq[1] = 0xe1;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreq[2] = 0xf5;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreq[3] = 0x05;

  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMin[0] = 0xc0;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMin[1] = 0x95;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMin[2] = 0xa9;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMin[3] = 0x05;

  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMax[0] = 0x40;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMax[1] = 0x2c;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMax[2] = 0x42;
  pClkCfgCon->clkCfgDescr.dirDescr.clockFreqMax[3] = 0x06;

  pClkCfgCon->rcrdHdr.record_cksum = calc_ipmi_xsum( ( uint8_t * )&( pClkCfgCon->rcrdHdr.manuf_id[0] ), 26);
  pClkCfgCon->rcrdHdr.header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pClkCfgCon->rcrdHdr.record_type_id ), 4);

  // configure FRU multirecord area--default current draw per PICMG
  pcurrec->hdr.rectypeID = 0xc0;				// OEM record type
  pcurrec->hdr.eolflag_recversion = 0x82;		// last record flag + version 02h
  pcurrec->hdr.reclength = 6;
  pcurrec->mfgID_LSB = 0x5a;
  pcurrec->mfgID_MidB = 0x31;
  pcurrec->mfgID_MSB = 0x00;
  pcurrec->PICMG_recID = 0x16;				// current requirements record
  pcurrec->record_fmt_version = 0x00;
  pcurrec->current_draw_100mA = 20;			// say 2A for now
  pcurrec->hdr.recxsum = calc_ipmi_xsum((uint8_t*) &(pcurrec->mfgID_LSB), 6);
  pcurrec->hdr.hdrxsum = calc_ipmi_xsum((uint8_t*) &(pcurrec->hdr.rectypeID), 4);

//  // Last record
//  pmrahdr->record_type_id = 0xC0;	/* For all records a value of C0h (OEM) shall be used. */
//  pmrahdr->eol = 1;		/* End of list. Set to one for the last record */
//  pmrahdr->reserved = 0;
//  pmrahdr->version = 2;	/* record format version (2h for this definition) */
//  pmrahdr->record_len = sizeof(MULTIRECORD_AREA_HEADER);	/* Record Length. */
//  pmrahdr->manuf_id[0] = 0x5A;
//  pmrahdr->manuf_id[1] = 0x31;
//  pmrahdr->manuf_id[2] = 0x00;
//  pmrahdr->picmg_rec_id = 0;	/* PICMG Record ID. */
//  pmrahdr->rec_fmt_ver = 0;	/* For this specification, the value 0h shall be used. */
//  pmrahdr->record_cksum = calc_ipmi_xsum( ( uint8_t * )&( pmrahdr->manuf_id[0] ), 5 );
//  pmrahdr->header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pmrahdr->record_type_id ), 4 );

}
#else
void fru_init(void)
{

  hardware_info_area_t* pHWarea         = (hardware_info_area_t*) &fru_buf[HW_HEADER_BYTE_OFFSET];
  App_Device_ID_record_t* pAppDevID     = (App_Device_ID_record_t*) &fru_buf[APP_DEV_ID_BYTE_OFFSET];
  FRU_area_common_header_t*     pFRUhdr = (FRU_area_common_header_t*) &fru_buf[COMMON_HEADER_BYTE_OFFSET];
  AMC_P2P_CONN_RECORD_t*        pP2PCon = (AMC_P2P_CONN_RECORD_t*) &fru_buf[MULTIRECORD_AREA_BYTE_OFFSET];
  module_current_req_record_t*  pcurrec = (module_current_req_record_t*) &fru_buf[172];
 // MULTIRECORD_AREA_HEADER*	pmrahdr	   = (MULTIRECORD_AREA_HEADER*) &fru_buf[175];

  uint8_t i;
  sio_putstr("Writing default Device ID image....\n");

  for (i=0; i<sizeof(fru_buf); i++)
    fru_buf[i] = 0;					// initialize to zeros

  // configure hardware info area (nonzero fields)
  pHWarea->version = 1;
  pHWarea->mmc_sw_major_vers = MAJOR_MMC_VERSION_NUMBER;
  pHWarea->mmc_sw_minor_vers = MINOR_MMC_VERSION_NUMBER;
  pHWarea->checksum = calc_ipmi_xsum((uint8_t*) pHWarea, sizeof(hardware_info_area_t)-1);

  // configure Application Device ID area (nonzero fields)
  pAppDevID->devicerev = 0x80;					// supports SDRs
  pAppDevID->fwrev1 = 0x00;
  pAppDevID->fwrev2 = 0x03;
  pAppDevID->ipmivers = 0x02;					// IPMI v2.0 support
  pAppDevID->adddevsuppt = 0x3b;				// device supports IPMB events, has SDR, FRU info, sensors

  // configure FRU area common header (nonzero fields)
  pFRUhdr->version = 0x01;						// v1.0 of storage definition
  pFRUhdr->multirecord_area_offset = (MULTIRECORD_AREA_BYTE_OFFSET-COMMON_HEADER_BYTE_OFFSET)/8;
  pFRUhdr->checksum = calc_ipmi_xsum((uint8_t*) pFRUhdr, sizeof(FRU_area_common_header_t)-1);


  /* Point-to-point connectivity record */
  pP2PCon->record_type_id = 0xC0;	/* For all records a value of C0h (OEM) shall be used. */
  pP2PCon->eol = 0;		/* [7:7] End of list. Set to one for the last record */
  pP2PCon->reserved = 0;		/* Reserved, write as 0h.*/
  pP2PCon->version = 2;		/* record format version (2h for this definition) */
  pP2PCon->record_len = 47;//39;//sizeof(AMC_P2P_CONN_RECORD_t);	/* Record Length. */

	/* Manufacturer ID - For the AMC specification the value 12634 (00315Ah) must be used. */
  pP2PCon->manuf_id[0] = 0x5A;
  pP2PCon->manuf_id[1] = 0x31;
  pP2PCon->manuf_id[2] = 0x00;
  pP2PCon->picmg_rec_id = 0x19;	/* 0x19 for AMC Point-to-Point Connectivity record */
  pP2PCon->rec_fmt_ver = 0;	/* Record Format Version, = 0 for this specification */
  pP2PCon->oem_guid_count = 0;	/* OEM GUID Count */
  pP2PCon->record_type = 1;	/* 1 = AMC Module */
  pP2PCon->conn_dev_id = 0;	/* Connected-device ID if Record Type = 0, Reserved, otherwise. */
  pP2PCon->ch_descr_count = 3;	/* AMC Channel Descriptor Count */

#ifdef PCIE_GEN2
  // AMC Link configuration for PCIe GEN2
  
  pP2PCon->amc_ch_descr0[0] = 0x00;//0xA4;
  pP2PCon->amc_ch_descr0[1] = 0x00;//0x98;
  pP2PCon->amc_ch_descr0[2] = 0x00;//0xF3;

  pP2PCon->amc_ch_descr1[0] = 0xA4;//0xA4;
  pP2PCon->amc_ch_descr1[1] = 0x98;//0x98;
  pP2PCon->amc_ch_descr1[2] = 0xF3;//0xF3;

  pP2PCon->amc_ch_descr2[0] = 0xA4;//0x28;
  pP2PCon->amc_ch_descr2[1] = 0x98;//0xA9;
  pP2PCon->amc_ch_descr2[2] = 0xF3;//0xF5;

  pP2PCon->amc_link_descr0[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr0[1] = 0x2F;
  pP2PCon->amc_link_descr0[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr0[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr0[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr1[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr1[1] = 0x2F;
  pP2PCon->amc_link_descr1[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr1[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr1[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr2[0] = 0x00;//0x01;
  pP2PCon->amc_link_descr2[1] = 0x23;//0x2F;
  pP2PCon->amc_link_descr2[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr2[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr2[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr3[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[1] = 0x23;//0x2F;
  pP2PCon->amc_link_descr3[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr3[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr4[0] = 0x01;//0x00;
  pP2PCon->amc_link_descr4[1] = 0x21;//0x23;
  pP2PCon->amc_link_descr4[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr4[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr4[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr5[0] = 0x01;//0x00;
  pP2PCon->amc_link_descr5[1] = 0x21;//0x21;
  pP2PCon->amc_link_descr5[2] = 0x20;//0x00;
  pP2PCon->amc_link_descr5[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr5[4] = 0xFD;//0xFD;
#else
  // AMC Link configuration for PCIe GEN1
  pP2PCon->amc_ch_descr0[0] = 0x00;//0xA4;
  pP2PCon->amc_ch_descr0[1] = 0x00;//0x98;
  pP2PCon->amc_ch_descr0[2] = 0x00;//0xF3;

  pP2PCon->amc_ch_descr1[0] = 0xA4;//0xA4;
  pP2PCon->amc_ch_descr1[1] = 0x98;//0x98;
  pP2PCon->amc_ch_descr1[2] = 0xF3;//0xF3;

  pP2PCon->amc_ch_descr2[0] = 0xA4;//0x28;
  pP2PCon->amc_ch_descr2[1] = 0x98;//0xA9;
  pP2PCon->amc_ch_descr2[2] = 0xF3;//0xF5;

  pP2PCon->amc_link_descr0[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr0[1] = 0x2F;
  pP2PCon->amc_link_descr0[2] = 0x00;//0x00;
  pP2PCon->amc_link_descr0[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr0[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr1[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr1[1] = 0x2F;
  pP2PCon->amc_link_descr1[2] = 0x10;//0x00;
  pP2PCon->amc_link_descr1[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr1[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr2[0] = 0x00;//0x01;
  pP2PCon->amc_link_descr2[1] = 0x23;//0x2F;
  pP2PCon->amc_link_descr2[2] = 0x00;//0x00;
  pP2PCon->amc_link_descr2[3] = 0x00;//0x01;
  pP2PCon->amc_link_descr2[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr3[0] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[1] = 0x23;//0x2F;
  pP2PCon->amc_link_descr3[2] = 0x10;//0x00;
  pP2PCon->amc_link_descr3[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr3[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr4[0] = 0x01;//0x00;
  pP2PCon->amc_link_descr4[1] = 0x21;//0x23;
  pP2PCon->amc_link_descr4[2] = 0x00;//0x00;
  pP2PCon->amc_link_descr4[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr4[4] = 0xFD;//0xFD;

  pP2PCon->amc_link_descr5[0] = 0x01;//0x00;
  pP2PCon->amc_link_descr5[1] = 0x21;//0x21;
  pP2PCon->amc_link_descr5[2] = 0x10;//0x00;
  pP2PCon->amc_link_descr5[3] = 0x00;//0x00;
  pP2PCon->amc_link_descr5[4] = 0xFD;//0xFD;
#endif

  pP2PCon->record_cksum =  calc_ipmi_xsum( ( uint8_t * )&( pP2PCon->manuf_id[0] ), 47 );
  pP2PCon->header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pP2PCon->record_type_id ), 4);


  // configure FRU multirecord area--default current draw per PICMG
  pcurrec->hdr.rectypeID = 0xc0;				// OEM record type
  pcurrec->hdr.eolflag_recversion = 0x82;		// last record flag + version 02h
  pcurrec->hdr.reclength = 6;
  pcurrec->mfgID_LSB = 0x5a;
  pcurrec->mfgID_MidB = 0x31;
  pcurrec->mfgID_MSB = 0x00;
  pcurrec->PICMG_recID = 0x16;				// current requirements record
  pcurrec->record_fmt_version = 0x00;
  pcurrec->current_draw_100mA = 80;			// say 2A for now
  pcurrec->hdr.recxsum = calc_ipmi_xsum((uint8_t*) &(pcurrec->mfgID_LSB), 6);
  pcurrec->hdr.hdrxsum = calc_ipmi_xsum((uint8_t*) &(pcurrec->hdr.rectypeID), 4);

//  // Last record
//  pmrahdr->record_type_id = 0xC0;	/* For all records a value of C0h (OEM) shall be used. */
//  pmrahdr->eol = 1;		/* End of list. Set to one for the last record */
//  pmrahdr->reserved = 0;
//  pmrahdr->version = 2;	/* record format version (2h for this definition) */
//  pmrahdr->record_len = sizeof(MULTIRECORD_AREA_HEADER);	/* Record Length. */
//  pmrahdr->manuf_id[0] = 0x5A;
//  pmrahdr->manuf_id[1] = 0x31;
//  pmrahdr->manuf_id[2] = 0x00;
//  pmrahdr->picmg_rec_id = 0;	/* PICMG Record ID. */
//  pmrahdr->rec_fmt_ver = 0;	/* For this specification, the value 0h shall be used. */
//  pmrahdr->record_cksum = calc_ipmi_xsum( ( uint8_t * )&( pmrahdr->manuf_id[0] ), 5 );
//  pmrahdr->header_cksum = calc_ipmi_xsum( ( uint8_t * )&( pmrahdr->record_type_id ), 4 );

}
#endif

void read_fru_data(uint8_t* prbuf, uint8_t saddr, uint8_t len)
{
	uint8_t* currptr = prbuf;
	uint8_t lenLeft = len;

	while(lenLeft)
	{
		*currptr = fru_buf[saddr+len-lenLeft];
		currptr ++;
		lenLeft--;
	}
}

