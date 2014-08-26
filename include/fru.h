
#ifndef FRU_H_
#define FRU_H_

typedef struct {
  uint8_t EEP_format_flag;
  uint8_t version;                    // version field for hardware info area--this is version 1
  uint8_t cmc_hw_vers;
  uint8_t cmc_hw_ser_num_lbyte;
  uint8_t cmc_hw_ser_num_ubyte;
  uint8_t mmc_sw_major_vers;
  uint8_t mmc_sw_minor_vers;
  uint8_t amc_hw_vers_lbyte;
  uint8_t amc_hw_vers_ubyte;
  uint8_t amc_hw_ser_num_lbyte;
  uint8_t amc_hw_ser_num_lmbyte;
  uint8_t amc_hw_ser_num_umbyte;
  uint8_t amc_hw_ser_num_ubyte;
  uint8_t amc_hw_id_lbyte;
  uint8_t amc_hw_id_ubyte;
  uint8_t amc_fw_id_lbyte;
  uint8_t amc_fw_id_ubyte;
  uint8_t checksum;
} hardware_info_area_t;


typedef struct multirecord_area_header {
	uint8_t	record_type_id;	/* Record Type ID. For all records defined
				   in this specification a value of C0h (OEM)
				   shall be used. */
#ifdef BF_MS_FIRST
	uint8_t 	eol:1,		/* [7:7] End of list. Set to one for the last record */
	      	reserved:3,	/* [6:4] Reserved, write as 0h.*/
		version:4;	/* [3:0] record format version (2h for this definition) */
#else
	uint8_t	version:4,
			reserved:3,
			eol:1;
#endif
	uint8_t	record_len;	/* Record Length. */
	uint8_t	record_cksum;	/* Record Checksum. Holds the zero checksum of
				   the record. */
	uint8_t	header_cksum;	/* Header Checksum. Holds the zero checksum of
				   the header. */
	uint8_t	manuf_id[3];	/* Manufacturer ID. LS Byte first. Write as the
				   three byte ID assigned to PICMG�. For this
				   specification, the value 12634 (00315Ah) shall
				   be used. */
	uint8_t	picmg_rec_id;	/* PICMG Record ID. */
	uint8_t	rec_fmt_ver;	/* Record Format Version. For this specification,
				   the value 0h shall be used. */
} MULTIRECORD_AREA_HEADER;


typedef struct {
  uint8_t deviceID;
  uint8_t devicerev;
  uint8_t fwrev1;
  uint8_t fwrev2;
  uint8_t ipmivers;
  uint8_t adddevsuppt;
  uint8_t manfIDlbyte;
  uint8_t manfIDmidbyte;
  uint8_t manfIDhbyte;
  uint8_t prodIDlbyte;
  uint8_t prodIDhbyte;
} App_Device_ID_record_t;

typedef struct {
  uint8_t version;
  uint8_t internal_use_area_offset;
  uint8_t chassis_info_area_offset;
  uint8_t board_area_offset;
  uint8_t product_area_offset;
  uint8_t multirecord_area_offset;
  uint8_t pad;
  uint8_t checksum;
} FRU_area_common_header_t;

typedef struct {
  uint8_t version;
  uint8_t length;
  uint8_t language;
  uint8_t filler[8];
  uint8_t endoffields_flag;
} FRU_default_board_info_area_t;

typedef struct {
  uint8_t rectypeID;
  uint8_t eolflag_recversion;
  uint8_t reclength;
  uint8_t recxsum;
  uint8_t hdrxsum;
} multirecord_header_t;

typedef struct {
  multirecord_header_t hdr;
  uint8_t mfgID_LSB;
  uint8_t mfgID_MidB;
  uint8_t mfgID_MSB;
  uint8_t PICMG_recID;
  uint8_t record_fmt_version;
  uint8_t current_draw_100mA;
} module_current_req_record_t;

typedef struct {
	/* AMC Table 3-16 AdvancedMC Point-to-Point Connectivity record */
  uint8_t	record_type_id;	/* Record Type ID. For all records defined
					   in this specification a value of C0h (OEM)
					   shall be used. */
  uint8_t	version:4,
			reserved:3,
			eol:1;
  uint8_t	record_len;	/* Record Length. # of bytes following rec cksum */
  uint8_t	record_cksum;	/* Record Checksum. Holds the zero checksum of
					   the record. */
  uint8_t	header_cksum;	/* Header Checksum. Holds the zero checksum of
					   the header. */
  uint8_t	manuf_id[3];	/* Manufacturer ID. LS Byte first. Write as the
					   three byte ID assigned to PICMG�. For this
					   specification, the value 12634 (00315Ah) shall
					   be used. */
  uint8_t	picmg_rec_id;	/* PICMG Record ID. For the AMC Point-to-Point
					   Connectivity record, the value 19h must be used  */
  uint8_t	rec_fmt_ver;	/* Record Format Version. For this specification,
					   the value 0h shall be used. */
  uint8_t	oem_guid_count;	/* OEM GUID Count. The number, n, of OEM GUIDs
					   defined in this record. */
	                          //OEM_GUID oem_guid_list[n];
					/* A list 16*n bytes of OEM GUIDs. */

  uint8_t	conn_dev_id:4,
			:3,
		record_type:1;

  uint8_t	ch_descr_count;	/* AMC Channel Descriptor Count. The number, m,
			           of AMC Channel Descriptors defined in this record. */
                                // AMC_CHANNEL_DESCR ch_descr[m];
                                // AMC Channel Descriptors. A variable length
                                // list of m three-byte AMC Channel Descriptors,
                                // each defining the Ports that make up an AMC
                                // Channel (least significant byte first).
                                // AMC_LINK_DESCR link_desrc[p];
                                // AMC Link Descriptors. A variable length list
                                // of p five-byte AMC Link Descriptors (Least
                                // significant byte first) (see Table 3-19, �AMC
                                // Link Descriptor�, Table 3-20, �AMC Link Designator�,
                                // and Table 3-21, �AMC Link Type�) totaling 5 * p
                                // bytes in length. The value of p and the length
                                // of the list are implied by Record Length, since
                                // the list is at the end of this record.
                                // Each AMC Link Descriptor details one type of
                                // point-to-point protocol supported by the
                                // referenced Ports.

  uint8_t amc_ch_descr0[3];
  uint8_t amc_ch_descr1[3];
  uint8_t amc_ch_descr2[3];
  uint8_t amc_link_descr0[5];
  uint8_t amc_link_descr1[5];
  uint8_t amc_link_descr2[5];
  uint8_t amc_link_descr3[5];
  uint8_t amc_link_descr4[5];
  uint8_t amc_link_descr5[5];

} AMC_P2P_CONN_RECORD_t;

typedef enum{
  TCLKA = 1,
  TCLKB,
  TCLKC,
  TCLKD,
  FCLKA
}AMC_CLK_ID;

typedef struct{
  uint8_t localClkID;
  uint8_t remoteClkID;
  uint8_t remoteClkRes;
}P2P_CLK_CONN_DESCRIPTOR;

typedef struct {
  uint8_t clkResID;
  uint8_t clkP2PConnCnt;
  P2P_CLK_CONN_DESCRIPTOR clkConnDescr;
}CLOCK_P2P_DESCRIPTOR;

typedef struct {
  MULTIRECORD_AREA_HEADER rcrdHdr;
  uint8_t clkP2PDescrCnt;
  CLOCK_P2P_DESCRIPTOR clkDescr;
}CLK_P2P_CONN_RECORT_t;

//typedef struct{
//
//}INDIRECT_CLOCK_DESCRIPTOR_t;

typedef struct{
  uint8_t clockFeatures;
  uint8_t clockFamily;
  uint8_t clockAccuracy;
  uint8_t clockFreq[4];
  uint8_t clockFreqMin[4];
  uint8_t clockFreqMax[4];
}DIRECT_CLOCK_DESCRIPTOR_t;

typedef struct{
  uint8_t clockID;
  uint8_t clockCtrl;
  uint8_t indirClkDescCnt;
  uint8_t dirClkDescCnt;
  DIRECT_CLOCK_DESCRIPTOR_t dirDescr;
}CLOCK_CFG_DESCRIPTOR;

typedef struct{
  MULTIRECORD_AREA_HEADER rcrdHdr;
  uint8_t clkResID;
  uint8_t clkCfgDescCnt;
  CLOCK_CFG_DESCRIPTOR clkCfgDescr;
}CLK_CFG_REC_t;

#define NONVOLATILE_FORMAT_VERSION      (0x00)      // version of format header

// size definitions
// size/location definitions for hardware header information
#define HW_HEADER_BYTE_OFFSET				  (0)
#define HW_HEADER_SIZE						   (32)

// size/location definitions for Application Device ID info
#define APP_DEV_ID_BYTE_OFFSET				(HW_HEADER_BYTE_OFFSET+HW_HEADER_SIZE)
#define APP_DEV_ID_SIZE						    (16)

// size definitions for FRU data area
#define COMMON_HEADER_BYTE_OFFSET			(APP_DEV_ID_BYTE_OFFSET+APP_DEV_ID_SIZE)
#define COMMON_HEADER_SIZE					(8)
#define BOARD_INFO_AREA_BYTE_OFFSET			(COMMON_HEADER_BYTE_OFFSET+COMMON_HEADER_SIZE)
#define BOARD_INFO_AREA_SIZE				(64)
#define MULTIRECORD_AREA_BYTE_OFFSET		(BOARD_INFO_AREA_BYTE_OFFSET+BOARD_INFO_AREA_SIZE)
#define P2P_CONNECTIVITY_REC_SIZE	        (52)
//#define CLK_P2P_CONN_REC_OFFSET                 (MULTIRECORD_AREA_BYTE_OFFSET+P2P_CONNECTIVITY_REC_SIZE)
//#define CLK_P2P_CONN_REC_SIZE                   (16)
//#define CLK_CFG_REC_OFFSET                      (CLK_P2P_CONN_REC_OFFSET+CLK_P2P_CONN_REC_SIZE)
#define CLK_CFG_REC_OFFSET                      (MULTIRECORD_AREA_BYTE_OFFSET+P2P_CONNECTIVITY_REC_SIZE)
#define CLK_CFG_REC_SIZE                        (31)
#define CURRENT_REQ_OFFSET                      (CLK_CFG_REC_OFFSET+CLK_CFG_REC_SIZE)
//#define CURRENT_REQ_OFFSET                      (CLK_P2P_CONN_REC_OFFSET+CLK_P2P_CONN_REC_SIZE)
#define CURRENT_REQ_SIZE                        (11)
#define MULTIRECORD_AREA_SIZE                   (P2P_CONNECTIVITY_REC_SIZE + CLK_CFG_REC_SIZE + CURRENT_REQ_SIZE)
//+ CLK_P2P_CONN_REC_SIZE
#define END_OF_FRU_AREA_OFFSET			(MULTIRECORD_AREA_SIZE+MULTIRECORD_AREA_BYTE_OFFSET)

// FPGA config area
#define FPGA_CONFIG_AREA_BYTE_OFFSET		(END_OF_FRU_AREA_OFFSET)
#define FPGA_CONFIG_AREA_SIZE				(256)

#define EEPSIZE								2048

// SDR data table
#define SDR_AREA_BYTE_OFFSET				              (480)		// define to be on 32-byte boundary
#define SDR_AREA_SIZE						     (32*64)

//#if SDR_AREA_BYTE_OFFSET < FPGA_CONFIG_AREA_BYTE_OFFSET+FPGA_CONFIG_AREA_SIZE
//#error "SDR area overlaps FPGA config area in EEPROM address allocation"
//#endif

// Payload Manager default settings
#define PAYLDMGR_AREA_BYTE_OFFSET   (SDR_AREA_BYTE_OFFSET+SDR_AREA_SIZE)
#define PAYLDMGR_AREA_SIZE          (64)

// ADC Scaling Factors
#define ADC_SCALING_AREA_BYTE_OFFSET    (PAYLDMGR_AREA_BYTE_OFFSET+PAYLDMGR_AREA_SIZE)
#define ADC_SCALING_AREA_SIZE           (96)

// FRU Data area definitions
#define EEP_USED_AREA_SIZE					(ADC_SCALING_AREA_BYTE_OFFSET+ADC_SCALING_AREA_SIZE)

#define FRU_AREA_SIZE						(MULTIRECORD_AREA_SIZE+MULTIRECORD_AREA_BYTE_OFFSET-COMMON_HEADER_BYTE_OFFSET)
#define FRU_AREA_SIZE_L						(FRU_AREA_SIZE & 0xff)
#define FRU_AREA_SIZE_H						(FRU_AREA_SIZE >> 8)


extern volatile uint8_t fru_buf[END_OF_FRU_AREA_OFFSET];

void fru_init(void);

void read_fru_data(uint8_t* prbuf, uint8_t  saddr, uint8_t  len);

#endif /* NONVOLATILE_H_ */
