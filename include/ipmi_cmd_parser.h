
#ifndef IPMI_CMD_PARSER_H_
#define IPMI_CMD_PARSER_H_

// known netFn codes (even request codes only)
#define NETFN_CHASSIS						(0x00)
#define NETFN_BRIDGE						(0x02)
#define NETFN_SE							(0x04)		// sensor/event netFN
#define NETFN_APP							(0x06)		// application netFN
#define NETFN_FIRMWARE						(0x08)
#define NETFN_STORAGE						(0x0a)
#define NETFN_TRANSPORT						(0x0c)
#define NETFN_GRPEXT						(0x2c)		// group extension, for PICMG, first data byte = 0x00
#define NETFN_CUSTOM						(0x32)		// custom extension for UWHEP MMC functions

#define NETFN_PICMG_IDENTIFIER				(0x00)		// PICMG group extension, first byte of data field
#define NETFN_PICMG_MIN_RQ_LENGTH			(8)			// minimum practical length for a PICMG group command

// supported IPMI commands
// cmds in Application NetFn (06/07)
#define IPMICMD_APP_GET_DEVICE_ID							(0x01)
#define IPMICMD_APP_COLD_RESET								(0x02)

// cmds in Sensor/Event NetFn (04/05)
#define IPMICMD_SE_SET_EVENT_RECEIVER						(0x00)
#define IPMICMD_SE_GET_EVENT_RECEIVER						(0x01)
#define IPMICMD_SE_PLATFORM_EVENT							(0x02)
#define IPMICMD_SE_GET_DEVICE_SDR_INFO						(0x20)
#define IPMICMD_SE_GET_DEVICE_SDR							(0x21)
#define IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY				(0x22)
#define IPMICMD_SE_GET_SENSOR_HYSTERESIS        			(0x25)
#define IPMICMD_SE_SET_SENSOR_THRESHOLD						(0x26)
#define IPMICMD_SE_GET_SENSOR_THRESHOLD						(0x27)
#define IPMICMD_SE_SET_SENSOR_EVENT_ENABLE					(0x28)
#define IPMICMD_SE_GET_SENSOR_EVENT_ENABLE					(0x29)
#define IPMICMD_SE_GET_SENSOR_EVENT_STATUS					(0x2b)
#define IPMICMD_SE_GET_SENSOR_READING						(0x2d)

// cmds in Storage NetFn (0A/0B)
#define IPMICMD_STOR_GET_FRU_INVEN_AREA_INFO				(0x10)
#define IPMICMD_STOR_READ_FRU_DATA							(0x11)
#define IPMICMD_STOR_WRITE_FRU_DATA							(0x12)

// cmds in Group Extension (PICMG) NetFn (2C/2d)
#define IPMICMD_PICMG_GET_PICMG_PROP						(0x00)
#define IPMICMD_PICMG_SET_AMC_PORT_STATE					(0x19)
#define IPMICMD_PICMG_GET_AMC_PORT_STATE                                        (0x1a)
#define IPMICMD_PICMG_SET_AMC_CLOCK_STATE                                       (0x2c)
#define IPMICMD_PICMG_GET_AMC_CLOCK_STATE                                       (0x2d)
#define IPMICMD_PICMG_FRU_CONTROL						(0x04)
#define IPMICMD_PICMG_GET_FRU_LED_PROP						(0x05)
#define IPMICMD_PICMG_GET_LED_COLOR_CAP						(0x06)
#define IPMICMD_PICMG_SET_FRU_LED_STATE						(0x07)
#define IPMICMD_PICMG_GET_FRU_LED_STATE						(0x08)
#define IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID				(0x0D)
#define IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES				(0x1E)

// cmds in Custom NetFn (32/33)
#define IPMICMD_CUSTOM_SET_BACKEND_PWR                (0x01)
#define IPMICMD_CUSTOM_GET_BACKEND_PWR                (0x02)
#define IPMICMD_CUSTOM_SET_PYLD_MGR_SETTING_REC       (0x03)
#define IPMICMD_CUSTOM_GET_PYLD_MGR_SETTING_REC       (0x04)
#define IPMICMD_CUSTOM_GET_FAULT_STATUS               (0x05)
#define IPMICMD_CUSTOM_SET_BOOT_MODE                  (0x06)
#define IPMICMD_CUSTOM_GET_BOOT_MODE                  (0x07)
#define IPMICMD_CUSTOM_SET_SENSOR_ALARM_MASK          (0x0B)
#define IPMICMD_CUSTOM_GET_SENSOR_ALARM_MASK          (0x0C)
#define IPMICMD_CUSTOM_SET_HANDLE_OVERRIDE            (0x0F)
#define IPMICMD_CUSTOM_SET_CURRENT_REQUIREMENT        (0x10)
#define IPMICMD_CUSTOM_SET_ANALOG_SCALE_FACTOR        (0x13)
#define IPMICMD_CUSTOM_GET_ANALOG_SCALE_FACTOR        (0x14)
#define IPMICMD_CUSTOM_GET_TIMESTATS                  (0x28)
#define IPMICMD_CUSTOM_SET_SYSTIME                    (0x29)
#define IPMICMD_CUSTOM_GET_SYSTIME                    (0x2A)
#define IPMICMD_CUSTOM_POLL_FPGA_CFG_PORT             (0x30)
#define IPMICMD_CUSTOM_FPGA_CFG_RDSR                  (0x31)
#define IPMICMD_CUSTOM_FPGA_CFG_WRCTL                 (0x32)
#define IPMICMD_CUSTOM_FPGA_CFG_WRITE                 (0x33)
#define IPMICMD_CUSTOM_FPGA_CFG_READ                  (0x34)
#define IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_WRITE      (0x35)
#define IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_READ       (0x36)
#define IPMICMD_CUSTOM_GET_NONVOLATILE_AREA_INFO      (0x40)
#define IPMICMD_CUSTOM_RAW_NONVOLATILE_WRITE          (0x41)
#define IPMICMD_CUSTOM_RAW_NONVOLATILE_READ           (0x42)
#define IPMICMD_CUSTOM_CHK_EEPROM_BUSY                (0x43)
#define IPMICMD_CUSTOM_EEPROM_ERASE                   (0x44)
#define NONVOLATILE_MAX_XFER_LEN                      (20)


// response codes
#define IPMI_RS_NORMAL_COMP_CODE		     (0x00)			// normal completion code
#define IPMI_RS_NODE_BUSY				     (0xC0)			// command processing resources (like eeprom) temp. unavail.
#define IPMI_RS_INVALID_CMD				     (0xC1)			// unknown or invalid command
#define IPMI_RS_INVALID_RSV_ID			     (0xC5)			// reservation cancelled or invalid reservation ID
#define IPMI_RS_REQ_DATA_TRUNCATED           (0xC6)      // request data truncated
#define IPMI_RS_INVALID_DATA_LENGTH		     (0xC7)			// data length invalid
#define IPMI_RS_INVALID_RECORD_CODE		     (0xCB)			// invalid sensor, data or record
#define IPMI_RS_INVALID_FIELD_IN_REQ	     (0xCC)			// invalid data field in request
#define IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE     (0xCD)      // command illegal for specified sensor or function type
#define IPMI_RS_INSUFFICIENT_PRIVILEGE       (0xD4)      // cannot execute command due to insufficient privilege or other restriction
#define IPMI_RS_ILLEGAL_PARAMETER            (0xD5)      // illegal parameter for unsupported subfunction



void ipmi_cmd_parser_init(void);

#endif /* IPMI_CMD_PARSER_H_ */
