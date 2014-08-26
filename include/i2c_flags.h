/*
 * i2c_flags.h
 *
 *  Created on: 04-04-2013
 *      Author: Bartek
 */

#ifndef I2C_FLAGS_H_
#define I2C_FLAGS_H_

/*==============================================================*/
/* I2C control flags						*/
/*==============================================================*/

#define I2C_CTRL_FL_AA		0x04	/* Assert acknowledge flag. */
#define I2C_CTRL_FL_SI		0x08	/* I2C serial interrupt flag. */
#define I2C_CTRL_FL_STO 	0x10	/* STOP flag. */
#define I2C_CTRL_FL_STA 	0x20	/* START flag.*/
#define	I2C_CTRL_FL_I2EN 	0x40	/* I2C interface enable. */

/*==============================================================*/
/* I2CSTAT Codes						*/
/*==============================================================*/

/* Slave standby */
#define	I2STAT_NADDR_SLAVE_MODE			0xFF
#define I2STAT_START_MASTER			0xFE

/* Master transmitter/receiver mode common */
#define	I2STAT_START_SENT			0x08
#define	I2STAT_REP_START_SENT			0x10
#define	I2STAT_ARBITRATION_LOST			0x38

/* Master transmitter mode */
#define	I2STAT_SLAW_SENT_ACKED			0x18
#define	I2STAT_SLAW_SENT_NOT_ACKED		0x20
#define	I2STAT_MASTER_DATA_SENT_ACKED		0x28
#define	I2STAT_MASTER_DATA_SENT_NOT_ACKED	0x30

/* Master receiver mode */
#define	I2STAT_SLAR_SENT_ACKED			0x40
#define	I2STAT_SLAR_SENT_NOT_ACKED		0x48
#define	I2STAT_MASTER_DATA_RCVD_ACKED		0x50
#define	I2STAT_MASTER_DATA_RCVD_NOT_ACKED	0x58

/* Slave receiver mode*/
#define	I2STAT_SLAW_RCVD_ACKED			0x60	/* Own SLA+W has been received;
							   ACK has been returned. */
#define	I2STAT_ARB_LOST_SLAW_RCVD_ACKED		0x68
#define	I2STAT_GENERAL_CALL_RCVD_ACKED		0x70
#define	I2STAT_ARB_LOST_GENERAL_CALL_RCVD_ACKED	0x78
#define	I2STAT_SLAVE_DATA_RCVD_ACKED		0x80
#define	I2STAT_SLAVE_DATA_RCVD_NOT_ACKED	0x88	/* Previously addressed with own SLV
							   address; DATA has been received;
							   ACK has been returned. */
#define	I2STAT_GENERAL_CALL_DATA_RCVD_ACKED	0x90
#define	I2STAT_GENERAL_CALL_DATA_RCVD_NOT_ACKED	0x98
#define	I2STAT_STOP_START_RCVD			0xA0

/* Slave transmitter mode */
#define	I2STAT_SLAR_RCVD_ACKED			0xA8
#define	I2STAT_ARB_LOST_SLAR_RCVD_ACKED		0xB0
#define	I2STAT_SLAVE_DATA_SENT_ACKED		0xB8
#define	I2STAT_SLAVE_DATA_SENT_NOT_ACKED	0xC0
#define	I2STAT_LAST_BYTE_SENT_ACKED		0xC8

#define	I2STAT_NO_INFO				0xF8
#define	I2STAT_BUS_ERROR			0x00

/* Channel utilization policies */
#define CH_POLICY_0_ONLY	0x0
#define CH_POLICY_1_ONLY	0x1
#define CH_POLICY_ALL		0x2

/* Channel health */
#define I2C_CH_STATE_DISABLED		0x0
#define I2C_CH_STATE_ENABLED_FUNCTIONAL	0x1
#define I2C_CH_STATE_ENABLED_DEGRADED	0x2
#define I2C_CH_STATE_ENABLED_DEAD	0x3

/* Error codes */
#define I2ERR_NOERR			0x0
#define I2ERR_STATE_TRANSITION		0x1
#define I2ERR_ARBITRATION_LOST		0x2
#define	I2ERR_SLARW_SENT_NOT_ACKED	0x3
#define I2ERR_NAK_RCVD			0x4
#define I2ERR_TIMEOUT			0x5
#define I2ERR_BUFFER_OVERFLOW		0x6


#endif /* I2C_FLAGS_H_ */
