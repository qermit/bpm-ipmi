
#ifndef PAYLOAD_MGR_H_
#define PAYLOAD_MGR_H_

#include "cmcpins.h"
#include "gpio.h"
#include "sensor_sdr.h"

#define get_backend_power_pin         (gpio_get_pin_value( PGOOD_PIN, PGOOD_PORT ))

#define FRU_CTLCODE_COLD_RST          (0)       // FRU Control command cold reset code
#define FRU_CTLCODE_WARM_RST          (1)       // FRU Control command warm reset code
#define FRU_CTLCODE_REBOOT            (2)       // FRU Control command reboot code
#define FRU_CTLCODE_QUIESCE           (4)       // FRU Control command quiesce code


#define BACKEND_PWR_OFF               (0)
#define BACKEND_PWR_ON                (1)
#define TRIGGER_BOOT_MODE             (0)
#define MAINT_BOOT_MODE               (1)
#define FPGA_CPU_RESET_ASSERT         (0)
#define FPGA_CPU_RESET_DEASSERT       (1)

#define COLD_RESET_TICKS              (30)                  // 100ms ticks for cold reset
#define WARM_RESET_TICKS              (10)                  // 100ms ticks for reset pulse
#define FPGA_LOAD_TICKS               (10)                  // 100ms ticks for FPGA load command
#define QUIESCE_DELAY_TICKS           (10)                  // 100ms ticks for quiesce command->delay
#define MONITOR_STARTUP_TICKS         (10)                  // 1 second delay for non-recoverable value monitoring
#define AUTOCONFIG_POLL_TICKS         (30)                  // 3 second delay for checking autoconfig port

#define PAYLDMGR_PWRON_ISR_DETECT               (1<<0)
#define PAYLDMGR_PWROFF_ISR_DETECT              (1<<1)
#define PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT     (1<<2)
#define PAYLDMGR_AUTOCONFIG_TIMER_FLAG          (1<<3)


typedef enum {status_OK=0, noncritical, critical, fault} alarm_level_t;
typedef enum {none=0, temperature, voltage, backend_device} alarm_source_class_t;

typedef struct {
  uint8_t cur_on_state;
  uint8_t prev_on_state;
  uint8_t upper_thr;                  // upper hysteresis threshold
  uint8_t lower_thr;                  // lower hysteresis threshold
} voltage_comparator_rec_t;


typedef struct {
  uint8_t default_bkend_auto_pwr_ena;       // default back end auto power-on enable value
  uint8_t default_boot_mode;                // default boot mode (0=trigger, 1=maintenance)
  uint8_t fpga_auto_cfg_inhibit;            // fpga auto-config disable value;
  uint8_t bkend_pwr_shtdn_evt_ena;          // setting for sending hotswap event when backend power is shut down
  uint8_t global_mask_level;                // global alarm mask level
  uint8_t reserved[3];
  uint8_t sensor_mask_level[MAX_SENSOR_CNT];
} payload_mgr_eeprom_rec_t;


typedef struct {
  alarm_level_t cur_alarm_level;            // current alarm level
  alarm_source_class_t cur_alarm_source;    // current alarm source
} backend_monitor_rec_t;


typedef struct {
  voltage_comparator_rec_t payload_pwr;     // comparator record for +12V payload power
  payload_mgr_eeprom_rec_t nonvol_settings; // settings which are sticky in EEPROM
  backend_monitor_rec_t bkend_monitor;      // state variables for backend monitor
  uint8_t bkend_pwr_ena;                    // enable for backend power
  uint8_t cold_reset_timer;          // countdown timer used during cold resets
  uint8_t quiesce_timer;             // countdown timer used during quiesce operation
  uint8_t quiesced_flag;                    // flag set when module is quiesced, used to signal "Quiesced" hotswap event
  uint8_t payload_startup_timer;     // timer for holding off monitoring during startup of payload power
  uint8_t backend_startup_timer;     // timer for holding off monitoring during startup of backend power
  uint8_t warm_reset_timer;          // timer for warm reset
  uint8_t fpga_load_timer;           // timer for fpga load strobe
  uint8_t autoconfig_poll_timer;     // timer for polling for autoconfig operation
  uint32_t mmc_uptime;                 // up time for mmc in seconds
  uint32_t bkend_hottime;              // hot time for backend, in seconds
} payload_mgr_state_t;

typedef enum {power_off=0, power_on} power_status_t;

extern volatile payload_mgr_state_t pyldmgr_state;

void pyldmgr_init(void);

void pyldmgr_service(void);

void pyldmgr_ipmicmd_fru_ctrl(const uint8_t ctlcode);

void pyldmgr_ipmicmd_backend_pwr(const ipmb_msg_desc_t* prqmsg);

power_status_t pyldmgr_get_payload_power_status(void);

power_status_t pyldmgr_get_backend_power_status(void);

#endif /* PAYLOAD_MGR_H_ */
