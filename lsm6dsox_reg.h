/**
  ******************************************************************************
  * @file    lsm6dsox_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lsm6dsox_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LSM6DSOX_REGS_H
#define LSM6DSOX_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LSM6DSOX
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t); // a pointer to the function;  Function pointers are used to store the address of a function, allowing it to be invoked indirectly
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg; //  store the address of a function responsible for writing data.
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay; // used to store the address of a function responsible for delaying execution for a specified duration.
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LSM6DSOX_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define LSM6DSOX_I2C_ADD_L                    0xD5U
#define LSM6DSOX_I2C_ADD_H                    0xD7U

/** Device Identification (Who am I) **/
#define LSM6DSOX_ID                           0x6CU // why 0x6C specifically? there're others as well - to avoid replicating?

/**
  * @}
  *
  */

#define LSM6DSOX_FUNC_CFG_ACCESS              0x01U // u: unsigned number
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_ctrl_from_ui         : 1; // Enable the full control from the Primary Interface of OIS configurations.
  uint8_t not_used_01              : 5;
  uint8_t reg_access               : 2; /* shub_reg_access + func_cfg_access */ // shub: enable access to the sensor hub (I2C master) registers; func: enable access to embedded functions configuration registers
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
uint8_t reg_access               :
  2; /* shub_reg_access + func_cfg_access */
  uint8_t not_used_01              : 5;
  uint8_t ois_ctrl_from_ui         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_func_cfg_access_t;

#define LSM6DSOX_PIN_CTRL                     0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t sdo_pu_en                : 1; // Enable pull-up on SDO pin; 0: SDO pin pull-up disconnected (default); 1: SDO pin with pull-up
  uint8_t ois_pu_dis               : 1; // Disable pull-up on both OCS_Aux and SDO_Aux pins.0: OCS_Aux and SDO_Aux pins with pull-up; 1: OCS_Aux and SDO_Aux pins pull-up disconnected
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ois_pu_dis               : 1;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_pin_ctrl_t;

#define LSM6DSOX_S4S_TPH_L                    0x04U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tph_l                    : 7;
  uint8_t tph_h_sel                : 1; //  Chooses if the TPH formula must be taken into account
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tph_h_sel                : 1;
  uint8_t tph_l                    : 7;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_s4s_tph_l_t;

#define LSM6DSOX_S4S_TPH_H                    0x05U
typedef struct
{
  uint8_t tph_h                    : 8;
} lsm6dsox_s4s_tph_h_t;

#define LSM6DSOX_S4S_RR                       0x06U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rr                       : 2; // Sensor synchronization resolution ratio register; 10: S4S, DT resolution 213
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t rr                       : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_s4s_rr_t;

#define LSM6DSOX_FIFO_CTRL1                   0x07U
typedef struct
{
  uint8_t wtm                      : 8; // What does in conjunction mean?
} lsm6dsox_fifo_ctrl1_t;

#define LSM6DSOX_FIFO_CTRL2                   0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm                      : 1;
  uint8_t uncoptr_rate             : 2; // Configures the compression algorithm to write non-compressed data at each rate. 2: Non-compressed data every 16 batch data rate
  uint8_t not_used_01              : 1;
  uint8_t odrchg_en                : 1; // Enables ODR CHANGE virtual sensor to be batched in FIFO
  uint8_t not_used_02              : 1;
  uint8_t fifo_compr_rt_en         : 1; // Enables compression algorithm runtime
  uint8_t stop_on_wtm              : 1; //Sensing chain FIFO stop values memorization at threshold level; 0: NOT LIMITED; 1: Limited to the threshold level
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_wtm              : 1;
  uint8_t fifo_compr_rt_en         : 1;
  uint8_t not_used_02              : 1;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_01              : 1;
  uint8_t uncoptr_rate             : 2;
  uint8_t wtm                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fifo_ctrl2_t;

#define LSM6DSOX_FIFO_CTRL3                   0x09U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdr_xl                   : 4; // 26 Hz
  uint8_t bdr_gy                   : 4; // 26 Hz
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bdr_gy                   : 4;
  uint8_t bdr_xl                   : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fifo_ctrl3_t;

#define LSM6DSOX_FIFO_CTRL4                   0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode                : 3; //  Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then FIFO mode;
  uint8_t not_used_01              : 1;
  uint8_t odr_t_batch              : 2; // Batching data rate (writing frequency in FIFO) for temperature data: 12.5 Hz
  uint8_t odr_ts_batch             : 2; // Decimation for timestamp batching in FIFO:  Decimation 8: max(BDR_XL[Hz],BDR_GY[Hz])/8 [Hz];
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_ts_batch             : 2;
  uint8_t odr_t_batch              : 2;
  uint8_t not_used_01              : 1;
  uint8_t fifo_mode                : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fifo_ctrl4_t;

#define LSM6DSOX_COUNTER_BDR_REG1             0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t cnt_bdr_th               : 3; // sets the threshold for the internal counter of batching events. When this counter reaches the threshold, the counter is reset and the COUNTER_BDR_IA flag in FIFO_STATUS2 (3Bh) is set to ‘1’.
  uint8_t not_used_01              : 2;
  uint8_t trig_counter_bdr         : 1; // Gyro; Selects the trigger for the internal counter of batching events between XL and gyro.
  uint8_t rst_counter_bdr          : 1; // 1: This bit is automatically reset to zero; Resets the internal counter of batching events for a single sensor.
  uint8_t dataready_pulsed         : 1; // Data-ready pulsed mode (the data ready pulses are 75 µs long)
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dataready_pulsed         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t trig_counter_bdr         : 1;
  uint8_t not_used_01              : 2;
  uint8_t cnt_bdr_th               : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_counter_bdr_reg1_t;

#define LSM6DSOX_COUNTER_BDR_REG2             0x0CU
typedef struct
{
  uint8_t cnt_bdr_th               : 8;
} lsm6dsox_counter_bdr_reg2_t;

#define LSM6DSOX_INT1_CTRL                    0x0D
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl             : 1; //  Enables accelerometer data-ready interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3CSM interface is used.
  uint8_t int1_drdy_g              : 1; // Enables gyroscope data-ready interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3CSM interface is used.
  uint8_t int1_boot                : 1; // Enables boot status on INT1 pin
  uint8_t int1_fifo_th             : 1; // Enables FIFO threshold interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3CSM interface is used.
  uint8_t int1_fifo_ovr            : 1; // Enables FIFO overrun interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3CSM interface is used.
  uint8_t int1_fifo_full           : 1; // Enables FIFO full flag interrupt on INT1 pin. It can be also used to trigger an IBI when the MIPI I3CSM interface is used.
  uint8_t int1_cnt_bdr             : 1; // Enables COUNTER_BDR_IA interrupt on INT1
  uint8_t den_drdy_flag            : 1; //  Sends DEN_DRDY (DEN stamped on Sensor Data flag) to INT1 pin
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy_flag            : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_int1_ctrl_t;

#define LSM6DSOX_INT2_CTRL                    0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_int2_ctrl_t;

#define LSM6DSOX_WHO_AM_I                     0x0FU
#define LSM6DSOX_CTRL1_XL                     0x10U // Accelerometer control register 1 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN // from the right to the left (little_endian)
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1; // Output from LPF2 second filtering stage selected
  uint8_t fs_xl                    : 2; // Accelerometer full-scale selection: 4g
  uint8_t odr_xl                   : 4; // Accelerometer ODR (output data rate) selection
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN // from left to right (big_endian)
  uint8_t odr_xl                   : 4;
  uint8_t fs_xl                    : 2;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl1_xl_t;

#define LSM6DSOX_CTRL2_G                      0x11U // Gyroscope control register 2 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */ // Gyroscope UI chain full-scale selection: 2000 dps
  uint8_t odr_g                    : 4; //  Gyroscope ODR configuration setting: 104 Hz
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g                    : 4;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl2_g_t;

#define LSM6DSOX_CTRL3_C                      0x12U // Control register 3 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1; // Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI).
  uint8_t sim                      : 1; // SPI Serial Interface Mode selection.0: 4-wire interface; 1: 3-wire interface
  uint8_t pp_od                    : 1; // Push-pull/open-drain selection on INT1 and INT2 pins. 0: push-pull mode; 1: open-drain mode
  uint8_t h_lactive                : 1; // 0: interrupt output pins active high; 1: interrupt output pins active low
  uint8_t bdu                      : 1; // Block data update: 0: continuous update; 1: output registers are not updated until MSB and LSB have been read
  uint8_t boot                     : 1; // Reboots memory content - 0: normal mode; 1: reboot memory content
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t h_lactive                : 1;
  uint8_t pp_od                    : 1;
  uint8_t sim                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t not_used_01              : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl3_c_t;

#define LSM6DSOX_CTRL4_C                      0x13U // Control register 4 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1; // Enables gyroscope digital LPF1 if auxiliary SPI is disabled; 0: disabled; 1: enabled
  uint8_t i2c_disable              : 1; // Disables I2C interface. 0: SPI, I2C and MIPI I3CSM interfaces enabled (default); 1: I²C interface disabled
  uint8_t drdy_mask                : 1; // Enables data available (0: disabled; 1: mask DRDY on pin (both XL & Gyro) until filter settling ends (XL and Gyro independently masked
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1; // All interrupt signals available on INT1 pin enable; 0: interrupt signals divided between INT1 and INT2 pins; 1: all interrupt signals in logic or on INT1 pin
  uint8_t sleep_g                  : 1; // Enables gyroscope Sleep mode; 1: Enabled
  uint8_t not_used_03              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t not_used_02              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t i2c_disable              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl4_c_t;

#define LSM6DSOX_CTRL5_C                      0x14U // Control register 5 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 2; // Linear acceleration sensor self-test enable; 00: Self-test disabled; 10: Negative sign self-test
  uint8_t st_g                     : 2; // Angular rate sensor self-test enable: 10: Not allowed
  uint8_t rounding_status          : 1; // Source register rounding function; 0: Rounding disabled; 1: Rounding enabled
  uint8_t rounding                 : 2; // Circular burst-mode (rounding) read from the output registers; 00: no rounding; 01: accelerometer only; 10: gyroscope only; 11: gyroscope + accelerometer
  uint8_t xl_ulp_en                : 1; // Accelerometer ultra-low-power mode enable. 1: enabled
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t xl_ulp_en                : 1;
  uint8_t rounding                 : 2;
  uint8_t rounding_status          : 1;
  uint8_t st_g                     : 2;
  uint8_t st_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl5_c_t;

#define LSM6DSOX_CTRL6_C                      0x15U // Control register 6 (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ftype                    : 3; // Gyroscope low-pass filter (LPF1) bandwidth selection; 011: Level-sensitive latched mode is selected
  uint8_t usr_off_w                : 1; // Weight of XL user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h), Z_OFS_USR (75h); 1:  = 2-6 g/LSB
  uint8_t xl_hm_mode               : 1; // High-performance operating mode disable for accelerometer.0: enabled; 1: disabled;
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */ // TRIGGER mode
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */
  uint8_t xl_hm_mode               : 1;
  uint8_t usr_off_w                : 1;
  uint8_t ftype                    : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl6_c_t;

#define LSM6DSOX_CTRL7_G                      0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_on                   : 1; //Enables/disables the OIS chain from primary interface when the OIS_ON_EN bit is '1'. 1: enabled
  uint8_t usr_off_on_out           : 1; // Enables accelerometer user offset correction block; it's valid for the low-pass path; 1: accelerometer user offset correction block enabled
  uint8_t ois_on_en                : 1; // Selects how to enable and disable the OIS chain, after first configuration and enabling through SPI2. 1: OIS chain is enabled/disabled with primary interface
  uint8_t not_used_01              : 1;
  uint8_t hpm_g                    : 2; // Gyroscope digital HP filter cutoff selection. 10: 260 mHz;
  uint8_t hp_en_g                  : 1; // Enables gyroscope digital high-pass filter. The filter is enabled only if the gyro is in HP mode. 1: HPF enabled
  uint8_t g_hm_mode                : 1; // Disables high-performance operating mode for gyroscope; 0: enabled; 1: disabled
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t g_hm_mode                : 1;
  uint8_t hp_en_g                  : 1;
  uint8_t hpm_g                    : 2;
  uint8_t not_used_01              : 1;
  uint8_t ois_on_en                : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t ois_on                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl7_g_t;

#define LSM6DSOX_CTRL8_XL                     0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t low_pass_on_6d           : 1; // LPF2 on 6D function selection. 0: ODR/2 low-pass filtered data sent to 6D interrupt function; 1: LPF2 output data sent to 6D interrupt function
  uint8_t xl_fs_mode               : 1; // Accelerometer full-scale management between UI chain and OIS chain; 1: New full-scale mode. Full scales are independent between the UI/OIS chain but both bound to ±8 g
  uint8_t hp_slope_xl_en           : 1; // Accelerometer slope filter / high-pass filter selection.
  uint8_t fastsettl_mode_xl        : 1; // Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the second samples after writing this bit. Active only during device exit from power- down mode.
  uint8_t hp_ref_mode_xl           : 1; // Enables accelerometer high-pass filter reference mode (valid for high-pass path - HP_SLOPE_XL_EN bit must be ‘1’).
  uint8_t hpcf_xl                  : 3; // Accelerometer LPF2 and HP filter configuration and cutoff setting. 011: ODR/45
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpcf_xl                  : 3;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t xl_fs_mode               : 1;
  uint8_t low_pass_on_6d           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl8_xl_t;

#define LSM6DSOX_CTRL9_XL                     0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t i3c_disable              : 1; // Disables MIPI I3CSM communication protocol; 0: SPI, I2C, MIPI I3CSM interfaces enabled; 1: MIPI I3CSM interface disabled SPI?
  uint8_t den_lh                   : 1; // DEN active level configuration. 0: active low; 1: active high
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */ // DEN stamping sensor selection (0: DEN pin info stamped in the gyro axis selected by bits [7:5]; 1: acce) + Extends DEN functionality to accelerometer sensor (0: disabled; 1: enabled).
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1; // Data enable value stored in LSB of X-axis; 0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_x                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_z                    : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_lh                   : 1;
  uint8_t i3c_disable              : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl9_xl_t;

#define LSM6DSOX_CTRL10_C                     0x19U // (r/w) for all CTRL registers
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t timestamp_en             : 1; // Enables timestamp counter; The counter is readable in TIMESTAMP0 (40h), TIMESTAMP1 (41h), TIMESTAMP2 (42h), and TIMESTAMP3 (43h).
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ctrl10_c_t;

#define LSM6DSOX_ALL_INT_SRC                  0x1AU // source register for all interrupts (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                    : 1; // Free-fall event status. 1: event detected (default: 0)
  uint8_t wu_ia                    : 1; // Wake-up event status.
  uint8_t single_tap               : 1; // Single-tap event status.
  uint8_t double_tap               : 1;
  uint8_t d6d_ia                   : 1; // Interrupt active for change in position of portrait, landscape, face-up, face-down. 0: change in position not detected; 1: change in position detected
  uint8_t sleep_change_ia          : 1; // Detects change event in activity/inactivity status. 0: change status not detected; 1: change status detected
  uint8_t not_used_01              : 1;
  uint8_t timestamp_endcount       : 1; // Alerts timestamp overflow within 6.4 ms
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount       : 1;
  uint8_t not_used_01              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t wu_ia                    : 1;
  uint8_t ff_ia                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_all_int_src_t;

#define LSM6DSOX_WAKE_UP_SRC                  0x1BU // Wake-up interrupt source register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                     : 1; // Wake-up event detection status on Z-axis. 0: wakeup event on Z-axis not detected; 1: wakeup event on Z-axis detected (Default: 0)
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1; // Wakeup event detection status.
  uint8_t sleep_state              : 1; // Sleep status bit. 0: Activity status; 1: Inactivity status
  uint8_t ff_ia                    : 1; // Free-fall event detection status. 0: free-fall event not detected; 1: free-fall event detected
  uint8_t sleep_change_ia          : 2; // Detects change event in activity/inactivity status. 0: change status not detected; 1: change status detected
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sleep_change_ia          : 2;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t wu_ia                    : 1;
  uint8_t x_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t z_wu                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_wake_up_src_t;

#define LSM6DSOX_TAP_SRC                      0x1CU //Tap source register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap                    : 1; // Tap event detection status on Z-axis. 0: tap event on Z-axis not detected; 1: tap event on Z-axis detected
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1; // Sign of acceleration detected by tap event. 0: positive sign of acceleration detected by tap event; 1: negative sign of acceleration detected by tap event
  uint8_t double_tap               : 1; // Double-tap event detection status.0: double-tap event not detected; 1: double-tap event detected.
  uint8_t single_tap               : 1; // Single-tap event status. 0: single tap event not detected; 1: single tap event detected
  uint8_t tap_ia                   : 1; // Tap event detection status; 0: tap event not detected; 1: tap event detected
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t tap_ia                   : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t tap_sign                 : 1;
  uint8_t x_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t z_tap                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_tap_src_t;

#define LSM6DSOX_D6D_SRC                      0x1DU // Portrait, landscape, face-up and face-down source register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                       : 1; // X-axis low event (under threshold).
  uint8_t xh                       : 1; // X-axis high event (over threshold). (0: event not detected; 1: event (over threshold) detected
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1; // Interrupt active for change position portrait, landscape, face-up, face-down.
  uint8_t den_drdy                 : 1; // DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active condition
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy                 : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t zh                       : 1;
  uint8_t zl                       : 1;
  uint8_t yh                       : 1;
  uint8_t yl                       : 1;
  uint8_t xh                       : 1;
  uint8_t xl                       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_d6d_src_t;

#define LSM6DSOX_STATUS_REG                   0x1EU // The STATUS_REG register is read by the primary interface SPI/I²C & MIPI I3CSM (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1; // Accelerometer new data available.
  uint8_t gda                      : 1; // Gyroscope new data available
  uint8_t tda                      : 1; // Temperature new data available. 0: no set of data is available at temperature sensor output; 1: a new set of data is available at temperature sensor output
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t tda                      : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_status_reg_t;

#define LSM6DSOX_OUT_TEMP_L                   0x20U // Temperature data output register (r). L and H registers together express a 16-bit word in two’s complement
#define LSM6DSOX_OUT_TEMP_H                   0x21U
#define LSM6DSOX_OUTX_L_G                     0x22U // Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement. Data are according to the full scale and ODR settings (CTRL2_G (11h)) of gyro user interface
#define LSM6DSOX_OUTX_H_G                     0x23U
#define LSM6DSOX_OUTY_L_G                     0x24U
#define LSM6DSOX_OUTY_H_G                     0x25U
#define LSM6DSOX_OUTZ_L_G                     0x26U
#define LSM6DSOX_OUTZ_H_G                     0x27U
#define LSM6DSOX_OUTX_L_A                     0x28U // Linear acceleration sensor X-axis output register (r). The value is expressed as a 16-bit word in two’s complement. Data are according to the full-scale and ODR settings (CTRL1_XL (10h)) of the accelerometer user interface.
#define LSM6DSOX_OUTX_H_A                     0x29U
#define LSM6DSOX_OUTY_L_A                     0x2AU
#define LSM6DSOX_OUTY_H_A                     0x2BU
#define LSM6DSOX_OUTZ_L_A                     0x2CU
#define LSM6DSOX_OUTZ_H_A                     0x2DU
#define LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE     0x35U // Embedded function status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01             : 3;
  uint8_t is_step_det             : 1; // Interrupt status bit for step detection; 1: interrupt detected; 0: no interrupt
  uint8_t is_tilt                 : 1; // Interrupt status bit for tilt detection
  uint8_t is_sigmot               : 1; // Interrupt status bit for significant motion detection
  uint8_t not_used_02             : 1;
  uint8_t is_fsm_lc               : 1; // Interrupt status bit for FSM long counter timeout interrupt event.
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc               : 1;
  uint8_t not_used_02             : 1;
  uint8_t is_sigmot               : 1;
  uint8_t is_tilt                 : 1;
  uint8_t is_step_det             : 1;
  uint8_t not_used_01             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_status_mainpage_t;

#define LSM6DSOX_FSM_STATUS_A_MAINPAGE        0x36U // Finite State Machine status register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1                 : 1; // Interrupt status bit for FSM1 interrupt event. 1: interrupt detected; 0: no interrupt
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm8                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm1                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_status_a_mainpage_t;

#define LSM6DSOX_FSM_STATUS_B_MAINPAGE        0x37U // Finite State Machine status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm9                 : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm16                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm16                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm9                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_status_b_mainpage_t;

#define LSM6DSOX_MLC_STATUS_MAINPAGE     0x38U // Machine Learning Core status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_mlc1             : 1; // Interrupt status bit for MLC1 interrupt event. 1: interrupt detected; 0: no interrupt
  uint8_t is_mlc2             : 1;
  uint8_t is_mlc3             : 1;
  uint8_t is_mlc4             : 1;
  uint8_t is_mlc5             : 1;
  uint8_t is_mlc6             : 1;
  uint8_t is_mlc7             : 1;
  uint8_t is_mlc8             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_mlc8             : 1;
  uint8_t is_mlc7             : 1;
  uint8_t is_mlc6             : 1;
  uint8_t is_mlc5             : 1;
  uint8_t is_mlc4             : 1;
  uint8_t is_mlc3             : 1;
  uint8_t is_mlc2             : 1;
  uint8_t is_mlc1             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mlc_status_mainpage_t;

#define LSM6DSOX_STATUS_MASTER_MAINPAGE       0x39U // Sensor hub source register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop          : 1; // Sensor hub communication status. 0: sensor hub communication not concluded; 1: sensor hub communication concluded
  uint8_t not_used_01             : 2;
  uint8_t slave0_nack             : 1; // This bit is set to 1 if Not acknowledge occurs on slave 0 communication. Default value: 0
  uint8_t slave1_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave3_nack             : 1;
  uint8_t wr_once_done            : 1; // When the bit WRITE_ONCE in MASTER_CONFIG (14h) is configured as 1, this bit is set to 1 when the write operation on slave 0 has been performed and completed. Default value: 0
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done            : 1;
  uint8_t slave3_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave0_nack             : 1;
  uint8_t not_used_01             : 2;
  uint8_t sens_hub_endop          : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_status_master_mainpage_t;

#define LSM6DSOX_FIFO_STATUS1                 0x3AU // FIFO status register 1 (r)
typedef struct
{
  uint8_t diff_fifo                : 8; // Number of unread sensor data (TAG + 6 bytes) stored in FIFO; In conjunction with DIFF_FIFO[9:8] in FIFO_STATUS2 (3Bh).
} lsm6dsox_fifo_status1_t;

#define LSM6DSOX_FIFO_STATUS2                 0x3B
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                : 2; // Number of unread sensor data (TAG + 6 bytes) stored in FIFO. Default value: 00 In conjunction with DIFF_FIFO[7:0] in FIFO_STATUS1 (3Ah
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1; // On datasheet: FIFO_OVR_LATCHED
  uint8_t counter_bdr_ia           : 1; // Counter BDR reaches the CNT_BDR_TH_[10:0] threshold set in COUNTER_BDR_REG1 (0Bh) and COUNTER_BDR_REG2 (0Ch). Default value: 0. This bit is reset when these registers are read.
  uint8_t fifo_full_ia             : 1; // Smart FIFO full status. Default value: 0; 0: FIFO is not full; 1: FIFO will be full at the next ODR
  uint8_t fifo_ovr_ia              : 1; // FIFO overrun status. Default value: 0; 0: FIFO is not completely filled; 1: FIFO is completely filled
  uint8_t fifo_wtm_ia              : 1; // FIFO watermark status. Default value: 0 (0: FIFO filling is lower than WTM; 1: FIFO filling is equal to or greater than WTM
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia              : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t over_run_latched         : 1;
  uint8_t not_used_01              : 1;
  uint8_t diff_fifo                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fifo_status2_t;

#define LSM6DSOX_TIMESTAMP0                   0x40U // Timestamp first data output register (r). The value is expressed as a 32-bit word and the bit resolution is 25 µs
#define LSM6DSOX_TIMESTAMP1                   0x41U
#define LSM6DSOX_TIMESTAMP2                   0x42U
#define LSM6DSOX_TIMESTAMP3                   0x43U
#define LSM6DSOX_UI_STATUS_REG_OIS            0x49U // OIS status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1; // Accelerometer data available (reset when one of the high parts of the output data is read)
  uint8_t gda                      : 1; // Gyroscope data available (reset when one of the high parts of the output data is read)
  uint8_t gyro_settling            : 1; // High when the gyroscope output is in the settling phase
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t gyro_settling            : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ui_status_reg_ois_t;

#define LSM6DSOX_UI_OUTX_L_G_OIS              0x4AU // Angular rate sensor pitch axis (X) angular rate output register (r). The value is expressed as a 16-bit word in two’s complement. Data are according to the gyroscope full scale and ODR (6.66 kHz) settings of the OIS gyro.
#define LSM6DSOX_UI_OUTX_H_G_OIS              0x4BU
#define LSM6DSOX_UI_OUTY_L_G_OIS              0x4CU
#define LSM6DSOX_UI_OUTY_H_G_OIS              0x4DU
#define LSM6DSOX_UI_OUTZ_L_G_OIS              0x4EU
#define LSM6DSOX_UI_OUTZ_H_G_OIS              0x4FU
#define LSM6DSOX_UI_OUTX_L_A_OIS              0x50U // Linear acceleration sensor X-axis output register (r). The value is expressed as a 16-bit word in two’s complement. Data are according to the accelerometer full scale and ODR (6.66 kHz) settings of the OIS accelerometer.
#define LSM6DSOX_UI_OUTX_H_A_OIS              0x51U
#define LSM6DSOX_UI_OUTY_L_A_OIS              0x52U
#define LSM6DSOX_UI_OUTY_H_A_OIS              0x53U
#define LSM6DSOX_UI_OUTZ_L_A_OIS              0x54U
#define LSM6DSOX_UI_OUTZ_H_A_OIS              0x55U

#define LSM6DSOX_TAP_CFG0                     0x56U // Activity/inactivity functions, configuration of filtering, and tap recognition functions (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                      : 1; // Latched Interrupt. Default value: 0 0: interrupt request not latched; 1: interrupt request latched
  uint8_t tap_z_en                 : 1; // Enable Z direction in tap recognition. Default value: 0 0: Z direction disabled; 1: Z direction enabled
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1; // HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions. Default value: 0 (0: SLOPE filter applied; 1: HPF applied)
  uint8_t sleep_status_on_int      : 1; // Activity/inactivity interrupt mode configuration. If INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits are enabled, drives the sleep status or sleep change on the INT pins. Default value: 0; 0: sleep change notification on INT pins; 1: sleep status reported on INT pins
  uint8_t int_clr_on_read          : 1; // This bit allows immediately clearing the latched interrupts of an event detection upon the read of the corresponding status register. It must be set to 1 together with LIR. Default value: 0. 0: latched interrupt signal cleared at the end of the ODR period; 1: latched interrupt signal immediately cleared
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t slope_fds                : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t lir                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_tap_cfg0_t;

#define LSM6DSOX_TAP_CFG1                     0x57U // Tap configuration register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_x                : 5; // X-axis tap recognition threshold. Default value: 0; 1 LSB = FS_XL / (25) 00101
  uint8_t tap_priority             : 3; // Selection of axis priority for TAP detection; 011: Z Y X
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tap_priority             : 3;
  uint8_t tap_ths_x                : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_tap_cfg1_t;

#define LSM6DSOX_TAP_CFG2                     0x58U // Enables interrupt and inactivity functions, and tap recognition functions (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_y                : 5; // Y-axis tap recognition threshold. Default value: 0 00101
  uint8_t inact_en                 : 2; // Enable activity/inactivity (sleep) function. Default value: 00 10: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro to sleep mode;
  uint8_t interrupts_enable        : 1; // Enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity). Default value: 0; 0: interrupt disabled; 1: interrupt enabled
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable        : 1;
  uint8_t inact_en                 : 2;
  uint8_t tap_ths_y                : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_tap_cfg2_t;

#define LSM6DSOX_TAP_THS_6D                   0x59U // Portrait/landscape position and tap function threshold register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_z                : 5; // Z-axis recognition threshold. Default value: 0
  uint8_t sixd_ths                 : 2; // Threshold for 4D/6D function. Default value: 0 60 degree
  uint8_t d4d_en                   : 1; // 4D orientation detection enable. Z-axis position detection is disabled (0: enabled; 1: disabled)
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t d4d_en                   : 1;
  uint8_t sixd_ths                 : 2;
  uint8_t tap_ths_z                : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_tap_ths_6d_t;

#define LSM6DSOX_INT_DUR2                     0x5AU // Tap recognition function setting register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock                    : 2; // Maximum duration of overthreshold event. Default value: 00. The default value of these bits is 00b which corresponds to 4*ODR_XL time. If the SHOCK[1:0] bits are set to a different value, 1LSB corresponds to 8*ODR_XL time.
  uint8_t quiet                    : 2; // Expected quiet time after a tap detection. Default value: 00 . The default value of these bits is 00b which corresponds to 2*ODR_XL time. If the QUIET[1:0] bits are set to a different value, 1LSB corresponds to 4*ODR_XL time
  uint8_t dur                      : 4; // Duration of maximum time gap for double tap recognition. Default: 0000; The default value of these bits is 0000b which corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a different value, 1LSB corresponds to 32*ODR_XL time ?
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dur                      : 4;
  uint8_t quiet                    : 2;
  uint8_t shock                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_int_dur2_t;

#define LSM6DSOX_WAKE_UP_THS                  0x5BU // Single/double-tap selection and wake-up configuration (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                   : 6; // Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in WAKE_UP_DUR (5Ch).
  uint8_t usr_off_on_wu            : 1; // Drives the low-pass filtered data with user offset correction (instead of high-pass filtered data) to the wakeup function.
  uint8_t single_double_tap        : 1; // Single/double-tap event enable. Default value: 0; 0: only single-tap event enabled; both single and double-tap events enabled
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap        : 1;
  uint8_t usr_off_on_wu            : 1;
  uint8_t wk_ths                   : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_wake_up_ths_t;

#define LSM6DSOX_WAKE_UP_DUR                  0x5CU // Free-fall, wakeup and sleep mode functions duration setting register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                : 4; // Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR) 1 LSB = 512 ODR (0100)
  uint8_t wake_ths_w               : 1; // Weight of 1 LSB of wakeup threshold. Default: 0; (0: 1 LSB = FS_XL / (2^6) 1: 1 LSB = FS_XL / (2^8) )
  uint8_t wake_dur                 : 2; // Wake up duration event. Default: 00. 1LSB = 1 ODR_time
  uint8_t ff_dur                   : 1; // Free fall duration event. Default: 0 For the complete configuration of the free-fall duration, refer to FF_DUR[4:0] in the FREE_FALL (5Dh) configuration. 1 LSB = 1 ODR_time ? FF_DUR5
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 1;
  uint8_t wake_dur                 : 2;
  uint8_t wake_ths_w               : 1;
  uint8_t sleep_dur                : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_wake_up_dur_t;

#define LSM6DSOX_FREE_FALL                    0x5DU // Free-fall function duration setting register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                   : 3; // Free fall threshold setting. Default: 000  011: 312 mg
  uint8_t ff_dur                   : 5; // Free-fall duration event. Default: 0 refer to FF_DUR5 in the WAKE_UP_DUR (5Ch) 00000
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                   : 5;
  uint8_t ff_ths                   : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_free_fall_t;

#define LSM6DSOX_MD1_CFG                      0x5EU // Functions routing on INT1 register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_shub                : 1; // Routing of sensor hub communication concluded event on INT1. Default value: 0 (0: routing of sensor hub communication concluded event on INT1 disabled; 1: routing of sensor hub communication concluded event on INT1 enabled)
  uint8_t int1_emb_func            : 1; // Routing of embedded functions event on INT1. Default value: 0
  uint8_t int1_6d                  : 1; // Routing of 6D event on INT1.
  uint8_t int1_double_tap          : 1; // Routing of tap event on INT1. Default value: 0
  uint8_t int1_ff                  : 1; // Routing of free-fall event on INT1. Default value: 0
  uint8_t int1_wu                  : 1; // Routing of wakeup event on INT1. Default value: 0
  uint8_t int1_single_tap          : 1; // Routing of single-tap recognition event on INT1. Default: 0
  uint8_t int1_sleep_change        : 1; // Routing of activity/inactivity recognition event on INT1. Default: 0
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_sleep_change        : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_emb_func            : 1;
  uint8_t int1_shub                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_md1_cfg_t;

#define LSM6DSOX_MD2_CFG                      0x5FU // Functions routing on INT2 register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_timestamp           : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_sleep_change        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_change        : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t int2_timestamp           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_md2_cfg_t;

#define LSM6DSOX_S4S_ST_CMD_CODE              0x60U // S4S Master command register (r/w)
typedef struct
{
  uint8_t s4s_st_cmd_code          : 8; // 00001000
} lsm6dsox_s4s_st_cmd_code_t;

#define LSM6DSOX_S4S_DT_REG                   0x61U // S4S DT register (r/w)
typedef struct
{
  uint8_t dt                       : 8;
} lsm6dsox_s4s_dt_reg_t;

#define LSM6DSOX_I3C_BUS_AVB                  0x62U // I3C_BUS_AVB register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pd_dis_int1              : 1; // This bit allows disabling the INT1 pull-down. 0: Pull-down on INT1 enabled (pull-down is effectively connected only when no interrupts are routed to the INT1 pin or when I3C dynamic address is assigned. 1: Pull-down on INT1 disabled (pull-down not connected
  uint8_t not_used_01              : 2;
  uint8_t i3c_bus_avb_sel          : 2; // These bits are used to select the bus available time when I3C IBI is used. 10:  bus available time equal to 1 msec
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t i3c_bus_avb_sel          : 2;
  uint8_t not_used_01              : 2;
  uint8_t pd_dis_int1              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_i3c_bus_avb_t;

#define LSM6DSOX_INTERNAL_FREQ_FINE           0x63U // Internal frequency register (r)
typedef struct
{
  uint8_t freq_fine                : 8;
} lsm6dsox_internal_freq_fine_t;

#define LSM6DSOX_UI_INT_OIS                   0x6F // OIS interrupt configuration register and accelerometer self-test enable setting.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t spi2_read_en             : 1; // In Primary IF full control mode, enables Auxiliary SPI for reading OIS data in registers. 1: enabled
  uint8_t not_used_02              : 1;
  uint8_t den_lh_ois               : 1; // Indicates polarity of DEN signal on OIS chain. 0: DEN pin is active-low; 1: DEN pin is active-high
  uint8_t lvl2_ois                 : 1; // Enables level-sensitive latched mode on the OIS chain. Default value: 0
  uint8_t int2_drdy_ois            : 1; // Enables OIS chain DRDY on INT2 pin. This setting has priority over all other INT2 settings.
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois            : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t den_lh_ois               : 1;
  uint8_t not_used_02              : 1;
  uint8_t spi2_read_en             : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ui_int_ois_t;

#define LSM6DSOX_UI_CTRL1_OIS                 0x70U // OIS configuration register. The primary interface can write this register when the OIS_CTRL_FROM_UI bit in the FUNC_CFG_ACCESS (01h) register is equal to 1 (Primary IF Full control mode); this register is read-only when the OIS_CTRL_FROM_UI bit is equal to 0 (SPI2 Full control mode) and shows the content of the SPI2_CTRL1_OIS (70h) register.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_en_spi2              : 1; // Enables OIS chain data processing for gyro in Mode 3 and Mode 4 (mode4_en = 1) and accelerometer data in and Mode 4 (mode4_en = 1).
  uint8_t fs_g_ois                 : 3; /* fs_125_ois + fs[1:0]_g_ois */ //fs_125_ois: Selects gyroscope OIS chain full-scale 125 dps (0: 0: FS selected through bits FS[1:0]_OIS_G; 1: 125 dps); fs_g_ois: Selects gyroscope OIS chain full-scale 011: 2000 dps)
  uint8_t mode4_en                 : 1; // Enables accelerometer OIS chain. OIS_EN_SPI2 must be enabled (i.e. set to ‘1’) to enable also the XL OIS chain.
  uint8_t sim_ois                  : 1;  // SPI2 3- or 4-wire interface. Default value: 0. 0: 4-wire; 1: 3-wire
  uint8_t lvl1_ois                 : 1; // Enables level-sensitive trigger mode on OIS chain. Default value: 0
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lvl1_ois                 : 1;
  uint8_t sim_ois                  : 1;
  uint8_t mode4_en                 : 1;
  uint8_t fs_g_ois                 : 3; /* fs_125_ois + fs[1:0]_g_ois */
  uint8_t ois_en_spi2              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ui_ctrl1_ois_t;

#define LSM6DSOX_UI_CTRL2_OIS                 0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hp_en_ois                : 1; // Enables gyroscope OIS chain digital high-pass filter.
  uint8_t ftype_ois                : 2; // Selects gyroscope digital LPF1 filter bandwidth. 10: cutoff@171.1 Hz, phase@20 Hz = -11.18
  uint8_t not_used_01              : 1;
  uint8_t hpm_ois                  : 2; // Selects gyroscope OIS chain digital high-pass filter cutoff. Default value: 00. 10: 260 mHz
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t hpm_ois                  : 2;
  uint8_t not_used_01              : 1;
  uint8_t ftype_ois                : 2;
  uint8_t hp_en_ois                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ui_ctrl2_ois_t;

#define LSM6DSOX_UI_CTRL3_OIS                 0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_ois_clampdis          : 1;
  uint8_t not_used_01              : 2;
  uint8_t filter_xl_conf_ois       : 3;
  uint8_t fs_xl_ois                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fs_xl_ois                : 2; // Selects accelerometer OIS channel full-scale. 10: 4g
  uint8_t filter_xl_conf_ois       : 3; // Selects accelerometer OIS channel bandwidth. 011: bandwidth: 65.1 Hz; overall phase: -21.5@20 Hz
  uint8_t not_used_01              : 2;
  uint8_t st_ois_clampdis          : 1; // Disables OIS chain clamp. 0: All OIS chain outputs = 8000h during self-test; 1: OIS chain self-test outputs as shown in
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_ui_ctrl3_ois_t;

#define LSM6DSOX_X_OFS_USR                    0x73U // Accelerometer X-axis user offset correction (r/w). The offset value set in the X_OFS_USR offset register is internally subtracted from the acceleration value measured on the X-axis.
#define LSM6DSOX_Y_OFS_USR                    0x74U
#define LSM6DSOX_Z_OFS_USR                    0x75U
#define LSM6DSOX_FIFO_DATA_OUT_TAG            0x78U // FIFO tag register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tag_parity               : 1; // Parity check of TAG content
  uint8_t tag_cnt                  : 2; // 2-bit counter which identifies sensor time slo
  uint8_t tag_sensor               : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tag_sensor               : 5;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_parity               : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fifo_data_out_tag_t;

#define LSM6DSOX_FIFO_DATA_OUT_X_L            0x79 // FIFO data output X (r)
#define LSM6DSOX_FIFO_DATA_OUT_X_H            0x7A
#define LSM6DSOX_FIFO_DATA_OUT_Y_L            0x7B
#define LSM6DSOX_FIFO_DATA_OUT_Y_H            0x7C
#define LSM6DSOX_FIFO_DATA_OUT_Z_L            0x7D
#define LSM6DSOX_FIFO_DATA_OUT_Z_H            0x7E

#define LSM6DSOX_SPI2_WHO_AM_I                0x0F
#define LSM6DSOX_SPI2_STATUS_REG_OIS          0x1E
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t gyro_settling            : 1;
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t gyro_settling            : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_spi2_status_reg_ois_t;

#define LSM6DSOX_SPI2_OUT_TEMP_L              0x20
#define LSM6DSOX_SPI2_OUT_TEMP_H              0x21
#define LSM6DSOX_SPI2_OUTX_L_G_OIS            0x22
#define LSM6DSOX_SPI2_OUTX_H_G_OIS            0x23
#define LSM6DSOX_SPI2_OUTY_L_G_OIS            0x24
#define LSM6DSOX_SPI2_OUTY_H_G_OIS            0x25
#define LSM6DSOX_SPI2_OUTZ_L_G_OIS            0x26
#define LSM6DSOX_SPI2_OUTZ_H_G_OIS            0x27
#define LSM6DSOX_SPI2_OUTX_L_A_OIS            0x28
#define LSM6DSOX_SPI2_OUTX_H_A_OIS            0x29
#define LSM6DSOX_SPI2_OUTY_L_A_OIS            0x2A
#define LSM6DSOX_SPI2_OUTY_H_A_OIS            0x2B
#define LSM6DSOX_SPI2_OUTZ_L_A_OIS            0x2C
#define LSM6DSOX_SPI2_OUTZ_H_A_OIS            0x2D
#define LSM6DSOX_SPI2_INT_OIS                 0x6F
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl_ois                : 2;
  uint8_t not_used_01              : 3;
  uint8_t den_lh_ois               : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t int2_drdy_ois            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois            : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t den_lh_ois               : 1;
  uint8_t not_used_01              : 3;
  uint8_t st_xl_ois                : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_spi2_int_ois_t;

#define LSM6DSOX_SPI2_CTRL1_OIS               0x70U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_en_spi2              : 1;
  uint8_t fs_g_ois                 : 3; /* fs_125_ois + fs[1:0]_g_ois */
  uint8_t mode4_en                 : 1;
  uint8_t sim_ois                  : 1;
  uint8_t lvl1_ois                 : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lvl1_ois                 : 1;
  uint8_t sim_ois                  : 1;
  uint8_t mode4_en                 : 1;
  uint8_t fs_g_ois                 : 3; /* fs_125_ois + fs[1:0]_g_ois */
  uint8_t ois_en_spi2              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_spi2_ctrl1_ois_t;

#define LSM6DSOX_SPI2_CTRL2_OIS               0x71U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hp_en_ois                : 1;
  uint8_t ftype_ois                : 2;
  uint8_t not_used_01              : 1;
  uint8_t hpm_ois                  : 2;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t hpm_ois                  : 2;
  uint8_t not_used_01              : 1;
  uint8_t ftype_ois                : 2;
  uint8_t hp_en_ois                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_spi2_ctrl2_ois_t;

#define LSM6DSOX_SPI2_CTRL3_OIS               0x72U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_ois_clampdis          : 1;
  uint8_t st_ois                   : 2;
  uint8_t filter_xl_conf_ois       : 3;
  uint8_t fs_xl_ois                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fs_xl_ois                : 2;
  uint8_t filter_xl_conf_ois       : 3;
  uint8_t st_ois                   : 2;
  uint8_t st_ois_clampdis          : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_spi2_ctrl3_ois_t;

#define LSM6DSOX_PAGE_SEL                     0x02U // Enable the full control from the Primary Interface of OIS configurations.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t page_sel                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel                 : 4;
  uint8_t not_used_01              : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_page_sel_t;

#define LSM6DSOX_EMB_FUNC_EN_A                0x04U // Embedded functions enable register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t pedo_en                  : 1; // Enable pedometer algorithm. Default value: 0
  uint8_t tilt_en                  : 1; // Enable tilt calculation. Default value: 0
  uint8_t sign_motion_en           : 1; // Enable significant motion detection function. Default value: 0
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t sign_motion_en           : 1;
  uint8_t tilt_en                  : 1;
  uint8_t pedo_en                  : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_en_a_t;

#define LSM6DSOX_EMB_FUNC_EN_B                0x05U // Embedded functions enable register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_en                   : 1; // Enable Finite State Machine (FSM) feature. Default value: 0
  uint8_t not_used_01              : 2;
  uint8_t fifo_compr_en            : 1; // Enables FIFO_COMPR_RT_EN bit in FIFO_CTRL2
  uint8_t mlc_en                   : 1; // Enable Machine Learning Core feature. Default value: 0
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t mlc_en                   : 1;
  uint8_t fifo_compr_en            : 1;
  uint8_t not_used_01              : 2;
  uint8_t fsm_en                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_en_b_t;

#define LSM6DSOX_PAGE_ADDRESS                 0x08U
typedef struct
{
  uint8_t page_addr                : 8;
} lsm6dsox_page_address_t;

#define LSM6DSOX_PAGE_VALUE                   0x09U
typedef struct
{
  uint8_t page_value               : 8;
} lsm6dsox_page_value_t;

#define LSM6DSOX_EMB_FUNC_INT1                0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t int1_step_detector       : 1; // Routing of pedometer step recognition event on INT1. Default value: 0
  uint8_t int1_tilt                : 1; // Routing of tilt event on INT1. Default value: 0
  uint8_t int1_sig_mot             : 1; // Routing of significant motion event on INT1. Default value: 0
  uint8_t not_used_02              : 1;
  uint8_t int1_fsm_lc              : 1; // Routing of FSM long counter timeout interrupt event on INT1. Default value: 0
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm_lc              : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_sig_mot             : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_step_detector       : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_int1_t;

#define LSM6DSOX_FSM_INT1_A                   0x0BU // INT1 pin control register (r/w). Each bit in this register enables a signal to be carried through INT1. The pin's output will supply the OR combination of the selected signals
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_fsm1                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm8                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm1                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_int1_a_t;

#define LSM6DSOX_FSM_INT1_B                   0x0CU // INT1 pin control register (r/w). Each bit in this register enables a signal to be carried through INT1. The pin's output will supply the OR combination of the selected signals.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_fsm16               : 1;
  uint8_t int1_fsm15               : 1;
  uint8_t int1_fsm14               : 1;
  uint8_t int1_fsm13               : 1;
  uint8_t int1_fsm12               : 1;
  uint8_t int1_fsm11               : 1;
  uint8_t int1_fsm10               : 1;
  uint8_t int1_fsm9                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_int1_b_t;

#define LSM6DSOX_MLC_INT1                     0x0DU // Each bit in this register enables a signal to be carried through INT1. The pin's output will supply the OR combination of the selected signal
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_mlc1                : 1;
  uint8_t int1_mlc2                : 1;
  uint8_t int1_mlc3                : 1;
  uint8_t int1_mlc4                : 1;
  uint8_t int1_mlc5                : 1;
  uint8_t int1_mlc6                : 1;
  uint8_t int1_mlc7                : 1;
  uint8_t int1_mlc8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_mlc8                : 1;
  uint8_t int1_mlc7                : 1;
  uint8_t int1_mlc6                : 1;
  uint8_t int1_mlc5                : 1;
  uint8_t int1_mlc4                : 1;
  uint8_t int1_mlc3                : 1;
  uint8_t int1_mlc2                : 1;
  uint8_t int1_mlc1                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mlc_int1_t;

#define LSM6DSOX_EMB_FUNC_INT2                0x0EU // INT2 pin control register (r/w). Each bit in this register enables a signal to be carried through INT2. The pin's output will supply the OR combination of the selected signals.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t int2_step_detector       : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_sig_mot             : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_fsm_lc              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm_lc              : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_sig_mot             : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_step_detector       : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_int2_t;

#define LSM6DSOX_FSM_INT2_A                   0x0FU // INT2 pin control register (r/w). Each bit in this register enables a signal to be carried through INT2. The pin's output will supply the OR combination of the selected signals.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_fsm1                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm8                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm1                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_int2_a_t;

#define LSM6DSOX_FSM_INT2_B                   0x10U // INT2 pin control register (r/w). Each bit in this register enables a signal to be carried through INT2. The pin's output will supply the OR combination of the selected signals
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_fsm9                : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm16               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm16               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm9                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_int2_b_t;

#define LSM6DSOX_MLC_INT2                     0x11U // INT2 pin control register (r/w). Each bit in this register enables a signal to be carried through INT2. The pin's output will supply the OR combination of the selected signals.
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_mlc1             : 1;
  uint8_t int2_mlc2             : 1;
  uint8_t int2_mlc3             : 1;
  uint8_t int2_mlc4             : 1;
  uint8_t int2_mlc5             : 1;
  uint8_t int2_mlc6             : 1;
  uint8_t int2_mlc7             : 1;
  uint8_t int2_mlc8             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_mlc8             : 1;
  uint8_t int2_mlc7             : 1;
  uint8_t int2_mlc6             : 1;
  uint8_t int2_mlc5             : 1;
  uint8_t int2_mlc4             : 1;
  uint8_t int2_mlc3             : 1;
  uint8_t int2_mlc2             : 1;
  uint8_t int2_mlc1             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mlc_int2_t;

#define LSM6DSOX_EMB_FUNC_STATUS              0x12U // Embedded function status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t is_step_det              : 1; // Interrupt status bit for step detection
  uint8_t is_tilt                  : 1; // Interrupt status bit for tilt detection
  uint8_t is_sigmot                : 1; // Interrupt status bit for significant motion detection
  uint8_t not_used_02              : 1;
  uint8_t is_fsm_lc                : 1; // Interrupt status bit for FSM long counter timeout interrupt event. 1: interrupt detected
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc                : 1;
  uint8_t not_used_02              : 1;
  uint8_t is_sigmot                : 1;
  uint8_t is_tilt                  : 1;
  uint8_t is_step_det              : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_status_t;

#define LSM6DSOX_FSM_STATUS_A                 0x13U // Finite State Machine status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1                  : 1; // Interrupt status bit for FSM1 interrupt event. 1: interrupt detected
  uint8_t is_fsm2                  : 1;
  uint8_t is_fsm3                  : 1;
  uint8_t is_fsm4                  : 1;
  uint8_t is_fsm5                  : 1;
  uint8_t is_fsm6                  : 1;
  uint8_t is_fsm7                  : 1;
  uint8_t is_fsm8                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8                  : 1;
  uint8_t is_fsm7                  : 1;
  uint8_t is_fsm6                  : 1;
  uint8_t is_fsm5                  : 1;
  uint8_t is_fsm4                  : 1;
  uint8_t is_fsm3                  : 1;
  uint8_t is_fsm2                  : 1;
  uint8_t is_fsm1                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_status_a_t;

#define LSM6DSOX_FSM_STATUS_B                 0x14U // Finite State Machine status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm9                  : 1; // Interrupt status bit for FSM9 interrupt event.
  uint8_t is_fsm10                 : 1;
  uint8_t is_fsm11                 : 1;
  uint8_t is_fsm12                 : 1;
  uint8_t is_fsm13                 : 1;
  uint8_t is_fsm14                 : 1;
  uint8_t is_fsm15                 : 1;
  uint8_t is_fsm16                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm16                 : 1;
  uint8_t is_fsm15                 : 1;
  uint8_t is_fsm14                 : 1;
  uint8_t is_fsm13                 : 1;
  uint8_t is_fsm12                 : 1;
  uint8_t is_fsm11                 : 1;
  uint8_t is_fsm10                 : 1;
  uint8_t is_fsm9                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_status_b_t;

#define LSM6DSOX_MLC_STATUS                   0x15U // Machine Learning Core status register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_mlc1            : 1; // Interrupt status bit for MLC1 interrupt event
  uint8_t is_mlc2            : 1;
  uint8_t is_mlc3            : 1;
  uint8_t is_mlc4            : 1;
  uint8_t is_mlc5            : 1;
  uint8_t is_mlc6            : 1;
  uint8_t is_mlc7            : 1;
  uint8_t is_mlc8            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_mlc8            : 1;
  uint8_t is_mlc7            : 1;
  uint8_t is_mlc6            : 1;
  uint8_t is_mlc5            : 1;
  uint8_t is_mlc4            : 1;
  uint8_t is_mlc3            : 1;
  uint8_t is_mlc2            : 1;
  uint8_t is_mlc1            : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mlc_status_t;

#define LSM6DSOX_PAGE_RW                      0x17U // Enable read and write mode of advanced features dedicated page (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t page_rw                  : 2;  /* page_write + page_read */ // Enable reads/writes from the selected advanced features dedicated page.
  uint8_t emb_func_lir             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_lir             : 1;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_page_rw_t;

#define LSM6DSOX_EMB_FUNC_FIFO_CFG             0x44U // Embedded functions batching configuration register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_00              : 6;
  uint8_t pedo_fifo_en             : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t pedo_fifo_en             : 1;
  uint8_t not_used_00              : 6;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_fifo_cfg_t;

#define LSM6DSOX_FSM_ENABLE_A                 0x46U // FSM enable register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm1_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm8_en                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm8_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm1_en                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_enable_a_t;

#define LSM6DSOX_FSM_ENABLE_B                 0x47U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm9_en                  : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm16_en                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm16_en                 : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm9_en                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_enable_b_t;

#define LSM6DSOX_FSM_LONG_COUNTER_L           0x48U // FSM long counter status register (r/w).
#define LSM6DSOX_FSM_LONG_COUNTER_H           0x49U
#define LSM6DSOX_FSM_LONG_COUNTER_CLEAR       0x4AU // FSM long counter reset register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_lc_clr               : 2;  /* fsm_lc_cleared + fsm_lc_clear */ // cleared: This read-only bit is automatically set to 1 when the long counter reset is done. Default value: 0. clear: Clear FSM long counter value. Default value: 0
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t fsm_lc_clr               : 2;  /* fsm_lc_cleared + fsm_lc_clear */
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_long_counter_clear_t;

#define LSM6DSOX_FSM_OUTS1                    0x4CU // FSM1 output register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1; // FSM1 output: negative event detected on the vector. 0: not detected
  uint8_t p_v                      : 1; // FSM1 output: positive event detected on the vector.
  uint8_t n_z                      : 1; // FSM1 output: negative event detected on the Z-axis
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs1_t;

#define LSM6DSOX_FSM_OUTS2                    0x4DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs2_t;

#define LSM6DSOX_FSM_OUTS3                    0x4EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs3_t;

#define LSM6DSOX_FSM_OUTS4                    0x4FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs4_t;

#define LSM6DSOX_FSM_OUTS5                    0x50U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs5_t;

#define LSM6DSOX_FSM_OUTS6                    0x51U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs6_t;

#define LSM6DSOX_FSM_OUTS7                    0x52U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs7_t;

#define LSM6DSOX_FSM_OUTS8                    0x53U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs8_t;

#define LSM6DSOX_FSM_OUTS9                    0x54U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs9_t;

#define LSM6DSOX_FSM_OUTS10                   0x55U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs10_t;

#define LSM6DSOX_FSM_OUTS11                   0x56U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs11_t;

#define LSM6DSOX_FSM_OUTS12                   0x57U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs12_t;

#define LSM6DSOX_FSM_OUTS13                   0x58U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs13_t;

#define LSM6DSOX_FSM_OUTS14                   0x59U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs14_t;

#define LSM6DSOX_FSM_OUTS15                   0x5AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs15_t;

#define LSM6DSOX_FSM_OUTS16                   0x5BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_fsm_outs16_t;

#define LSM6DSOX_EMB_FUNC_ODR_CFG_B           0x5FU // Finite State Machine output data rate (ODR) configuration register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_odr_cfg_b_t;

#define LSM6DSOX_EMB_FUNC_ODR_CFG_C           0x60U // Machine Learning Core output data rate configuration register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01             : 4;
  uint8_t mlc_odr                 : 2;
  uint8_t not_used_02             : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02             : 2;
  uint8_t mlc_odr                 : 2;
  uint8_t not_used_01             : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_odr_cfg_c_t;

#define LSM6DSOX_STEP_COUNTER_L               0x62U // Step counter output register (r)
#define LSM6DSOX_STEP_COUNTER_H               0x63U
#define LSM6DSOX_EMB_FUNC_SRC                 0x64U // Embedded function source register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t stepcounter_bit_set      : 1;
  uint8_t step_overflow            : 1;
  uint8_t step_count_delta_ia      : 1;
  uint8_t step_detected            : 1;
  uint8_t not_used_02              : 1;
  uint8_t pedo_rst_step            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pedo_rst_step            : 1;
  uint8_t not_used_02              : 1;
  uint8_t step_detected            : 1;
  uint8_t step_count_delta_ia      : 1;
  uint8_t step_overflow            : 1;
  uint8_t stepcounter_bit_set      : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_src_t;

#define LSM6DSOX_EMB_FUNC_INIT_A              0x66U // Embedded functions initialization register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01               : 3;
  uint8_t step_det_init             : 1;
  uint8_t tilt_init                 : 1;
  uint8_t sig_mot_init              : 1;
  uint8_t not_used_02               : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02               : 2;
  uint8_t sig_mot_init              : 1;
  uint8_t tilt_init                 : 1;
  uint8_t step_det_init             : 1;
  uint8_t not_used_01               : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_init_a_t;

#define LSM6DSOX_EMB_FUNC_INIT_B              0x67U // Embedded functions initialization register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_init                 : 1;
  uint8_t not_used_01              : 2;
  uint8_t fifo_compr_init          : 1;
  uint8_t mlc_init                 : 1;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t mlc_init                 : 1;
  uint8_t fifo_compr_init          : 1;
  uint8_t not_used_01              : 2;
  uint8_t fsm_init                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_emb_func_init_b_t;

#define LSM6DSOX_MLC0_SRC                     0x70U // Machine Learning Core source register (r)
#define LSM6DSOX_MLC1_SRC                     0x71U
#define LSM6DSOX_MLC2_SRC                     0x72U
#define LSM6DSOX_MLC3_SRC                     0x73U
#define LSM6DSOX_MLC4_SRC                     0x74U
#define LSM6DSOX_MLC5_SRC                     0x75U
#define LSM6DSOX_MLC6_SRC                     0x76U
#define LSM6DSOX_MLC7_SRC                     0x77U
#define LSM6DSOX_MAG_SENSITIVITY_L            0xBAU // External magnetometer sensitivity value register for the Finite State Machine (r/w)
#define LSM6DSOX_MAG_SENSITIVITY_H            0xBBU
#define LSM6DSOX_MAG_OFFX_L                   0xC0U // Offset for X-axis hard-iron compensation register (r/w).
#define LSM6DSOX_MAG_OFFX_H                   0xC1U
#define LSM6DSOX_MAG_OFFY_L                   0xC2U
#define LSM6DSOX_MAG_OFFY_H                   0xC3U
#define LSM6DSOX_MAG_OFFZ_L                   0xC4U
#define LSM6DSOX_MAG_OFFZ_H                   0xC5U
#define LSM6DSOX_MAG_SI_XX_L                  0xC6U // Soft-iron (3x3 symmetric) matrix correction register (r/w).
#define LSM6DSOX_MAG_SI_XX_H                  0xC7U
#define LSM6DSOX_MAG_SI_XY_L                  0xC8U
#define LSM6DSOX_MAG_SI_XY_H                  0xC9U
#define LSM6DSOX_MAG_SI_XZ_L                  0xCAU
#define LSM6DSOX_MAG_SI_XZ_H                  0xCBU
#define LSM6DSOX_MAG_SI_YY_L                  0xCCU
#define LSM6DSOX_MAG_SI_YY_H                  0xCDU
#define LSM6DSOX_MAG_SI_YZ_L                  0xCEU
#define LSM6DSOX_MAG_SI_YZ_H                  0xCFU
#define LSM6DSOX_MAG_SI_ZZ_L                  0xD0U
#define LSM6DSOX_MAG_SI_ZZ_H                  0xD1U
#define LSM6DSOX_MAG_CFG_A                    0xD4U // External magnetometer coordinates (Z and Y axes) rotation register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t mag_z_axis               : 3; // Magnetometer Z-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation)
  uint8_t not_used_01              : 1;
  uint8_t mag_y_axis               : 3; // Magnetometer Y-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation)
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t mag_y_axis               : 3;
  uint8_t not_used_01              : 1;
  uint8_t mag_z_axis               : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mag_cfg_a_t;

#define LSM6DSOX_MAG_CFG_B                    0xD5U // External magnetometer coordinates (X-axis) rotation register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t mag_x_axis               : 3; // Magnetometer X-axis coordinates rotation (to be aligned to accelerometer/gyroscope axes orientation)
  uint8_t not_used_01              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t mag_x_axis               : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_mag_cfg_b_t;

#define LSM6DSOX_FSM_LC_TIMEOUT_L             0x17AU // FSM long counter timeout register (r/w). The long counter timeout value is an unsigned integer value (16-bit format). When the long counter value reached this value, the FSM generates an interrupt.
#define LSM6DSOX_FSM_LC_TIMEOUT_H             0x17BU
#define LSM6DSOX_FSM_PROGRAMS                 0x17CU // FSM number of programs register (r/w).
#define LSM6DSOX_FSM_START_ADD_L              0x17EU // FSM start address register (r/w). First available address is 0x033C.
#define LSM6DSOX_FSM_START_ADD_H              0x17FU
#define LSM6DSOX_PEDO_CMD_REG                 0x183U // Pedometer configuration register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ad_det_en                : 1; // Enables the advanced detection feature.
  uint8_t not_used_01              : 1;
  uint8_t fp_rejection_en          : 1; // Enables the false-positive rejection feature
  uint8_t carry_count_en           : 1; // Set when user wants to generate interrupt only on count overflow event.
  uint8_t not_used_02              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 4;
  uint8_t carry_count_en           : 1;
  uint8_t fp_rejection_en          : 1;
  uint8_t not_used_01              : 1;
  uint8_t ad_det_en                : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_pedo_cmd_reg_t;

#define LSM6DSOX_PEDO_DEB_STEPS_CONF          0x184U // Pedometer debounce configuration register (r/w)
#define LSM6DSOX_PEDO_SC_DELTAT_L             0x1D0U // Time period register for step detection on delta time (r/w)
#define LSM6DSOX_PEDO_SC_DELTAT_H             0x1D1U

#define LSM6DSOX_MLC_MAG_SENSITIVITY_L        0x1E8U // External magnetometer sensitivity value register for the Machine Learning Core (r/w).
#define LSM6DSOX_MLC_MAG_SENSITIVITY_H        0x1E9U

#define LSM6DSOX_SENSOR_HUB_1                 0x02U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_1_t;

#define LSM6DSOX_SENSOR_HUB_2                 0x03U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_2_t;

#define LSM6DSOX_SENSOR_HUB_3                 0x04U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_3_t;

#define LSM6DSOX_SENSOR_HUB_4                 0x05U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_4_t;

#define LSM6DSOX_SENSOR_HUB_5                 0x06U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_5_t;

#define LSM6DSOX_SENSOR_HUB_6                 0x07U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_6_t;

#define LSM6DSOX_SENSOR_HUB_7                 0x08U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_7_t;

#define LSM6DSOX_SENSOR_HUB_8                 0x09U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_8_t;

#define LSM6DSOX_SENSOR_HUB_9                 0x0AU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_9_t;

#define LSM6DSOX_SENSOR_HUB_10                0x0BU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_10_t;

#define LSM6DSOX_SENSOR_HUB_11                0x0CU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_11_t;

#define LSM6DSOX_SENSOR_HUB_12                0x0DU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_12_t;

#define LSM6DSOX_SENSOR_HUB_13                0x0EU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_13_t;

#define LSM6DSOX_SENSOR_HUB_14                0x0FU // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_14_t;

#define LSM6DSOX_SENSOR_HUB_15                0x10U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_15_t;

#define LSM6DSOX_SENSOR_HUB_16                0x11U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_16_t;

#define LSM6DSOX_SENSOR_HUB_17                0x12U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_17_t;

#define LSM6DSOX_SENSOR_HUB_18                0x13U // Sensor hub output register (r)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_sensor_hub_18_t;

#define LSM6DSOX_MASTER_CONFIG                0x14U // Master configuration register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t aux_sens_on              : 2; // Number of external sensors to be read by the sensor hub.
  uint8_t master_on                : 1; // Sensor hub I2C master enable. Default: 0 . (0: master I2C of sensor hub disabled; 1: master I2C of sensor hub enabled
  uint8_t shub_pu_en               : 1; // Master I²C pull-up enable. Default value: 0. 0: disabled; 1: enabled
  uint8_t pass_through_mode        : 1; // I²C interface pass-through. Default value: 0
  uint8_t start_config             : 1; // Sensor hub trigger signal selection. Default value: 0 (0: sensor hub trigger signal is the accelerometer/gyro data-ready; 1: sensor hub trigger signal external from INT2 pin)
  uint8_t write_once               : 1; // Slave 0 write operation is performed only at the first sensor hub cycle. 0: write operation for each sensor hub cycle; 1: write operation only for the first sensor hub cycle
  uint8_t rst_master_regs          : 1; // Reset Master logic and output registers. Must be set to ‘1’ and then set it to ‘0’. Default value: 0
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rst_master_regs          : 1;
  uint8_t write_once               : 1;
  uint8_t start_config             : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t shub_pu_en               : 1;
  uint8_t master_on                : 1;
  uint8_t aux_sens_on              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_master_config_t;

#define LSM6DSOX_SLV0_ADD                     0x15U // I2C slave address of the first external sensor (Sensor 1) register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rw_0                     : 1;
  uint8_t slave0                   : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0                   : 7;
  uint8_t rw_0                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv0_add_t;

#define LSM6DSOX_SLV0_SUBADD                  0x16U // Address of register on the first external sensor (Sensor 1) register (r/w).
typedef struct
{
  uint8_t slave0_reg               : 8;
} lsm6dsox_slv0_subadd_t;

#define LSM6DSOX_SLV0_CONFIG                  0x17U // First external sensor (Sensor1) configuration and sensor hub settings register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_numop             : 3; // Number of read operations on Sensor 1. Default value: 000
  uint8_t batch_ext_sens_0_en      : 1; // Enable FIFO batching data of first slave. Default value: 0
  uint8_t not_used_01              : 2;
  uint8_t shub_odr                 : 2; // Rate at which the master communicates. Default value: 00
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t shub_odr                 : 2;
  uint8_t not_used_01              : 2;
  uint8_t batch_ext_sens_0_en      : 1;
  uint8_t slave0_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv0_config_t;

#define LSM6DSOX_SLV1_ADD                     0x18U // I²C slave address of the second external sensor (Sensor 2) register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_1                      : 1; // Read operation on Sensor 2 enable. Default value: 0. 0: read operation disabled; 1: read operation enabled
  uint8_t slave1_add               : 7; // I²C slave address of Sensor 2 that can be read by the sensor hub.
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_add               : 7;
  uint8_t r_1                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv1_add_t;

#define LSM6DSOX_SLV1_SUBADD                  0x19U // Address of register on the second external sensor (Sensor 2) register (r/w).
typedef struct
{
  uint8_t slave1_reg               : 8;
} lsm6dsox_slv1_subadd_t;

#define LSM6DSOX_SLV1_CONFIG                  0x1AU // Second external sensor (Sensor 2) configuration register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave1_numop             : 3; // Number of read operations on Sensor 2. Default value: 000
  uint8_t batch_ext_sens_1_en      : 1; // Enable FIFO batching data of second slave. Default value: 0
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t batch_ext_sens_1_en      : 1;
  uint8_t slave1_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv1_config_t;

#define LSM6DSOX_SLV2_ADD                     0x1BU // I2C slave address of the third external sensor (Sensor 3) register (r/w)
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_2                      : 1;
  uint8_t slave2_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_add               : 7;
  uint8_t r_2                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv2_add_t;

#define LSM6DSOX_SLV2_SUBADD                  0x1CU // Address of register on the third external sensor (Sensor 3) register (r/w).
typedef struct
{
  uint8_t slave2_reg               : 8;
} lsm6dsox_slv2_subadd_t;

#define LSM6DSOX_SLV2_CONFIG                  0x1DU // Third external sensor (Sensor 3) configuration register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave2_numop             : 3; // Number of read operations on Sensor 3. Default value: 000
  uint8_t batch_ext_sens_2_en      : 1; // Enable FIFO batching data of third slave. Default value: 0
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t batch_ext_sens_2_en      : 1;
  uint8_t slave2_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv2_config_t;

#define LSM6DSOX_SLV3_ADD                     0x1EU // I²C slave address of the fourth external sensor (Sensor 4) register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_3                      : 1; // Read operation on Sensor 4 enable. Default value: 0
  uint8_t slave3_add               : 7; // I2C slave address of Sensor 4 that can be read by the sensor hub
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_add               : 7;
  uint8_t r_3                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv3_add_t;

#define LSM6DSOX_SLV3_SUBADD                  0x1FU // Address of register on the fourth external sensor (Sensor 4) register (r/w)
typedef struct
{
  uint8_t slave3_reg               : 8;
} lsm6dsox_slv3_subadd_t;

#define LSM6DSOX_SLV3_CONFIG                  0x20U // Fourth external sensor (Sensor 4) configuration register (r/w).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave3_numop             : 3; // Number of read operations on Sensor 4. Default value: 000
  uint8_t  batch_ext_sens_3_en     : 1; // Enable FIFO batching data of fourth slave. Default value: 0
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t  batch_ext_sens_3_en     : 1;
  uint8_t slave3_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_slv3_config_t;

#define LSM6DSOX_DATAWRITE_SLV0               0x21U // Data to be written into the slave device register (r/w).
typedef struct
{
  uint8_t slave0_dataw             : 8;
} lsm6dsox_datawrite_slv0_t;

#define LSM6DSOX_STATUS_MASTER                0x22U // Sensor hub source register (r).
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop           : 1; // Sensor hub communication status. Default value: 0. 0: sensor hub communication not concluded; 1: sensor hub communication concluded
  uint8_t not_used_01              : 2;
  uint8_t slave0_nack              : 1; // This bit is set to 1 if Not acknowledge occurs on slave 0 communication. Default value: 0
  uint8_t slave1_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t wr_once_done             : 1; // When the bit WRITE_ONCE in MASTER_CONFIG (14h) is configured as 1, this bit is set to 1 when the write operation on slave 0 has been performed and completed. Default value: 0
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done             : 1;
  uint8_t slave3_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave0_nack              : 1;
  uint8_t not_used_01              : 2;
  uint8_t sens_hub_endop           : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsox_status_master_t;

#define LSM6DSOX_START_FSM_ADD                0x0400U

/**
  * @defgroup LSM6DSOX_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lsm6dsox_func_cfg_access_t               func_cfg_access;
  lsm6dsox_pin_ctrl_t                      pin_ctrl;
  lsm6dsox_s4s_tph_l_t                     s4s_tph_l;
  lsm6dsox_s4s_tph_h_t                     s4s_tph_h;
  lsm6dsox_s4s_rr_t                        s4s_rr;
  lsm6dsox_fifo_ctrl1_t                    fifo_ctrl1;
  lsm6dsox_fifo_ctrl2_t                    fifo_ctrl2;
  lsm6dsox_fifo_ctrl3_t                    fifo_ctrl3;
  lsm6dsox_fifo_ctrl4_t                    fifo_ctrl4;
  lsm6dsox_counter_bdr_reg1_t              counter_bdr_reg1;
  lsm6dsox_counter_bdr_reg2_t              counter_bdr_reg2;
  lsm6dsox_int1_ctrl_t                     int1_ctrl;
  lsm6dsox_int2_ctrl_t                     int2_ctrl;
  lsm6dsox_ctrl1_xl_t                      ctrl1_xl;
  lsm6dsox_ctrl2_g_t                       ctrl2_g;
  lsm6dsox_ctrl3_c_t                       ctrl3_c;
  lsm6dsox_ctrl4_c_t                       ctrl4_c;
  lsm6dsox_ctrl5_c_t                       ctrl5_c;
  lsm6dsox_ctrl6_c_t                       ctrl6_c;
  lsm6dsox_ctrl7_g_t                       ctrl7_g;
  lsm6dsox_ctrl8_xl_t                      ctrl8_xl;
  lsm6dsox_ctrl9_xl_t                      ctrl9_xl;
  lsm6dsox_ctrl10_c_t                      ctrl10_c;
  lsm6dsox_all_int_src_t                   all_int_src;
  lsm6dsox_wake_up_src_t                   wake_up_src;
  lsm6dsox_tap_src_t                       tap_src;
  lsm6dsox_d6d_src_t                       d6d_src;
  lsm6dsox_status_reg_t                    status_reg;
  lsm6dsox_fifo_status1_t                  fifo_status1;
  lsm6dsox_fifo_status2_t                  fifo_status2;
  lsm6dsox_ui_status_reg_ois_t             ui_status_reg_ois;
  lsm6dsox_tap_cfg0_t                      tap_cfg0;
  lsm6dsox_tap_cfg1_t                      tap_cfg1;
  lsm6dsox_tap_cfg2_t                      tap_cfg2;
  lsm6dsox_tap_ths_6d_t                    tap_ths_6d;
  lsm6dsox_int_dur2_t                      int_dur2;
  lsm6dsox_wake_up_ths_t                   wake_up_ths;
  lsm6dsox_wake_up_dur_t                   wake_up_dur;
  lsm6dsox_free_fall_t                     free_fall;
  lsm6dsox_md1_cfg_t                       md1_cfg;
  lsm6dsox_md2_cfg_t                       md2_cfg;
  lsm6dsox_s4s_st_cmd_code_t               s4s_st_cmd_code;
  lsm6dsox_s4s_dt_reg_t                    s4s_dt_reg;
  lsm6dsox_i3c_bus_avb_t                   i3c_bus_avb;
  lsm6dsox_internal_freq_fine_t            internal_freq_fine;
  lsm6dsox_ui_int_ois_t                    ui_int_ois;
  lsm6dsox_ui_ctrl1_ois_t                  ui_ctrl1_ois;
  lsm6dsox_ui_ctrl2_ois_t                  ui_ctrl2_ois;
  lsm6dsox_ui_ctrl3_ois_t                  ui_ctrl3_ois;
  lsm6dsox_fifo_data_out_tag_t             fifo_data_out_tag;
  lsm6dsox_spi2_status_reg_ois_t           spi2_status_reg_ois;
  lsm6dsox_spi2_int_ois_t                  spi2_int_ois;
  lsm6dsox_spi2_ctrl1_ois_t                spi2_ctrl1_ois;
  lsm6dsox_spi2_ctrl2_ois_t                spi2_ctrl2_ois;
  lsm6dsox_spi2_ctrl3_ois_t                spi2_ctrl3_ois;
  lsm6dsox_page_sel_t                      page_sel;
  lsm6dsox_emb_func_en_a_t                 emb_func_en_a;
  lsm6dsox_emb_func_en_b_t                 emb_func_en_b;
  lsm6dsox_page_address_t                  page_address;
  lsm6dsox_page_value_t                    page_value;
  lsm6dsox_emb_func_int1_t                 emb_func_int1;
  lsm6dsox_fsm_int1_a_t                    fsm_int1_a;
  lsm6dsox_fsm_int1_b_t                    fsm_int1_b;
  lsm6dsox_emb_func_int2_t                 emb_func_int2;
  lsm6dsox_fsm_int2_a_t                    fsm_int2_a;
  lsm6dsox_fsm_int2_b_t                    fsm_int2_b;
  lsm6dsox_emb_func_status_t               emb_func_status;
  lsm6dsox_fsm_status_a_t                  fsm_status_a;
  lsm6dsox_fsm_status_b_t                  fsm_status_b;
  lsm6dsox_page_rw_t                       page_rw;
  lsm6dsox_emb_func_fifo_cfg_t             emb_func_fifo_cfg;
  lsm6dsox_fsm_enable_a_t                  fsm_enable_a;
  lsm6dsox_fsm_enable_b_t                  fsm_enable_b;
  lsm6dsox_fsm_long_counter_clear_t        fsm_long_counter_clear;
  lsm6dsox_fsm_outs1_t                     fsm_outs1;
  lsm6dsox_fsm_outs2_t                     fsm_outs2;
  lsm6dsox_fsm_outs3_t                     fsm_outs3;
  lsm6dsox_fsm_outs4_t                     fsm_outs4;
  lsm6dsox_fsm_outs5_t                     fsm_outs5;
  lsm6dsox_fsm_outs6_t                     fsm_outs6;
  lsm6dsox_fsm_outs7_t                     fsm_outs7;
  lsm6dsox_fsm_outs8_t                     fsm_outs8;
  lsm6dsox_fsm_outs9_t                     fsm_outs9;
  lsm6dsox_fsm_outs10_t                    fsm_outs10;
  lsm6dsox_fsm_outs11_t                    fsm_outs11;
  lsm6dsox_fsm_outs12_t                    fsm_outs12;
  lsm6dsox_fsm_outs13_t                    fsm_outs13;
  lsm6dsox_fsm_outs14_t                    fsm_outs14;
  lsm6dsox_fsm_outs15_t                    fsm_outs15;
  lsm6dsox_fsm_outs16_t                    fsm_outs16;
  lsm6dsox_emb_func_odr_cfg_b_t            emb_func_odr_cfg_b;
  lsm6dsox_emb_func_odr_cfg_c_t            emb_func_odr_cfg_c;
  lsm6dsox_emb_func_src_t                  emb_func_src;
  lsm6dsox_emb_func_init_a_t               emb_func_init_a;
  lsm6dsox_emb_func_init_b_t               emb_func_init_b;
  lsm6dsox_mag_cfg_a_t                     mag_cfg_a;
  lsm6dsox_mag_cfg_b_t                     mag_cfg_b;
  lsm6dsox_pedo_cmd_reg_t                  pedo_cmd_reg;
  lsm6dsox_sensor_hub_1_t                  sensor_hub_1;
  lsm6dsox_sensor_hub_2_t                  sensor_hub_2;
  lsm6dsox_sensor_hub_3_t                  sensor_hub_3;
  lsm6dsox_sensor_hub_4_t                  sensor_hub_4;
  lsm6dsox_sensor_hub_5_t                  sensor_hub_5;
  lsm6dsox_sensor_hub_6_t                  sensor_hub_6;
  lsm6dsox_sensor_hub_7_t                  sensor_hub_7;
  lsm6dsox_sensor_hub_8_t                  sensor_hub_8;
  lsm6dsox_sensor_hub_9_t                  sensor_hub_9;
  lsm6dsox_sensor_hub_10_t                 sensor_hub_10;
  lsm6dsox_sensor_hub_11_t                 sensor_hub_11;
  lsm6dsox_sensor_hub_12_t                 sensor_hub_12;
  lsm6dsox_sensor_hub_13_t                 sensor_hub_13;
  lsm6dsox_sensor_hub_14_t                 sensor_hub_14;
  lsm6dsox_sensor_hub_15_t                 sensor_hub_15;
  lsm6dsox_sensor_hub_16_t                 sensor_hub_16;
  lsm6dsox_sensor_hub_17_t                 sensor_hub_17;
  lsm6dsox_sensor_hub_18_t                 sensor_hub_18;
  lsm6dsox_master_config_t                 master_config;
  lsm6dsox_slv0_add_t                      slv0_add;
  lsm6dsox_slv0_subadd_t                   slv0_subadd;
  lsm6dsox_slv0_config_t                   slv0_config;
  lsm6dsox_slv1_add_t                      slv1_add;
  lsm6dsox_slv1_subadd_t                   slv1_subadd;
  lsm6dsox_slv1_config_t                   slv1_config;
  lsm6dsox_slv2_add_t                      slv2_add;
  lsm6dsox_slv2_subadd_t                   slv2_subadd;
  lsm6dsox_slv2_config_t                   slv2_config;
  lsm6dsox_slv3_add_t                      slv3_add;
  lsm6dsox_slv3_subadd_t                   slv3_subadd;
  lsm6dsox_slv3_config_t                   slv3_config;
  lsm6dsox_datawrite_slv0_t                datawrite_slv0;
  lsm6dsox_status_master_t                 status_master;
  bitwise_t                                bitwise;
  uint8_t                                  byte;
} lsm6dsox_reg_t;

/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */

int32_t lsm6dsox_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t lsm6dsox_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

float_t lsm6dsox_from_fs2_to_mg(int16_t lsb);
float_t lsm6dsox_from_fs4_to_mg(int16_t lsb);
float_t lsm6dsox_from_fs8_to_mg(int16_t lsb);
float_t lsm6dsox_from_fs16_to_mg(int16_t lsb);

float_t lsm6dsox_from_fs125_to_mdps(int16_t lsb);
float_t lsm6dsox_from_fs500_to_mdps(int16_t lsb);
float_t lsm6dsox_from_fs250_to_mdps(int16_t lsb);
float_t lsm6dsox_from_fs1000_to_mdps(int16_t lsb);
float_t lsm6dsox_from_fs2000_to_mdps(int16_t lsb);

float_t lsm6dsox_from_lsb_to_celsius(int16_t lsb);

float_t lsm6dsox_from_lsb_to_nsec(int16_t lsb);

typedef enum
{
  LSM6DSOX_2g   = 0,
  LSM6DSOX_16g  = 1, /* if XL_FS_MODE = â€˜1â€™ -> LSM6DSOX_2g */
  LSM6DSOX_4g   = 2,
  LSM6DSOX_8g   = 3,
} lsm6dsox_fs_xl_t;
int32_t lsm6dsox_xl_full_scale_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_fs_xl_t val);
int32_t lsm6dsox_xl_full_scale_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_fs_xl_t *val);

typedef enum
{
  LSM6DSOX_XL_ODR_OFF    = 0,
  LSM6DSOX_XL_ODR_12Hz5  = 1,
  LSM6DSOX_XL_ODR_26Hz   = 2,
  LSM6DSOX_XL_ODR_52Hz   = 3,
  LSM6DSOX_XL_ODR_104Hz  = 4,
  LSM6DSOX_XL_ODR_208Hz  = 5,
  LSM6DSOX_XL_ODR_417Hz  = 6,
  LSM6DSOX_XL_ODR_833Hz  = 7,
  LSM6DSOX_XL_ODR_1667Hz = 8,
  LSM6DSOX_XL_ODR_3333Hz = 9,
  LSM6DSOX_XL_ODR_6667Hz = 10,
  LSM6DSOX_XL_ODR_1Hz6   = 11, /* (low power only) */
} lsm6dsox_odr_xl_t;
int32_t lsm6dsox_xl_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_odr_xl_t val);
int32_t lsm6dsox_xl_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_odr_xl_t *val);

typedef enum
{
  LSM6DSOX_250dps   = 0,
  LSM6DSOX_125dps   = 1,
  LSM6DSOX_500dps   = 2,
  LSM6DSOX_1000dps  = 4,
  LSM6DSOX_2000dps  = 6,
} lsm6dsox_fs_g_t;
int32_t lsm6dsox_gy_full_scale_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_fs_g_t val);
int32_t lsm6dsox_gy_full_scale_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_fs_g_t *val);

typedef enum
{
  LSM6DSOX_GY_ODR_OFF    = 0,
  LSM6DSOX_GY_ODR_12Hz5  = 1,
  LSM6DSOX_GY_ODR_26Hz   = 2,
  LSM6DSOX_GY_ODR_52Hz   = 3,
  LSM6DSOX_GY_ODR_104Hz  = 4,
  LSM6DSOX_GY_ODR_208Hz  = 5,
  LSM6DSOX_GY_ODR_417Hz  = 6,
  LSM6DSOX_GY_ODR_833Hz  = 7,
  LSM6DSOX_GY_ODR_1667Hz = 8,
  LSM6DSOX_GY_ODR_3333Hz = 9,
  LSM6DSOX_GY_ODR_6667Hz = 10,
} lsm6dsox_odr_g_t;
int32_t lsm6dsox_gy_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_odr_g_t val);
int32_t lsm6dsox_gy_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_odr_g_t *val);

int32_t lsm6dsox_block_data_update_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lsm6dsox_block_data_update_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum
{
  LSM6DSOX_LSb_1mg  = 0,
  LSM6DSOX_LSb_16mg = 1,
} lsm6dsox_usr_off_w_t;
int32_t lsm6dsox_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_usr_off_w_t val);
int32_t lsm6dsox_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_usr_off_w_t *val);

typedef enum
{
  LSM6DSOX_HIGH_PERFORMANCE_MD  = 0,
  LSM6DSOX_LOW_NORMAL_POWER_MD  = 1,
  LSM6DSOX_ULTRA_LOW_POWER_MD   = 2,
} lsm6dsox_xl_hm_mode_t;
int32_t lsm6dsox_xl_power_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_xl_hm_mode_t val);
int32_t lsm6dsox_xl_power_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_xl_hm_mode_t *val);

typedef enum
{
  LSM6DSOX_GY_HIGH_PERFORMANCE  = 0,
  LSM6DSOX_GY_NORMAL            = 1,
} lsm6dsox_g_hm_mode_t;
int32_t lsm6dsox_gy_power_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_g_hm_mode_t val);
int32_t lsm6dsox_gy_power_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_g_hm_mode_t *val);

int32_t lsm6dsox_status_reg_get(stmdev_ctx_t *ctx,
                                lsm6dsox_status_reg_t *val);

int32_t lsm6dsox_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_gy_flag_data_ready_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_temp_flag_data_ready_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

int32_t lsm6dsox_xl_usr_offset_x_set(stmdev_ctx_t *ctx,
                                     uint8_t *buff);
int32_t lsm6dsox_xl_usr_offset_x_get(stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t lsm6dsox_xl_usr_offset_y_set(stmdev_ctx_t *ctx,
                                     uint8_t *buff);
int32_t lsm6dsox_xl_usr_offset_y_get(stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t lsm6dsox_xl_usr_offset_z_set(stmdev_ctx_t *ctx,
                                     uint8_t *buff);
int32_t lsm6dsox_xl_usr_offset_z_get(stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t lsm6dsox_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_timestamp_rst(stmdev_ctx_t *ctx);

int32_t lsm6dsox_timestamp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_timestamp_raw_get(stmdev_ctx_t *ctx, uint32_t *val);

typedef enum
{
  LSM6DSOX_NO_ROUND      = 0,
  LSM6DSOX_ROUND_XL      = 1,
  LSM6DSOX_ROUND_GY      = 2,
  LSM6DSOX_ROUND_GY_XL   = 3,
} lsm6dsox_rounding_t;
int32_t lsm6dsox_rounding_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_rounding_t val);
int32_t lsm6dsox_rounding_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_rounding_t *val);

typedef enum
{
  LSM6DSOX_STAT_RND_DISABLE  = 0,
  LSM6DSOX_STAT_RND_ENABLE   = 1,
} lsm6dsox_rounding_status_t;
int32_t lsm6dsox_rounding_on_status_set(stmdev_ctx_t *ctx,
                                        lsm6dsox_rounding_status_t val);
int32_t lsm6dsox_rounding_on_status_get(stmdev_ctx_t *ctx,
                                        lsm6dsox_rounding_status_t *val);

int32_t lsm6dsox_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsox_angular_rate_raw_get(stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t lsm6dsox_acceleration_raw_get(stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t lsm6dsox_fifo_out_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsox_ois_angular_rate_raw_get(stmdev_ctx_t *ctx,
                                          int16_t *val);

int32_t lsm6dsox_ois_acceleration_raw_get(stmdev_ctx_t *ctx,
                                          int16_t *val);

int32_t lsm6dsox_aux_temperature_raw_get(stmdev_ctx_t *ctx,
                                         int16_t *val);

int32_t lsm6dsox_aux_ois_angular_rate_raw_get(stmdev_ctx_t *ctx,
                                              int16_t *val);

int32_t lsm6dsox_aux_ois_acceleration_raw_get(stmdev_ctx_t *ctx,
                                              int16_t *val);

int32_t lsm6dsox_number_of_steps_get(stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t lsm6dsox_steps_reset(stmdev_ctx_t *ctx);

int32_t lsm6dsox_mlc_out_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsox_odr_cal_reg_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_odr_cal_reg_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_USER_BANK           = 0,
  LSM6DSOX_SENSOR_HUB_BANK     = 1,
  LSM6DSOX_EMBEDDED_FUNC_BANK  = 2,
} lsm6dsox_reg_access_t;
int32_t lsm6dsox_mem_bank_set(stmdev_ctx_t *ctx,
                              lsm6dsox_reg_access_t val);
int32_t lsm6dsox_mem_bank_get(stmdev_ctx_t *ctx,
                              lsm6dsox_reg_access_t *val);

int32_t lsm6dsox_ln_pg_write_byte(stmdev_ctx_t *ctx, uint16_t address,
                                  uint8_t *val);
int32_t lsm6dsox_ln_pg_read_byte(stmdev_ctx_t *ctx, uint16_t address,
                                 uint8_t *val);

int32_t lsm6dsox_ln_pg_write(stmdev_ctx_t *ctx, uint16_t address,
                             uint8_t *buf, uint8_t len);
int32_t lsm6dsox_ln_pg_read(stmdev_ctx_t *ctx, uint16_t address,
                            uint8_t *val);

typedef enum
{
  LSM6DSOX_DRDY_LATCHED = 0,
  LSM6DSOX_DRDY_PULSED  = 1,
} lsm6dsox_dataready_pulsed_t;
int32_t lsm6dsox_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_dataready_pulsed_t val);
int32_t lsm6dsox_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_dataready_pulsed_t *val);

int32_t lsm6dsox_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsox_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_XL_ST_DISABLE  = 0,
  LSM6DSOX_XL_ST_POSITIVE = 1,
  LSM6DSOX_XL_ST_NEGATIVE = 2,
} lsm6dsox_st_xl_t;
int32_t lsm6dsox_xl_self_test_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_xl_t val);
int32_t lsm6dsox_xl_self_test_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_xl_t *val);

typedef enum
{
  LSM6DSOX_GY_ST_DISABLE  = 0,
  LSM6DSOX_GY_ST_POSITIVE = 1,
  LSM6DSOX_GY_ST_NEGATIVE = 3,
} lsm6dsox_st_g_t;
int32_t lsm6dsox_gy_self_test_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_g_t val);
int32_t lsm6dsox_gy_self_test_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_g_t *val);

int32_t lsm6dsox_xl_filter_lp2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_xl_filter_lp2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_gy_filter_lp1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_gy_filter_lp1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_filter_settling_mask_set(stmdev_ctx_t *ctx,
                                          uint8_t val);
int32_t lsm6dsox_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

typedef enum
{
  LSM6DSOX_ULTRA_LIGHT  = 0,
  LSM6DSOX_VERY_LIGHT   = 1,
  LSM6DSOX_LIGHT        = 2,
  LSM6DSOX_MEDIUM       = 3,
  LSM6DSOX_STRONG       = 4,
  LSM6DSOX_VERY_STRONG  = 5,
  LSM6DSOX_AGGRESSIVE   = 6,
  LSM6DSOX_XTREME       = 7,
} lsm6dsox_ftype_t;
int32_t lsm6dsox_gy_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_ftype_t val);
int32_t lsm6dsox_gy_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_ftype_t *val);

int32_t lsm6dsox_xl_lp2_on_6d_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_xl_lp2_on_6d_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_HP_PATH_DISABLE_ON_OUT    = 0x00,
  LSM6DSOX_SLOPE_ODR_DIV_4           = 0x10,
  LSM6DSOX_HP_ODR_DIV_10             = 0x11,
  LSM6DSOX_HP_ODR_DIV_20             = 0x12,
  LSM6DSOX_HP_ODR_DIV_45             = 0x13,
  LSM6DSOX_HP_ODR_DIV_100            = 0x14,
  LSM6DSOX_HP_ODR_DIV_200            = 0x15,
  LSM6DSOX_HP_ODR_DIV_400            = 0x16,
  LSM6DSOX_HP_ODR_DIV_800            = 0x17,
  LSM6DSOX_HP_REF_MD_ODR_DIV_10      = 0x31,
  LSM6DSOX_HP_REF_MD_ODR_DIV_20      = 0x32,
  LSM6DSOX_HP_REF_MD_ODR_DIV_45      = 0x33,
  LSM6DSOX_HP_REF_MD_ODR_DIV_100     = 0x34,
  LSM6DSOX_HP_REF_MD_ODR_DIV_200     = 0x35,
  LSM6DSOX_HP_REF_MD_ODR_DIV_400     = 0x36,
  LSM6DSOX_HP_REF_MD_ODR_DIV_800     = 0x37,
  LSM6DSOX_LP_ODR_DIV_10             = 0x01,
  LSM6DSOX_LP_ODR_DIV_20             = 0x02,
  LSM6DSOX_LP_ODR_DIV_45             = 0x03,
  LSM6DSOX_LP_ODR_DIV_100            = 0x04,
  LSM6DSOX_LP_ODR_DIV_200            = 0x05,
  LSM6DSOX_LP_ODR_DIV_400            = 0x06,
  LSM6DSOX_LP_ODR_DIV_800            = 0x07,
} lsm6dsox_hp_slope_xl_en_t;
int32_t lsm6dsox_xl_hp_path_on_out_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_hp_slope_xl_en_t val);
int32_t lsm6dsox_xl_hp_path_on_out_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_hp_slope_xl_en_t *val);

int32_t lsm6dsox_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_xl_fast_settling_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSOX_USE_SLOPE = 0,
  LSM6DSOX_USE_HPF   = 1,
} lsm6dsox_slope_fds_t;
int32_t lsm6dsox_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_slope_fds_t val);
int32_t lsm6dsox_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_slope_fds_t *val);

typedef enum
{
  LSM6DSOX_HP_FILTER_NONE     = 0x00,
  LSM6DSOX_HP_FILTER_16mHz    = 0x80,
  LSM6DSOX_HP_FILTER_65mHz    = 0x81,
  LSM6DSOX_HP_FILTER_260mHz   = 0x82,
  LSM6DSOX_HP_FILTER_1Hz04    = 0x83,
} lsm6dsox_hpm_g_t;
int32_t lsm6dsox_gy_hp_path_internal_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_hpm_g_t val);
int32_t lsm6dsox_gy_hp_path_internal_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_hpm_g_t *val);

typedef enum
{
  LSM6DSOX_OIS_CTRL_AUX_DATA_UI          = 0x00,
  LSM6DSOX_OIS_CTRL_AUX_DATA_UI_AUX      = 0x01,
  LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI       = 0x02,
  LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI_AUX   = 0x03,
} lsm6dsox_spi2_read_en_t;
int32_t lsm6dsox_ois_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_spi2_read_en_t val);
int32_t lsm6dsox_ois_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_spi2_read_en_t *val);

typedef enum
{
  LSM6DSOX_AUX_PULL_UP_DISC       = 0,
  LSM6DSOX_AUX_PULL_UP_CONNECT    = 1,
} lsm6dsox_ois_pu_dis_t;
int32_t lsm6dsox_aux_sdo_ocs_mode_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_ois_pu_dis_t val);
int32_t lsm6dsox_aux_sdo_ocs_mode_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_ois_pu_dis_t *val);

typedef enum
{
  LSM6DSOX_AUX_ON                    = 1,
  LSM6DSOX_AUX_ON_BY_AUX_INTERFACE   = 0,
} lsm6dsox_ois_on_t;
int32_t lsm6dsox_aux_pw_on_ctrl_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_ois_on_t val);
int32_t lsm6dsox_aux_pw_on_ctrl_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_ois_on_t *val);

typedef enum
{
  LSM6DSOX_USE_SAME_XL_FS        = 0,
  LSM6DSOX_USE_DIFFERENT_XL_FS   = 1,
} lsm6dsox_xl_fs_mode_t;
int32_t lsm6dsox_aux_xl_fs_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_xl_fs_mode_t val);
int32_t lsm6dsox_aux_xl_fs_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_xl_fs_mode_t *val);

int32_t lsm6dsox_aux_status_reg_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_spi2_status_reg_ois_t *val);

int32_t lsm6dsox_aux_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm6dsox_aux_gy_flag_data_ready_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm6dsox_aux_gy_flag_settling_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

typedef enum
{
  LSM6DSOX_AUX_DEN_ACTIVE_LOW     = 0,
  LSM6DSOX_AUX_DEN_ACTIVE_HIGH    = 1,
} lsm6dsox_den_lh_ois_t;
int32_t lsm6dsox_aux_den_polarity_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_den_lh_ois_t val);
int32_t lsm6dsox_aux_den_polarity_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_den_lh_ois_t *val);

typedef enum
{
  LSM6DSOX_AUX_DEN_DISABLE         = 0,
  LSM6DSOX_AUX_DEN_LEVEL_LATCH     = 3,
  LSM6DSOX_AUX_DEN_LEVEL_TRIG      = 2,
} lsm6dsox_lvl2_ois_t;
int32_t lsm6dsox_aux_den_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_lvl2_ois_t val);
int32_t lsm6dsox_aux_den_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_lvl2_ois_t *val);

int32_t lsm6dsox_aux_drdy_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_aux_drdy_on_int2_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSOX_AUX_DISABLE  = 0,
  LSM6DSOX_MODE_3_GY    = 1,
  LSM6DSOX_MODE_4_GY_XL = 3,
} lsm6dsox_ois_en_spi2_t;
int32_t lsm6dsox_aux_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_ois_en_spi2_t val);
int32_t lsm6dsox_aux_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_ois_en_spi2_t *val);

typedef enum
{
  LSM6DSOX_250dps_AUX  = 0,
  LSM6DSOX_125dps_AUX  = 1,
  LSM6DSOX_500dps_AUX  = 2,
  LSM6DSOX_1000dps_AUX = 4,
  LSM6DSOX_2000dps_AUX = 6,
} lsm6dsox_fs_g_ois_t;
int32_t lsm6dsox_aux_gy_full_scale_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_fs_g_ois_t val);
int32_t lsm6dsox_aux_gy_full_scale_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_fs_g_ois_t *val);

typedef enum
{
  LSM6DSOX_AUX_SPI_4_WIRE = 0,
  LSM6DSOX_AUX_SPI_3_WIRE = 1,
} lsm6dsox_sim_ois_t;
int32_t lsm6dsox_aux_spi_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_sim_ois_t val);
int32_t lsm6dsox_aux_spi_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_sim_ois_t *val);

typedef enum
{
  LSM6DSOX_351Hz39 = 0,
  LSM6DSOX_236Hz63 = 1,
  LSM6DSOX_172Hz70 = 2,
  LSM6DSOX_937Hz91 = 3,
} lsm6dsox_ftype_ois_t;
int32_t lsm6dsox_aux_gy_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                          lsm6dsox_ftype_ois_t val);
int32_t lsm6dsox_aux_gy_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                          lsm6dsox_ftype_ois_t *val);

typedef enum
{
  LSM6DSOX_AUX_HP_DISABLE = 0x00,
  LSM6DSOX_AUX_HP_Hz016   = 0x10,
  LSM6DSOX_AUX_HP_Hz065   = 0x11,
  LSM6DSOX_AUX_HP_Hz260   = 0x12,
  LSM6DSOX_AUX_HP_1Hz040  = 0x13,
} lsm6dsox_hpm_ois_t;
int32_t lsm6dsox_aux_gy_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_hpm_ois_t val);
int32_t lsm6dsox_aux_gy_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_hpm_ois_t *val);

typedef enum
{
  LSM6DSOX_ENABLE_CLAMP  = 0,
  LSM6DSOX_DISABLE_CLAMP = 1,
} lsm6dsox_st_ois_clampdis_t;
int32_t lsm6dsox_aux_gy_clamp_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_ois_clampdis_t val);
int32_t lsm6dsox_aux_gy_clamp_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_st_ois_clampdis_t *val);

typedef enum
{
  LSM6DSOX_289Hz = 0,
  LSM6DSOX_258Hz = 1,
  LSM6DSOX_120Hz = 2,
  LSM6DSOX_65Hz2 = 3,
  LSM6DSOX_33Hz2 = 4,
  LSM6DSOX_16Hz6 = 5,
  LSM6DSOX_8Hz30 = 6,
  LSM6DSOX_4Hz15 = 7,
} lsm6dsox_filter_xl_conf_ois_t;
int32_t lsm6dsox_aux_xl_bandwidth_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_filter_xl_conf_ois_t val);
int32_t lsm6dsox_aux_xl_bandwidth_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_filter_xl_conf_ois_t *val);

typedef enum
{
  LSM6DSOX_AUX_2g  = 0,
  LSM6DSOX_AUX_16g = 1,
  LSM6DSOX_AUX_4g  = 2,
  LSM6DSOX_AUX_8g  = 3,
} lsm6dsox_fs_xl_ois_t;
int32_t lsm6dsox_aux_xl_full_scale_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_fs_xl_ois_t val);
int32_t lsm6dsox_aux_xl_full_scale_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_fs_xl_ois_t *val);

typedef enum
{
  LSM6DSOX_PULL_UP_DISC       = 0,
  LSM6DSOX_PULL_UP_CONNECT    = 1,
} lsm6dsox_sdo_pu_en_t;
int32_t lsm6dsox_sdo_sa0_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_sdo_pu_en_t val);
int32_t lsm6dsox_sdo_sa0_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_sdo_pu_en_t *val);

typedef enum
{
  LSM6DSOX_SPI_4_WIRE = 0,
  LSM6DSOX_SPI_3_WIRE = 1,
} lsm6dsox_sim_t;
int32_t lsm6dsox_spi_mode_set(stmdev_ctx_t *ctx, lsm6dsox_sim_t val);
int32_t lsm6dsox_spi_mode_get(stmdev_ctx_t *ctx, lsm6dsox_sim_t *val);

typedef enum
{
  LSM6DSOX_I2C_ENABLE  = 0,
  LSM6DSOX_I2C_DISABLE = 1,
} lsm6dsox_i2c_disable_t;
int32_t lsm6dsox_i2c_interface_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_i2c_disable_t val);
int32_t lsm6dsox_i2c_interface_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_i2c_disable_t *val);

typedef enum
{
  LSM6DSOX_I3C_DISABLE         = 0x80,
  LSM6DSOX_I3C_ENABLE_T_50us   = 0x00,
  LSM6DSOX_I3C_ENABLE_T_2us    = 0x01,
  LSM6DSOX_I3C_ENABLE_T_1ms    = 0x02,
  LSM6DSOX_I3C_ENABLE_T_25ms   = 0x03,
} lsm6dsox_i3c_disable_t;
int32_t lsm6dsox_i3c_disable_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_i3c_disable_t val);
int32_t lsm6dsox_i3c_disable_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_i3c_disable_t *val);

typedef enum
{
  LSM6DSOX_PUSH_PULL                         = 0x00,
  LSM6DSOX_OPEN_DRAIN                        = 0x01,
  LSM6DSOX_INT1_NOPULL_DOWN_INT2_PUSH_PULL   = 0x02,
  LSM6DSOX_INT1_NOPULL_DOWN_INT2_OPEN_DRAIN  = 0x03,
} lsm6dsox_pp_od_t;
int32_t lsm6dsox_pin_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_pp_od_t val);
int32_t lsm6dsox_pin_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_pp_od_t *val);

typedef enum
{
  LSM6DSOX_ACTIVE_HIGH = 0,
  LSM6DSOX_ACTIVE_LOW  = 1,
} lsm6dsox_h_lactive_t;
int32_t lsm6dsox_pin_polarity_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_h_lactive_t val);
int32_t lsm6dsox_pin_polarity_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_h_lactive_t *val);

int32_t lsm6dsox_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_ALL_INT_PULSED            = 0,
  LSM6DSOX_BASE_LATCHED_EMB_PULSED   = 1,
  LSM6DSOX_BASE_PULSED_EMB_LATCHED   = 2,
  LSM6DSOX_ALL_INT_LATCHED           = 3,
} lsm6dsox_lir_t;
int32_t lsm6dsox_int_notification_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_lir_t val);
int32_t lsm6dsox_int_notification_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_lir_t *val);

typedef enum
{
  LSM6DSOX_LSb_FS_DIV_64       = 0,
  LSM6DSOX_LSb_FS_DIV_256      = 1,
} lsm6dsox_wake_ths_w_t;
int32_t lsm6dsox_wkup_ths_weight_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_wake_ths_w_t val);
int32_t lsm6dsox_wkup_ths_weight_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_wake_ths_w_t *val);

int32_t lsm6dsox_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_xl_usr_offset_on_wkup_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm6dsox_xl_usr_offset_on_wkup_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm6dsox_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_gy_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_gy_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_DRIVE_SLEEP_CHG_EVENT = 0,
  LSM6DSOX_DRIVE_SLEEP_STATUS    = 1,
} lsm6dsox_sleep_status_on_int_t;
int32_t lsm6dsox_act_pin_notification_set(stmdev_ctx_t *ctx,
                                          lsm6dsox_sleep_status_on_int_t val);
int32_t lsm6dsox_act_pin_notification_get(stmdev_ctx_t *ctx,
                                          lsm6dsox_sleep_status_on_int_t *val);

typedef enum
{
  LSM6DSOX_XL_AND_GY_NOT_AFFECTED      = 0,
  LSM6DSOX_XL_12Hz5_GY_NOT_AFFECTED    = 1,
  LSM6DSOX_XL_12Hz5_GY_SLEEP           = 2,
  LSM6DSOX_XL_12Hz5_GY_PD              = 3,
} lsm6dsox_inact_en_t;
int32_t lsm6dsox_act_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_inact_en_t val);
int32_t lsm6dsox_act_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_inact_en_t *val);

int32_t lsm6dsox_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_tap_detection_on_z_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsox_tap_detection_on_z_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_tap_detection_on_y_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsox_tap_detection_on_y_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_tap_detection_on_x_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsox_tap_detection_on_x_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_tap_threshold_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_threshold_x_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_XYZ = 0,
  LSM6DSOX_YXZ = 1,
  LSM6DSOX_XZY = 2,
  LSM6DSOX_ZYX = 3,
  LSM6DSOX_YZX = 5,
  LSM6DSOX_ZXY = 6,
} lsm6dsox_tap_priority_t;
int32_t lsm6dsox_tap_axis_priority_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_tap_priority_t val);
int32_t lsm6dsox_tap_axis_priority_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_tap_priority_t *val);

int32_t lsm6dsox_tap_threshold_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_threshold_y_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_tap_threshold_z_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_threshold_z_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_ONLY_SINGLE = 0,
  LSM6DSOX_BOTH_SINGLE_DOUBLE = 1,
} lsm6dsox_single_double_tap_t;
int32_t lsm6dsox_tap_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_single_double_tap_t val);
int32_t lsm6dsox_tap_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_single_double_tap_t *val);

typedef enum
{
  LSM6DSOX_DEG_80  = 0,
  LSM6DSOX_DEG_70  = 1,
  LSM6DSOX_DEG_60  = 2,
  LSM6DSOX_DEG_50  = 3,
} lsm6dsox_sixd_ths_t;
int32_t lsm6dsox_6d_threshold_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_sixd_ths_t val);
int32_t lsm6dsox_6d_threshold_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_sixd_ths_t *val);

int32_t lsm6dsox_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_FF_TSH_156mg = 0,
  LSM6DSOX_FF_TSH_219mg = 1,
  LSM6DSOX_FF_TSH_250mg = 2,
  LSM6DSOX_FF_TSH_312mg = 3,
  LSM6DSOX_FF_TSH_344mg = 4,
  LSM6DSOX_FF_TSH_406mg = 5,
  LSM6DSOX_FF_TSH_469mg = 6,
  LSM6DSOX_FF_TSH_500mg = 7,
} lsm6dsox_ff_ths_t;
int32_t lsm6dsox_ff_threshold_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_ff_ths_t val);
int32_t lsm6dsox_ff_threshold_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_ff_ths_t *val);

int32_t lsm6dsox_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsox_fifo_watermark_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsox_compression_algo_init_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm6dsox_compression_algo_init_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

typedef enum
{
  LSM6DSOX_CMP_DISABLE  = 0x00,
  LSM6DSOX_CMP_ALWAYS   = 0x04,
  LSM6DSOX_CMP_8_TO_1   = 0x05,
  LSM6DSOX_CMP_16_TO_1  = 0x06,
  LSM6DSOX_CMP_32_TO_1  = 0x07,
} lsm6dsox_uncoptr_rate_t;
int32_t lsm6dsox_compression_algo_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_uncoptr_rate_t val);
int32_t lsm6dsox_compression_algo_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_uncoptr_rate_t *val);

int32_t lsm6dsox_fifo_virtual_sens_odr_chg_set(stmdev_ctx_t *ctx,
                                               uint8_t val);
int32_t lsm6dsox_fifo_virtual_sens_odr_chg_get(stmdev_ctx_t *ctx,
                                               uint8_t *val);

int32_t lsm6dsox_compression_algo_real_time_set(stmdev_ctx_t *ctx,
                                                uint8_t val);
int32_t lsm6dsox_compression_algo_real_time_get(stmdev_ctx_t *ctx,
                                                uint8_t *val);

int32_t lsm6dsox_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSOX_XL_NOT_BATCHED       =  0,
  LSM6DSOX_XL_BATCHED_AT_12Hz5   =  1,
  LSM6DSOX_XL_BATCHED_AT_26Hz    =  2,
  LSM6DSOX_XL_BATCHED_AT_52Hz    =  3,
  LSM6DSOX_XL_BATCHED_AT_104Hz   =  4,
  LSM6DSOX_XL_BATCHED_AT_208Hz   =  5,
  LSM6DSOX_XL_BATCHED_AT_417Hz   =  6,
  LSM6DSOX_XL_BATCHED_AT_833Hz   =  7,
  LSM6DSOX_XL_BATCHED_AT_1667Hz  =  8,
  LSM6DSOX_XL_BATCHED_AT_3333Hz  =  9,
  LSM6DSOX_XL_BATCHED_AT_6667Hz  = 10,
  LSM6DSOX_XL_BATCHED_AT_6Hz5    = 11,
} lsm6dsox_bdr_xl_t;
int32_t lsm6dsox_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_bdr_xl_t val);
int32_t lsm6dsox_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_bdr_xl_t *val);

typedef enum
{
  LSM6DSOX_GY_NOT_BATCHED         = 0,
  LSM6DSOX_GY_BATCHED_AT_12Hz5    = 1,
  LSM6DSOX_GY_BATCHED_AT_26Hz     = 2,
  LSM6DSOX_GY_BATCHED_AT_52Hz     = 3,
  LSM6DSOX_GY_BATCHED_AT_104Hz    = 4,
  LSM6DSOX_GY_BATCHED_AT_208Hz    = 5,
  LSM6DSOX_GY_BATCHED_AT_417Hz    = 6,
  LSM6DSOX_GY_BATCHED_AT_833Hz    = 7,
  LSM6DSOX_GY_BATCHED_AT_1667Hz   = 8,
  LSM6DSOX_GY_BATCHED_AT_3333Hz   = 9,
  LSM6DSOX_GY_BATCHED_AT_6667Hz   = 10,
  LSM6DSOX_GY_BATCHED_AT_6Hz5     = 11,
} lsm6dsox_bdr_gy_t;
int32_t lsm6dsox_fifo_gy_batch_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_bdr_gy_t val);
int32_t lsm6dsox_fifo_gy_batch_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_bdr_gy_t *val);

typedef enum
{
  LSM6DSOX_BYPASS_MODE             = 0,
  LSM6DSOX_FIFO_MODE               = 1,
  LSM6DSOX_STREAM_TO_FIFO_MODE     = 3,
  LSM6DSOX_BYPASS_TO_STREAM_MODE   = 4,
  LSM6DSOX_STREAM_MODE             = 6,
  LSM6DSOX_BYPASS_TO_FIFO_MODE     = 7,
} lsm6dsox_fifo_mode_t;
int32_t lsm6dsox_fifo_mode_set(stmdev_ctx_t *ctx,
                               lsm6dsox_fifo_mode_t val);
int32_t lsm6dsox_fifo_mode_get(stmdev_ctx_t *ctx,
                               lsm6dsox_fifo_mode_t *val);

typedef enum
{
  LSM6DSOX_TEMP_NOT_BATCHED        = 0,
  LSM6DSOX_TEMP_BATCHED_AT_1Hz6    = 1,
  LSM6DSOX_TEMP_BATCHED_AT_12Hz5   = 2,
  LSM6DSOX_TEMP_BATCHED_AT_52Hz    = 3,
} lsm6dsox_odr_t_batch_t;
int32_t lsm6dsox_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_odr_t_batch_t val);
int32_t lsm6dsox_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_odr_t_batch_t *val);

typedef enum
{
  LSM6DSOX_NO_DECIMATION = 0,
  LSM6DSOX_DEC_1         = 1,
  LSM6DSOX_DEC_8         = 2,
  LSM6DSOX_DEC_32        = 3,
} lsm6dsox_odr_ts_batch_t;
int32_t lsm6dsox_fifo_timestamp_decimation_set(stmdev_ctx_t *ctx,
                                               lsm6dsox_odr_ts_batch_t val);
int32_t lsm6dsox_fifo_timestamp_decimation_get(stmdev_ctx_t *ctx,
                                               lsm6dsox_odr_ts_batch_t *val);

typedef enum
{
  LSM6DSOX_XL_BATCH_EVENT   = 0,
  LSM6DSOX_GYRO_BATCH_EVENT = 1,
} lsm6dsox_trig_counter_bdr_t;

typedef enum
{
  LSM6DSOX_GYRO_NC_TAG    = 1,
  LSM6DSOX_XL_NC_TAG,
  LSM6DSOX_TEMPERATURE_TAG,
  LSM6DSOX_TIMESTAMP_TAG,
  LSM6DSOX_CFG_CHANGE_TAG,
  LSM6DSOX_XL_NC_T_2_TAG,
  LSM6DSOX_XL_NC_T_1_TAG,
  LSM6DSOX_XL_2XC_TAG,
  LSM6DSOX_XL_3XC_TAG,
  LSM6DSOX_GYRO_NC_T_2_TAG,
  LSM6DSOX_GYRO_NC_T_1_TAG,
  LSM6DSOX_GYRO_2XC_TAG,
  LSM6DSOX_GYRO_3XC_TAG,
  LSM6DSOX_SENSORHUB_SLAVE0_TAG,
  LSM6DSOX_SENSORHUB_SLAVE1_TAG,
  LSM6DSOX_SENSORHUB_SLAVE2_TAG,
  LSM6DSOX_SENSORHUB_SLAVE3_TAG,
  LSM6DSOX_STEP_CPUNTER_TAG,
  LSM6DSOX_GAME_ROTATION_TAG,
  LSM6DSOX_GEOMAG_ROTATION_TAG,
  LSM6DSOX_ROTATION_TAG,
  LSM6DSOX_SENSORHUB_NACK_TAG  = 0x19,
} lsm6dsox_fifo_tag_t;
int32_t lsm6dsox_fifo_cnt_event_batch_set(stmdev_ctx_t *ctx,
                                          lsm6dsox_trig_counter_bdr_t val);
int32_t lsm6dsox_fifo_cnt_event_batch_get(stmdev_ctx_t *ctx,
                                          lsm6dsox_trig_counter_bdr_t *val);

int32_t lsm6dsox_rst_batch_counter_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t lsm6dsox_rst_batch_counter_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t lsm6dsox_batch_counter_threshold_set(stmdev_ctx_t *ctx,
                                             uint16_t val);
int32_t lsm6dsox_batch_counter_threshold_get(stmdev_ctx_t *ctx,
                                             uint16_t *val);

int32_t lsm6dsox_fifo_data_level_get(stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t lsm6dsox_fifo_status_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_fifo_status2_t *val);

int32_t lsm6dsox_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_fifo_sensor_tag_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_fifo_tag_t *val);

int32_t lsm6dsox_fifo_pedo_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_fifo_pedo_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_sh_batch_slave_0_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_batch_slave_0_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsox_sh_batch_slave_1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_batch_slave_1_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsox_sh_batch_slave_2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_batch_slave_2_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsox_sh_batch_slave_3_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_batch_slave_3_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LSM6DSOX_DEN_DISABLE    = 0,
  LSM6DSOX_LEVEL_FIFO     = 6,
  LSM6DSOX_LEVEL_LETCHED  = 3,
  LSM6DSOX_LEVEL_TRIGGER  = 2,
  LSM6DSOX_EDGE_TRIGGER   = 4,
} lsm6dsox_den_mode_t;
int32_t lsm6dsox_den_mode_set(stmdev_ctx_t *ctx,
                              lsm6dsox_den_mode_t val);
int32_t lsm6dsox_den_mode_get(stmdev_ctx_t *ctx,
                              lsm6dsox_den_mode_t *val);

typedef enum
{
  LSM6DSOX_DEN_ACT_LOW  = 0,
  LSM6DSOX_DEN_ACT_HIGH = 1,
} lsm6dsox_den_lh_t;
int32_t lsm6dsox_den_polarity_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_den_lh_t val);
int32_t lsm6dsox_den_polarity_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_den_lh_t *val);

typedef enum
{
  LSM6DSOX_STAMP_IN_GY_DATA     = 0,
  LSM6DSOX_STAMP_IN_XL_DATA     = 1,
  LSM6DSOX_STAMP_IN_GY_XL_DATA  = 2,
} lsm6dsox_den_xl_g_t;
int32_t lsm6dsox_den_enable_set(stmdev_ctx_t *ctx,
                                lsm6dsox_den_xl_g_t val);
int32_t lsm6dsox_den_enable_get(stmdev_ctx_t *ctx,
                                lsm6dsox_den_xl_g_t *val);

int32_t lsm6dsox_den_mark_axis_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_den_mark_axis_x_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_den_mark_axis_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_den_mark_axis_y_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_den_mark_axis_z_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_den_mark_axis_z_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_PEDO_BASE_MODE            = 0x00,
  LSM6DSOX_FALSE_STEP_REJ            = 0x10,
  LSM6DSOX_FALSE_STEP_REJ_ADV_MODE   = 0x30,
} lsm6dsox_pedo_md_t;
int32_t lsm6dsox_pedo_sens_set(stmdev_ctx_t *ctx,
                               lsm6dsox_pedo_md_t val);
int32_t lsm6dsox_pedo_sens_get(stmdev_ctx_t *ctx,
                               lsm6dsox_pedo_md_t *val);

int32_t lsm6dsox_pedo_step_detect_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lsm6dsox_pedo_debounce_steps_set(stmdev_ctx_t *ctx,
                                         uint8_t *buff);
int32_t lsm6dsox_pedo_debounce_steps_get(stmdev_ctx_t *ctx,
                                         uint8_t *buff);

int32_t lsm6dsox_pedo_steps_period_set(stmdev_ctx_t *ctx,
                                       uint16_t val);
int32_t lsm6dsox_pedo_steps_period_get(stmdev_ctx_t *ctx,
                                       uint16_t *val);

int32_t lsm6dsox_pedo_adv_detection_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lsm6dsox_pedo_adv_detection_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm6dsox_pedo_false_step_rejection_set(stmdev_ctx_t *ctx,
                                               uint8_t val);
int32_t lsm6dsox_pedo_false_step_rejection_get(stmdev_ctx_t *ctx,
                                               uint8_t *val);

typedef enum
{
  LSM6DSOX_EVERY_STEP     = 0,
  LSM6DSOX_COUNT_OVERFLOW = 1,
} lsm6dsox_carry_count_en_t;
int32_t lsm6dsox_pedo_int_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_carry_count_en_t val);
int32_t lsm6dsox_pedo_int_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_carry_count_en_t *val);

int32_t lsm6dsox_motion_flag_data_ready_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm6dsox_tilt_flag_data_ready_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

int32_t lsm6dsox_sh_mag_sensitivity_set(stmdev_ctx_t *ctx,
                                        uint16_t val);
int32_t lsm6dsox_sh_mag_sensitivity_get(stmdev_ctx_t *ctx,
                                        uint16_t *val);

int32_t lsm6dsox_mlc_mag_sensitivity_set(stmdev_ctx_t *ctx,
                                         uint16_t val);
int32_t lsm6dsox_mlc_mag_sensitivity_get(stmdev_ctx_t *ctx,
                                         uint16_t *val);

int32_t lsm6dsox_mag_offset_set(stmdev_ctx_t *ctx, int16_t *val);
int32_t lsm6dsox_mag_offset_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm6dsox_mag_soft_iron_set(stmdev_ctx_t *ctx, uint16_t *val);
int32_t lsm6dsox_mag_soft_iron_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum
{
  LSM6DSOX_Z_EQ_Y     = 0,
  LSM6DSOX_Z_EQ_MIN_Y = 1,
  LSM6DSOX_Z_EQ_X     = 2,
  LSM6DSOX_Z_EQ_MIN_X = 3,
  LSM6DSOX_Z_EQ_MIN_Z = 4,
  LSM6DSOX_Z_EQ_Z     = 5,
} lsm6dsox_mag_z_axis_t;
int32_t lsm6dsox_mag_z_orient_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_z_axis_t val);
int32_t lsm6dsox_mag_z_orient_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_z_axis_t *val);

typedef enum
{
  LSM6DSOX_Y_EQ_Y     = 0,
  LSM6DSOX_Y_EQ_MIN_Y = 1,
  LSM6DSOX_Y_EQ_X     = 2,
  LSM6DSOX_Y_EQ_MIN_X = 3,
  LSM6DSOX_Y_EQ_MIN_Z = 4,
  LSM6DSOX_Y_EQ_Z     = 5,
} lsm6dsox_mag_y_axis_t;
int32_t lsm6dsox_mag_y_orient_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_y_axis_t val);
int32_t lsm6dsox_mag_y_orient_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_y_axis_t *val);

typedef enum
{
  LSM6DSOX_X_EQ_Y     = 0,
  LSM6DSOX_X_EQ_MIN_Y = 1,
  LSM6DSOX_X_EQ_X     = 2,
  LSM6DSOX_X_EQ_MIN_X = 3,
  LSM6DSOX_X_EQ_MIN_Z = 4,
  LSM6DSOX_X_EQ_Z     = 5,
} lsm6dsox_mag_x_axis_t;
int32_t lsm6dsox_mag_x_orient_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_x_axis_t val);
int32_t lsm6dsox_mag_x_orient_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_mag_x_axis_t *val);

int32_t lsm6dsox_long_cnt_flag_data_ready_get(stmdev_ctx_t *ctx,
                                              uint8_t *val);

typedef struct
{
  lsm6dsox_fsm_enable_a_t          fsm_enable_a;
  lsm6dsox_fsm_enable_b_t          fsm_enable_b;
} lsm6dsox_emb_fsm_enable_t;
int32_t lsm6dsox_fsm_enable_set(stmdev_ctx_t *ctx,
                                lsm6dsox_emb_fsm_enable_t *val);
int32_t lsm6dsox_fsm_enable_get(stmdev_ctx_t *ctx,
                                lsm6dsox_emb_fsm_enable_t *val);

int32_t lsm6dsox_long_cnt_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsox_long_cnt_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum
{
  LSM6DSOX_LC_NORMAL     = 0,
  LSM6DSOX_LC_CLEAR      = 1,
  LSM6DSOX_LC_CLEAR_DONE = 2,
} lsm6dsox_fsm_lc_clr_t;
int32_t lsm6dsox_long_clr_set(stmdev_ctx_t *ctx,
                              lsm6dsox_fsm_lc_clr_t val);
int32_t lsm6dsox_long_clr_get(stmdev_ctx_t *ctx,
                              lsm6dsox_fsm_lc_clr_t *val);

typedef struct
{
  lsm6dsox_fsm_outs1_t    fsm_outs1;
  lsm6dsox_fsm_outs2_t    fsm_outs2;
  lsm6dsox_fsm_outs3_t    fsm_outs3;
  lsm6dsox_fsm_outs4_t    fsm_outs4;
  lsm6dsox_fsm_outs5_t    fsm_outs5;
  lsm6dsox_fsm_outs6_t    fsm_outs6;
  lsm6dsox_fsm_outs7_t    fsm_outs7;
  lsm6dsox_fsm_outs8_t    fsm_outs8;
  lsm6dsox_fsm_outs1_t    fsm_outs9;
  lsm6dsox_fsm_outs2_t    fsm_outs10;
  lsm6dsox_fsm_outs3_t    fsm_outs11;
  lsm6dsox_fsm_outs4_t    fsm_outs12;
  lsm6dsox_fsm_outs5_t    fsm_outs13;
  lsm6dsox_fsm_outs6_t    fsm_outs14;
  lsm6dsox_fsm_outs7_t    fsm_outs15;
  lsm6dsox_fsm_outs8_t    fsm_outs16;
} lsm6dsox_fsm_out_t;
int32_t lsm6dsox_fsm_out_get(stmdev_ctx_t *ctx,
                             lsm6dsox_fsm_out_t *val);

typedef enum
{
  LSM6DSOX_ODR_FSM_12Hz5 = 0,
  LSM6DSOX_ODR_FSM_26Hz  = 1,
  LSM6DSOX_ODR_FSM_52Hz  = 2,
  LSM6DSOX_ODR_FSM_104Hz = 3,
} lsm6dsox_fsm_odr_t;
int32_t lsm6dsox_fsm_data_rate_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_fsm_odr_t val);
int32_t lsm6dsox_fsm_data_rate_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_fsm_odr_t *val);

int32_t lsm6dsox_fsm_init_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_fsm_init_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_long_cnt_int_value_set(stmdev_ctx_t *ctx,
                                        uint16_t val);
int32_t lsm6dsox_long_cnt_int_value_get(stmdev_ctx_t *ctx,
                                        uint16_t *val);

int32_t lsm6dsox_fsm_number_of_programs_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm6dsox_fsm_number_of_programs_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm6dsox_fsm_start_address_set(stmdev_ctx_t *ctx,
                                       uint16_t val);
int32_t lsm6dsox_fsm_start_address_get(stmdev_ctx_t *ctx,
                                       uint16_t *val);

int32_t lsm6dsox_mlc_status_get(stmdev_ctx_t *ctx,
                                lsm6dsox_mlc_status_mainpage_t *val);

typedef enum
{
  LSM6DSOX_ODR_PRGS_12Hz5 = 0,
  LSM6DSOX_ODR_PRGS_26Hz  = 1,
  LSM6DSOX_ODR_PRGS_52Hz  = 2,
  LSM6DSOX_ODR_PRGS_104Hz = 3,
} lsm6dsox_mlc_odr_t;
int32_t lsm6dsox_mlc_data_rate_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_mlc_odr_t val);
int32_t lsm6dsox_mlc_data_rate_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_mlc_odr_t *val);

typedef struct
{
  lsm6dsox_sensor_hub_1_t   sh_byte_1;
  lsm6dsox_sensor_hub_2_t   sh_byte_2;
  lsm6dsox_sensor_hub_3_t   sh_byte_3;
  lsm6dsox_sensor_hub_4_t   sh_byte_4;
  lsm6dsox_sensor_hub_5_t   sh_byte_5;
  lsm6dsox_sensor_hub_6_t   sh_byte_6;
  lsm6dsox_sensor_hub_7_t   sh_byte_7;
  lsm6dsox_sensor_hub_8_t   sh_byte_8;
  lsm6dsox_sensor_hub_9_t   sh_byte_9;
  lsm6dsox_sensor_hub_10_t  sh_byte_10;
  lsm6dsox_sensor_hub_11_t  sh_byte_11;
  lsm6dsox_sensor_hub_12_t  sh_byte_12;
  lsm6dsox_sensor_hub_13_t  sh_byte_13;
  lsm6dsox_sensor_hub_14_t  sh_byte_14;
  lsm6dsox_sensor_hub_15_t  sh_byte_15;
  lsm6dsox_sensor_hub_16_t  sh_byte_16;
  lsm6dsox_sensor_hub_17_t  sh_byte_17;
  lsm6dsox_sensor_hub_18_t  sh_byte_18;
} lsm6dsox_emb_sh_read_t;
int32_t lsm6dsox_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_emb_sh_read_t *val,
                                      uint8_t len);

typedef enum
{
  LSM6DSOX_SLV_0       = 0,
  LSM6DSOX_SLV_0_1     = 1,
  LSM6DSOX_SLV_0_1_2   = 2,
  LSM6DSOX_SLV_0_1_2_3 = 3,
} lsm6dsox_aux_sens_on_t;
int32_t lsm6dsox_sh_slave_connected_set(stmdev_ctx_t *ctx,
                                        lsm6dsox_aux_sens_on_t val);
int32_t lsm6dsox_sh_slave_connected_get(stmdev_ctx_t *ctx,
                                        lsm6dsox_aux_sens_on_t *val);

int32_t lsm6dsox_sh_master_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_EXT_PULL_UP      = 0,
  LSM6DSOX_INTERNAL_PULL_UP = 1,
} lsm6dsox_shub_pu_en_t;
int32_t lsm6dsox_sh_pin_mode_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_shub_pu_en_t val);
int32_t lsm6dsox_sh_pin_mode_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_shub_pu_en_t *val);

int32_t lsm6dsox_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_sh_pass_through_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_EXT_ON_INT2_PIN = 1,
  LSM6DSOX_XL_GY_DRDY      = 0,
} lsm6dsox_start_config_t;
int32_t lsm6dsox_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_start_config_t val);
int32_t lsm6dsox_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_start_config_t *val);

typedef enum
{
  LSM6DSOX_EACH_SH_CYCLE    = 0,
  LSM6DSOX_ONLY_FIRST_CYCLE = 1,
} lsm6dsox_write_once_t;
int32_t lsm6dsox_sh_write_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_write_once_t val);
int32_t lsm6dsox_sh_write_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_write_once_t *val);

int32_t lsm6dsox_sh_reset_set(stmdev_ctx_t *ctx);
int32_t lsm6dsox_sh_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LSM6DSOX_SH_ODR_104Hz = 0,
  LSM6DSOX_SH_ODR_52Hz  = 1,
  LSM6DSOX_SH_ODR_26Hz  = 2,
  LSM6DSOX_SH_ODR_13Hz  = 3,
} lsm6dsox_shub_odr_t;
int32_t lsm6dsox_sh_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_shub_odr_t val);
int32_t lsm6dsox_sh_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_shub_odr_t *val);

typedef struct
{
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} lsm6dsox_sh_cfg_write_t;
int32_t lsm6dsox_sh_cfg_write(stmdev_ctx_t *ctx,
                              lsm6dsox_sh_cfg_write_t *val);

typedef struct
{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dsox_sh_cfg_read_t;
int32_t lsm6dsox_sh_slv0_cfg_read(stmdev_ctx_t *ctx,
                                  lsm6dsox_sh_cfg_read_t *val);
int32_t lsm6dsox_sh_slv1_cfg_read(stmdev_ctx_t *ctx,
                                  lsm6dsox_sh_cfg_read_t *val);
int32_t lsm6dsox_sh_slv2_cfg_read(stmdev_ctx_t *ctx,
                                  lsm6dsox_sh_cfg_read_t *val);
int32_t lsm6dsox_sh_slv3_cfg_read(stmdev_ctx_t *ctx,
                                  lsm6dsox_sh_cfg_read_t *val);

int32_t lsm6dsox_sh_status_get(stmdev_ctx_t *ctx,
                               lsm6dsox_status_master_t *val);
typedef enum
{
  LSM6DSOX_S4S_TPH_7bit   = 0,
  LSM6DSOX_S4S_TPH_15bit  = 1,
} lsm6dsox_s4s_tph_res_t;
int32_t lsm6dsox_s4s_tph_res_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_s4s_tph_res_t val);
int32_t lsm6dsox_s4s_tph_res_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_s4s_tph_res_t *val);

int32_t lsm6dsox_s4s_tph_val_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm6dsox_s4s_tph_val_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum
{
  LSM6DSOX_S4S_DT_RES_11 = 0,
  LSM6DSOX_S4S_DT_RES_12 = 1,
  LSM6DSOX_S4S_DT_RES_13 = 2,
  LSM6DSOX_S4S_DT_RES_14 = 3,
} lsm6dsox_s4s_res_ratio_t;
int32_t lsm6dsox_s4s_res_ratio_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_s4s_res_ratio_t val);
int32_t lsm6dsox_s4s_res_ratio_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_s4s_res_ratio_t *val);

int32_t lsm6dsox_s4s_command_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_s4s_command_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsox_s4s_dt_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm6dsox_s4s_dt_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  uint8_t ui;
  uint8_t aux;
} lsm6dsox_id_t;
int32_t lsm6dsox_id_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                        lsm6dsox_id_t *val);

typedef struct
{
  enum
  {
    LSM6DSOX_SEL_BY_HW   = 0x00, /* bus mode select by HW (SPI 3W disable) */
    LSM6DSOX_SPI_4W      = 0x06, /* Only SPI: SDO / SDI separated pins */
    LSM6DSOX_SPI_3W      = 0x07, /* Only SPI: SDO / SDI share the same pin */
    LSM6DSOX_I2C         = 0x04, /* Only I2C */
    LSM6DSOX_I3C_T_50us  = 0x02, /* I3C: available time equal to 50 Î¼s */
    LSM6DSOX_I3C_T_2us   = 0x12, /* I3C: available time equal to 2 Î¼s */
    LSM6DSOX_I3C_T_1ms   = 0x22, /* I3C: available time equal to 1 ms */
    LSM6DSOX_I3C_T_25ms  = 0x32, /* I3C: available time equal to 25 ms */
  } ui_bus_md;
  enum
  {
    LSM6DSOX_SPI_4W_AUX  = 0x00,
    LSM6DSOX_SPI_3W_AUX  = 0x01,
  } aux_bus_md;
} lsm6dsox_bus_mode_t;
int32_t lsm6dsox_bus_mode_set(stmdev_ctx_t *ctx,
                              stmdev_ctx_t *aux_ctx,
                              lsm6dsox_bus_mode_t val);
int32_t lsm6dsox_bus_mode_get(stmdev_ctx_t *ctx,
                              stmdev_ctx_t *aux_ctx,
                              lsm6dsox_bus_mode_t *val);

typedef enum
{
  LSM6DSOX_DRV_RDY   = 0x00, /* Initialize the device for driver usage */
  LSM6DSOX_BOOT      = 0x01, /* Restore calib. param. ( it takes 10ms ) */
  LSM6DSOX_RESET     = 0x02, /* Reset configuration registers */
  LSM6DSOX_FIFO_COMP = 0x04, /* FIFO compression initialization request. */
  LSM6DSOX_FSM       = 0x08, /* Finite State Machine initialization request */
  LSM6DSOX_MLC       = 0x10, /* Machine Learning Core initialization request */
  LSM6DSOX_PEDO      = 0x20, /* Pedometer algo initialization request. */
  LSM6DSOX_TILT      = 0x40, /* Tilt algo initialization request */
  LSM6DSOX_SMOTION   = 0x80, /* Significant Motion initialization request */
} lsm6dsox_init_t;
int32_t lsm6dsox_init_set(stmdev_ctx_t *ctx, lsm6dsox_init_t val);

typedef struct
{
uint8_t sw_reset           :
  1; /* Restoring configuration registers */
  uint8_t boot               : 1; /* Restoring calibration parameters */
  uint8_t drdy_xl            : 1; /* Accelerometer data ready */
  uint8_t drdy_g             : 1; /* Gyroscope data ready */
  uint8_t drdy_temp          : 1; /* Temperature data ready */
  uint8_t ois_drdy_xl        : 1; /* Accelerometer data ready on OIS */
  uint8_t ois_drdy_g         : 1; /* Gyroscope data ready on OIS */
uint8_t ois_gyro_settling  :
  1; /* Gyroscope is in the settling phase */
} lsm6dsox_status_t;
int32_t lsm6dsox_status_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                            lsm6dsox_status_t *val);

typedef struct
{
  uint8_t sdo_sa0_pull_up     : 1; /* 1 = pull-up on SDO/SA0 pin */
uint8_t aux_sdo_ocs_pull_up :
  1; /* 1 = pull-up on OCS_Aux/SDO_Aux pins */
  uint8_t int1_int2_push_pull : 1; /* 1 = push-pull / 0 = open-drain*/
uint8_t int1_pull_down      :
  1; /* 1 = pull-down always disabled (0=auto) */
} lsm6dsox_pin_conf_t;
int32_t lsm6dsox_pin_conf_set(stmdev_ctx_t *ctx,
                              lsm6dsox_pin_conf_t val);
int32_t lsm6dsox_pin_conf_get(stmdev_ctx_t *ctx,
                              lsm6dsox_pin_conf_t *val);

typedef struct
{
  uint8_t active_low   : 1; /* 1 = active low / 0 = active high */
uint8_t base_latched :
  1; /* base functions are: FF, WU, 6D, Tap, Act/Inac */
uint8_t emb_latched  :
  1; /* emb functions are: Pedo, Tilt, SMot, Timestamp */
} lsm6dsox_int_mode_t;
int32_t lsm6dsox_interrupt_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_int_mode_t val);
int32_t lsm6dsox_interrupt_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_int_mode_t *val);

typedef struct
{
  uint8_t drdy_xl       : 1; /* Accelerometer data ready */
  uint8_t drdy_g        : 1; /* Gyroscope data ready */
uint8_t drdy_temp     :
  1; /* Temperature data ready (1 = int2 pin disable) */
  uint8_t boot          : 1; /* Restoring calibration parameters */
  uint8_t fifo_th       : 1; /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1; /* FIFO overrun */
  uint8_t fifo_full     : 1; /* FIFO full */
  uint8_t fifo_bdr      : 1; /* FIFO Batch counter threshold reached */
uint8_t den_flag      :
  1; /* external trigger level recognition (DEN) */
  uint8_t sh_endop      : 1; /* sensor hub end operation */
uint8_t timestamp     :
  1; /* timestamp overflow (1 = int2 pin disable) */
  uint8_t six_d         : 1; /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1; /* double-tap event */
  uint8_t free_fall     : 1; /* free fall event */
  uint8_t wake_up       : 1; /* wake up event */
  uint8_t single_tap    : 1; /* single-tap event */
uint8_t sleep_change  :
  1; /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1; /* Step detected */
  uint8_t tilt          : 1; /* Relative tilt event detected */
  uint8_t sig_mot       : 1; /* "significant motion" event detected */
uint8_t fsm_lc        :
  1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1; /* fsm 1 interrupt event */
  uint8_t fsm2          : 1; /* fsm 2 interrupt event */
  uint8_t fsm3          : 1; /* fsm 3 interrupt event */
  uint8_t fsm4          : 1; /* fsm 4 interrupt event */
  uint8_t fsm5          : 1; /* fsm 5 interrupt event */
  uint8_t fsm6          : 1; /* fsm 6 interrupt event */
  uint8_t fsm7          : 1; /* fsm 7 interrupt event */
  uint8_t fsm8          : 1; /* fsm 8 interrupt event */
  uint8_t fsm9          : 1; /* fsm 9 interrupt event */
  uint8_t fsm10         : 1; /* fsm 10 interrupt event */
  uint8_t fsm11         : 1; /* fsm 11 interrupt event */
  uint8_t fsm12         : 1; /* fsm 12 interrupt event */
  uint8_t fsm13         : 1; /* fsm 13 interrupt event */
  uint8_t fsm14         : 1; /* fsm 14 interrupt event */
  uint8_t fsm15         : 1; /* fsm 15 interrupt event */
  uint8_t fsm16         : 1; /* fsm 16 interrupt event */
  uint8_t mlc1          : 1; /* mlc 1 interrupt event */
  uint8_t mlc2          : 1; /* mlc 2 interrupt event */
  uint8_t mlc3          : 1; /* mlc 3 interrupt event */
  uint8_t mlc4          : 1; /* mlc 4 interrupt event */
  uint8_t mlc5          : 1; /* mlc 5 interrupt event */
  uint8_t mlc6          : 1; /* mlc 6 interrupt event */
  uint8_t mlc7          : 1; /* mlc 7 interrupt event */
  uint8_t mlc8          : 1; /* mlc 8 interrupt event */
} lsm6dsox_pin_int1_route_t;

int32_t lsm6dsox_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_pin_int1_route_t val);
int32_t lsm6dsox_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_pin_int1_route_t *val);

typedef struct
{
  uint8_t drdy_ois      : 1; /* OIS chain data ready */
  uint8_t drdy_xl       : 1; /* Accelerometer data ready */
  uint8_t drdy_g        : 1; /* Gyroscope data ready */
  uint8_t drdy_temp     : 1; /* Temperature data ready */
  uint8_t fifo_th       : 1; /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1; /* FIFO overrun */
  uint8_t fifo_full     : 1; /* FIFO full */
  uint8_t fifo_bdr      : 1; /* FIFO Batch counter threshold reached */
  uint8_t timestamp     : 1; /* timestamp overflow */
  uint8_t six_d         : 1; /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1; /* double-tap event */
  uint8_t free_fall     : 1; /* free fall event */
  uint8_t wake_up       : 1; /* wake up event */
  uint8_t single_tap    : 1; /* single-tap event */
uint8_t sleep_change  :
  1; /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1; /* Step detected */
  uint8_t tilt          : 1; /* Relative tilt event detected */
  uint8_t sig_mot       : 1; /* "significant motion" event detected */
uint8_t fsm_lc        :
  1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1; /* fsm 1 interrupt event */
  uint8_t fsm2          : 1; /* fsm 2 interrupt event */
  uint8_t fsm3          : 1; /* fsm 3 interrupt event */
  uint8_t fsm4          : 1; /* fsm 4 interrupt event */
  uint8_t fsm5          : 1; /* fsm 5 interrupt event */
  uint8_t fsm6          : 1; /* fsm 6 interrupt event */
  uint8_t fsm7          : 1; /* fsm 7 interrupt event */
  uint8_t fsm8          : 1; /* fsm 8 interrupt event */
  uint8_t fsm9          : 1; /* fsm 9 interrupt event */
  uint8_t fsm10         : 1; /* fsm 10 interrupt event */
  uint8_t fsm11         : 1; /* fsm 11 interrupt event */
  uint8_t fsm12         : 1; /* fsm 12 interrupt event */
  uint8_t fsm13         : 1; /* fsm 13 interrupt event */
  uint8_t fsm14         : 1; /* fsm 14 interrupt event */
  uint8_t fsm15         : 1; /* fsm 15 interrupt event */
  uint8_t fsm16         : 1; /* fsm 16 interrupt event */
  uint8_t mlc1          : 1; /* mlc 1 interrupt event */
  uint8_t mlc2          : 1; /* mlc 2 interrupt event */
  uint8_t mlc3          : 1; /* mlc 3 interrupt event */
  uint8_t mlc4          : 1; /* mlc 4 interrupt event */
  uint8_t mlc5          : 1; /* mlc 5 interrupt event */
  uint8_t mlc6          : 1; /* mlc 6 interrupt event */
  uint8_t mlc7          : 1; /* mlc 7 interrupt event */
  uint8_t mlc8          : 1; /* mlc 8 interrupt event */
} lsm6dsox_pin_int2_route_t;

int32_t lsm6dsox_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    stmdev_ctx_t *aux_ctx,
                                    lsm6dsox_pin_int2_route_t val);
int32_t lsm6dsox_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    stmdev_ctx_t *aux_ctx,
                                    lsm6dsox_pin_int2_route_t *val);

typedef struct
{
  uint8_t drdy_xl          :  1; /* Accelerometer data ready */
  uint8_t drdy_g           :  1; /* Gyroscope data ready */
  uint8_t drdy_temp        :  1; /* Temperature data ready */
uint8_t den_flag         :
  1; /* external trigger level recognition (DEN) */
uint8_t timestamp        :
  1; /* timestamp overflow (1 = int2 pin disable) */
  uint8_t free_fall        :  1; /* free fall event */
  uint8_t wake_up          :  1; /* wake up event */
  uint8_t wake_up_z        :  1; /* wake up on Z axis event */
  uint8_t wake_up_y        :  1; /* wake up on Y axis event */
  uint8_t wake_up_x        :  1; /* wake up on X axis event */
  uint8_t single_tap       :  1; /* single-tap event */
  uint8_t double_tap       :  1; /* double-tap event */
  uint8_t tap_z            :  1; /* single-tap on Z axis event */
  uint8_t tap_y            :  1; /* single-tap on Y axis event */
  uint8_t tap_x            :  1; /* single-tap on X axis event */
  uint8_t tap_sign         :  1; /* sign of tap event (0-pos / 1-neg) */
uint8_t six_d            :
  1; /* orientation change (6D/4D detection) */
uint8_t six_d_xl         :
  1; /* X-axis low 6D/4D event (under threshold) */
uint8_t six_d_xh         :
  1; /* X-axis high 6D/4D event (over threshold) */
uint8_t six_d_yl         :
  1; /* Y-axis low 6D/4D event (under threshold) */
uint8_t six_d_yh         :
  1; /* Y-axis high 6D/4D event (over threshold) */
uint8_t six_d_zl         :
  1; /* Z-axis low 6D/4D event (under threshold) */
uint8_t six_d_zh         :
  1; /* Z-axis high 6D/4D event (over threshold) */
uint8_t sleep_change     :
  1; /* Act/Inact (or Vice-versa) status changed */
uint8_t sleep_state      :
  1; /* Act/Inact status flag (0-Act / 1-Inact) */
  uint8_t step_detector    :  1; /* Step detected */
  uint8_t tilt             :  1; /* Relative tilt event detected */
uint8_t sig_mot          :
  1; /* "significant motion" event detected */
uint8_t fsm_lc           :
  1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1             :  1; /* fsm 1 interrupt event */
  uint8_t fsm2             :  1; /* fsm 2 interrupt event */
  uint8_t fsm3             :  1; /* fsm 3 interrupt event */
  uint8_t fsm4             :  1; /* fsm 4 interrupt event */
  uint8_t fsm5             :  1; /* fsm 5 interrupt event */
  uint8_t fsm6             :  1; /* fsm 6 interrupt event */
  uint8_t fsm7             :  1; /* fsm 7 interrupt event */
  uint8_t fsm8             :  1; /* fsm 8 interrupt event */
  uint8_t fsm9             :  1; /* fsm 9 interrupt event */
  uint8_t fsm10            :  1; /* fsm 10 interrupt event */
  uint8_t fsm11            :  1; /* fsm 11 interrupt event */
  uint8_t fsm12            :  1; /* fsm 12 interrupt event */
  uint8_t fsm13            :  1; /* fsm 13 interrupt event */
  uint8_t fsm14            :  1; /* fsm 14 interrupt event */
  uint8_t fsm15            :  1; /* fsm 15 interrupt event */
  uint8_t fsm16            :  1; /* fsm 16 interrupt event */
  uint8_t mlc1             :  1; /* mlc 1 interrupt event */
  uint8_t mlc2             :  1; /* mlc 2 interrupt event */
  uint8_t mlc3             :  1; /* mlc 3 interrupt event */
  uint8_t mlc4             :  1; /* mlc 4 interrupt event */
  uint8_t mlc5             :  1; /* mlc 5 interrupt event */
  uint8_t mlc6             :  1; /* mlc 6 interrupt event */
  uint8_t mlc7             :  1; /* mlc 7 interrupt event */
  uint8_t mlc8             :  1; /* mlc 8 interrupt event */
  uint8_t sh_endop         :  1; /* sensor hub end operation */
uint8_t sh_slave0_nack   :
  1; /* Not acknowledge on sensor hub slave 0 */
uint8_t sh_slave1_nack   :
  1; /* Not acknowledge on sensor hub slave 1 */
uint8_t sh_slave2_nack   :
  1; /* Not acknowledge on sensor hub slave 2 */
uint8_t sh_slave3_nack   :
  1; /* Not acknowledge on sensor hub slave 3 */
uint8_t sh_wr_once       :
  1; /* "WRITE_ONCE" end on sensor hub slave 0 */
uint16_t fifo_diff       :
  10; /* Number of unread sensor data in FIFO*/
 uint8_t fifo_ovr_latched :
  1; /* Latched FIFO overrun status */
uint8_t fifo_bdr         :
  1; /* FIFO Batch counter threshold reached */
  uint8_t fifo_full        :  1; /* FIFO full */
  uint8_t fifo_ovr         :  1; /* FIFO overrun */
  uint8_t fifo_th          :  1; /* FIFO threshold reached */
} lsm6dsox_all_sources_t;
int32_t lsm6dsox_all_sources_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_all_sources_t *val);

typedef struct
{
  uint8_t odr_fine_tune;
} lsm6dsox_dev_cal_t;
int32_t lsm6dsox_calibration_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_dev_cal_t *val);

typedef struct
{
  struct
  {
    struct
    {
      enum
      {
        LSM6DSOX_XL_UI_OFF       = 0x00, /* in power down */
        LSM6DSOX_XL_UI_1Hz6_LP   = 0x1B, /* @1Hz6 (low power) */
        LSM6DSOX_XL_UI_1Hz6_ULP  = 0x2B, /* @1Hz6 (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_12Hz5_HP  = 0x01, /* @12Hz5 (high performance) */
        LSM6DSOX_XL_UI_12Hz5_LP  = 0x11, /* @12Hz5 (low power) */
        LSM6DSOX_XL_UI_12Hz5_ULP = 0x21, /* @12Hz5 (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_26Hz_HP   = 0x02, /* @26Hz  (high performance) */
        LSM6DSOX_XL_UI_26Hz_LP   = 0x12, /* @26Hz  (low power) */
        LSM6DSOX_XL_UI_26Hz_ULP  = 0x22, /* @26Hz  (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_52Hz_HP   = 0x03, /* @52Hz  (high performance) */
        LSM6DSOX_XL_UI_52Hz_LP   = 0x13, /* @52Hz  (low power) */
        LSM6DSOX_XL_UI_52Hz_ULP  = 0x23, /* @52Hz  (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_104Hz_HP  = 0x04, /* @104Hz (high performance) */
        LSM6DSOX_XL_UI_104Hz_NM  = 0x14, /* @104Hz (normal mode) */
        LSM6DSOX_XL_UI_104Hz_ULP = 0x24, /* @104Hz (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_208Hz_HP  = 0x05, /* @208Hz (high performance) */
        LSM6DSOX_XL_UI_208Hz_NM  = 0x15, /* @208Hz (normal mode) */
        LSM6DSOX_XL_UI_208Hz_ULP = 0x25, /* @208Hz (ultra low/Gy, OIS imu off) */
        LSM6DSOX_XL_UI_416Hz_HP  = 0x06, /* @416Hz (high performance) */
        LSM6DSOX_XL_UI_833Hz_HP  = 0x07, /* @833Hz (high performance) */
        LSM6DSOX_XL_UI_1667Hz_HP = 0x08, /* @1kHz66 (high performance) */
        LSM6DSOX_XL_UI_3333Hz_HP = 0x09, /* @3kHz33 (high performance) */
        LSM6DSOX_XL_UI_6667Hz_HP = 0x0A, /* @6kHz66 (high performance) */
      } odr;
      enum
      {
        LSM6DSOX_XL_UI_2g   = 0,
        LSM6DSOX_XL_UI_4g   = 2,
        LSM6DSOX_XL_UI_8g   = 3,
        LSM6DSOX_XL_UI_16g  = 1, /* OIS full scale is also forced to be 16g */
      } fs;
    } xl;
    struct
    {
      enum
      {
        LSM6DSOX_GY_UI_OFF       = 0x00, /* gy in power down */
        LSM6DSOX_GY_UI_12Hz5_LP  = 0x11, /* gy @12Hz5 (low power) */
        LSM6DSOX_GY_UI_12Hz5_HP  = 0x01, /* gy @12Hz5 (high performance) */
        LSM6DSOX_GY_UI_26Hz_LP   = 0x12, /* gy @26Hz  (low power) */
        LSM6DSOX_GY_UI_26Hz_HP   = 0x02, /* gy @26Hz  (high performance) */
        LSM6DSOX_GY_UI_52Hz_LP   = 0x13, /* gy @52Hz  (low power) */
        LSM6DSOX_GY_UI_52Hz_HP   = 0x03, /* gy @52Hz  (high performance) */
        LSM6DSOX_GY_UI_104Hz_NM  = 0x14, /* gy @104Hz (low power) */
        LSM6DSOX_GY_UI_104Hz_HP  = 0x04, /* gy @104Hz (high performance) */
        LSM6DSOX_GY_UI_208Hz_NM  = 0x15, /* gy @208Hz (low power) */
        LSM6DSOX_GY_UI_208Hz_HP  = 0x05, /* gy @208Hz (high performance) */
        LSM6DSOX_GY_UI_416Hz_HP  = 0x06, /* gy @416Hz (high performance) */
        LSM6DSOX_GY_UI_833Hz_HP  = 0x07, /* gy @833Hz (high performance) */
        LSM6DSOX_GY_UI_1667Hz_HP = 0x08, /* gy @1kHz66 (high performance) */
        LSM6DSOX_GY_UI_3333Hz_HP = 0x09, /* gy @3kHz33 (high performance) */
        LSM6DSOX_GY_UI_6667Hz_HP = 0x0A, /* gy @6kHz66 (high performance) */
      } odr;
      enum
      {
        LSM6DSOX_GY_UI_250dps   = 0,
        LSM6DSOX_GY_UI_125dps   = 1,
        LSM6DSOX_GY_UI_500dps   = 2,
        LSM6DSOX_GY_UI_1000dps  = 4,
        LSM6DSOX_GY_UI_2000dps  = 6,
      } fs;
    } gy;
  } ui;
  struct
  {
    enum
    {
      LSM6DSOX_OIS_ONLY_AUX    = 0x00, /* Auxiliary SPI full control */
      LSM6DSOX_OIS_ONLY_UI     = 0x02, /* Primary interface full control */
      LSM6DSOX_OIS_MIXED       = 0x01, /* Enabling by UI / read-config by AUX */
    } ctrl_md;
    struct
    {
      enum
      {
        LSM6DSOX_XL_OIS_OFF       = 0x00, /* in power down */
        LSM6DSOX_XL_OIS_6667Hz_HP = 0x01, /* @6kHz OIS imu active/NO ULP on UI */
      } odr;
      enum
      {
        LSM6DSOX_XL_OIS_2g   = 0,
        LSM6DSOX_XL_OIS_4g   = 2,
        LSM6DSOX_XL_OIS_8g   = 3,
        LSM6DSOX_XL_OIS_16g  = 1, /* UI full scale is also forced to be 16g */
      } fs;
    } xl;
    struct
    {
      enum
      {
        LSM6DSOX_GY_OIS_OFF       = 0x00, /* in power down */
        LSM6DSOX_GY_OIS_6667Hz_HP = 0x01, /* @6kHz No Ultra Low Power*/
      } odr;
      enum
      {
        LSM6DSOX_GY_OIS_250dps   = 0,
        LSM6DSOX_GY_OIS_125dps   = 1,
        LSM6DSOX_GY_OIS_500dps   = 2,
        LSM6DSOX_GY_OIS_1000dps  = 4,
        LSM6DSOX_GY_OIS_2000dps  = 6,
      } fs;
    } gy;
  } ois;
  struct
  {
    enum
    {
      LSM6DSOX_FSM_DISABLE = 0x00,
      LSM6DSOX_FSM_XL      = 0x01,
      LSM6DSOX_FSM_GY      = 0x02,
      LSM6DSOX_FSM_XL_GY   = 0x03,
    } sens;
    enum
    {
      LSM6DSOX_FSM_12Hz5 = 0x00,
      LSM6DSOX_FSM_26Hz  = 0x01,
      LSM6DSOX_FSM_52Hz  = 0x02,
      LSM6DSOX_FSM_104Hz = 0x03,
    } odr;
  } fsm;
  struct
  {
    enum
    {
      LSM6DSOX_MLC_DISABLE = 0x00,
      LSM6DSOX_MLC_XL      = 0x01,
      LSM6DSOX_MLC_XL_GY   = 0x03,
    } sens;
    enum
    {
      LSM6DSOX_MLC_12Hz5 = 0x00,
      LSM6DSOX_MLC_26Hz  = 0x01,
      LSM6DSOX_MLC_52Hz  = 0x02,
      LSM6DSOX_MLC_104Hz = 0x03,
    } odr;
  } mlc;
} lsm6dsox_md_t;
int32_t lsm6dsox_mode_set(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *val);
int32_t lsm6dsox_mode_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *val);

typedef struct
{
  struct
  {
    struct
    {
      float_t mg[3];
      int16_t raw[3];
    } xl;
    struct
    {
      float_t mdps[3];
      int16_t raw[3];
    } gy;
    struct
    {
      float_t deg_c;
      int16_t raw;
    } heat;
  } ui;
  struct
  {
    struct
    {
      float_t mg[3];
      int16_t raw[3];
    } xl;
    struct
    {
      float_t mdps[3];
      int16_t raw[3];
    } gy;
  } ois;
} lsm6dsox_data_t;
int32_t lsm6dsox_data_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *md, lsm6dsox_data_t *data);

typedef struct
{
  uint8_t sig_mot      : 1; /* significant motion */
  uint8_t tilt         : 1; /* tilt detection  */
  uint8_t step         : 1; /* step counter/detector */
  uint8_t mlc          : 1; /* machine learning core */
  uint8_t fsm          : 1; /* finite state machine */
  uint8_t fifo_compr   : 1; /* mlc 8 interrupt event */
} lsm6dsox_emb_sens_t;
int32_t lsm6dsox_embedded_sens_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_emb_sens_t *emb_sens);
int32_t lsm6dsox_embedded_sens_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_emb_sens_t *emb_sens);
int32_t lsm6dsox_embedded_sens_off(stmdev_ctx_t *ctx);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*LSM6DSOX_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
