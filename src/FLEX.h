#ifndef FELX_H_
#define FLEX_H_

#include <Wire.h>
#include <sys/time.h>
#include "esp_err.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"  // mask/unmask a single GPIO IRQ (IRAM-safe)e
#include "esp_log.h"

#define ADS_RESET_PIN          (15)           // Pin number attached to ads reset line.
#define ADS1_RESET_PIN          (16) 
#define ADS2_INTERRUPT_PIN     (18)           // Pin number attached to the ads data ready line.  two axis
#define ADS1_INTERRUPT_PIN     (17) 
#define I2C1_SDA                21
#define I2C1_SCL                22
#define ADS2_DEFAULT_ADDR      (0x13)			   // Default I2C address of the Two Axis

/* ---------------- Shared constants (match library) ---------------- */
#define ADS2_TRANSFER_SIZE     (5)  // 1 header + two int16s
#define ADS_COUNT              (10)

#define ADS_OK                 (0)
#define ADS_ERR                (-1)
#define ADS_ERR_BAD_PARAM      (-2)
#define ADS_ERR_OP_IN_PROGRESS (-3)
#define ADS_ERR_IO             (-4)
#define ADS_ERR_DEV_ID         (-5)
#define ADS_ERR_TIMEOUT        (-6)

#define ADS_AXIS_0_EN          (0x01)
#define ADS_AXIS_1_EN          (0x02)


extern volatile bool ADS1_dataReady;
extern float Horiz_Angle, Verti_Angle;
void ads2_data_callback(float * sample);
void ads2_hal_delay(uint16_t delay_ms);
void ads2_hal_pin_int_enable(bool enable);
static void ads2_service_irq_once();
void ads_hal_pin_int_enable(bool enable);

void deadzone_filter(float * sample);
void signal_filter(float * sample);
void parse_serial_port(void);

/* Flex Sensor variable and structures */
typedef void (*ads2_callback)(float*);



/* Command set for ADS */

typedef enum 
{
	ADS2_RUN = 0,
	ADS2_SPS,
	ADS2_RESET,
	ADS2_DFU,
	ADS2_SET_ADDRESS,
	ADS2_INTERRUPT_ENABLE,
	ADS2_GET_FW_VER,
	ADS2_CALIBRATE,
	ADS2_AXES_ENALBED,
	ADS2_SHUTDOWN,
	ADS2_GET_DEV_ID
} ADS2_COMMAND_T;
/* Identifier for packet received from ADS */

typedef enum 
{
	ADS2_SAMPLE = 0,
	ADS2_FW_VER,
	ADS2_DEV_ID
} ADS2_PACKET_T;


typedef enum 
{
	ADS2_DEV_UNKNOWN     = 0,
	ADS2_DEV_ONE_AXIS_V1 = 1,
	ADS2_DEV_TWO_AXIS_V1 = 2,
	ADS2_DEV_ONE_AXIS_V2 = 12,
	ADS2_DEV_TWO_AXIS_V2 = 22
} ADS2_DEV_TYPE_T;


typedef enum 
{
	ADS2_1_HZ   = 16384,
	ADS2_10_HZ  = 1638,
	ADS2_20_HZ  = 819,
	ADS2_50_HZ  = 327,
	ADS2_100_HZ = 163,
	ADS2_200_HZ = 81,
	ADS2_333_HZ = 49,
	ADS2_500_HZ = 32,
} ADS2_SPS_T;


/* Device IDS */
typedef enum 
{
	ADS_TWO_AXIS_1 = 1,
	ADS_TWO_AXIS = 2,
} ADS_DEV_IDS_T;



typedef enum 
{
	ADS2_CALIBRATE_FIRST = 0,		   	// First calibration point, typically 0 degrees
	ADS2_CALIBRATE_SECOND,			     	// Second calibration point, 45-255 degrees
	ADS2_CALIBRATE_CLEAR,				    // Clears user calibration, restores factory calibration
//	ADS2_CALIBRATE_STRETCH_ZERO,			// 0mm strain calibration point
//	ADS2_CALIBRATE_STRETCH_SECOND,		// Second calibration point for stretch, typically 30mm
} ADS2_CALIBRATION_STEP_T;

typedef struct 
{
  ADS2_SPS_T sps2;
	ads2_callback ads2_sample_callback;
	uint32_t reset_pin;
	uint32_t ADS2_datardy_pin;
  uint8_t  addr2;      // kannna
} ads2_init_t;

extern volatile bool _ads2_int_enabled ;


extern volatile float bend_value ;             // ADS_SAMPLE
extern volatile float stretch_value ;          // ADS_STRETCH_SAMPLE
extern volatile bool  bend_new ;
extern volatile bool  stretch_new ;


#endif