
/* inmplemented UDP and HTTP server */
#include <Adafruit_NeoPixel.h>
#include <sys/time.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include "driver/uart.h"
#include <Adafruit_NeoPixel.h>
#include <sys/time.h>
#include "esp_vfs_fat.h"
#include "FS.h"
#include "esp_err.h"
#include <queue>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"  // mask/unmask a single GPIO IRQ (IRAM-safe)e
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include <USB.h>
#include <USBMSC.h>




#define ADS_RESET_PIN          (4)           // Pin number attached to ads reset line.
#define ADS2_INTERRUPT_PIN     (5)           // Pin number attached to the ads data ready line.  two axis
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


//static (uint8_t* buffer);
static uint8_t* buffer = NULL;
void ads2_data_callback(float * sample);
void ads2_hal_delay(uint16_t delay_ms);
void ads2_hal_pin_int_enable(bool enable);
static void ads2_service_irq_once();
void ads_hal_pin_int_enable(bool enable);

void deadzone_filter(float * sample);
void signal_filter(float * sample);
void deadzone_filter1(float * sample);
void signal_filter1(float * sample);
void parse_serial_port(void);

float prev_ads2_x = 0;
float prev_ads2_y = 0;

//const unsigned long emailInterval = 5UL * 60UL * 1000UL; // 5 minutes in milliseconds

volatile bool ADS1_dataReady = false;
// New I2C address being written to the attached ads one axis sensor
uint8_t new_address = 0x15;
//bool NVS_STORE_DONE = "";

long i =0;

String default_deviceName = "Nitto_Flex1";    //  device name
String Location_ID = "B1234";         // Location_ID  replced the BED_ID   this used for the database index  *** important parameter
int packetSize;

float Horiz_Angle=0, Verti_Angle=0;

bool rebootRequested = false;  // from telegram

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
/* Device type */

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
	ADS_ONE_AXIS = 1,
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

// Global callback pointers
float sample1[2]={0,0};
float sample[2]={0,0};


static uint8_t read_buffer[ADS2_TRANSFER_SIZE];
static volatile bool ads2_irq_pending = false;
static volatile uint32_t ads2_lastTickISR = 0;  // tiny debounce window

static inline void ads2_mask_irq_from_isr() { gpio_intr_disable((gpio_num_t)ADS2_INTERRUPT_PIN); }
static inline void ads2_unmask_irq()        { gpio_intr_enable((gpio_num_t)ADS2_INTERRUPT_PIN);  }



volatile bool _ads2_int_enabled = false;


volatile float bend_value = 0.0f;             // ADS_SAMPLE
volatile float stretch_value = 0.0f;          // ADS_STRETCH_SAMPLE
volatile bool  bend_new = false;
volatile bool  stretch_new = false;

static uint8_t _address2 = ADS2_DEFAULT_ADDR;   // 0x13    Two Axis
int Kan_test = 0;

/* Device I2C address array. Use ads_hal_update_addr() to 
 * populate this array. */
static uint8_t ads2_addrs[ADS_COUNT] = 
{
	ADS2_DEFAULT_ADDR,
};

float ang[2]={0,0};
volatile bool ads2_newData = false;

// holds the app’s sample callback
static ads2_callback ads2_sample_callback = nullptr;
static void (*ads2_read_callback)(uint8_t *);

/* End of Flexsensr variables */

char Buffer[100];    // Make sure buffer is large enough


/* BLE Functions*/

// --- helpers to send BLE->UART regardless of return type ---
static inline void writeToSerial(const std::string& s) 
{
  Serial1.write((const uint8_t*)s.data(), s.size());
  Serial1.flush();
}
static inline void writeToSerial(const String& s) 
{
  Serial1.write((const uint8_t*)s.c_str(), s.length());
  Serial1.flush();
}
     
/* Function belongs to Flex Sensor */

static bool stretch_en = false;

// Two axis 
void IRAM_ATTR ads2_hal_interrupt(void)
{
  // One-shot mask to avoid re-entrancy / bounce storms
  ads2_mask_irq_from_isr();

  // Optional debounce (~2 ms). Use 0–5 ms depending on your source.
 uint32_t now = xTaskGetTickCountFromISR();
 if ((now - ads2_lastTickISR) >= pdMS_TO_TICKS(2)) 
	{
    ads2_lastTickISR = now;
    ads2_irq_pending = true;
    //Serial.println("INTERRUPT");
    //Serial.println("IRQ RECEIVED");  			
		 // Do the I2C read in loop()
  } 
 else 
	{
    // If it's just bounce, re-arm immediately
    ads2_unmask_irq();
  }
}

static void ads2_service_irq_once() 
{
 if (ads2_hal_read_buffer(read_buffer, ADS2_TRANSFER_SIZE) == ADS_OK)
 {
    if (ads2_read_callback) 
    ads2_read_callback(read_buffer);
 }

 ads2_unmask_irq();  // re-arm the GPIO interrupt
}

/**
 * @brief Initializes the hardware abstraction layer 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_hal_init(void (*callback)(uint8_t*), uint32_t reset_pin, uint32_t ADS2_datardy_pin)
{
	 pinMode(ADS_RESET_PIN, OUTPUT);
   pinMode(ADS2_INTERRUPT_PIN, INPUT_PULLUP);
	// Set callback pointer
	ads2_read_callback = callback;
	
	
	// Reset the ads
	ads_hal_reset();
	// Wait for ads to initialize
	ads_hal_delay(2000);
	
	// Configure I2C bus
	//Wire.setPins(sdaPin, sclPin); // Replace sdaPin and sclPin with your chosen GPIOs // kannana
	//Wire.setPins(18, 17); // Replace sdaPin and sclPin with your chosen GPIOs   // kannan Nitto
	Wire.begin(I2C1_SDA,I2C1_SCL,400000); // Pin 8 = I2C_SDA , Pin 9 = I2C_SCL
	//Wire.setClock(100000);
	Wire.setTimeOut(50);          // ms
// Configure and enable interrupt pin
	ads2_hal_pin_int_init();

	return ADS_OK;
}

void on_ads2_raw(uint8_t *data) 
{
    Serial.print("Raw: ");
    Serial.print(data[0]); Serial.print(" ");
    Serial.println(data[1]);
}

void on_ads2_sample(float *samples) 
{
    Serial.print("Samples: X=");
    Serial.print(samples[0]);
    Serial.print(" Y=");
    Serial.println(samples[1]);
}

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *			all settings on ADS will be reset to default. Reinitilaztion necessary
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_wake(void)
{
	// Reset ADS to wake from shutdown
	ads_hal_reset();
	
	// Allow time for ADS to reinitialize 
	ads_hal_delay(100);	
	
	return ADS_OK;
}

/**
 * @brief Checks that the device id is ADS_ONE_AXIS. ADS should not be in free run
 * 			when this function is called.
 *
 * @return	ADS_OK if dev_id is ADS_ONE_AXIS, ADS_ERR_DEV_ID if not
 */

/**
 * @brief Write pin to level of val	
 */

// /**
//  * @brief Reset the Angular Displacement Sensor
/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address1
 */

/**
 * @brief Configure I2C bus, 7 bit address, 400kHz frequency enable clock stretching
 *			if available.
 */
static void ads1_hal_i2c_init(void)
{
	//Wire.begin();
	//Wire.begin(I2C_SDA,I2C_SCL,400000);  // Pin 8 = I2C_SDA , Pin 9 = I2C_SCL
	Wire.begin(21,22,400000);  // Pin 8 = I2C_SDA , Pin 9 = I2C_SCL
	//Wire.setClock(400000);	
	Wire.setTimeOut(50);          // ms
}

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of data to read in number of bytes.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads1_hal_read_buffer(uint8_t * buffer, uint8_t len)
{
	Wire.requestFrom(_address2, len);     // Single axis 
	
	uint8_t i = 0; 
	
	while(Wire.available())
	{
		buffer[i] = Wire.read();
		i++;
	}
	//Serial.println("INTR 1 ads_hal_read_buffer");  

	if(i == len)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

void signal_filter1(float * sample1)   //singale axis 
{
    static float filter_samples1[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples1[i][5] = filter_samples1[i][4];
      filter_samples1[i][4] = filter_samples1[i][3];
      filter_samples1[i][3] = (float)sample1[i];
      filter_samples1[i][2] = filter_samples1[i][1];
      filter_samples1[i][1] = filter_samples1[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples1[i][0] = filter_samples1[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples1[i][2] + \
        0.20657208382614792f*(filter_samples1[i][3] + 2*filter_samples1[i][4] + filter_samples1[i][5]);   

      sample1[i] = filter_samples1[i][0];
    }
}

void deadzone_filter1(float * sample1)
{
  static float prev_sample1[2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample1[i]-prev_sample1[i]) > dead_zone)
      prev_sample1[i] = sample1[i];
    else
      sample1[i] = prev_sample1[i];
  }
}

/* End of single axis fucntion*/

void signal_filter(float * sample)   // two axis
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 2.0f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}

void ads2_data_callback(float * sample)
{
  // Low pass IIR filter
  signal_filter(sample);

   //Serial.println("ads2_data_callback kan");		
  // Deadzone filter
  deadzone_filter(sample);
  
  ang[0] = sample[0];
  ang[1] = sample[1];
  ads2_newData = true;
}

/* DDDDDDDDDDDDDDDDDD %%%%%%%%%%%%%%%%%% @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@&&&&&@@@@@@@@@@@@@@  */

/**@brief Function for decoding a int16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @return      Decoded value.
 */
inline int16_t ads_int16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)(p_encoded_data)[0])) |
                 (((int16_t)(p_encoded_data)[1]) << 8 ));
}

/**@brief Function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 * @return      Decoded value.
 */
inline uint16_t ads_uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)(p_encoded_data)[0])) |
                 (((uint16_t)(p_encoded_data)[1]) << 8 ));
}

/**@brief Function for encoding a uint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
inline uint8_t ads_uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

/*  ads_two_axis.cpp %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%@@@@@@@@@@@@@@@@@@@@@@@@@############## */

 /**
 * @brief Parses sample buffer from two axis ADS. Scales to degrees and
 *				executes callback registered in ads2_two_axis_init. 
 *				This function is called from ads_two_axis_hal. Application should never call this function.
 */	
void ads2_two_axis_parse_read_buffer(uint8_t * buffer)
{
	if(buffer[0] == ADS2_SAMPLE)
	{
		float sample[2];
				
		int16_t temp = ads_int16_decode(&buffer[1]); 
		sample[0] = (float)temp/32.0f;
		
		temp = ads_int16_decode(&buffer[3]);
		sample[1] = (float)temp/32.0f;
		
   //Serial.println("ads_two_axis_parse_read_buffer");		

    if (ads2_sample_callback) ads2_sample_callback(sample);  // call user sample callback  

	}	 
}

/**
 * @brief Places ADS in free run or sleep mode
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_two_axis_run(bool run)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
		
	buffer[0] = ADS2_RUN;
	buffer[1] = run;
		
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

/**
 * @brief Sets the sample rate of the ADS in free run mode
 *
 * @param	sps ADS2_SPS_T sample rate
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_two_axis_set_sample_rate(ADS2_SPS_T sps2)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_SPS;
	ads_uint16_encode(sps2, &buffer[1]);
	
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

/**
 * @brief Enables the ADS data ready interrupt line
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_two_axis_enable_interrupt(bool enable)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_INTERRUPT_ENABLE;
	buffer[1] = enable;
	
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

/**
 * @brief Updates the I2C address of the selected ADS. The default address 
 *		  is 0x13. Use this function to program an ADS to allow multiple
 *		  devices on the same I2C bus. 
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_ERR_BAD_PARAM if failed
 */
int ads2_two_axis_update_device_address(uint8_t device, uint8_t address)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_SET_ADDRESS;
	buffer[1] = address;
	
	if(ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE) != ADS_OK)
		return ADS_ERR_IO;
	
	ads2_hal_set_address(address);
	
	return ADS_OK;
}

/**
 * @brief Initializes the hardware abstraction layer and sample rate of the ADS
 *
 * @param	ads2_init_t	initialization structure of the ADS
 * @return	ADS_OK if successful ADS_ERR if failed
 */
int ads2_two_axis_init(ads2_init_t *ads2_init)
{	
	ads2_hal_init(&ads2_two_axis_parse_read_buffer, ads2_init->reset_pin, ads2_init->ADS2_datardy_pin);	// ads_read_callback was set in ads2_hal_init() to point to ads_two_axis_parse_read_buffer().
	                                                                                            // ads_two_axis_parse_read_buffer() is called with the raw I2C data.

  ads2_sample_callback = ads2_init->ads2_sample_callback;     
	// Check that the device id matched ADS_TWO_AXIS
	// Check that the device type is a one axis
	ADS2_DEV_TYPE_T ads2_dev_type;
	if (ads2_get_dev_type(&ads2_dev_type) != ADS_OK)
		return ADS_ERR_DEV_ID;

	switch (ads2_dev_type)
	{
	case ADS2_DEV_TWO_AXIS_V1:
	case ADS2_DEV_TWO_AXIS_V2:
		break;
	default:
		return ADS_ERR_DEV_ID;
	}
	Serial.println("INTR2 ads2_two_axis_init ");   
	ads_hal_delay(2);

	if(ads2_two_axis_set_sample_rate(ads2_init->sps2))     // Set sample rate 
		return ADS_ERR;

	ads_hal_delay(2);

	return ADS_OK;
}
/**
 * @brief Enables/disables individual axes of the sensor. Both axes are
 *				enabled at reset. 
 *				ADS_AXIS_0_EN | ADS_AXIS_1_EN enables both.
 *				ADS_AXIS_0_EN enables axis zero and disables axis one
 *				ADS_AXIS_1_EN enables axis one and disables axis zero
 *
 * @param	axes_enabled	bit mask of which axes to enable
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 */
int ads2_two_axis_enable_axis(uint8_t axes_enable)
{
	if(!(axes_enable & (ADS_AXIS_0_EN | ADS_AXIS_1_EN)))
			return ADS_ERR_BAD_PARAM;
	
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_AXES_ENALBED;
	buffer[1] = axes_enable;
	
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

/**
 * @brief Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_two_axis_shutdown(void)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_SHUTDOWN;
	
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_two_axis_wake(void)
{
	// Reset ADS to wake from shutdown
	ads_hal_reset();
	
	// Allow time for ADS to reinitialize 
	ads_hal_delay(100);	
	
	return ADS_OK;
}

/**
 * @brief Checks that the device id is ADS_TWO_AXIS. ADS should not be in free run
 *				when this function is called.
 *
 * @return	ADS_OK if dev_id is ADS_TWO_AXIS, ADS_ERR_DEV_ID if not
 */
int ads2_get_dev_id(void)
{
	ADS2_DEV_TYPE_T device_type;
	
	if (ads2_get_dev_type(&device_type) == ADS_OK)
	{
		switch (device_type)
		{
		case ADS2_DEV_TWO_AXIS_V1:
		case ADS2_DEV_TWO_AXIS_V2:
			return ADS_OK;
		}
	}	
	return ADS_ERR_DEV_ID;
}

 /**
 * @brief Returns the device type in device_type. ADS should not be in free run
 * 			when this function is called.
 *
 * @param device_type  recipient of the device type
 * @return	ADS_OK if dev_id is one of ADS2_DEV_TYPE_T, ADS_ERR_DEV_ID if not
 */
int ads2_get_dev_type(ADS2_DEV_TYPE_T * ads2_dev_type)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
	buffer[0] = ADS2_GET_DEV_ID;
	
	// Disable interrupt to prevent callback from reading out device id
	ads2_hal_pin_int_enable(false);
	
	ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
	ads_hal_delay(2);
	ads2_hal_read_buffer(buffer, ADS2_TRANSFER_SIZE);
	
	ads2_hal_pin_int_enable(true);
	
	if (buffer[0] == ADS2_DEV_ID)
	{
		switch (buffer[1])
		{
		case ADS2_DEV_ONE_AXIS_V1:
		case ADS2_DEV_ONE_AXIS_V2:
		case ADS2_DEV_TWO_AXIS_V1:
		case ADS2_DEV_TWO_AXIS_V2:
			*ads2_dev_type = static_cast<ADS2_DEV_TYPE_T>(buffer[1]);
			return ADS_OK;
		}
	}
	
	*ads2_dev_type = ADS2_DEV_UNKNOWN;
	return ADS_ERR_DEV_ID;
}

static void ads2_hal_pin_int_init(void)
{
	pinMode(ADS2_INTERRUPT_PIN, INPUT_PULLUP);
	ads2_hal_pin_int_enable(true);
}


static inline void ads2_hal_gpio_pin_write(uint8_t pin, uint8_t val)
{
	digitalWrite(pin, val);
}

void ads_hal_delay(uint16_t delay_ms)
{
	delay(delay_ms);
}

void ads2_hal_pin_int_enable(bool enable)
{
	_ads2_int_enabled = enable;
	
	if(enable)
	{
		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);
	}
	else
	{
		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));
	}
}

/**
 * @brief Write buffer of data to the Angular Displacement Sensor
 *
 * @param buffer[in]	Write buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_hal_write_buffer(uint8_t * buffer, uint8_t len)
{
	// Disable the interrupt
	if(_ads2_int_enabled)
		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));
	
	Wire.beginTransmission(_address2);
	uint8_t nb_written = Wire.write(buffer, len);
	Wire.endTransmission();
	
	// Enable the interrupt
	if(_ads2_int_enabled)
	{
		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);
		
		// Read data packet if interrupt was missed
		if(digitalRead(ADS2_INTERRUPT_PIN) == 0)
		{
			if(ads2_hal_read_buffer(read_buffer, ADS2_TRANSFER_SIZE) == ADS_OK)
			{
				ads2_read_callback(read_buffer);
			}
		}
	}
	
	if(nb_written == len)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads2_hal_read_buffer(uint8_t * buffer, uint8_t len)
{
	Wire.requestFrom(_address2, len);
	
	uint8_t i = 0; 
	
	while(Wire.available())
	{
		buffer[i] = Wire.read();
		i++;
	}
//	Serial.println("INTR2 ads_hal_read_buffer kan");   
	if(i == len)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}
/**
 * @brief Reset the Angular Displacement Sensor
 *
 * @param dfuMode	Resets ADS into firmware update mode if true
 */
void ads_hal_reset(void)
{
	// Configure reset line as an output
	pinMode(ADS_RESET_PIN, OUTPUT);
	
	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 0);
	ads_hal_delay(10);
	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 1);
	
	pinMode(ADS_RESET_PIN, INPUT_PULLUP);
}

/**
 * @brief Selects the current device address of the ADS driver is communicating with
 *
 * @param device select device 0 - ADS_COUNT
 * @return	ADS_OK if successful ADS_ERR_BAD_PARAM if invalid device number
 */
int ads2_hal_select_device(uint8_t device)
{
	if(device < ADS_COUNT)
		_address2 = ads2_addrs[device];
	else
		return ADS_ERR_BAD_PARAM;
		
	return ADS_OK;
}

/**
 * @brief Updates the I2C address in the ads_addrs[] array. Updates the current
 *		  selected address.
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_BAD_PARAM if failed
 */
int ads2_hal_update_device_addr(uint8_t device, uint8_t address)
{
	if(device < ADS_COUNT)
		ads2_addrs[device] = address;
	else
		return ADS_ERR_BAD_PARAM;
		
	_address2 = address;
		
	return ADS_OK;	
}

/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address2
 */
uint8_t ads2_hal_get_address(void)
{
	return _address2;
}

/**
 * @brief Sets the i2c address that the hal layer is addressing *	
 *				Used by device firmware update (dfu)
 */
void ads2_hal_set_address(uint8_t address)
{
	_address2 = address;
}
/* End Flex sensor Functions */


/*Serail port 0 and 1 */


/**
 * @brief Calibrates one axis ADS. ADS_CALIBRATE_FIRST should be at 0 degrees on
 *				ADS_CALIBRATE_SECOND can be at 45 - 255 degrees, recommended 90 degrees.
 *
 * @param	ads_calibration_step 	ADS_CALIBRATE_STEP_T to perform
 * @param degrees uint8_t angle at which sensor is bent when performing 
 *				ADS_CALIBRATE_FIRST, and ADS_CALIBRATE_SECOND
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 */

int ads2_calibrate(ADS2_CALIBRATION_STEP_T ads2_calibration_step, uint8_t degrees)
 {
	uint8_t buffer[ADS2_TRANSFER_SIZE] = {0};
	
	buffer[0] = ADS2_CALIBRATE;
	buffer[1] = ads2_calibration_step;
	buffer[2] = degrees;	
	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
 }
 

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *			all settings on ADS will be reset to default. Reinitilaztion necessary
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
struct VP { float v; int pct; };
static const VP curve[] = 
 {
  {4.20,100},{4.10,90},{3.98,80},{3.92,70},{3.87,60},
  {3.82,50},{3.79,40},{3.77,30},{3.72,20},{3.69,10},{3.50,0}
 };

int liionPercentFromVoltage(float vbat) 
{
  if (vbat >= curve[0].v) return 100;
  if (vbat <= curve[sizeof(curve)/sizeof(curve[0]) - 1].v) return 0;

  // find segment and interpolate
  for (size_t i = 0; i < (sizeof(curve)/sizeof(curve[0]) - 1); ++i) 
  {
    if (vbat <= curve[i].v && vbat >= curve[i+1].v) 
    {
      float x = (vbat - curve[i+1].v) / (curve[i].v - curve[i+1].v);
      return (int)roundf(curve[i+1].pct + x * (curve[i].pct - curve[i+1].pct));
    }
  }
  return 0;
}
  

void setup()

 {  
  /*ESP32-S3-WROOM-1 (16MB flash 8MB RAM) */

  Serial.begin(115200);      //     43 Tx ,44 Rx  console port
  delay(30);
  Serial.println("\n[BOOT] Starting...");
      // ================= BLE =================
  delay(30);
   // Attach the interrupt handler
  //MDNS.begin("Nitto_Flex1"); // host will be esp-sensor.local  
  Serial.println("Initializing Two Axis sensor");   
  ads2_init_t init2;

  init2.sps2                  = ADS2_10_HZ;  
  init2.ads2_sample_callback  = &ads2_data_callback;   // ads1_sample_callback is set to point to your function ads2_data_callback.
	init2.reset_pin             = ADS_RESET_PIN;
  init2.ADS2_datardy_pin      = ADS2_INTERRUPT_PIN;
	//init.addr2                  = 0x13;            // Initialize ADS hardware abstraction layer, and set the sample rate

	int ret_val = ads2_two_axis_init(&init2);       //   ads2_two_axis_init       

  if(ret_val == ADS_OK)
  {
    Serial.println("Two Axis ADS initialization succeeded");
  }
  else
  {
    Serial.print("Two Axis ADS initialization failed with reason: ");
    Serial.println(ret_val);		
  }
  
  delay(1000);

/* ================================================ */

// RUN ADS2 SENSOR (0x13)
ads2_hal_set_address(ADS2_DEFAULT_ADDR);
ads2_two_axis_run(true);
delay(1200);

Serial.println("] Device Ready");
 }  /*End of setup */
 
void loop() 
 { 
 
    Serial.print("Horiz_Angle(X-Axis):  ");
    Serial.println(ang[0], 2);
    Serial.print("Verti_Angle (Y-Axis): ");
    Serial.println(ang[1], 2);
// sendAnglesFrame(ang[0],  ang[1], 'H' , 'V');
  delay(100);

/* I2C Data receing interupt calling from two axis */
if (ads2_irq_pending)  
 {
  noInterrupts(); 
	ads2_irq_pending = false; 
	interrupts();
  ads2_service_irq_once();
 }

if(ads2_newData)
 {
   ads2_newData = false;      
   Horiz_Angle = ang[0]; 
   Verti_Angle = ang[1]; 
  
  // Serial.print(ang[0]); 
  // Serial.print(","); 
  // Serial.println(ang[1]);
  }  
 /* Event based data Transfer to server  call  Update_data3, eMAIL, ble  */ 

}

/* End of Loop */


