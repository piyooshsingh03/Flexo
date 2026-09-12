// #include <Adafruit_NeoPixel.h>
// #include <sys/time.h>
// #include "Wire.h"
// #include <ArduinoJson.h>
// #include <Arduino.h>
// #include <ArduinoWebsockets.h>
// #include "driver/uart.h"
// #include <Adafruit_NeoPixel.h>
// #include <sys/time.h>
// #include "esp_vfs_fat.h"
// #include "FS.h"
// #include "esp_err.h"
// #include <queue>
// #include <math.h>
// #include <stdint.h>
// #include <stdbool.h>
// #include "driver/gpio.h"  // mask/unmask a single GPIO IRQ (IRAM-safe)e
// #include "esp_log.h"
// #include "esp_intr_alloc.h"
// #include "esp_heap_caps.h"
// #include <USB.h>
// #include <USBMSC.h>
// #include "BLE.h"

// #define ADS_RESET_PIN          (15)           // Pin number attached to ads reset line.
// #define ADS2_INTERRUPT_PIN     (18)           // Pin number attached to the ads data ready line.  two axis
// #define I2C1_SDA                21
// #define I2C1_SCL                22
// #define ADS2_DEFAULT_ADDR      (0x13)			   // Default I2C address of the Two Axis
// #define ADS2_TRANSFER_SIZE     (5)  // 1 header + two int16s
// #define ADS_COUNT              (10)
// #define ADS_OK                 (0)
// #define ADS_ERR                (-1)
// #define ADS_ERR_BAD_PARAM      (-2)
// #define ADS_ERR_OP_IN_PROGRESS (-3)
// #define ADS_ERR_IO             (-4)
// #define ADS_ERR_DEV_ID         (-5)
// #define ADS_ERR_TIMEOUT        (-6)

// #define ADS_AXIS_0_EN          (0x01)
// #define ADS_AXIS_1_EN          (0x02)

// TwoWire I2C_1 = TwoWire(0);

// float prev_ads2_x = 0;
// float prev_ads2_y = 0;

// float Horiz_Angle=0, Verti_Angle=0;
// /* Flex Sensor variable and structures */
// typedef void (*ads2_callback)(float*);

// typedef enum 
// {
// 	ADS2_RUN = 0,
// 	ADS2_SPS,
// 	ADS2_RESET,
// 	ADS2_DFU,
// 	ADS2_SET_ADDRESS,
// 	ADS2_INTERRUPT_ENABLE,
// 	ADS2_GET_FW_VER,
// 	ADS2_CALIBRATE,
// 	ADS2_AXES_ENALBED,
// 	ADS2_SHUTDOWN,
// 	ADS2_GET_DEV_ID
// } ADS2_COMMAND_T;
// /* Identifier for packet received from ADS */

// typedef enum 
// {
// 	ADS2_SAMPLE = 0,
// 	ADS2_FW_VER,
// 	ADS2_DEV_ID
// } ADS2_PACKET_T;

// typedef enum 
// {
// 	ADS2_DEV_UNKNOWN     = 0,
// 	ADS2_DEV_ONE_AXIS_V1 = 1,
// 	ADS2_DEV_TWO_AXIS_V1 = 2,
// 	ADS2_DEV_ONE_AXIS_V2 = 12,
// 	ADS2_DEV_TWO_AXIS_V2 = 22
// } ADS2_DEV_TYPE_T;

// typedef enum 
// {
// 	ADS2_1_HZ   = 16384,
// 	ADS2_10_HZ  = 1638,
// 	ADS2_20_HZ  = 819,
// 	ADS2_50_HZ  = 327,
// 	ADS2_100_HZ = 163,
// 	ADS2_200_HZ = 81,
// 	ADS2_333_HZ = 49,
// 	ADS2_500_HZ = 32,
// } ADS2_SPS_T;

// /* Device IDS */
// typedef enum 
// {
// 	ADS_TWO_AXIS_1 = 1,
// 	ADS_TWO_AXIS = 2,
// } ADS_DEV_IDS_T;

// typedef enum 
// {
// 	ADS2_CALIBRATE_FIRST = 0,		   	// First calibration point, typically 0 degrees
// 	ADS2_CALIBRATE_SECOND,			     	// Second calibration point, 45-255 degrees
// 	ADS2_CALIBRATE_CLEAR,				    // Clears user calibration, restores factory calibration
// //	ADS2_CALIBRATE_STRETCH_ZERO,			// 0mm strain calibration point
// //	ADS2_CALIBRATE_STRETCH_SECOND,		// Second calibration point for stretch, typically 30mm
// } ADS2_CALIBRATION_STEP_T;

// typedef struct 
// {
//   ADS2_SPS_T sps2;
//   ads2_callback ads2_sample_callback;
//   uint32_t reset_pin;
//   uint32_t ADS2_datardy_pin;
//   uint8_t  addr2;      // kannna
// } ads2_init_t;

// float sample[2]={0,0};
// float sample2[2]={0,0};

// static uint8_t read_buffer[ADS2_TRANSFER_SIZE];
// static volatile bool ads2_irq_pending = false;
// static volatile uint32_t ads2_lastTickISR = 0;  // tiny debounce window

// static inline void ads2_mask_irq_from_isr() { gpio_intr_disable((gpio_num_t)ADS2_INTERRUPT_PIN); }
// static inline void ads2_unmask_irq()        { gpio_intr_enable((gpio_num_t)ADS2_INTERRUPT_PIN);  }

// volatile bool _ads2_int_enabled = false;
// volatile float bend_value = 0.0f;             // ADS_SAMPLE
// volatile float stretch_value = 0.0f;          // ADS_STRETCH_SAMPLE
// volatile bool  bend_new = false;
// volatile bool  stretch_new = false;
// static uint8_t _address2 = ADS2_DEFAULT_ADDR;   // 0x13    Two Axis
// static uint8_t ads2_addrs[ADS_COUNT] = 
// {
// 	ADS2_DEFAULT_ADDR,
// };

// float ang[2]={0,0};
// volatile bool ads2_newData = false;

// // holds the app’s sample callback
// static ads2_callback ads2_sample_callback = nullptr;
// static void (*ads2_read_callback)(uint8_t *);
// char Buffer[100];    // Make sure buffer is large enough
// int ads2_hal_read_buffer(uint8_t * buffer, uint8_t len);
// void ads2_hal_set_address(uint8_t address);
// void ads2_hal_reset(void);


// static uint8_t* buffer = NULL;
// void ads2_data_callback(float * sample);
// void ads2_hal_delay(uint16_t delay_ms);
// void ads2_hal_pin_int_enable(bool enable);
// static void ads2_service_irq_once();
// void ads_hal_pin_int_enable(bool enable);
// void deadzone_filter(float * sample);
// void signal_filter(float * sample);
// void parse_serial_port(void);
// void ads_hal_delay(uint16_t delay_ms);
// static void ads2_hal_pin_int_init(void);
// int ads2_hal_write_buffer(uint8_t * buffer, uint8_t len);
// int ads2_get_dev_type(ADS2_DEV_TYPE_T * ads2_dev_type);



// void IRAM_ATTR ads2_hal_interrupt(void)
// {
//   // One-shot mask to avoid re-entrancy / bounce storms
//   ads2_mask_irq_from_isr();

//   // Optional debounce (~2 ms). Use 0–5 ms depending on your source.
//  uint32_t now = xTaskGetTickCountFromISR();
//  if ((now - ads2_lastTickISR) >= pdMS_TO_TICKS(2)) 
// 	{
//     ads2_lastTickISR = now;
//     ads2_irq_pending = true;
//     //Serial.println("INTERRUPT");
//     //Serial.println("IRQ RECEIVED");  			
// 		 // Do the I2C read in loop()
//   } 
//  else 
// 	{
//     // If it's just bounce, re-arm immediately
//     ads2_unmask_irq();
//   }
// }

// int ads2_hal_init(void (*callback)(uint8_t*), uint32_t reset_pin, uint32_t ADS2_datardy_pin)
// {
//    pinMode(ADS_RESET_PIN, OUTPUT);
//    pinMode(ADS2_INTERRUPT_PIN, INPUT_PULLUP);
// 	// Set callback pointer
// 	ads2_read_callback = callback;
	
	
// 	// Reset the ads
// 	ads2_hal_reset();
// 	// Wait for ads to initialize
// 	ads_hal_delay(2000);
	
// 	// Configure I2C bus
// 	//Wire.setPins(sdaPin, sclPin); // Replace sdaPin and sclPin with your chosen GPIOs // kannana
// 	//Wire.setPins(18, 17); // Replace sdaPin and sclPin with your chosen GPIOs   // kannan Nitto
// 	I2C_1.begin(I2C1_SDA,I2C1_SCL,400000); // Pin 8 = I2C_SDA , Pin 9 = I2C_SCL
// 	//Wire.setClock(100000);  
// 	I2C_1.setTimeOut(50);       // ms
// // Configure and enable interrupt pin
// 	ads2_hal_pin_int_init();

// 	return ADS_OK;
// }

// void on_ads2_raw(uint8_t *data) 
// {
//     Serial.print("Raw: ");
//     Serial.print(data[0]); Serial.print(" ");
//     Serial.println(data[1]);
// }

// void on_ads2_sample(float *samples) 
// {
//     Serial.print("Samples: X=");
//     Serial.print(samples[0]);
//     Serial.print(" Y=");
//     Serial.println(samples[1]);
// }

// int ads2_wake(void)
// {
// 	// Reset ADS to wake from shutdown
// 	ads2_hal_reset();
	
// 	// Allow time for ADS to reinitialize 
// 	ads_hal_delay(100);	
	
// 	return ADS_OK;
// }

// /* End of single axis fucntion*/

// void signal_filter(float * sample)   // two axis
// {
//     static float filter_samples[2][6];

//     for(uint8_t i=0; i<2; i++)
//     {
//       filter_samples[i][5] = filter_samples[i][4];
//       filter_samples[i][4] = filter_samples[i][3];
//       filter_samples[i][3] = (float)sample[i];
//       filter_samples[i][2] = filter_samples[i][1];
//       filter_samples[i][1] = filter_samples[i][0];
  
//       // 20 Hz cutoff frequency @ 100 Hz Sample Rate
//       filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
//         0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

//       sample[i] = filter_samples[i][0];
//     }
// }

// void deadzone_filter(float * sample)
// {
//   static float prev_sample[2];
//   float dead_zone = 2.0f;

//   for(uint8_t i=0; i<2; i++)
//   {
//     if(fabs(sample[i]-prev_sample[i]) > dead_zone)
//       prev_sample[i] = sample[i];
//     else
//       sample[i] = prev_sample[i];
//   }
// }

// void ads2_data_callback(float * sample)
// {
//   // Low pass IIR filter
//   signal_filter(sample);
// //Serial.println("ads2_data_callback kan");		
// // Deadzone filter
//   deadzone_filter(sample);
//   ang[0] = sample[0];
//   ang[1] = sample[1];
//   ads2_newData = true;
// }

// inline int16_t ads_int16_decode(const uint8_t * p_encoded_data)
// {
//         return ( (((uint16_t)(p_encoded_data)[0])) |
//                  (((int16_t)(p_encoded_data)[1]) << 8 ));
// }
// inline uint16_t ads_uint16_decode(const uint8_t * p_encoded_data)
// {
//         return ( (((uint16_t)(p_encoded_data)[0])) |
//                  (((uint16_t)(p_encoded_data)[1]) << 8 ));
// }

// inline uint8_t ads_uint16_encode(uint16_t value, uint8_t * p_encoded_data)
// {
//     p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
//     p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
//     return sizeof(uint16_t);
// }

// void ads2_two_axis_parse_read_buffer(uint8_t * buffer)
// {
// 	if(buffer[0] == ADS2_SAMPLE)
// 	{
// 		float sample[2];
				
// 		int16_t temp = ads_int16_decode(&buffer[1]); 
// 		sample[0] = (float)temp/32.0f;
		
// 		temp = ads_int16_decode(&buffer[3]);
// 		sample[1] = (float)temp/32.0f;
		
//    //Serial.println("ads_two_axis_parse_read_buffer");		

//     if (ads2_sample_callback) ads2_sample_callback(sample);  // call user sample callback  

// 	}	 
// }

// int ads2_two_axis_run(bool run)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
		
// 	buffer[0] = ADS2_RUN;
// 	buffer[1] = run;
		
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// }

// int ads2_two_axis_set_sample_rate(ADS2_SPS_T sps2)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_SPS;
// 	ads_uint16_encode(sps2, &buffer[1]);
	
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// }

// int ads2_two_axis_enable_interrupt(bool enable)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_INTERRUPT_ENABLE;
// 	buffer[1] = enable;
	
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// }

// int ads2_two_axis_update_device_address(uint8_t device, uint8_t address)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_SET_ADDRESS;
// 	buffer[1] = address;
	
// 	if(ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE) != ADS_OK)
// 		return ADS_ERR_IO;
	
// 	ads2_hal_set_address(address);
	
// 	return ADS_OK;
// }

// int ads2_two_axis_init(ads2_init_t *ads2_init)
// {	
// 	ads2_hal_init(&ads2_two_axis_parse_read_buffer, ads2_init->reset_pin, ads2_init->ADS2_datardy_pin);	// ads_read_callback was set in ads2_hal_init() to point to ads_two_axis_parse_read_buffer().
// 	                                                                                            // ads_two_axis_parse_read_buffer() is called with the raw I2C data.

//   ads2_sample_callback = ads2_init->ads2_sample_callback;     
// 	// Check that the device id matched ADS_TWO_AXIS
// 	// Check that the device type is a one axis
// 	ADS2_DEV_TYPE_T ads2_dev_type;
// 	if (ads2_get_dev_type(&ads2_dev_type) != ADS_OK)
// 		return ADS_ERR_DEV_ID;

// 	switch (ads2_dev_type)
// 	{
// 	case ADS2_DEV_TWO_AXIS_V1:
// 	case ADS2_DEV_TWO_AXIS_V2:
// 	break;
// 	default:
// 	return ADS_ERR_DEV_ID;
// 	}
// 	Serial.println("INTR2 ads2_two_axis_init ");   
// 	ads_hal_delay(2);

// 	if(ads2_two_axis_set_sample_rate(ads2_init->sps2))     // Set sample rate 
// 		return ADS_ERR;

// 	ads_hal_delay(2);

// 	return ADS_OK;
// }
// int ads2_two_axis_enable_axis(uint8_t axes_enable)
// {
// 	if(!(axes_enable & (ADS_AXIS_0_EN | ADS_AXIS_1_EN)))
// 			return ADS_ERR_BAD_PARAM;
	
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_AXES_ENALBED;
// 	buffer[1] = axes_enable;
	
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// }

// int ads_two_axis_shutdown(void)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_SHUTDOWN;
	
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// }

// int ads2_two_axis_wake(void)
// {
// 	// Reset ADS to wake from shutdown
// 	ads2_hal_reset();
	
// 	// Allow time for ADS to reinitialize 
// 	ads_hal_delay(100);	
	
// 	return ADS_OK;
// }

// int ads2_get_dev_id(void)
// {
// 	ADS2_DEV_TYPE_T device_type;
	
// 	if (ads2_get_dev_type(&device_type) == ADS_OK)
// 	{
// 		switch (device_type)
// 		{
// 		case ADS2_DEV_TWO_AXIS_V1:
// 		case ADS2_DEV_TWO_AXIS_V2:
// 			return ADS_OK;
// 		}
// 	}	
// 	return ADS_ERR_DEV_ID;
// }

// int ads2_get_dev_type(ADS2_DEV_TYPE_T * ads2_dev_type)
// {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE];
	
// 	buffer[0] = ADS2_GET_DEV_ID;
	
// 	// Disable interrupt to prevent callback from reading out device id
// 	ads2_hal_pin_int_enable(false);
	
// 	ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
// 	ads_hal_delay(2);
// 	ads2_hal_read_buffer(buffer, ADS2_TRANSFER_SIZE);
	
// 	ads2_hal_pin_int_enable(true);
	
// 	if (buffer[0] == ADS2_DEV_ID)
// 	{
// 		switch (buffer[1])
// 		{
// 		case ADS2_DEV_ONE_AXIS_V1:
// 		case ADS2_DEV_ONE_AXIS_V2:
// 		case ADS2_DEV_TWO_AXIS_V1:
// 		case ADS2_DEV_TWO_AXIS_V2:
// 			*ads2_dev_type = static_cast<ADS2_DEV_TYPE_T>(buffer[1]);
// 			return ADS_OK;
// 		}
// 	}
	
// 	*ads2_dev_type = ADS2_DEV_UNKNOWN;
// 	return ADS_ERR_DEV_ID;
// }

// static void ads2_hal_pin_int_init(void)
// {
// 	pinMode(ADS2_INTERRUPT_PIN, INPUT_PULLUP);
// 	ads2_hal_pin_int_enable(true);
// }

// static inline void ads2_hal_gpio_pin_write(uint8_t pin, uint8_t val)
// {
// 	digitalWrite(pin, val);
// }

// static inline void ads1_hal_gpio_pin_write(uint8_t pin, uint8_t val)
// {
// 	digitalWrite(pin, val);
// }

// void ads_hal_delay(uint16_t delay_ms)
// {
// 	delay(delay_ms);
// }
// static void ads2_service_irq_once() 
// {
//  if (ads2_hal_read_buffer(read_buffer, ADS2_TRANSFER_SIZE) == ADS_OK)
//  {
//     if (ads2_read_callback) 
//     ads2_read_callback(read_buffer);
//  }
 
//  ads2_unmask_irq();  // re-arm the GPIO interrupt
// }
// void ads2_hal_pin_int_enable(bool enable)
// {
// 	_ads2_int_enabled = enable;
	
// 	if(enable)
// 	{
// 		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);
// 	}
// 	else
// 	{
// 		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));
// 	}
// }


// int ads2_hal_write_buffer(uint8_t * buffer, uint8_t len)
// {
// 	// Disable the interrupt
// 	if(_ads2_int_enabled)
// 		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));
	
// 	I2C_1.beginTransmission(0x13);
// 	uint8_t nb_written = I2C_1.write(buffer, len);
// 	I2C_1.endTransmission();
	
// 	// Enable the interrupt
// 	if(_ads2_int_enabled)
// 	{
// 		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);
		
// 		// Read data packet if interrupt was missed
// 		if(digitalRead(ADS2_INTERRUPT_PIN) == 0)
// 		{
// 			if(ads2_hal_read_buffer(read_buffer, ADS2_TRANSFER_SIZE) == ADS_OK)
// 			{
// 				ads2_read_callback(read_buffer);
// 			}
// 		}
// 	}
// 	if(nb_written == len)
// 		return ADS_OK;
// 	else
// 		return ADS_ERR_IO;
// }

// int ads2_hal_read_buffer(uint8_t * buffer, uint8_t len)
// {
// 	I2C_1.requestFrom(_address2, len);
	
// 	uint8_t i = 0; 
	
// 	while(I2C_1.available())
// 	{
// 		buffer[i] = I2C_1.read();
// 		//  Serial.println(buffer[i], HEX);
// 		i++;
// 	}
// //	Serial.println("INTR2 ads_hal_read_buffer kan");   
// 	if(i == len)
// 		return ADS_OK;
// 	else
// 		return ADS_ERR_IO;
// }

// void ads2_hal_reset(void)
// {
// 	// Configure reset line as an output
// 	pinMode(ADS_RESET_PIN, OUTPUT);
// 	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 0);
// 	ads_hal_delay(10);
// 	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 1);
	
// 	pinMode(ADS_RESET_PIN, INPUT_PULLUP);
// }

// int ads2_hal_select_device(uint8_t device)
// {
// 	if(device < ADS_COUNT)
// 		_address2 = ads2_addrs[device];
// 	else
// 		return ADS_ERR_BAD_PARAM;
		
// 	return ADS_OK;
// }

// int ads2_hal_update_device_addr(uint8_t device, uint8_t address)
// {
// 	if(device < ADS_COUNT)
// 		ads2_addrs[device] = address;
// 	else
// 		return ADS_ERR_BAD_PARAM;
		
// 	_address2 = address;
		
// 	return ADS_OK;	
// }

// uint8_t ads2_hal_get_address(void)
// {
// 	return _address2;
// }

// void ads2_hal_set_address(uint8_t address)
// {
// 	_address2 = address;
// }

// int ads2_calibrate(ADS2_CALIBRATION_STEP_T ads2_calibration_step, uint8_t degrees)
//  {
// 	uint8_t buffer[ADS2_TRANSFER_SIZE] = {0};
	
// 	buffer[0] = ADS2_CALIBRATE;
// 	buffer[1] = ads2_calibration_step;
// 	buffer[2] = degrees;	
// 	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
//  }
