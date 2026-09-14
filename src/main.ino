#include <Arduino.h>
#include "FLEX.h"
#include <BLE.h>
#include "espnow_pairing.h"
#include "nvs_pairing.h"
TwoWire I2C_1 = TwoWire(0);
static uint8_t *buffer = NULL;
volatile bool ADS1_dataReady = false;

float Horiz_Angle = 0, Verti_Angle = 0;

float sample[2] = {0, 0};
void paired_status(void);
static uint8_t read_buffer[ADS2_TRANSFER_SIZE];
static volatile bool ads2_irq_pending = false;
static volatile uint32_t ads2_lastTickISR = 0; // tiny debounce window

static inline void ads2_mask_irq_from_isr() { gpio_intr_disable((gpio_num_t)ADS2_INTERRUPT_PIN); }
static inline void ads2_unmask_irq() { gpio_intr_enable((gpio_num_t)ADS2_INTERRUPT_PIN); }

volatile bool _ads2_int_enabled = false;

volatile float bend_value = 0.0f;	 // ADS_SAMPLE
volatile float stretch_value = 0.0f; // ADS_STRETCH_SAMPLE
volatile bool bend_new = false;
volatile bool stretch_new = false;

static uint8_t _address2 = ADS2_DEFAULT_ADDR; // 0x13    Two Axis

int Kan_test = 0;

static uint8_t ads2_addrs[ADS_COUNT] =
	{
		ADS2_DEFAULT_ADDR,
};

float ang[4] = {0, 0, 0, 0};

volatile bool ads2_newData = false;

// holds the app’s sample callback
static ads2_callback ads2_sample_callback = nullptr;
static void (*ads2_read_callback)(uint8_t *);

char Buffer[100]; // Make sure buffer is large enough

/* Function belongs to Flex Sensor */

static bool stretch_en = false;
/* BLE Functions*/

// --- helpers to send BLE->UART regardless of return type ---
static inline void writeToSerial(const std::string &s)
{
	Serial1.write((const uint8_t *)s.data(), s.size());
	Serial1.flush();
}
static inline void writeToSerial(const String &s)
{
	Serial1.write((const uint8_t *)s.c_str(), s.length());
	Serial1.flush();
}

int ads2_hal_read_buffer(uint8_t *buffer, uint8_t len);

void ads2_hal_set_address(uint8_t address);

void ads2_hal_reset(void);

void IRAM_ATTR ads2_hal_interrupt(void)
{
	// One-shot mask to avoid re-entrancy / bounce storms
	ads2_mask_irq_from_isr();

	uint32_t now = xTaskGetTickCountFromISR();
	if ((now - ads2_lastTickISR) >= pdMS_TO_TICKS(2))
	{
		ads2_lastTickISR = now;
		ads2_irq_pending = true;
		// Serial.println("INTERRUPT");
		// Serial.println("IRQ RECEIVED");
		//  Do the I2C read in loop()
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

	ads2_unmask_irq(); // re-arm the GPIO interrupt
}

int ads2_hal_init(void (*callback)(uint8_t *), uint32_t reset_pin, uint32_t ADS2_datardy_pin)
{
	pinMode(ADS_RESET_PIN, OUTPUT);
	pinMode(ADS2_INTERRUPT_PIN, INPUT_PULLUP);
	// Set callback pointer
	ads2_read_callback = callback;

	// Reset the ads
	ads2_hal_reset();
	// Wait for ads to initialize
	ads_hal_delay(2000);

	// Configure I2C bus
	// Wire.setPins(sdaPin, sclPin); // Replace sdaPin and sclPin with your chosen GPIOs // kannana
	// Wire.setPins(18, 17); // Replace sdaPin and sclPin with your chosen GPIOs   // kannan Nitto
	I2C_1.begin(I2C1_SDA, I2C1_SCL, 400000); // Pin 8 = I2C_SDA , Pin 9 = I2C_SCL
	// Wire.setClock(100000);
	I2C_1.setTimeOut(50); // ms
						  // Configure and enable interrupt pin
	ads2_hal_pin_int_init();

	return ADS_OK;
}

void on_ads2_raw(uint8_t *data)
{
	Serial.print("Raw: ");
	Serial.print(data[0]);
	Serial.print(" ");
	Serial.println(data[1]);
}

void on_ads2_sample(float *samples)
{
	Serial.print("Samples: X=");
	Serial.print(samples[0]);
	Serial.print(" Y=");
	Serial.println(samples[1]);
}

int ads2_wake(void)
{
	// Reset ADS to wake from shutdown
	ads2_hal_reset();

	// Allow time for ADS to reinitialize
	ads_hal_delay(100);

	return ADS_OK;
}

void signal_filter(float *sample) // two axis
{
	static float filter_samples[2][6];

	for (uint8_t i = 0; i < 2; i++)
	{
		filter_samples[i][5] = filter_samples[i][4];
		filter_samples[i][4] = filter_samples[i][3];
		filter_samples[i][3] = (float)sample[i];
		filter_samples[i][2] = filter_samples[i][1];
		filter_samples[i][1] = filter_samples[i][0];

		// 20 Hz cutoff frequency @ 100 Hz Sample Rate
		filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) - 0.19581571265583314f * filter_samples[i][2] +
							   0.20657208382614792f * (filter_samples[i][3] + 2 * filter_samples[i][4] + filter_samples[i][5]);

		sample[i] = filter_samples[i][0];
	}
}

void deadzone_filter(float *sample)
{
	static float prev_sample[2];
	float dead_zone = 2.0f;

	for (uint8_t i = 0; i < 2; i++)
	{
		if (fabs(sample[i] - prev_sample[i]) > dead_zone)
			prev_sample[i] = sample[i];
		else
			sample[i] = prev_sample[i];
	}
}

void ads2_data_callback(float *sample)
{
	// Low pass IIR filter
	signal_filter(sample);
	deadzone_filter(sample);

	ang[0] = sample[0];
	ang[1] = sample[1];
	ads2_newData = true;
}

inline int16_t ads_int16_decode(const uint8_t *p_encoded_data)
{
	return ((((uint16_t)(p_encoded_data)[0])) |
			(((int16_t)(p_encoded_data)[1]) << 8));
}

inline uint16_t ads_uint16_decode(const uint8_t *p_encoded_data)
{
	return ((((uint16_t)(p_encoded_data)[0])) |
			(((uint16_t)(p_encoded_data)[1]) << 8));
}

inline uint8_t ads_uint16_encode(uint16_t value, uint8_t *p_encoded_data)
{
	p_encoded_data[0] = (uint8_t)((value & 0x00FF) >> 0);
	p_encoded_data[1] = (uint8_t)((value & 0xFF00) >> 8);
	return sizeof(uint16_t);
}

void ads2_two_axis_parse_read_buffer(uint8_t *buffer)
{
	if (buffer[0] == ADS2_SAMPLE)
	{
		float sample[2];

		int16_t temp = ads_int16_decode(&buffer[1]);
		sample[0] = (float)temp / 32.0f;

		temp = ads_int16_decode(&buffer[3]);
		sample[1] = (float)temp / 32.0f;

		// Serial.println("ads_two_axis_parse_read_buffer");

		if (ads2_sample_callback)
			ads2_sample_callback(sample); // call user sample callback
	}
}

int ads2_two_axis_run(bool run)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_RUN;
	buffer[1] = run;

	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

int ads2_two_axis_set_sample_rate(ADS2_SPS_T sps2)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_SPS;
	ads_uint16_encode(sps2, &buffer[1]);

	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

int ads2_two_axis_enable_interrupt(bool enable)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_INTERRUPT_ENABLE;
	buffer[1] = enable;

	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

int ads2_two_axis_update_device_address(uint8_t device, uint8_t address)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_SET_ADDRESS;
	buffer[1] = address;

	if (ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE) != ADS_OK)
		return ADS_ERR_IO;

	ads2_hal_set_address(address);

	return ADS_OK;
}

int ads2_two_axis_init(ads2_init_t *ads2_init)
{
	ads2_hal_init(&ads2_two_axis_parse_read_buffer, ads2_init->reset_pin, ads2_init->ADS2_datardy_pin); // ads_read_callback was set in ads2_hal_init() to point to ads_two_axis_parse_read_buffer().
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

	if (ads2_two_axis_set_sample_rate(ads2_init->sps2)) // Set sample rate
		return ADS_ERR;

	ads_hal_delay(2);

	return ADS_OK;
}

int ads2_two_axis_enable_axis(uint8_t axes_enable)
{
	if (!(axes_enable & (ADS_AXIS_0_EN | ADS_AXIS_1_EN)))
		return ADS_ERR_BAD_PARAM;

	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_AXES_ENALBED;
	buffer[1] = axes_enable;

	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

int ads_two_axis_shutdown(void)
{
	uint8_t buffer[ADS2_TRANSFER_SIZE];

	buffer[0] = ADS2_SHUTDOWN;

	return ads2_hal_write_buffer(buffer, ADS2_TRANSFER_SIZE);
}

int ads2_two_axis_wake(void)
{
	// Reset ADS to wake from shutdown
	ads2_hal_reset();

	// Allow time for ADS to reinitialize
	ads_hal_delay(100);

	return ADS_OK;
}

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

int ads2_get_dev_type(ADS2_DEV_TYPE_T *ads2_dev_type)
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

	if (enable)
	{
		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);
	}
	else
	{
		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));
	}
}

int ads2_hal_write_buffer(uint8_t *buffer, uint8_t len)
{
	// Disable the interrupt
	if (_ads2_int_enabled)
		detachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN));

	I2C_1.beginTransmission(0x13);
	uint8_t nb_written = I2C_1.write(buffer, len);
	I2C_1.endTransmission();

	// Enable the interrupt
	if (_ads2_int_enabled)
	{
		attachInterrupt(digitalPinToInterrupt(ADS2_INTERRUPT_PIN), ads2_hal_interrupt, FALLING);

		// Read data packet if interrupt was missed
		if (digitalRead(ADS2_INTERRUPT_PIN) == 0)
		{
			if (ads2_hal_read_buffer(read_buffer, ADS2_TRANSFER_SIZE) == ADS_OK)
			{
				ads2_read_callback(read_buffer);
			}
		}
	}
	if (nb_written == len)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

int ads2_hal_read_buffer(uint8_t *buffer, uint8_t len)
{
	I2C_1.requestFrom(_address2, len);

	uint8_t i = 0;

	while (I2C_1.available())
	{
		buffer[i] = I2C_1.read();
		//  Serial.println(buffer[i], HEX);
		i++;
	}
	if (i == len)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

void ads2_hal_reset(void)
{
	// Configure reset line as an output
	pinMode(ADS_RESET_PIN, OUTPUT);
	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 0);
	ads_hal_delay(10);
	ads2_hal_gpio_pin_write(ADS_RESET_PIN, 1);

	pinMode(ADS_RESET_PIN, INPUT_PULLUP);
}

int ads2_hal_select_device(uint8_t device)
{
	if (device < ADS_COUNT)
		_address2 = ads2_addrs[device];
	else
		return ADS_ERR_BAD_PARAM;

	return ADS_OK;
}

int ads2_hal_update_device_addr(uint8_t device, uint8_t address)
{
	if (device < ADS_COUNT)
		ads2_addrs[device] = address;
	else
		return ADS_ERR_BAD_PARAM;

	_address2 = address;

	return ADS_OK;
}

uint8_t ads2_hal_get_address(void)
{
	return _address2;
}

void ads2_hal_set_address(uint8_t address)
{
	_address2 = address;
}

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
struct VP
{
	float v;
	int pct;
};
static const VP curve[] =
	{
		{4.20, 100}, {4.10, 90}, {3.98, 80}, {3.92, 70}, {3.87, 60}, {3.82, 50}, {3.79, 40}, {3.77, 30}, {3.72, 20}, {3.69, 10}, {3.50, 0}};

int liionPercentFromVoltage(float vbat)
{
	if (vbat >= curve[0].v)
		return 100;
	if (vbat <= curve[sizeof(curve) / sizeof(curve[0]) - 1].v)
		return 0;

	// find segment and interpolate
	for (size_t i = 0; i < (sizeof(curve) / sizeof(curve[0]) - 1); ++i)
	{
		if (vbat <= curve[i].v && vbat >= curve[i + 1].v)
		{
			float x = (vbat - curve[i + 1].v) / (curve[i].v - curve[i + 1].v);
			return (int)roundf(curve[i + 1].pct + x * (curve[i].pct - curve[i + 1].pct));
		}
	}
	return 0;
}

void setup()

{
	/*ESP32-S3-WROOM-1 (16MB flash 8MB RAM) */

	Serial.begin(115200); //     43 Tx ,44 Rx  console port
	// espnow_setup();

	BLE_Init();

	Serial.println("BLE Started");
	Serial.println("Initializing Two Axis sensor");
	ads2_init_t init2;

	init2.sps2 = ADS2_10_HZ;
	init2.ads2_sample_callback = &ads2_data_callback; // ads1_sample_callback is set to point to your function ads2_data_callback.
	init2.reset_pin = ADS_RESET_PIN;
	init2.ADS2_datardy_pin = ADS2_INTERRUPT_PIN;

	int ret_val = ads2_two_axis_init(&init2); //   ads2_two_axis_init

	if (ret_val == ADS_OK)
	{
		Serial.println("Two Axis ADS2 initialization succeeded");
	}
	else
	{
		Serial.print("Two Axis ADS2 initialization failed with reason: ");
		Serial.println(ret_val);
	}
	// RUN ADS2 SENSOR (0x13)
	ads2_hal_set_address(ADS2_DEFAULT_ADDR);
	ads2_two_axis_run(true);
	delay(1200);

	Serial.println("] Device Ready");
	check_saved_mac();
}

void loop()
{
	// paired_status();
	/* I2C Data receing interupt calling from two axis */
	if (ads2_irq_pending)
	{
		noInterrupts();
		ads2_irq_pending = false;
		interrupts();
		ads2_service_irq_once();
	}

	if (ads2_newData)
	{
		// ads2_newData = false;
		Horiz_Angle = ang[0];
		Verti_Angle = ang[1];

		// Serial.print(ang[0]);
		// Serial.print(",");
		// Serial.println(ang[1]);
	}
	if (ads2_newData)
	{

		// ads2_newData = false;
		int16_t s2x = (int16_t)(ang[0] * 100);
		int16_t s2y = (int16_t)(ang[1] * 100);

		if (deviceConnected && sendData)
		{
			uint8_t dataPacket[10] = {0};
			sampleCount++;
			dataPacket[0] = 0xBC;

			dataPacket[1] = (s2x >> 8) & 0xFF;
			dataPacket[2] = s2x & 0xFF;

			dataPacket[3] = (s2y >> 8) & 0xFF;
			dataPacket[4] = s2y & 0xFF;

			dataPacket[5] = (sampleCount >> 24) & 0xFF;
			dataPacket[6] = (sampleCount >> 16) & 0xFF;
			dataPacket[7] = (sampleCount >> 8) & 0xFF;
			dataPacket[8] = (sampleCount) & 0xFF;

			dataPacket[9] = 0xBD;
			// Serial.println(sampleCount);
			pTxCharacteristic->setValue(dataPacket, sizeof(dataPacket));
			pTxCharacteristic->notify();
			delay(10);
			memset(dataPacket, 0, sizeof(dataPacket));
			// Debug Print
			// Serial.print("Packet Count = ");
			// Serial.print(sampleCount);
			//  Serial.print(" | Data = ");

			//  for(int i = 0; i < 14; i++)
			//  {
			//      Serial.printf("%02X ", dataPacket[i]);
			//     }

			//        Serial.println();
		}
	}
}

void paired_status(void)
{
	if (!paired)
	{
		if (millis() - lastPairRequest >= 2000)
		{
			lastPairRequest = millis();

			sendPairRequest();
		}
	}
	// ========================================================
	// PAIRED
	// ========================================================

	else
	{
		static bool printed = false;

		if (!printed)
		{
			printed = true;

			Serial.println();
			Serial.println(
				"NORMAL MODE");

			Serial.print(
				"Partner MAC: ");

			printMAC(peerMAC);

			Serial.println();

			Serial.println(
				"Pairing stopped.");
		}
		// ----------------------------------------------------
		// SEND COUNTER
		// ----------------------------------------------------

		// if (millis() - lastDataSend >= 1000)
		// {
		//     lastDataSend = millis();

		//     sendCounter();
		// }
	}
}
/* End of Loop */
