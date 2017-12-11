//#include <SoftwareSerial.h> // software serial does not work on the pins used for serial to the CO2 sensor unit due to not being interrupt pins

//#include "NeoSWSerial.h"



#include <Wire.h>
#include <I2C_FRAM.h>
#include <K30_CO2_lib.h>
#include "RTClib.h"
#include <XBee.h>

#include <avr/power.h>


#define BAUD_RATE 115200
#define RADIO_BAUD_RATE 19200

#define HOLD_TIME_TO_PAIRING 8000

#define debug true
#define device_id 0x01000001 // top 2 bytes reserved for the board type
//#define device_id_low  0x00000001
#define sensor_type 2 //Board id type. 1: ADC, 2: CO2 sensor, 3: Temp/RH, 4: --- (undefined)

#define network_channel_default 0x0E
#define network_ID_default 0xABCD
#define network_channel_PAIRING 0x0F
#define network_ID_PAIRING 0xAAAA

#define base_station_address_high_default 0x00000001
#define base_station_address_low_default 0x00000001

#define buffer_size_default             64
#define sample_interval_seconds_default 600
#define upload_time_interval_sampling_default 600
#define upload_time_interval_idle_default 300
#define next_upload_Hour_default        255
#define next_upload_Minutes_default     255
#define next_upload_Seconds_default     0
#define next_upload_Date_default        255
#define next_upload_Months_default      255
#define next_upload_years_default       255

/*============ Radio Variables =======================*/

XBee xbee = XBee();

#define maxPayloadsize 48
#define maxReceiveLoadSize 49

const uint8_t samplePayloadDataOffset = 4;
uint8_t payload[maxPayloadsize];
uint8_t receivedLoad[maxReceiveLoadSize];
uint8_t recievedDataLength;

uint8_t errorCode;

uint16_t sample_index = 0;

//uint32_t base_station_address_high;
//uint32_t base_station_address_low;

// SH + SL Address of receiving XBee
XBeeAddress64 addr64;


ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBRxResponse radio_rx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
AtCommandRequest AtCMD = AtCommandRequest();
AtCommandResponse AtCMDres = AtCommandResponse();

#define radio_DTR 10
#define radio_RX_strength 5
#define radio_reset 13
#define radio_CTS 8
#define radio_RTS 12
#define radio_PWR_EN 4
#define radio_sleep_status 6


/*============ Radio data packet Commands =======================*/
#define ACK               0x01
#define NACK              0x02
//#define DACK              0x02
#define ACK_REQUEST       0x03
#define STATUS_REQUEST    0x04
#define STATUS_DATA       0x05


#define SAMPLE_TRANSMIT_REQUEST_NEXT 0X0A
#define SAMPLE_TRANSMIT_DATA    0X0B
#define SAMPLE_END_OF_DATA      0X0C
#define SAMPLE_TRANSMIT_REQUEST_INDEX 0X0D

#define END_COMMUNICATION       0X10
#define END_COMMUNICATION_ACK   0X11

#define PAIR_REQUEST_CHECK      0X20
#define PAIR_REQUEST            0X22

#define SU_INFO_HIGH_REQUEST    0X30
#define SU_INFO_HIGH_DATA       0X31
#define SU_INFO_LOW_REQUEST     0X32
#define SU_INFO_LOW_DATA        0X33

#define SU_SENSOR_SETTING_REQUEST  0X36
#define SU_SENSOR_SETTING_DATA     0X37
#define SU_SENSOR_SETTING_UPDATE   0X38

#define UPDATE_BASE_STATION_INFO   0X40
#define UPDATE_DATE_TIME_INFO      0X42
#define DATA_SAMPLE_LAYOUT_REQUEST 0X50
#define DATA_SAMPLE_LAYOUT         0X51



/*============ Misc constants =======================*/
#define led_R 20
#define led_G 21
#define Vbus_line A5
#define Vbat_level A4

#define main_power_en 7

volatile uint8_t LED_PULSE_BIT = 0;

bool pairing_mode = false;

/*=============== RTC variables ========================*/

RTC_DS3231 rtc;

uint8_t nextAlarmTime_minutes = 0;
uint8_t nextAlarmTime_hours = 0;

const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

bool powerLoss = false;

// scratch space for comparing date times in seconds etc that need 32 bits or higher
uint64_t temp_time_1;
uint64_t temp_time_2;
uint64_t temp_time_3;

/*=============== CO2 sensor variables =======*/
#define CO2_RX 19 //19
#define CO2_TX 18 //18
#define CO2_PWR_EN 11

const uint16_t CO2_warmup_time = 90; // the time in seconds the CO2 sensor must run before reliable results can be obtained
const uint16_t CO2_powerDown_offset = 30; // The minimum time interval that the CO2 sensor can be powered down before loosing the benefit of saving power to the sensors auto calibration.
//#define C02_STATUS_PIN // cant define PD5 as a pin number as it is not selectable with this board firmware. Use port registers to change its pin value/direction

// --- variables for using Serial communication----

//SoftwareSerial CO2Serial(CO2_RX, CO2_TX);
volatile uint32_t newlines = 0UL;

byte CO2buff[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
const int valMultiplier = 1;


// ---- variables for using I2C communication----

K30_CO2_lib K30_CO2;
#define CO2Addr 0x7F



/*============Variables for FRAM =======================*/
I2C_FRAM fram;

// Config FRAM addresses
const uint8_t su_Config_addr = 0; // assume config byte size is max 128 bytes long
const uint8_t data_sample_buffer_count = 128; // two bytes long. address to the current count of buffered data samples in FRAM

//const uint8_t baseStation_address64_high = 130; // network address of the paired base station unit. 4 bytes each
//const uint8_t baseStation_address64_low = 134;


const uint8_t next_sampleTime_seconds_addr = 138; // next time to wake up and take a data sample. 1 byte each
const uint8_t next_sampleTime_minutes_addr = 139;
const uint8_t next_sampleTime_hours_addr = 140;
const uint8_t next_sampleTime_day_addr = 141;
const uint8_t next_sampleTime_month_addr = 142;
const uint8_t next_sampleTime_year_addr = 143;

const uint8_t next_upload_attempt_minutes = 144; // next time to attempt to upload to the base station, both 1 byte long
const uint8_t next_upload_attempt_seconds = 145;

const uint8_t wakeup_check_flag = 146; // byte used to store the current wakeup source to help determining if device should be in pairing mode.


// Error history FRAM addresses

const uint8_t errCodeFramAddr  = 147; //FRAM address for stored error codes. (2 bytes)
const uint8_t errCodeNumTotal_addr = 149; // address for total number of error codes stored, 1 byte
const uint8_t errCodeNum_addr = 150;  // current position counter for error code history, 1 byte
const uint8_t errCodeHistory_start_addr = 151; // each error code is 2 bytes with time data of 6 bytes, thus 8 bytes long

// extra fixed variables for finding the sample data start address
const uint8_t errCodeMaxNum = 32; // maximum number of error codes to store
const uint8_t errCode_byte_length = 8;

// sample data starting point FRAM address.
const uint16_t sample_data_start_addr  = errCodeHistory_start_addr + errCode_byte_length * errCodeMaxNum; //start address for sample data. Must not save sample data before this.


//uint16_t currAddrFRAM = 0;
//uint16_t sampleCount = 0;
//uint16_t co2SampleCount = 0;



/*============ Sensor unit config settings    ========================*/
#define SUConfig_FRAM_byte_length 32;

struct SUConfig {
	uint8_t network_channel;
	uint16_t network_ID;
	uint32_t base_station_address_high;	//Address to the base station unit
	uint32_t base_station_address_low;
	
	uint32_t sample_interval_seconds; // sample interval in seconds
	uint16_t buffer_size;
	uint8_t run_sampling;
	
	uint32_t upload_time_interval_sampling ; // the time interval used to set the upload time while sampling is enabled
	uint32_t upload_time_interval_idle ; // the time interval used to set the upload time while sampling is disabled
	
	uint8_t next_upload_Hour;
	uint8_t next_upload_Minutes;
	uint8_t next_upload_Seconds;
	uint8_t next_upload_Date;
	uint8_t next_upload_Months;
	uint8_t next_upload_years;
	
	uint8_t errorCountSinceLastUpload;

	uint8_t checksum;

};
SUConfig SUConfig;


/*============ Sample data settings    ========================*/
#define datasample_FRAM_byte_length 9 // max possible byte length assumed by the entire system is 46 bytes. last 1 byte for the checksum.
const uint16_t max_sample_count = 256;
struct DataSample {
  uint8_t sample_hour;
  uint8_t sample_minute;
  uint8_t sample_second;
  uint8_t sample_date;
  uint8_t sample_month;
  uint8_t sample_year;

  //float si7051_temperature;
  //float SHT35_temperature;
  //float SHT35_humidity;
  uint16_t CO2;

  //uint16_t adsSingle[MAX_ADS1115_INPUTS]; //variables for storing temporary sinlge ended analog readings
  //int16_t adsDouble[MAX_ADS1115_INPUTS / 2]; //variables for storing temporary double ended analog readings

  uint8_t checksumValue;
};
DataSample temp_sample;

/*
   Data type layout is used to inform the base station of which and what order of bytes are
   used for each variable type in a data sample when broken down into the wireless transmission payload.
*/
const char data_type_layout[] = {'c', 'E'}; // data type layout does not include date/time as they should be all uint8_t, and also does not include checksum as it is always uint8_t
/* Data type    | char symbol representation
-------------|------------------------------------
char         | a
uint8        | b
uint16       | c
uint32       | d
int8         | e
int16        | f
int32        | g
float        | F
double       | D
|
END          | E


*/
/*======================================================*/







static void handleRxChar( uint8_t c )
{
	if (c == '\n')
	newlines++;
}

/*
	Interrupt Service Routine to asynchronously handle the Pairing LED notification pulsing.
*/
ISR(TIMER1_COMPA_vect)
{
	if( OCF1A == 1){
		//digitalWrite(led_R, ~LED_PULSE_BIT);
		//digitalWrite(led_G, LED_PULSE_BIT);
		if(LED_PULSE_BIT != 0){
			LED_PULSE_BIT = ~LED_PULSE_BIT;
			//digitalWrite(led_R, LOW);
			//digitalWrite(led_G, HIGH);
		}
		else{
			//digitalWrite(led_R, HIGH);
			//digitalWrite(led_G, LOW);
			PINF |= _BV(PINB5);
			PINF |= _BV(PINB4);
			LED_PULSE_BIT = ~LED_PULSE_BIT;
		}

		//TCNT1H = 0x00;
		//TCNT1L = 0x00;
		//LED_PULSE_BIT = ~LED_PULSE_BIT;
		
		TIFR1 &= ~_BV(OCF1A);
	}
}



void setup() {
	// put your setup code here, to run once:
	//delay(10000);
	// pull stay awake pin High to keep micro on
	digitalWrite(main_power_en, HIGH);
	pinMode(main_power_en, OUTPUT);

	// enable CO2 sensor power
	//digitalWrite(CO2_PWR_EN, LOW);		
	//pinMode(CO2_PWR_EN, OUTPUT);
	
	startCO2Sensor(); // CO2 turned on due to issue with I2C connection on CO2 unit disabling I2C for whole board if powered off
	


	delay(500); // delay to help upload new code before JTAG interface is software disabled

	MCUCR = (1 << JTD); // software disable the JTAG interface to allow Status LEDs to work,
	MCUCR = (1 << JTD); // while still allowing the JTAG interface to program the ATMEGA32u4.
	delay(20);

	pinMode(led_G, OUTPUT);
	digitalWrite(led_G, HIGH);  // turn on green LED to indicate setting up

	//SPCR &= ~(1<<4); // set SPI unit in master mode
	//SPCR &= ~(1 << 6); //disable SPI unit on startup
	Serial.begin(BAUD_RATE);
//	if(debug && checkUSBConnected() == true){
		while (!Serial);
//	}

	power_spi_disable();
	
	if(debug){
		Serial.println(F("Setting up I2C devices"));
	}
	
	beginI2cDevices();




	/*
		set all pins to Radio low
	*/

	//digitalWrite(17, LOW);
	//pinMode(17, INPUT);
	//set slave select pin to tri state
	DDRB &= ~(1 << 0); //
	PORTB &= ~(1 << 0);

	pinMode(radio_PWR_EN, INPUT);
	pinMode(radio_sleep_status, INPUT);
	pinMode(radio_CTS, INPUT);
	pinMode(radio_reset, INPUT);
	digitalWrite(radio_PWR_EN, LOW);
	digitalWrite(radio_sleep_status, LOW);
	digitalWrite(radio_CTS, LOW);
	digitalWrite(radio_reset, LOW);

	//


	// load in SUConfig settings from FRAM
	if(debug){
		Serial.println(F("Loading Configuration data.."));
	}
	loadSUConfig();
	if (debug) {
		Serial.println(F("starting"));
		Serial.print(F("Device ID: "));
		Serial.println(device_id);
		//Serial.print(F("Device ID Low: "));
		//Serial.println(device_id_low);
		Serial.print(F("Device Type: "));
		Serial.println(sensor_type);

	}

	pinMode(led_G, OUTPUT);
	pinMode(led_R, OUTPUT);
	pinMode(Vbus_line, INPUT);
	digitalWrite(led_G, HIGH);
	digitalWrite(led_R, HIGH);

	Serial.print(F("Vbus level = "));
	Serial.println(digitalRead(Vbus_line));

	//DateTime alarmtime(2017, 8, 9, 255, 255, 255);

	//rtc.setAlarm2Time(255, 255, 255, false);
	//rtc.enableRTC_DS3231Interrupts(true);
	//rtc.enableRTCAlarm(false, false);

	digitalWrite(led_G, LOW); // set Green LED to off as setup should be completed

	//delay(1000);
	//Serial1.begin(RADIO_BAUD_RATE);
	//xbee.begin(Serial1);
	//delay(5000);
	//Serial.println("waiting for Xbee to initialize");

	//SUConfig.buffer_size = 32;

}

// used to re-initialize all sensors that rely on I2C or SPI modules
void beginI2cDevices() {
	Wire.begin();
	if(debug){
		Serial.println(F("Starting FRAM"));
	}
	fram.begin(0x51);
	
	if(debug){
		Serial.println(F("Setting I2C clock for RTC"));
	}
	Wire.setClock(400000); // set I2C clock rate to 400 KHz
	
	if(debug){
		Serial.println(F("Starting RTC"));
	}
	
	rtc.begin();

	if (rtc.lostPower()) {
		//Serial.println("RTC lost power, lets set the time!");
		// following line sets the RTC to the date & time this sketch was compiled
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
		powerLoss = true;
	}

	rtc.writeSqwPinMode(DS3231_OFF);  // disable square wave output
	//Wire.setClock(400000); // set I2C clock rate to 400 KHz

}

bool checkUSBConnected() {
	digitalRead(Vbus_line);
	return (digitalRead(Vbus_line)) == 1;
}


/*

*/
void loop() {
  //digitalWrite(led_G, LOW);
  //digitalWrite(led_R, LOW);
  if(powerLoss == true){
	  
	  radio_pairing_mode();
	  if(SUConfig.run_sampling == 1){
		  updateNextSampleTime(true);
	  }
	  powerLoss = false;
  }

  
  // check if RTC was wakeup source, if is was, check if time matches a known wakeup time.
  if(rtc.clearRTCAlarm1(false)){
	  if(debug){
		  Serial.println(F("Alarm triggered!"));
	  }
	  
	  if(checkSampleTimeNow() && SUConfig.run_sampling == 1){
		  takeSample();
		  //sample_count = fram.readUInt16(data_sample_buffer_count);
		  //if (sample_count >= SUConfig.buffer_size) {															// remove this later!!!!!!
		  //	//uploadDataWireless();
		  //}
	  }
	  if(checkUploadTimeNow()){
		  waitForPacket(45000);
		  updateNextSampleTime(true);
		  //uploadDataWireless();
	  }
	  
	  // If the wakeup check flag was set, run the method to verify if the device had a normal wake button press.
	  if(fram.readByte(wakeup_check_flag) == 0x01){
		  if(debug){
			  Serial.println(F("Verifying wakeup source..."));
		  }
		  check_wakeup_event(true);
	  }
	  
  }
  // if wakeup event was not the RTC,
  else {
	  if(check_wakeup_event(false)){
		  powerDown();
	  }
	  
	  
  }


  

  //if (!checkUSBConnected()) {
  
  // set new alarm time and disable power to micro
  if(debug){
	  Serial.println(F("Setting next alarm time."));
  }
  setNextAlarmTime();
  if(debug){
	  Serial.println(F("Powering Down."));
  }
  powerDown();
}




/*

*/
void takeSample() {
	uint16_t sample_count;
	DateTime now;
	sample_count = fram.readUInt16(data_sample_buffer_count);
	Serial.println();

	if ( sample_count >= max_sample_count) {
		sample_count = 0;
		if (debug && checkUSBConnected()) {
			Serial.println(F("max sample count reached. resetting sampling buffer index"));
			Serial.print(F("sample buffer size: "));
			Serial.println(SUConfig.buffer_size);
		}
	}
	
	readCO2();

	now = rtc.now();

	temp_sample.sample_hour = now.hour();
	temp_sample.sample_minute = now.minute();
	temp_sample.sample_second = now.second();
	temp_sample.sample_date = now.day();
	temp_sample.sample_month = now.month();
	if(now.year() >= 2000){
		temp_sample.sample_year = now.year() - 2000;
	}
	else{
		temp_sample.sample_year = now.year();
	}
	
	if (debug && checkUSBConnected()) {
		Serial.print(temp_sample.sample_hour);
		Serial.print(':');
		Serial.print(temp_sample.sample_minute);
		Serial.print(':');
		Serial.print(temp_sample.sample_second);
		Serial.print(' ');
		Serial.print(temp_sample.sample_date);
		Serial.print('/');
		Serial.print(temp_sample.sample_month);
		Serial.print('/');
		Serial.println(temp_sample.sample_year);
		
		
		Serial.print(F(" Current Sample count: "));
		Serial.println(sample_count);
	}
	
	

	
	saveDataSampleToFRAM();
	fram.writeUInt16(data_sample_buffer_count, sample_count + 1); // increment sample count
	sample_count = fram.readUInt16(data_sample_buffer_count);
	stopCO2Sensor();
	updateNextSampleTime(true);
	
	//delay(20);
}



void readCO2() {
	uint32_t startTime = millis();
	uint32_t warmup_time = CO2_warmup_time; 
	uint8_t buffersize = 5;
	int16_t buffer[buffersize]; 
	uint32_t avg = 0;
	uint8_t count = 0;
	
	//startCO2Sensor();
	
	//Serial.println(F("start reading test"));
	//sendCO2Request();

	//Serial.println(F("start reading"));
	// read CO2 samples into a buffer until the warm up time has passed.
	while( startTime + (warmup_time*1000) > millis()){
		if(count >= buffersize ){
			count = 0;
		}
		//Serial.print('.');
		//sendCO2Request();	// read CO2 value using serial
		//Serial.println(F("Received response"));
		//buffer[count] = getCO2Value(response);// read CO2 value using serial
		
		buffer[count] = K30_CO2.readCO2();	// read co2 value using I2C
		count++;
	}
	
	count = 0;
	for(uint8_t i = 0; i < buffersize; i++){
		if(buffer[i] >= 0){
			avg += buffer[i]; // careful here, as its a signed integer being added to an unsigned integer.
			count++;
		}
	}

	// store the averaged sample 
	temp_sample.CO2 = avg/count;


	if (debug && checkUSBConnected()) {
		Serial.print(F(" (K30) CO2= "));
		Serial.print(temp_sample.CO2);
		Serial.println(F(" ppm"));
	}
	
	stopCO2Sensor();

}


/*
void sendCO2Request()
{
	//CO2Serial.flush();
	Serial.println(F("sending command"));
	while (!CO2Serial.available()) //keep sending request until we start to get a response
	{

		CO2Serial.write(CO2buff, 7);

		delay(50);
	}
	Serial.println(F("Receiving response"));
	int timeout = 0; //set a timeout counter
	while (CO2Serial.available() < 7 ) //Wait to get a 7 byte response
	{
		timeout++;
		if (timeout > 10)   //if it takes to long there was probably an error
		{
			while (CO2Serial.available()) //flush whatever we have
			CO2Serial.read();

			break;                        //exit and try again
		}
		delay(50);
	}
	
	Serial.println(F("Reading response"));

	for (int i = 0; i < 7; i++)
	{
		response[i] = CO2Serial.read();

	}

}
*/

unsigned long getCO2Value(byte packet[])
{
	int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
	int low = packet[4];                         //low byte for value is 5th byte in the packet


	unsigned long val = high * 256 + low;              //Combine high byte and low byte with this formula to get value
	return val * valMultiplier;
}



void startCO2Sensor(){
	uint8_t port_D_val = PIND;
	pinMode(CO2_PWR_EN, OUTPUT);
	digitalWrite(CO2_PWR_EN, HIGH);
	while( ((port_D_val >> 5) & 0x01) != 1){//wait until the status pin goes high to indicate successful power on of CO2 sensor.
		port_D_val = PIND;
		//Serial.print(((PIND >> 5) ), BIN)	; 
	}
	//delay(200);
	K30_CO2.begin(CO2Addr);	// used with I2C
	//CO2Serial.begin(9600);		// used with SoftwareSerial
	//CO2Serial.attachInterrupt( handleRxChar ); // used with NeoSWSerial
	//CO2Serial.begin( 9600 );
	
}


void stopCO2Sensor(){

	if(SUConfig.sample_interval_seconds >= ((uint32_t)CO2_warmup_time + (uint32_t)CO2_powerDown_offset)){
		digitalWrite(CO2_PWR_EN, LOW);
		pinMode(CO2_PWR_EN, INPUT);
	}
	//CO2Serial.end();
}








void saveDataSampleToFRAM() {
	uint16_t addr = sample_data_start_addr + fram.readUInt16(data_sample_buffer_count) * datasample_FRAM_byte_length;

	if (debug && checkUSBConnected()) {
		Serial.print(F("Saving to Fram at address:"));
		Serial.println(addr, HEX);
		//printDataSample();
	}

	while (fram.readByte(addr) != temp_sample.sample_hour ) {
		fram.writeByte(addr, temp_sample.sample_hour );
	}
	temp_sample.checksumValue = temp_sample.sample_hour;
	addr += sizeof(byte);

	while (fram.readByte(addr) != temp_sample.sample_minute ) {
		fram.writeByte(addr, temp_sample.sample_minute );
	}
	temp_sample.checksumValue += temp_sample.sample_minute;
	addr += sizeof(byte);

	while (fram.readByte(addr) != temp_sample.sample_second ) {
		fram.writeByte(addr, temp_sample.sample_second );
	}
	temp_sample.checksumValue += temp_sample.sample_second;
	addr += sizeof(byte);

	while (fram.readByte(addr) != temp_sample.sample_date ) {
		fram.writeByte(addr, temp_sample.sample_date );
	}
	temp_sample.checksumValue += temp_sample.sample_date;
	addr += sizeof(byte);

	while (fram.readByte(addr) != temp_sample.sample_month ) {
		fram.writeByte(addr, temp_sample.sample_month );
	}
	temp_sample.checksumValue += temp_sample.sample_month;
	addr += sizeof(byte);

	while (fram.readByte(addr) != temp_sample.sample_year ) {
		fram.writeByte(addr, temp_sample.sample_year );
	}
	temp_sample.checksumValue += temp_sample.sample_year;
	addr += sizeof(byte);


	while (fram.readUInt16(addr) != temp_sample.CO2 ) {
		fram.writeUInt16(addr, temp_sample.CO2 );
	}
	temp_sample.checksumValue += temp_sample.CO2;
	addr += sizeof(uint16_t);



  // write checksum value of data sample to FRAM
  while ((fram.readByte( addr) != temp_sample.checksumValue) ) {
    fram.writeByte( addr, temp_sample.checksumValue);
  }

}



/*
This method checks what source was used to wake up the device.
If the real time clock woke the device up, verify that the wakeup source was either a sample, upload data or wakeup check source. log error otherwise.

If the RTC did not trigger the event, then either a power enable jumper is set, USB power on jumper is set or a wake button was pressed.
If the wake button is held for longer than a threshold, the device should be put into pairing mode.

*/
bool check_wakeup_event(bool wake_flag_was_set){
	uint8_t delayTimeToWake = 7;
	uint32_t start_time = millis();
	bool start_pairing_mode = false;
	uint8_t temp;
	
	uint8_t usb_connected = digitalRead(Vbus_line); // if this pin reads high, the usb is connected and possibly set to be always on when usb is present.
	
	// if the power was lost but the wake flag was set, a normal wake button press occurred, so proceed as normal.
	if(wake_flag_was_set && fram.readByte(wakeup_check_flag ) == 1 ){
		if(debug){
			Serial.println(F("Continuing as normal."));
		}
		setNextAlarmTime();
		return true;
	}
	else if(wake_flag_was_set && fram.readByte(wakeup_check_flag ) == 0 ){
		return false;
	}
	else{
		// if the wake flag was not set, set it and turn off the main power enable pin to see if the device looses power.
		// If it does not lose power, the wake button is being held so enter pairing mode.
		if(debug){
			Serial.println(F("Setting wake flag and waiting..."));
		}
		fram.writeByte(wakeup_check_flag, 0x01 );
		
		// set the alarm to wake up device if button is not being held
		DateTime now = rtc.now();
		temp = now.second() + delayTimeToWake;
		if(temp > 59){
			temp -= 60;
		}
		setNextAlarmTime(true, temp, 255, 255 ,255); // set unused time value to 255 to disable them
		
		pinMode(main_power_en, OUTPUT);
		digitalWrite(main_power_en, LOW);
		while(true){
			
			if(HOLD_TIME_TO_PAIRING + start_time < millis()){
				start_pairing_mode = true;
				digitalWrite(main_power_en, HIGH);
				//Serial.println(F("Wake button was held..."));
				break;
			}
		}
	}
	
	
	
	if(start_pairing_mode == true){
		
		fram.writeByte(wakeup_check_flag, 0x00 );
		if(debug){
			Serial.println(F("Starting pairing mode."));
		}
		radio_pairing_mode();
		
	}
	
	return false;
	
	
}








/*
	Switches off the micro controllers control over the main power converter. external events such as a wake button press,
	jumpers or a real time clock interrupt can turn/ keep the main power converter on.
*/
void powerDown() {
	//rtc.clearRTCAlarm2(true); // be carefull, this could be dangerous clearing it here. should be cleared as soon as possible.
	stopCO2Sensor();
	if(debug){
		Serial.end();
	}
	delay(5);
	digitalWrite(main_power_en, LOW);
	delay(3000); // add delay to allow time for capacitors to discharge.
}





/*
	This method turns on (enables power to it) and does some basic configuring of the Radio module.
*/
void start_radio() {
	pinMode(radio_PWR_EN, OUTPUT);
	pinMode(radio_RTS, OUTPUT);
	pinMode(radio_CTS, INPUT);

	digitalWrite(radio_PWR_EN, HIGH);
	digitalWrite(radio_RTS, LOW);


	Serial1.begin(RADIO_BAUD_RATE);
	xbee.setSerial(Serial1);
	//digitalWrite(radio_RTS, LOW);
	memset(receivedLoad, 0, sizeof(receivedLoad));



	// pause till Radio is able to receive data
	while (digitalRead(radio_CTS) == 1) {
	}
	delay(200);
	
	if(pairing_mode == true){
		setRadioNetworkCH(network_channel_PAIRING);
		setRadioNetworkID(network_ID_PAIRING);
		
	}
	else{
		//while(!Serial);
		
		while(!setRadioNetworkID(SUConfig.network_ID));
		while(!setRadioNetworkCH(SUConfig.network_channel));

		
		if(debug){
			Serial.print(F("Network Settings: "));
			Serial.print(SUConfig.network_ID, HEX);
			Serial.print(' ');
			Serial.print(SUConfig.network_channel, HEX);
			Serial.print(' ');
			Serial.print(SUConfig.base_station_address_high, HEX);
			Serial.print(' ');
			Serial.println(SUConfig.base_station_address_low, HEX);
			
			
		}
		
		//SUConfig.base_station_address_high = 0x0013A200;
		//SUConfig.base_station_address_low = 0x415B5F2B;
		//Serial.print(SUConfig.base_station_address_high, HEX);
		//Serial.println(SUConfig.base_station_address_low, HEX);
		
		addr64 = XBeeAddress64(SUConfig.base_station_address_high, SUConfig.base_station_address_low);
	}

	while(!radioExitCommandMode());

	zbTx = ZBTxRequest(addr64, payload, sizeof(payload));

}







/*
	This method ends the serial connection to the radio and turns off the power to the radio
*/
void stop_radio() {
	pinMode(radio_PWR_EN, OUTPUT);
	pinMode(radio_RTS, OUTPUT);
	pinMode(radio_CTS, INPUT);

	Serial1.end();

	digitalWrite(radio_PWR_EN, LOW);
	digitalWrite(radio_RTS, LOW);

}




void radio_pairing_mode(){
	pairing_mode = true;
	set_led_pulse();
	
	waitForPacket(120000); // set time out to 2 minutes
	
	stop_led_pulse();
}





/*
	Configures the LED's and Interrupt Service Routine to asynchronously handle the Pairing LED notification pulsing.
*/
void set_led_pulse(){
	pinMode(led_R, OUTPUT);
	pinMode(led_G, OUTPUT);
	
	digitalWrite(led_R, HIGH);
	digitalWrite(led_G, LOW);
	
	//delay(10000);
	
	//Serial.println(F("Setting Timer Prescaler"));
	//set timer/counter 1 to have a clock pre-scaler of 256
	TCCR1B	|=  _BV(CS12);
	TCCR1B	&= ~_BV(CS11);
	TCCR1B	|=  _BV(CS10);
	
	
	//Serial.println(F("Setting Timer Compare value"));
	
	// set the timer compare value on compare register A for timer 1 to 0x7FFF. The high register must be written to before the low byte register.
	OCR1AH = 0xFF;
	OCR1AL = 0xFF;
	
	
	
	TIFR1 = 0; // clear timer 1 flags
	
	//Serial.println(F("Setting Timer interrupt trigger"));
	// set timer 1 interrupt register to trigger on compare register A matching.
	TIMSK1 |= _BV(OCIE1A);
	TIMSK1 &= ~_BV(OCIE1B);
	TIMSK1 &= ~_BV(OCIE1C);
	TIMSK1 &= ~_BV(TOIE1);
	TIMSK1 &= ~_BV(ICIE1);
	
	//Serial.print(F("Setting Timer done"));

}



/*
	Stops the Interrupt Service Routine for handling the Pairing LED notification pulsing.
*/
void stop_led_pulse(){

	TIFR1 = 0; // clear timer 1 flags
	// disable timer 1 interrupt register.
	TIMSK1 &= ~_BV(OCIE1A);
	TIMSK1 &= ~_BV(OCIE1B);
	TIMSK1 &= ~_BV(OCIE1C);
	TIMSK1 &= ~_BV(TOIE1);
	TIMSK1 &= ~_BV(ICIE1);
	
	digitalWrite(led_R, HIGH);
	digitalWrite(led_G, LOW);

}




void waitForPacket(){
	waitForPacket(30000);
	
}




/*
	Starts the radio module and waits for a packet to be received. 
	Ends when the previous packet command sent is the end communication acknowledge command.
*/
void waitForPacket(uint32_t timeout_length){
	uint32_t startTime = 0;
	//uint32_t timeout_length = timeout;
	uint8_t previous_cmd = 0;
	uint8_t count  =0;
	XBeeAddress64 tempAddress = XBeeAddress64(SUConfig.base_station_address_high, SUConfig.base_station_address_low);// initialize the temp address with a non NULL value if pairing mode is enabled
	
	//if(pairing_mode != true){
	//	tempAddress = NULL; // 
	//}
	
	
	start_radio();
	
	startTime = millis();
	while(startTime + timeout_length > millis() && previous_cmd != END_COMMUNICATION_ACK ){
		
		Serial.print('.');
		if(count >= 10){
			Serial.println();
			count = 0;
		}
		if(recievePacket(300, &tempAddress)){
			
			if(debug){
				Serial.println(F("Packet Received!"));
			}
			
			
			if(verifyRecievedPacketChecksum()){
				if(debug){
					Serial.println(F("Checksum verified!"));
				}
				if(pairing_mode == true){
					addr64 = tempAddress;
					if(debug){
						Serial.print(F("Found Address: "));
						Serial.print(addr64.getMsb(), HEX);
						Serial.println(addr64.getLsb(), HEX);
					}
				}
				previous_cmd = processPacket(receivedLoad[0], previous_cmd);
				
			}
			else{
				if(debug){
					Serial.println(F("Checksum not valid!"));
				}
			}
		}
		
		count++;
	}
	Serial.println(F("Turning off Radio. . ."));
	delay(100);
	stop_radio();
	
}



bool recievePacket() {
	return recievePacket(500, NULL);
}


bool recievePacket(uint16_t timeout) {
	return recievePacket(timeout, NULL);
}




/*

*/
bool recievePacket(uint16_t timeout, XBeeAddress64 *response_address) {
	//bool recieved_command = false;
	
	if (!xbee.readPacket(timeout)) {
		return false;
	}
	if (xbee.getResponse().isAvailable()) {
		// got something

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			xbee.getResponse().getZBRxResponse(radio_rx);
			savePacketToRAM(radio_rx.getData(), radio_rx.getDataLength());
			if(pairing_mode == true && response_address != NULL){
				*response_address = radio_rx.getRemoteAddress64();
			}
			errorCode = 0xFF; //clear error code
			//Serial.println(F("packet received!"));
			return true;
		}
		else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
			xbee.getResponse().getAtCommandResponse(AtCMDres);
			saveATResponseToRAM(AtCMDres.getCommand(), AtCMDres.getStatus(), AtCMDres.getValue(), AtCMDres.getValueLength());
			if(pairing_mode == true && response_address != NULL){
				*response_address = radio_rx.getRemoteAddress64();
			}
			// the local XBee sends this response if an AT command was sent to it
			processRecievedATCommand(AtCMDres);
			return true;
		}
		else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
			xbee.getResponse().getModemStatusResponse(msr);
			// the local XBee sends this response on certain events, like association/dissociation

			if (msr.getStatus() == ASSOCIATED) {
				errorCode = 0xFF; //clear error code
				return false;
			}
			//else if (msr.getStatus() == DISASSOCIATED) {
			//        // this is awful.. flash led to show our discontent
			//        log_error(RADIO_ERROR, msr.getStatus());
			//        return false;
			//}
			else {
				// this is awful..
				//log_error(RADIO_MODEM_STATUS, msr.getStatus() );
				return false;
			}
			} else {
			// not something we were expecting
			return false;
		}
		} else if (xbee.getResponse().isError()) {
		//Error reading packet.
		errorCode = xbee.getResponse().getErrorCode();
		//log_error(RADIO_ERROR, errorCode );

		return false;

	}
	return false;
}




/*
	Checks a received packets data to see if the received checksum matches the newly calculated checksum.
*/
bool verifyRecievedPacketChecksum() {
	uint8_t checksum = 0;
	uint8_t datalength = receivedLoad[1];
	
	

	 // Serial.print(F("Data length: "));
	//  Serial.println(datalength);

	for (uint8_t i = 0; i < (datalength + 2) ; i++) { // use offset of 2 byte to include the command and length bytes, but not the checksum
		checksum += receivedLoad[i];
		//Serial.print(receivedLoad[i], HEX);
		//Serial.print(' ');
	}
	

	if ( receivedLoad[datalength + 2] == checksum) { // use offset of 1 byte to get the checksum byte
		return true;
	}
	else {
		//Serial.println();
		//Serial.print(F("Calculated checksum: "));
		//Serial.println(checksum, HEX);
		//Serial.print(F("Recieved checksum: "));
		//Serial.println(receivedLoad[datalength + 2], HEX);
		return false;
	}

}





uint8_t processPacket(uint8_t command, uint8_t previous_cmd){
	
	//uint16_t sample_count = fram.readUInt16(data_sample_buffer_count);
	uint8_t cmd = 0;
	if(debug){
		Serial.print(F("Received command is: "));
		Serial.println(command, HEX);
	}

	// If the checksum is correct, run the appropriate method for the command byte (first byte) of the received packet
	switch (command) {
		
		case ACK:
		//Serial.println(F("Recieved ACK"));
		if (previous_cmd == SAMPLE_TRANSMIT_REQUEST_NEXT || previous_cmd == SAMPLE_TRANSMIT_REQUEST_INDEX) {
			cmd = 0xFF; // clear previous command
		}
		//else if (previous_cmd == SAMPLE_END_OF_DATA) {
		//fram.writeUInt16(data_sample_buffer_count, 0);
		//sample_index = 0;
		//}

		break;


		case NACK:
		//Serial.println(F("Recieved NACK"));
		if (previous_cmd == SAMPLE_TRANSMIT_REQUEST_NEXT ) {
			sendDataSamples(sample_index - 1 );
		}
		if (previous_cmd == SAMPLE_TRANSMIT_REQUEST_INDEX) {
			sendDataSamples(sample_index);
		}
		else if ( previous_cmd == SAMPLE_END_OF_DATA) {
			sendCommand(SAMPLE_END_OF_DATA);
		}
		else if (previous_cmd == STATUS_REQUEST) {
			sendStatus();
		}
		else if (previous_cmd == SU_SENSOR_SETTING_REQUEST) {
			sendSettings();
		}
		else if (previous_cmd == ACK_REQUEST) {
			sendCommand(ACK);
		}
		break;
		

		case ACK_REQUEST:
		sendCommand(ACK);
		cmd = ACK_REQUEST;
		break;
		
		
		case STATUS_REQUEST:
		sendStatus();
		cmd = STATUS_REQUEST;
		break;
		

		case SAMPLE_TRANSMIT_REQUEST_NEXT:
		//Serial.println(F("Transmit data Request:"));
		if ( sendDataSamples(sample_index + 1) ) {
			cmd = SAMPLE_TRANSMIT_REQUEST_NEXT;
			sample_index += 1;
		}
		else {
			sendCommand(SAMPLE_END_OF_DATA);
			cmd = SAMPLE_END_OF_DATA;
		}
		//endTime = millis();
		//Serial.println(endTime - startTime);
		break;
		
		case SAMPLE_TRANSMIT_REQUEST_INDEX:	// this command allows requesting a specific sample number
		
		//if(debug){
		//	Serial.print(F("\nCurrent Index: "));
		//	Serial.print(sample_index);
		//}
		sample_index = receivedLoad[2];
		sample_index = (sample_index << 8) + receivedLoad[3];
		
		//if(debug){
		//	Serial.print(F(" Requested Index: "));
		//	Serial.println(sample_index);
		//}
		
		if ( sendDataSamples(sample_index ) ) {
			cmd = SAMPLE_TRANSMIT_REQUEST_INDEX;
		}
		else {
			sendCommand(SAMPLE_END_OF_DATA);
			cmd = SAMPLE_END_OF_DATA;
		}
		break;
		

		case END_COMMUNICATION:
		sendCommand(END_COMMUNICATION_ACK);
		sample_index = 0;
		fram.writeUInt16(data_sample_buffer_count, 0);	// clear sample buffer
		
		//SUConfig.errorCountSinceLastUpload = 0;
		saveSUConfig();
		
		cmd = END_COMMUNICATION_ACK;
		
		//delay(5000);
		//stop_radio();	//
		
		break;

		//case END_COMMUNICATION_ACK:
		//delay(1000);	// temporary delay
		//stop_radio();
		//	break;
		
		
		case PAIR_REQUEST_CHECK:
		if(pairing_mode == true){
			sendCommand(ACK);
		}
		else{
			sendCommand(NACK);
		}
		break;
		
		case PAIR_REQUEST:
		if(pairing_mode == true){
			setPairingInfo();
		}
		
		
		break;
		
		case SU_INFO_HIGH_REQUEST:
		if(debug){
			Serial.println(F("Sending SUInfoHigh data"));
		}
		sendSUInfoHighBytes();
		cmd = SU_INFO_HIGH_DATA;
		break;
		
		case SU_INFO_LOW_REQUEST:
		if(debug){
			Serial.println(F("Sending SUInfoLow data"));
		}
		sendSUInfoLowBytes();
		cmd = SU_INFO_LOW_DATA;
		break;
		

		case SU_SENSOR_SETTING_REQUEST:
		sendSettings();
		cmd = SU_SENSOR_SETTING_REQUEST;
		break;

		case SU_SENSOR_SETTING_UPDATE:
		updateSettings();
		sendCommand(ACK);
		break;
		
		case UPDATE_BASE_STATION_INFO:
		
		break;
		
		case UPDATE_DATE_TIME_INFO:
		updateDateTime();
		break;
		
		case DATA_SAMPLE_LAYOUT_REQUEST:
		sendDataSampleLayout();
		break;

		default:
		if(debug){
			Serial.println(F("Unknown Command received"));
		}
		//sendCommand(NACK);
		break;
	}
	return cmd;

}


void transmitPayload(uint8_t payloadSizeBytes){
	
	zbTx = ZBTxRequest(addr64, payload, payloadSizeBytes);
	//zbTx.setFrameId(0); //does not increase data transfer speed
	while (digitalRead(radio_CTS) == 1);
	xbee.send(zbTx);
	//Serial.println(freeRam());
}



/*

*/
void savePacketToRAM(uint8_t *data, uint8_t data_length) {
	if ( data_length < MAX_FRAME_DATA_SIZE) { // check to ensure the data length does not exceed the maximum index of the buffer array. (assuming array size is defined by MAX_FRAME_DATA_SIZE)
		for (uint8_t i = 0; i < data_length; i++) {
			receivedLoad[i] = data[i];
		}
		receivedLoad[data_length] = '\0'; // add delimiter to the end
	}
	recievedDataLength = data_length;
}



void saveATResponseToRAM(uint8_t *command, uint8_t status, uint8_t *value, uint8_t valueLength){
	receivedLoad[0] = command[0];
	receivedLoad[1] = command[1];
	receivedLoad[2] = status;
	receivedLoad[3] = valueLength;
	if ( (valueLength + 4) < MAX_FRAME_DATA_SIZE) { // check to ensure the data length does not exceed the maximum index of the buffer array. (assuming array size is defined by MAX_FRAME_DATA_SIZE)
		for (uint8_t i = 0; i < valueLength; i++) {
			receivedLoad[i + 4] = value[i];
		}
		receivedLoad[valueLength] = '\0'; // add delimiter to the end
	}
	recievedDataLength = valueLength;

}


uint8_t processRecievedATCommand(AtCommandResponse cmdResponse) {
	uint8_t * cmd = cmdResponse.getCommand();
	uint8_t statusCode = cmdResponse.getStatus ();
	uint8_t * cmdValue = cmdResponse.getValue();
	uint8_t valueLength = cmdResponse.getValueLength();
	receivedLoad[0] = cmd[0];
	receivedLoad[1] = cmd[1];
	receivedLoad[2] = statusCode;
	receivedLoad[3] = valueLength;
	for (int i = 0; i < valueLength; i++) {
		receivedLoad[i + 4] = cmdValue[i];
	}

	return statusCode;

}


/*

*/
void sendCommand(uint8_t cmd) {
	//uint8_t cmd_payload[4];
	if(debug){
		Serial.print(F("Sending command: "));
		Serial.println(cmd, HEX);
	}
	payload[0] = cmd; // add command to payload
	payload[1] = 0;  // add data length to payload
	payload[2] = cmd; // add checksum byte. (sum of all bytes)
	
	transmitPayload(3);
}




/*
this method sends AT commands to set the Network ID of the Xbee
*/
bool setRadioNetworkID(uint16_t networkID) {
	uint8_t cmd[] = {'I', 'D'};
	uint8_t cmdValue[2];
	cmdValue[0] = (networkID >> 8) & 0xFF;
	cmdValue[1] = (networkID & 0xFF);
	return sendATCommand(cmd, cmdValue, 2);

}

/*
this method sends AT commands to set the Network channel of the Xbee
*/
bool setRadioNetworkCH(uint8_t networkChannel) {
	uint8_t cmd[] = {'C', 'H'};
	//uint8_t cmdValue;
	
	return sendATCommand(cmd, &networkChannel, 1);
	
}



/*
this method sends AT commands to set the Baud of the Xbee's serial interface
*/
bool setRadioBaud(uint32_t baud) {
	uint8_t cmd[] = {'C', 'H'};
	uint8_t cmdValue = baudToByte(baud);

	
	return sendATCommand(cmd, &cmdValue, 1);
}

uint8_t baudToByte(uint32_t baud) {
	switch (baud) {
		case 1200:
		return 0x00;

		case 2400:
		return 0x01;

		case 4800:
		return 0x02;

		case 9600:
		return 0x03;

		case 19200:
		return 0x04;

		case 38400:
		return 0x05;

		case 57600:
		return 0x06;

		case 115200:
		return 0x07;

		case 230400:
		return 0x08;

		default:
		return 0x03;
	}
}


/*
this method sends AT commands to make the XBee exit command mode
*/
bool radioExitCommandMode() {
	uint8_t cmd[] = {'C', 'N', '\r'};
	
	return sendATCommand(cmd, NULL, 0);
}


/*
this method sends AT commands to the XBee
*/
bool sendATCommand(uint8_t *command, uint8_t * value, uint8_t value_length) {
	if( value_length == 0){
		AtCMD = AtCommandRequest(command);
	}
	else{
		AtCMD = AtCommandRequest(command, value, value_length);
	}

	xbee.send(AtCMD); //send AT command packet
	
	recievePacket(1000); // wait for a response
	
	// check if the status code byte of the command returned OK (0x00)
	if ( receivedLoad[2] == 0x00) {
		return true;
	}
	else {
		return false;
	}
}









void sendStatus() {
	uint8_t errorCount = 0;
	uint8_t checksum = 0;
	uint16_t tempUint16 = 0;
	uint8_t status_packet_length = 11;

	//Serial.println(F("Sending status:"));

	tempUint16 = fram.readUInt16(data_sample_buffer_count);
	//Serial.println(tempUint16, BIN);
	DateTime now = rtc.now();

	//Serial.println(now.hour());

	payload[0] = STATUS_DATA;
	payload[1] = status_packet_length;
	payload[2] = (tempUint16 >> 8) & 0xFF;
	payload[3] = tempUint16 & 0xFF;
	payload[4] = now.hour();
	payload[5] = now.minute();
	payload[6] = now.second();
	payload[7] = now.day();
	payload[8] = now.month();
	if(now.year() >= 2000){
		payload[9] = (now.year() - 2000) & 0xFF; //truncate year to ensure it fits into one byte
	}
	else{
		payload[9] = now.year();
	}
	payload[10] = SUConfig.errorCountSinceLastUpload;
	payload[11] = 0;
	payload[12] = getBateryLevel();


	// Calculate payload checksum
	for ( uint8_t i = 0;  i < status_packet_length + 2; i++) {
		checksum += payload[i];
	}
	payload[status_packet_length + 2] = checksum;

	transmitPayload(status_packet_length + 3);

}



/*
Takes ~5000 ms to transfer all 64 samples and stop transmitting with the same size data and command packets
Takes ~3650 ms to transmit all 64 samples with the command packet size reduced to 4 bytes before sending to radio
Takes ~550 ms TO load in 64 data samples
Almost 60 ms to calculate the checksum for 64 data samples.
128 ms to send 64 samples to the radio
700 ms from verifying data packet to finish sending data out to radio
*/
bool sendDataSamples(uint16_t sampleIndex) {
	uint16_t sample_count = fram.readUInt16(data_sample_buffer_count);
	
	uint16_t addr;


	//Serial.print(F("Sample Index: "));
	//Serial.println(sampleIndex);
	uint8_t checksum = 0;
	//Serial.print(F("Sample count: "));
	//Serial.println(sample_count);

	// if the sample index to be sent is valid, send it and return true
	if (sampleIndex < sample_count) {
		payload[0] = SAMPLE_TRANSMIT_DATA; // add command to payload
		payload[1] = datasample_FRAM_byte_length + samplePayloadDataOffset - 2;  // add data length to payload
		payload[2] = (sampleIndex >> 8);
		payload[3] = sampleIndex & 0xFF;

		//loadSampleIntoPayload(sampleIndex);
		
		addr = sample_data_start_addr + sampleIndex * datasample_FRAM_byte_length;
		
		for ( uint16_t i = 0;  i < datasample_FRAM_byte_length; i++) {
			payload[i + samplePayloadDataOffset] = fram.readByte(addr);
			addr += 1;
		}

		// calculate checksum,
		for (uint8_t i = 0; i < datasample_FRAM_byte_length + samplePayloadDataOffset ; i++) {
			checksum += payload[i];
			//Serial.print(payload[i],HEX);
			//Serial.print(' ');
		}
		//Serial.println();


		//Serial.println();
		payload[datasample_FRAM_byte_length + samplePayloadDataOffset ] = checksum; // add checksum to end of payload
		//Serial.print(F("Calculated checksum to send: "));
		//Serial.println(checksum, HEX);

		transmitPayload( datasample_FRAM_byte_length + samplePayloadDataOffset + 1);
		

		return true;
	}
	else {
		return false;
	}

}

/*

*/
void loadSampleIntoPayload(uint16_t sampleIndex) {
	uint16_t addr = sample_data_start_addr + sampleIndex * datasample_FRAM_byte_length;

	//if (debug && checkUSBConnected()) {
	//Serial.print(F("Loading Payload from address: "));
	//Serial.println(addr, HEX);
	//printDataSample();
	//}

	// payload includes two checksums here. one inside the data sample, and one for the transmit data packet
	for ( uint16_t i = 0;  i < datasample_FRAM_byte_length; i++) {
		payload[i + samplePayloadDataOffset] = fram.readByte(addr);
		addr += 1;
	}

}



/*
*	This method updates the real time clocks date and time from the recieved date/time
*	from the base station unit
*/
void updateDateTime(){
	uint16_t temp;
	
	bool timeValid = true;
	for (uint8_t i = 0; i < (receivedLoad[1] + 2) ; i++) { // use offset of 2 byte to include the command and length bytes, but not the checksum
		Serial.print(receivedLoad[i], HEX);
		Serial.print(' ');
	}
	

	// verify if new time is valid
	// Should be in format of HH,MM,SS,D,M,Y
	if( receivedLoad[2] > 23){	//hours
		timeValid = false;
	}
	if( receivedLoad[3] > 59){	// minutes
		timeValid = false;
	}
	if( receivedLoad[4] > 59){	//seconds
		timeValid = false;
	}
	if( receivedLoad[5] > 31){	//day
		timeValid = false;
	}
	if( receivedLoad[6] > 12){	//month
		timeValid = false;
	}
	if( receivedLoad[7] > 99){	//year
		timeValid = false;
	}
	if(timeValid == false){
		if(debug){
			Serial.println(F("time update not valid"));
		}
		sendCommand(NACK);
		return;
	}
	
	
	// January 21, 2014 at 3:21 am you would call:
	// rtc.adjust(DateTime(2014, 1, 21, 3, 21, 0));
	if(receivedLoad[7] < 2000){
		rtc.adjust(DateTime(receivedLoad[7] + 2000, receivedLoad[6], receivedLoad[5], receivedLoad[2], receivedLoad[3], receivedLoad[4]));
	}
	else{
		rtc.adjust(DateTime(receivedLoad[7], receivedLoad[6], receivedLoad[5], receivedLoad[2], receivedLoad[3], receivedLoad[4]));
	}
	DateTime now = rtc.now();
	
	fram.writeByte(next_sampleTime_seconds_addr, now.second());
	fram.writeByte(next_sampleTime_minutes_addr, now.minute());
	fram.writeByte(next_sampleTime_hours_addr, now.hour());
	fram.writeByte(next_sampleTime_day_addr, now.day());
	fram.writeByte(next_sampleTime_month_addr, now.month());
	temp = now.year();
	if(temp >= 2000){
		temp -= 2000;
	}
	fram.writeByte(next_sampleTime_year_addr, temp);
	
	
	sendCommand(ACK); // return ACK that time is updated.
}


/*
*
*/
void updateSettings() {
	uint32_t temp;
	
	SUConfig.buffer_size = receivedLoad[2];
	SUConfig.buffer_size = (SUConfig.buffer_size << 8) + receivedLoad[3];
	
	temp = receivedLoad[4];
	temp = (temp << 8) + receivedLoad[5];
	temp = (temp << 8) + receivedLoad[6];
	temp = (temp << 8) + receivedLoad[7];
	
	SUConfig.sample_interval_seconds = temp;
	
	temp = receivedLoad[8];
	temp = (temp << 8) + receivedLoad[9];
	temp = (temp << 8) + receivedLoad[10];
	temp = (temp << 8) + receivedLoad[11];
	
	SUConfig.upload_time_interval_sampling = temp; // the time interval used to set the upload time while sampling is enabled
	
	//if(debug){
	//Serial.print(F("upload_time_interval_sampling = "));
	//Serial.println(SUConfig.upload_time_interval_sampling);
	//}
	
	temp = receivedLoad[12];
	temp = (temp << 8) + receivedLoad[13];
	temp = (temp << 8) + receivedLoad[14];
	temp = (temp << 8) + receivedLoad[15];

	
	SUConfig.upload_time_interval_idle = temp; // the time interval used to set the upload time while sampling is disabled

	
	if(receivedLoad[16] == 1){
		SUConfig.run_sampling = 1;
	}
	else if(receivedLoad[16] == 0){
		SUConfig.run_sampling = 0;
	}
	
	// update upload time if the settings changed
	
	findNextUploadTime();
	
	
	saveSUConfig();
	
}



void sendSettings() {
	uint8_t checksum = 0;
	uint8_t status_packet_length = 15;

	//Serial.print(F("Sending Current Settings: "));

	payload[0] = SU_SENSOR_SETTING_DATA;
	payload[1] = status_packet_length;
	
	payload[2] = (SUConfig.buffer_size >> 8) & 0xFF;
	payload[3] = SUConfig.buffer_size & 0xFF;
	
	
	payload[4] = (SUConfig.sample_interval_seconds >> 24) & 0xFF;
	payload[5] = (SUConfig.sample_interval_seconds >> 16) & 0xFF;
	payload[6] = (SUConfig.sample_interval_seconds >> 8) & 0xFF;
	payload[7] = SUConfig.sample_interval_seconds & 0xFF;

	payload[8] = (SUConfig.upload_time_interval_sampling >> 24) & 0xff;
	payload[9] = (SUConfig.upload_time_interval_sampling >> 16) & 0xff;
	payload[10] = (SUConfig.upload_time_interval_sampling >> 8) & 0xff;
	payload[11] = SUConfig.upload_time_interval_sampling & 0xff;

	payload[12] = (SUConfig.upload_time_interval_idle >> 24) & 0xff;
	payload[13] = (SUConfig.upload_time_interval_idle >> 16) & 0xff;
	payload[14] = (SUConfig.upload_time_interval_idle >> 8) & 0xff;
	payload[15] = SUConfig.upload_time_interval_idle & 0xff;

	payload[16] = SUConfig.run_sampling;


	// Calculate payload checksum
	for ( uint8_t i = 0;  i < status_packet_length +2; i++) {
		checksum += payload[i];
	}
	payload[status_packet_length +2] = checksum;
	//Serial.print(checksum);

	transmitPayload(status_packet_length + 3);
	
	//printSU_settings();
}

/*
*	test method for outputting settings payload to serial.
*/
void printSU_settings() {
	uint16_t temp16;
	uint32_t temp32;
	char space = ' ';
	
	
	Serial.print(space);
	Serial.print(SUConfig.network_channel, HEX);
	
	Serial.print(space);
	Serial.print(SUConfig.network_ID, HEX);
	
	Serial.print(space);
	Serial.print(SUConfig.base_station_address_high, HEX);
	
	Serial.print(space);
	Serial.print(SUConfig.base_station_address_low, HEX);
	
	
	
	Serial.print(space);
	Serial.print(SUConfig.buffer_size);
	Serial.print(space);
	Serial.print(SUConfig.sample_interval_seconds);
	Serial.print(space);
	
	Serial.print(SUConfig.next_upload_Hour);
	Serial.print(space);
	Serial.print(SUConfig.next_upload_Minutes);
	Serial.print(space);
	Serial.print(SUConfig.next_upload_Seconds);
	Serial.print(space);
	
	Serial.print(SUConfig.next_upload_Date);
	Serial.print(space);
	Serial.print(SUConfig.next_upload_Months);
	Serial.print(space);
	Serial.print(SUConfig.next_upload_years);
	Serial.print(space);
	Serial.print(SUConfig.run_sampling);
	Serial.print(space);
	Serial.print(space);
	Serial.print(space);
	for ( uint8_t i = 0;  i < 15 ; i++) {
		Serial.print(payload[i]);
		Serial.print(space);
	}
	Serial.println();
}


void sendSUInfoHighBytes() {
	uint8_t checksum = 0;
	uint8_t high_info_packet_length = 16;
	uint8_t cmd_high[] = {'S', 'H'};
	uint8_t cmd_low[] = {'S', 'L'};
	

	//Serial.print(F("Sending Current Settings: "));
	
	payload[0] = SU_INFO_HIGH_DATA;
	payload[1] = high_info_packet_length;
	
	payload[2] = SUConfig.network_channel;
	
	payload[3] = (SUConfig.network_ID >> 8) & 0xFF;
	payload[4] = SUConfig.network_ID & 0xFF;
	
	//Serial.println(F("Retrieving radio address..."));
	
	// send the AT command 'SH' to retrieve the high address of the radio module, and load it into the payload
	sendATCommand(cmd_high, NULL, 0);
	//recievePacket();
	if(receivedLoad[0] == cmd_high[0] && receivedLoad[1] == cmd_high[1] ){
		for (int i = 0; i < receivedLoad[3]; i++) {
			payload[5 + i] = receivedLoad[i + 4] ;
			//Serial.print(receivedLoad[i + 4], HEX);
			//Serial.print(' ');
		}
	}
	
	
	// send the AT command 'SL' to retrieve the low address of the radio module, and load it into the payload
	sendATCommand(cmd_low, NULL, 0);
	//recievePacket();
	if(receivedLoad[0] == cmd_low[0] && receivedLoad[1] == cmd_low[1] ){
		for (int i = 0; i < receivedLoad[3]; i++) {
			payload[9 + i] = receivedLoad[i + 4] ;
			//Serial.print(receivedLoad[i + 4], HEX);
			//Serial.print(' ');
		}
	}
	//Serial.println();
	//Serial.println(F("Radio address received."));
	radioExitCommandMode();
	//Serial.println(F("Exited command mode.."));
	
	
	payload[13] = sensor_type;
	
	payload[14] = (device_id >> 24) & 0xFF;
	payload[15] = (device_id >> 16) & 0xFF;
	payload[16] = (device_id >> 8) & 0xFF;
	payload[17] = device_id & 0xFF;

	// extra info can go here


	//Serial.println(F("Calculating checksum..."));
	// Calculate payload checksum
	for ( uint8_t i = 0;  i < high_info_packet_length +2; i++) {
		//Serial.print(payload[i], HEX);
		//Serial.print(' ');
		checksum += payload[i];
	}
	//Serial.println(checksum, HEX);
	payload[high_info_packet_length +2] = checksum;
	
	
	transmitPayload(high_info_packet_length + 3);
	//Serial.print(F("Free Ram: "));
	//Serial.println(freeRam());
	
	//printSU_settings();
}



void sendSUInfoLowBytes() {
	uint8_t checksum = 0;
	uint8_t low_info_packet_length = 20;
	uint16_t temp16;

	//Serial.print(F("Sending Current Settings: "));
	temp16 = getBateryLevel();

	payload[0] = SU_INFO_LOW_DATA;
	payload[1] = low_info_packet_length;
	
	payload[2] = (SUConfig.buffer_size >> 8) & 0xFF;
	payload[3] = SUConfig.buffer_size & 0xFF;
	
	payload[4] = (max_sample_count >> 8) & 0xFF;
	payload[5] = max_sample_count & 0xFF;
	
	payload[6] = (SUConfig.sample_interval_seconds >> 24) & 0xFF;
	payload[7] = (SUConfig.sample_interval_seconds >> 16) & 0xFF;
	payload[8] = (SUConfig.sample_interval_seconds >> 8) & 0xFF;
	payload[9] = SUConfig.sample_interval_seconds & 0xFF;

	
	payload[10] = (SUConfig.upload_time_interval_sampling >> 24) & 0xFF;
	payload[11] = (SUConfig.upload_time_interval_sampling >> 16) & 0xFF;
	payload[12] = (SUConfig.upload_time_interval_sampling >> 8) & 0xFF;
	payload[13] = SUConfig.upload_time_interval_sampling & 0xFF;
	
	payload[14] = (SUConfig.upload_time_interval_idle >> 24) & 0xFF;
	payload[15] = (SUConfig.upload_time_interval_idle >> 16) & 0xFF;
	payload[16] = (SUConfig.upload_time_interval_idle >> 8) & 0xFF;
	payload[17] = SUConfig.upload_time_interval_idle & 0xFF;

	payload[18] = SUConfig.run_sampling;
	payload[19] = SUConfig.errorCountSinceLastUpload;
	
	payload[20] = (temp16 >> 8) & 0xFF; // battery life info
	payload[21] = temp16 & 0xFF;
	
	//extra info can go here
	

	// Calculate payload checksum
	for ( uint8_t i = 0;  i < low_info_packet_length +2; i++) {
		checksum += payload[i];
	}
	payload[low_info_packet_length +2] = checksum;
	//Serial.print(checksum);

	transmitPayload(low_info_packet_length + 3);
	
	//printSU_settings();
}




void setPairingInfo(){
	uint32_t temp;
	
	SUConfig.network_channel = receivedLoad[2];
	//Serial.print(F("NET CHN "));
	//Serial.println(SUConfig.network_channel, HEX);
	SUConfig.network_ID =  receivedLoad[3];
	SUConfig.network_ID = (SUConfig.network_ID << 8) + receivedLoad[4];
	
	temp = receivedLoad[5];
	temp = (temp << 8) + receivedLoad[6];
	temp = (temp << 8) + receivedLoad[7];
	temp = (temp << 8) + receivedLoad[8];
	
	SUConfig.base_station_address_high = temp;
	
	temp = receivedLoad[9];
	temp = (temp << 8) + receivedLoad[10];
	temp = (temp << 8) + receivedLoad[11];
	temp = (temp << 8) + receivedLoad[12];
	
	SUConfig.base_station_address_low = temp;
	
	saveSUConfig();
	
	sendCommand(ACK);
}



void sendDataSampleLayout(){
	uint8_t checksum = 0;
	uint8_t packet_length = 2;

	payload[0] = DATA_SAMPLE_LAYOUT;
	payload[1] = packet_length;
	
	
	payload[2] = data_type_layout[0];
	payload[3] = data_type_layout[1];
	
	//payload[4] = data_type_layout[2];
	//payload[5] = data_type_layout[3];


	// Calculate payload checksum
	for ( uint8_t i = 0;  i < packet_length +2; i++) {
		checksum += payload[i];
	}
	payload[packet_length +2] = checksum;


	transmitPayload(packet_length + 3);
	
}







/*

*/
void loadSUConfig() {
	uint16_t addr = su_Config_addr;
	
	uint8_t checksumCheck = 0;

	SUConfig.network_channel          = fram.readByte(addr);
	addr += sizeof(uint8_t);
	SUConfig.network_ID               = fram.readUInt16(addr);
	addr += sizeof(uint16_t);
	SUConfig.base_station_address_high     = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	SUConfig.base_station_address_low      = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	SUConfig.sample_interval_seconds  = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	SUConfig.buffer_size              = fram.readUInt16(addr);
	addr += sizeof(uint16_t);

	SUConfig.run_sampling         = fram.readByte(addr);
	addr += sizeof(byte);
	
	SUConfig.upload_time_interval_sampling     = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	SUConfig.upload_time_interval_idle     = fram.readUInt32(addr);
	addr += sizeof(uint32_t);

	SUConfig.next_upload_Hour         = fram.readByte(addr);
	addr += sizeof(byte);
	SUConfig.next_upload_Minutes      = fram.readByte(addr);
	addr += sizeof(byte);
	SUConfig.next_upload_Seconds      = fram.readByte(addr);
	addr += sizeof(byte);
	
	SUConfig.next_upload_Date         = fram.readByte(addr);
	addr += sizeof(byte);
	SUConfig.next_upload_Months       = fram.readByte(addr);
	addr += sizeof(byte);
	SUConfig.next_upload_years        = fram.readByte(addr);
	addr += sizeof(byte);
	SUConfig.errorCountSinceLastUpload = fram.readByte(addr); // not included in checksum calculations
	addr += sizeof(byte);

	SUConfig.checksum                 = fram.readByte(addr);
	addr += sizeof(byte);



	// Calculate Checksum for the Settings

	checksumCheck = SUConfig.network_channel;
	checksumCheck += calcChecksum(SUConfig.network_ID);
	checksumCheck += calcChecksum(SUConfig.base_station_address_high);
	checksumCheck += calcChecksum(SUConfig.base_station_address_low);
	checksumCheck += calcChecksum(SUConfig.sample_interval_seconds);
	checksumCheck += calcChecksum(SUConfig.buffer_size);

	checksumCheck += SUConfig.run_sampling;
	
	checksumCheck += calcChecksum(SUConfig.upload_time_interval_sampling);
	checksumCheck += calcChecksum(SUConfig.upload_time_interval_idle);

	checksumCheck += SUConfig.next_upload_Hour;
	checksumCheck += SUConfig.next_upload_Minutes;
	checksumCheck += SUConfig.next_upload_Seconds;
	checksumCheck += SUConfig.next_upload_Date;
	checksumCheck += SUConfig.next_upload_Months;
	checksumCheck += SUConfig.next_upload_years;


	if(debug){
		Serial.print(F("New Checksum: "));
		Serial.println(checksumCheck);
		Serial.print(F("Stored Checksum: "));
		Serial.println(SUConfig.checksum);
	}
	
	//printsuconfig();
	//Serial.println();

	if ( checksumCheck != SUConfig.checksum ) {
		// config settings have not been saved properly or they are not set after a first turn on
		Serial.println(F("Config Checksum Incorrect! Default settings restored!"));
		loadDefaultSUCongifg();
		saveSUConfig();
		if(debug){
			printsuconfig();
		}
	}
	if(!SUConfigValid()){
		Serial.println(F("Config not valid! Default settings restored!"));
		loadDefaultSUCongifg();
		saveSUConfig();
		if(debug){
			printsuconfig();
		}
	}
}



/*

*/
bool SUConfigValid() {
	bool config_valid = true;
	Serial.println(F("Checking config for valid settings"));
	if ( SUConfig.base_station_address_high == 0x00000000) {
		config_valid = false;
		Serial.println(0);
		
	}
	if ( SUConfig.base_station_address_low == 0x00000000 ) {
		config_valid = false;
		Serial.println(1);

	}
	if ( SUConfig.sample_interval_seconds < 10) {
		config_valid = false;
		Serial.println(2);
	}

	if (SUConfig.buffer_size < 4) {
		config_valid = false;
		Serial.println(3);
	}

	/*
	values of 255 indicate the upload time is invalid and should not be used.
	*/
	if (SUConfig.next_upload_Hour        == 255) {
		config_valid = false;
		Serial.println(4);
	}
	if (SUConfig.next_upload_Minutes     == 255) {
		config_valid = false;
		Serial.println(5);
	}
	if (SUConfig.next_upload_Seconds    == 255) {
		config_valid = false;
		Serial.println(6);
	}
	
	if (SUConfig.next_upload_Date        == 255 || SUConfig.next_upload_Date == 0) {
		config_valid = false;
		Serial.println(7);
	}
	if (SUConfig.next_upload_Months      == 255 || SUConfig.next_upload_Months == 0) {
		config_valid = false;
		Serial.println(8);
	}
	if (SUConfig.next_upload_years       == 255 || SUConfig.next_upload_years == 0) {
		config_valid = false;
		Serial.println(9);
	}
	
	
	if (SUConfig.upload_time_interval_sampling < 60) {
		config_valid = false;
		Serial.println(10);
	}
	
	if (SUConfig.upload_time_interval_idle < 60) {
		config_valid = false;
		Serial.println(11);
	}



	return config_valid;
}



/*

*/
void loadDefaultSUCongifg() {

	/*
	this network address is invalid and thus should used to indicate that it is not connected to any base station
	*/
	SUConfig.network_channel          = network_channel_default;
	SUConfig.network_ID               = network_ID_default;
	SUConfig.base_station_address_high     = base_station_address_high_default;
	SUConfig.base_station_address_low      = base_station_address_low_default;
	SUConfig.sample_interval_seconds  = sample_interval_seconds_default; // default 10 minute sample time
	SUConfig.buffer_size              = buffer_size_default; // default buffer size
	SUConfig.run_sampling             = 0;
	/*
	values of 255 indicate the upload time is invalid and should not be used.
	*/
	SUConfig.upload_time_interval_sampling = upload_time_interval_sampling_default;
	SUConfig.upload_time_interval_idle     = upload_time_interval_idle_default;
	
	SUConfig.next_upload_Hour        = next_upload_Hour_default;
	SUConfig.next_upload_Minutes     = next_upload_Minutes_default;
	SUConfig.next_upload_Seconds	 = next_upload_Seconds_default;
	SUConfig.next_upload_Date        = next_upload_Date_default;
	SUConfig.next_upload_Months      = next_upload_Months_default;
	SUConfig.next_upload_years       = next_upload_years_default;
	SUConfig.errorCountSinceLastUpload = 1;     // not included in checksum calculations
	/*
	calculate new checksum
	*/
	SUConfig.checksum = SUConfig.network_channel;
	SUConfig.checksum += calcChecksum(SUConfig.network_ID);
	SUConfig.checksum += calcChecksum(SUConfig.base_station_address_high);
	SUConfig.checksum += calcChecksum(SUConfig.base_station_address_low);
	SUConfig.checksum += calcChecksum(SUConfig.sample_interval_seconds);
	SUConfig.checksum += calcChecksum(SUConfig.buffer_size);

	SUConfig.checksum += SUConfig.run_sampling;
	
	SUConfig.checksum += calcChecksum(SUConfig.upload_time_interval_sampling);
	SUConfig.checksum += calcChecksum(SUConfig.upload_time_interval_idle);

	SUConfig.checksum += SUConfig.next_upload_Hour;
	SUConfig.checksum += SUConfig.next_upload_Minutes;
	SUConfig.checksum += SUConfig.next_upload_Seconds;
	
	SUConfig.checksum += SUConfig.next_upload_Date;
	SUConfig.checksum += SUConfig.next_upload_Months;
	SUConfig.checksum += SUConfig.next_upload_years;

}


void printsuconfig(){
	uint16_t addr = su_Config_addr;
	
	for(uint8_t i = 0; i < 26; i++){
		Serial.print(fram.readByte(addr));
		addr++;
		Serial.print(' ');
	}
	
}




/*

*/
void saveSUConfig() {
	uint16_t addr = su_Config_addr;
	uint8_t checksumCheck = 0;

	fram.writeByte(addr, SUConfig.network_channel);
	addr += sizeof(byte);
	fram.writeUInt16(addr, SUConfig.network_ID);
	addr += sizeof(uint16_t);

	fram.writeUInt32(addr, SUConfig.base_station_address_high);
	addr += sizeof(uint32_t);
	fram.writeUInt32(addr, SUConfig.base_station_address_low);
	addr += sizeof(uint32_t);
	fram.writeUInt32(addr, SUConfig.sample_interval_seconds);
	addr += sizeof(uint32_t);
	fram.writeUInt16(addr, SUConfig.buffer_size);
	addr += sizeof(uint16_t);

	fram.writeByte(addr, SUConfig.run_sampling);
	addr += sizeof(byte);
	
	fram.writeUInt32(addr, SUConfig.upload_time_interval_sampling);
	addr += sizeof(uint32_t);
	fram.writeUInt32(addr, SUConfig.upload_time_interval_idle);
	addr += sizeof(uint32_t);

	fram.writeByte(addr, SUConfig.next_upload_Hour);
	addr += sizeof(byte);
	fram.writeByte(addr, SUConfig.next_upload_Minutes);
	addr += sizeof(byte);
	fram.writeByte(addr, SUConfig.next_upload_Seconds);
	addr += sizeof(byte);
	fram.writeByte(addr, SUConfig.next_upload_Date);
	addr += sizeof(byte);
	fram.writeByte(addr, SUConfig.next_upload_Months);
	addr += sizeof(byte);
	fram.writeByte(addr, SUConfig.next_upload_years);
	addr += sizeof(byte);

	fram.writeByte(addr, SUConfig.errorCountSinceLastUpload); // not included in checksum calculations
	addr += sizeof(byte);


	checksumCheck = SUConfig.network_channel;
	checksumCheck += calcChecksum(SUConfig.network_ID);
	checksumCheck += calcChecksum(SUConfig.base_station_address_high);
	checksumCheck += calcChecksum(SUConfig.base_station_address_low);
	checksumCheck += calcChecksum(SUConfig.sample_interval_seconds);
	checksumCheck += calcChecksum(SUConfig.buffer_size);

	checksumCheck += SUConfig.run_sampling;
	
	checksumCheck += calcChecksum(SUConfig.upload_time_interval_sampling);
	checksumCheck += calcChecksum(SUConfig.upload_time_interval_idle);

	checksumCheck += SUConfig.next_upload_Hour;
	checksumCheck += SUConfig.next_upload_Minutes;
	checksumCheck += SUConfig.next_upload_Seconds;
	checksumCheck += SUConfig.next_upload_Date;
	checksumCheck += SUConfig.next_upload_Months;
	checksumCheck += SUConfig.next_upload_years;


	fram.writeByte(addr, checksumCheck);
	
	if(debug){
		Serial.print(F("Saving config, Checksum = "));
		Serial.println(checksumCheck);
	}


}


/*

*/
uint8_t getBateryLevel() {
	return 100;
}




/*

*/
uint8_t calcChecksum(uint32_t value) {
  uint8_t temp32[4];
  memcpy(&temp32[0], &value, sizeof(uint32_t));
  temp32[0] += temp32[1];
  temp32[0] += temp32[2];
  temp32[0] += temp32[3];
  return temp32[0];
}
/*

*/
uint8_t calcChecksum(uint16_t value) {
  uint8_t temp16[2];
  memcpy(&temp16[0], &value, sizeof(uint16_t));
  temp16[0] += temp16[1];
  return temp16[0];
}
/*

*/
uint8_t calcChecksum(int16_t value) {
  uint8_t temp16[2];
  memcpy(&temp16[0], &value, sizeof(int16_t));
  temp16[0] += temp16[1];
  return temp16[0];
}

/*

*/
uint8_t calcChecksum(float value) {
  uint8_t temp32[4];
  memcpy(&temp32[0], &value, sizeof(float));
  temp32[0] += temp32[1];
  temp32[0] += temp32[2];
  temp32[0] += temp32[3];
  return temp32[0];
}

//============ Time methods ======================


/*
	This method returns the time in seconds from the start of the year 2000 (same as Unix systems that start from 1970).
	64 bit time is used so there "should not" be an issue past the year 2038 like 32 bit Unix time.
*/
uint64_t timeToSeconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds) {
	uint64_t timeInSeconds;
	uint16_t timeInDays;
	uint16_t years;
	uint32_t monthInDays = 0;

	
	if(year >= 2000 ){
		years = year - 2000;
	}
	else{
		years = year;
	}
	
	if(month < 32){
		for(uint8_t i = 1; i < month; ++i){
			monthInDays += pgm_read_byte(daysInMonth + i - 1);
		}
		if( month > 2 && years % 4 == 0){
			monthInDays += 1;

		}
	}
	else{
		monthInDays = 384;
	}
	
	timeInDays += (years )*365 + monthInDays + (years + 3) / 4 ; // the (years + 3) / 4 - 1 part is exploiting integer divisions which round to zero
	
	timeInSeconds = seconds;
	timeInSeconds += (uint32_t)minute*60;
	timeInSeconds += (uint32_t)hour*3600;
	timeInSeconds += (uint32_t)(day-1)*86400;
	
	timeInSeconds += timeInDays*86400;
	
	return timeInSeconds;
}



void updateNextSampleTime( bool useCurrentTime){
	uint64_t sampleTime;
	uint64_t currentTime;
	uint16_t temp;
	DateTime now = rtc.now();
	
	uint8_t next_seconds = fram.readByte(next_sampleTime_seconds_addr);
	uint8_t next_minutes = fram.readByte(next_sampleTime_minutes_addr);
	uint8_t next_hours = fram.readByte(next_sampleTime_hours_addr);
	uint8_t next_days = fram.readByte(next_sampleTime_day_addr);
	uint8_t next_months = fram.readByte(next_sampleTime_month_addr);
	uint8_t next_years = fram.readByte(next_sampleTime_year_addr);
	
	//SUConfig.sample_interval_seconds = 5;
	
	
	
	temp = now.year();
	if(temp >= 2000){
		temp -=2000;
	}
	
	if(useCurrentTime == true){
		next_seconds = now.second();
		next_minutes = now.minute();
		next_hours = now.hour();
		next_days = now.day();
		next_months = now.month();
		next_years = temp;
		
	}
	
	next_seconds += SUConfig.sample_interval_seconds;
	
	while(next_seconds > 59){
		next_minutes += 1;
		next_seconds -= 60;
	}
	while(next_minutes > 59){
		next_hours += 1;
		next_minutes -= 60;
	}
	while(next_hours > 23){
		next_days += 1;
		next_hours -= 24;
	}
	
	if(next_months < 8 && next_months % 2 == 0){
		if(next_months == 2 && next_years % 4 != 0)	{
			while(next_days > 28){
				next_months += 1;
				next_days -= 28;
			}
		}
		else{
			while(next_days > 29){
				next_months += 1;
				next_days -= 29;
			}
		}
	}
	
	else if(next_months < 8 && next_months % 2 != 0)		{
		while(next_days > 31)		{
			next_months += 1;
			next_days -= 31;
		}
	}

	else if(next_months > 7 && next_months % 2 == 0){
		while(next_days > 31)		{
			next_months += 1;
			next_days -= 31;
		}
	}
	else{
		
		while(next_days > 30)		{
			next_months += 1;
			next_days -= 30;
		}
	}
	
	while(next_months > 12){
		next_years += 1;
		next_months -= 12;
	}
	
	
	
	
	
	
	
	// truncate years value to fit in an 8bit unsigned integer
	//temp =  now.year();
	//if(temp >= 2000){
	//	temp -= 2000;
	//}
	
	temp_time_1 = timeToSeconds(temp, now.month(), now.day(), now.hour(), now.minute(), now.second()); // current time in seconds from 0:0:0 1/1/2000
	temp_time_2 = timeToSeconds(next_years, next_months, next_days, next_hours, next_minutes, next_seconds);	// next calculated sample time in seconds
	
	if(temp_time_1 >= temp_time_2 - 5 && useCurrentTime == true){
		fram.writeByte(next_sampleTime_seconds_addr, now.second());
		fram.writeByte(next_sampleTime_minutes_addr, now.minute());
		fram.writeByte(next_sampleTime_hours_addr, now.hour());
		fram.writeByte(next_sampleTime_day_addr, now.day());
		fram.writeByte(next_sampleTime_month_addr, now.month());
		fram.writeByte(next_sampleTime_year_addr, temp);
		
		updateNextSampleTime(false);
	}
	else{
		
		fram.writeByte(next_sampleTime_seconds_addr, next_seconds);
		fram.writeByte(next_sampleTime_minutes_addr, next_minutes);
		fram.writeByte(next_sampleTime_hours_addr, next_hours);
		fram.writeByte(next_sampleTime_day_addr, next_days);
		fram.writeByte(next_sampleTime_month_addr, next_months);
		fram.writeByte(next_sampleTime_year_addr, next_years);
	}
	
	
	
	

	
}



void waitTillAlarm(){
	uint16_t temp16;
	uint32_t waitTime;
	
	uint8_t seconds = fram.readByte(next_sampleTime_seconds_addr);
	uint8_t minutes = fram.readByte(next_sampleTime_minutes_addr);
	uint8_t hours   = fram.readByte(next_sampleTime_hours_addr);
	uint8_t days    = fram.readByte(next_sampleTime_day_addr);
	uint8_t months = fram.readByte(next_sampleTime_month_addr);
	uint8_t years = fram.readByte(next_sampleTime_year_addr);
		
	DateTime now = rtc.now();
	
	// convert date/times to seconds (e.g. Unix time) to simplify comparisons
	
	temp_time_1 = timeToSeconds(SUConfig.next_upload_years, SUConfig.next_upload_Months, SUConfig.next_upload_Date, SUConfig.next_upload_Hour, SUConfig.next_upload_Minutes, SUConfig.next_upload_Seconds); // upload time
	temp_time_2 = timeToSeconds(years, months, days, hours, minutes, seconds);	// sample time
			
	temp16 = now.year();
	if(temp16 >= 2000){
		temp16 -= 2000;
	}
	temp_time_3 = timeToSeconds(temp16, now.month(), now.day(), now.hour(), now.minute(), now.second()); // current time
	
	if(temp_time_1 <= temp_time_2 && temp_time_1 >= temp_time_3){
		waitTime = temp_time_1 - temp_time_3;
	}
	else if(temp_time_2 < temp_time_1 && temp_time_2 >= temp_time_3){
		waitTime = temp_time_2 - temp_time_3;
	}
	else{
		waitTime = 0;
		return;
	}
	
	// use large delay till close to next alarm time
	delay((waitTime) * 1000 - 500);
	
	
	// read rtc until its alarm triggers.
	while(rtc.clearRTCAlarm1(false) == false);
	
}










/*
This method checks the current time from the RTC and returns true if the current time
matches the known time to take a sample at.

This method assumes the SUconfig struct gets imported from FRAM upon each wakeup.
*/
bool checkSampleTimeNow() {
	bool nextSampleTimeMatches = true;
	uint8_t tollerance_Seconds = 20;
	
	uint16_t temp16;
	
	DateTime now = rtc.now();

	uint8_t seconds = fram.readByte(next_sampleTime_seconds_addr);
	uint8_t minutes = fram.readByte(next_sampleTime_minutes_addr);
	uint8_t hours   = fram.readByte(next_sampleTime_hours_addr);
	uint8_t days    = fram.readByte(next_sampleTime_day_addr);
	uint8_t months = fram.readByte(next_sampleTime_month_addr);
	uint8_t years = fram.readByte(next_sampleTime_year_addr);
	
	
	temp16 = now.year();
	if(temp16 >= 2000){
		temp16 -= 2000;
	}
	
	temp_time_1 = timeToSeconds(temp16, now.month(), now.day(), now.hour(), now.minute(), now.second()); // current time		// slower but safer(?) and simpler way of comparing times
	temp_time_2 = timeToSeconds(years, months, days, hours, minutes, seconds);		// next sample time in memory
	
	if( temp_time_1 <= temp_time_2 - tollerance_Seconds){ // only having a lower bound should help with instances of the current time passing the sample time before being called.
		nextSampleTimeMatches = false;
	}
	
	
	return nextSampleTimeMatches;
}




/*
this method checks the current time from the RTC and returns true if the current time
matches the known time to upload data at.

This method assumes the SUconfig struct gets imported from FRAM upon each wakeup.
*/
bool checkUploadTimeNow() {
	bool nextUploadTimeMatches = true;
	uint8_t tollerance_Seconds = 3;
	uint16_t temp16;
	DateTime now = rtc.now();
	uint32_t temp32;
	
	
	temp16 = now.year();
	if(temp16 >= 2000){
		temp16 -= 2000;
	}
	
	
	temp_time_1 = timeToSeconds(temp16, now.month(), now.day(), now.hour(), now.minute(), now.second());	// current time in seconds
	temp_time_2 = timeToSeconds(SUConfig.next_upload_years, SUConfig.next_upload_Months, SUConfig.next_upload_Date, SUConfig.next_upload_Hour, SUConfig.next_upload_Minutes, SUConfig.next_upload_Seconds); // next upload time in seconds

	// catch if the time was incorrectly set, or a reset occurred
	if (SUConfig.next_upload_years == 255 && SUConfig.next_upload_Months == 255 && SUConfig.next_upload_Date == 255 && SUConfig.next_upload_Hour == 255 && SUConfig.next_upload_Minutes == 255){ // check the the time to see if the upload date is set to a default
		findNextUploadTime();
		temp_time_2 = timeToSeconds(SUConfig.next_upload_years, SUConfig.next_upload_Months, SUConfig.next_upload_Date, SUConfig.next_upload_Hour, SUConfig.next_upload_Minutes, SUConfig.next_upload_Seconds); // next upload time in seconds
		saveSUConfig();
	}

	
	if( temp_time_1 <= temp_time_2 - tollerance_Seconds){
		nextUploadTimeMatches = false;
	}
	
	if(debug){
		Serial.print(F("Upload Time Match = "));
		Serial.println(nextUploadTimeMatches);
	}
	return nextUploadTimeMatches;
}




void setNextAlarmTime(){
	setNextAlarmTime(false, 0, 0, 0, 0);
}



/*
	This method compares the next upload and take sample times and sets the RTC alarm to the time which will occur first.
	
	The next alarm time can be overridden by passing the new time and setting the override parameter to true.
	If the override is not needed, just use the setNextAlarmTime() method call which by default does not override the methods function.
*/
void setNextAlarmTime(bool override, uint8_t override_seconds, uint8_t override_minutes, uint8_t override_hours, uint8_t override_days) {
	uint16_t temp16 = 0;
	uint32_t nextAlarmTimeSeconds;
	uint8_t seconds = fram.readByte(next_sampleTime_seconds_addr);
	uint8_t minutes = fram.readByte(next_sampleTime_minutes_addr);
	uint8_t hours   = fram.readByte(next_sampleTime_hours_addr);
	uint8_t days    = fram.readByte(next_sampleTime_day_addr);
	uint8_t months = fram.readByte(next_sampleTime_month_addr);
	uint8_t years = fram.readByte(next_sampleTime_year_addr);
	
	rtc.clearRTCAlarm1(true);
	DateTime now = rtc.now();
	
	temp16 = now.year();
	if(temp16 >= 2000){
		temp16 -= 2000;
	}
	
	
	findNextUploadTime(); // update the alarm time 
	
	if(!override){
		
		// convert date/times to seconds (e.g. Unix time) to simplify comparisons 
			
		temp_time_1 = timeToSeconds(SUConfig.next_upload_years, SUConfig.next_upload_Months, SUConfig.next_upload_Date, SUConfig.next_upload_Hour, SUConfig.next_upload_Minutes, SUConfig.next_upload_Seconds); // upload time
		
		temp_time_2 = timeToSeconds(years, months, days, hours, minutes, seconds);	// sample time
		temp_time_3 = timeToSeconds(temp16, now.month(), now.day(), now.hour(), now.minute(), now.second()); // current time
		

		if(debug){
			Serial.println(F("Comparing Times..."));
		}
		
		// if the next sample time is less than the current time, update the time so they are valid
		if(temp_time_2 <= temp_time_3 && SUConfig.run_sampling == 1){
			if(debug){
				Serial.println(F("Updating Sample time to be valid..."));
			}
			updateNextSampleTime(true); // reset sample time so that it is in the future.
		}
		
		//// if the next upload time is less than the current time, update the time so they are valid
		//if(temp_time_1 <= temp_time_3){
			//
			//if(SUConfig.run_sampling == 1){
				//if(debug){
					//Serial.println(F("Updating times to be valid..."));
				//}
				//
				//findNextUploadTime(true); //set the next upload time to be the next time it would have woken up otherwise. Safety net of sorts. will recall this method to update the alarm.
				//
				//setNextAlarmTime(); // now retry setting the next alarm time
				//saveSUConfig();
				//return;
			//}
			//else{
				//// wake up every default time interval to allow for checks from base station
				//if(debug){
					//Serial.println(F("Setting next Idle alarm time"));
				//}
				//findNextUploadTime(true);
				//rtc.setAlarm1Time(SUConfig.next_upload_Seconds, SUConfig.next_upload_Minutes, SUConfig.next_upload_Hour, SUConfig.next_upload_Date, false);
			//}
			//
			//
			//
			////rtc.setAlarm1Time(SUConfig.next_upload_Seconds, SUConfig.next_upload_Minutes, SUConfig.next_upload_Hour, SUConfig.next_upload_Date, false);
			//
			//
		//}
		
		if(temp_time_2 <= temp_time_1 && SUConfig.run_sampling == 1){
			// if the next sample time is sooner than upload time, set the next sample time as the wakeup time
			//DateTime alarmtime(years + 2000, months, days, hours, minutes, seconds);
			rtc.setAlarm1Time(seconds, minutes, hours, days, false);
			if(debug){
				Serial.println(F("Next wake is Sample: "));
			}
		
		
		}
		else{
			// set the next upload time as the wakeup time
			//DateTime alarmtime(SUConfig.next_upload_years + 2000, SUConfig.next_upload_Months, SUConfig.next_upload_Date, SUConfig.next_upload_Hour, SUConfig.next_upload_Minutes, SUConfig.next_upload_Seconds);
			//rtc.setAlarm1Time(alarmtime);

			rtc.setAlarm1Time(SUConfig.next_upload_Seconds, SUConfig.next_upload_Minutes, SUConfig.next_upload_Hour, SUConfig.next_upload_Date, false);
		
			if(debug){
				Serial.println(F("Next wake is Upload: "));
			}
		}
	
		if(debug){
			Serial.println((uint32_t)temp_time_2); // print next sample time
			Serial.println((uint32_t)temp_time_1);// print next upload time
		
		
			Serial.print(F("Next Sample time is: "));
			Serial.print(hours);
			Serial.print(':');
			Serial.print(minutes);
			Serial.print(':');
			Serial.print(seconds);
			Serial.print(' ');
			Serial.print(days);
			Serial.print('/');
			Serial.print(months);
			Serial.print('/');
			Serial.println(years);
		
			Serial.print(F("Next upload time is: "));
			Serial.print(SUConfig.next_upload_Hour);
			Serial.print(':');
			Serial.print(SUConfig.next_upload_Minutes);
			Serial.print(':');
			Serial.print(SUConfig.next_upload_Seconds);
			Serial.print(' ');
			Serial.print(SUConfig.next_upload_Date);
			Serial.print('/');
			Serial.print(SUConfig.next_upload_Months);
			Serial.print('/');
			Serial.println(SUConfig.next_upload_years);
		
			DateTime now = rtc.now();
			Serial.print(F("Current time is: "));
			Serial.print(now.hour());
			Serial.print(':');
			Serial.print(now.minute());
			Serial.print(':');
			Serial.println(now.second());
		
			printSU_settings();
		}
	}
	else{
		rtc.setAlarm1Time(override_seconds, override_minutes, override_hours, override_days, false);
	}
	
	
	rtc.writeSqwPinMode(DS3231_OFF);	// turn off square wave output from RTC
	rtc.setAlarm2Time(255, 255, 255, false);	// disable alarm 2 time
	rtc.enableRTC_DS3231Interrupts(true);		// enable TC to generate an interrupt on its output pin
	rtc.clearRTCAlarm2(true);					// clear alarm 2 if it was set for some reason
	rtc.enableRTCAlarm(true, false);			// ensure alarm 1 can trigger an interrupt.

	saveSUConfig();

}




/*
	
*/ 
void findNextUploadTime(){
	uint32_t time_interval_seconds;
	uint16_t temp;
	DateTime now = rtc.now();
	
	
	uint32_t next_seconds = SUConfig.next_upload_Seconds;
	uint16_t next_minutes = SUConfig.next_upload_Minutes;
	uint8_t next_hours = SUConfig.next_upload_Hour;
	uint8_t next_days = SUConfig.next_upload_Date;
	uint8_t next_months = SUConfig.next_upload_Months;
	uint8_t next_years = SUConfig.next_upload_years;
	
	
	//ensures upload time is always in the future or now, and also prevents it from being incremented multiple times
	if(SUConfig.run_sampling == 1){
		time_interval_seconds = getSecondsToNextIdleUpload(now, SUConfig.upload_time_interval_sampling);
	}
	else{
		time_interval_seconds = getSecondsToNextIdleUpload(now, SUConfig.upload_time_interval_idle);
	}

	next_seconds = now.second();
	next_minutes = now.minute();
	next_hours = now.hour();
	next_days	= now.day();
	next_months	= now.month();
	temp = now.year();
	if(temp >= 2000){
		temp -= 2000;
	}
	next_years = temp;


	next_seconds += time_interval_seconds;
	if(debug){
		Serial.println(F("Finding new time from seconds offset..."));
	}
	
	while(next_seconds > 59){
		next_minutes += 1;
		next_seconds -= 60;
	}
	while(next_minutes > 59){
		next_hours += 1;
		next_minutes -= 60;
	}
	while(next_hours > 23){
		next_days += 1;
		next_hours -= 24;
	}
	
	if(next_months < 8 && next_months % 2 == 0){
		
		if(next_months == 2 && next_years % 4 != 0)	{
			
			while(next_days > 28){
				
				next_months += 1;
				next_days -= 28;
			}
		}
		else{
			while(next_days > 29){
				next_months += 1;
				next_days -= 29;
			}
		}
	}
	
	else if(next_months < 8 && next_months % 2 != 0)		{
		while(next_days > 31)		{
			next_months += 1;
			next_days -= 31;
		}
	}

	else if(next_months > 7 && next_months % 2 == 0){
		while(next_days > 31)		{
			next_months += 1;
			next_days -= 31;
		}
	}
	else{
		
		while(next_days > 30)		{
			next_months += 1;
			next_days -= 30;
		}
	}
	
	while(next_months > 12){
		next_years += 1;
		next_months -= 12;
	}
	
	// truncate years value to fit in an 8bit unsigned integer
	temp =  now.year();
	if(temp >= 2000){
		temp -= 2000;
	}
	if(debug){
		Serial.println(F("Saving new time."));
	}
	
	SUConfig.next_upload_Seconds = next_seconds;
	SUConfig.next_upload_Minutes = next_minutes;
	SUConfig.next_upload_Hour = next_hours;
	SUConfig.next_upload_Date = next_days;
	SUConfig.next_upload_Months = next_months;
	SUConfig.next_upload_years = next_years;
	
	saveSUConfig();

	
}




/*
	returns the time in seconds until the next x minutes in an hour. e.g. returns the time in seconds till the next 5 minute interval (12:00pm, 12:05pm, 12:10pm etc)
	Even multiples of 60 minutes should be used, e.g 1,2,3,4,5,6,10,20,30,60. 45 can be used but unsure of the results
*/
uint32_t getSecondsToNextIdleUpload(DateTime now, uint32_t wakeupUploadInterval){
	uint64_t idleWakeupUploadInterval = wakeupUploadInterval; // idle wakeup interval in seconds
	uint16_t temp = now.year();
	uint64_t currentTime;
	uint64_t newTime;
	
	if(idleWakeupUploadInterval == 0){
		idleWakeupUploadInterval = upload_time_interval_idle_default;
	}
	
	if(temp >= 2000){
		temp -=2000;
	}
	currentTime = timeToSeconds(temp, now.month(), now.day(), now.hour(), now.minute(), now.second());
	newTime = currentTime;
	if(debug){
		Serial.println(F("finding time in seconds to next upload"));
		Serial.print(F("interval = "));
		Serial.println(wakeupUploadInterval);
	}
	while(newTime % idleWakeupUploadInterval != 0){
		newTime +=1;
	}
	return (uint32_t)( newTime - currentTime);
}









int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}






