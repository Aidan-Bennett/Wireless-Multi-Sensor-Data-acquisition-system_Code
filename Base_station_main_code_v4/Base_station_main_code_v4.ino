
//#if  defined(Serial_PORT_USBVIRTUAL) //defined(ARDUINO_SAMD_ZERO) &&
////  // Required for Serial on Zero based boards

//#endif
#include "arduino.h"

#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_SharpMemSPI1.h"
#include "I2C_FRAM.h"
#include "samd21_Sleep_function.h"
#include <XBee.h>
//#define Serial SerialUSB
samd21_Sleep_function sleep;

#define DeviceVersionNumber 2

/* --- User configurable global Settings --- */
#define SPI_DIVISOR_GLOBAL 2
#define Serial_BAUD 115200
#define RADIO_BAUD_RATE 57600
#define I2C_FRAM_CLOCK 1000000
#define I2C_RTC_CLOCK  400000




#define debug true
bool keepUSBConnected = true;

bool runSampling = false; // variable controls whether sampling is allowed to run. is set via GUI
bool waitingSamplingEnd = false;
bool waitingSamplingStart = false;

/* ---------- Display variables ---------------- */
#define displayCS 8
#define displayRefresh 10
#define display_PWR_EN 9
#define display_MOSI 38
Adafruit_SharpMem ePaper(displayCS); // display uses pin 10 as chip select
#define BLACK 0
#define WHITE 1

bool displayOn = false;
volatile bool turnOffDisplay = false;


uint8_t timeOutLimitcCount = 104; // 22 for ~ 30 sec

volatile uint8_t timeOnCount = 0;
volatile uint8_t EXTCOM_PWM_OUTPUT = 0;

/*============ Button settings    ========================*/
#define button_menu 17
#define button_back 14
#define button_down 12
#define button_up 13

volatile bool menu_button_flag = false;
volatile bool back_button_flag = false;
volatile bool up_button_flag = false;
volatile bool down_button_flag = false;

/*============ Menu settings    ========================*/
const char **textList = ( const char **)malloc(32); //allocate memory for a pointer to pointers to char arrays, pointers are 4 bytes in size




uint32_t temp1 = 0;
uint32_t temp2 = 0;
uint32_t temp3 = 0;

uint16_t inMenuNum = 0;
uint16_t prevMenuNum = 0;
uint8_t cursorPos = 0;
uint8_t maxCursor = 0;
bool buttonPressedContinuous = false;
uint16_t lastButtonPress = 0;
bool loopButtonPress = false;

bool menuButtonFlag = false;
bool backButtonFlag = false;



/*=============== SDcard variables ======================*/
//SD library may not pull chip select low automatically
//SPISettings SdSPISetting(4000000, MSBFIRST, SPI_MODE0); //not needed now
#define SD_CS_PIN 7
#define SD_card_detect 6
#define sd_PWR_EN 5
#define SD_MOSI 37
#define SPI_DIVISOR_GLOBAL 1
const char commar  = ','; // this is used so often that its better to have as global, i think
const char zero = '0';
const char space = ' ';
//volatile bool wakeFlag = false;

SdFile txtFile;
SdFat SD;



/* --- User configurable Settings --- */
const char fileName[]  = "DATA.CSV" ; //
const char tableName[] = "LOOKUP.CSV" ; //
const char CO2fileName [] = "CO2.CSV" ;

/* ---------------------------------- */



/*=============== RTC variables ========================*/

RTC_DS3231 rtc;
const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

#define RTC_alarm_pin 11
volatile bool rtcTimerInterrupt = false;

uint8_t nextAlarmTime_seconds = 0;
uint8_t nextAlarmTime_minutes = 0;
uint8_t nextAlarmTime_hours = 0;
uint8_t  nextAlarmTime_days = 0;
uint8_t  nextAlarmTime_months = 0;
uint8_t  nextAlarmTime_years = 0;

uint32_t upload_time_interval_sampling = 300; // the time interval used to set the upload time while sampling is enabled
uint32_t upload_time_interval_idle = 300; // the time interval used to set the upload time while sampling is disabled

uint8_t defaultSampleRateHour = 0;
uint8_t defaultSampleRateMin = 5;

/*============ Radio Variables =======================*/

XBee xbee = XBee();
// serial high
//uint8_t shCmd[] = {'S', 'H'};
// serial low
//uint8_t slCmd[] = {'S', 'L'};
// association status
//uint8_t assocCmd[] = {'A', 'I'};

//AtCommandRequest atRequest = AtCommandRequest(shCmd);

//AtCommandResponse atResponse = AtCommandResponse();

#define radio_DTR 2
#define radio_RX_strength 3
#define radio_reset 4
#define radio_CTS 16
#define radio_RTS 15
#define radio_PWR_EN 21
#define radio_sleep_status 18

#define network_channel_PAIRING 0x0F
#define network_ID_PAIRING 0xAAAA

#define network_channel_default 0x0E
#define network_ID_default 0xABCD


#define maxPacketSize 49
uint8_t payload[maxPacketSize];
const uint8_t payloadDataOffset = 2;

uint8_t receivedLoad[maxPacketSize];
uint8_t receivedDataLength;

uint8_t errorCode;
bool pairing_mode = false;

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x415B5F70);

ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBRxResponse radio_rx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
AtCommandRequest AtCMD = AtCommandRequest();
AtCommandResponse AtCMDres = AtCommandResponse();


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

//#define PAIR_REQUEST_POLL			0X20
//#define PAIR_REQUEST_RESPONSE_ACK	0X20
#define PAIR_REQUEST_CHECK			0X20
#define PAIR_REQUEST				0X22

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
#define led_R_1 42
#define led_G_1 41
#define led_R_2 44
#define led_G_2 43

#define PWRLED led_G_2

#define USB_detect A3

#define VBATPIN A2

/*============Variables for FRAM =======================*/
I2C_FRAM fram;



#define FRAM_SIZE 65536; // byte size of installed FRAM

// fix variables
const uint16_t maxSampleBuffersize = 512; // max number of samples to store in FRAM before writing out to SD card, allows some wiggle room for larger sample sizes
const uint8_t maxSampleByteLength = 46;



uint16_t uploadInterval_bufferLength = 5;
const uint16_t minimumSamplingUploadTime = 5; //in minutes
const uint32_t maxSamplingUploadTime = 1440;
const uint16_t samplingUploadTimeIncrement = 5; // change this to set the upload interval time (in minutes) while sampling is active

//FRAM addresses
const uint8_t errCodeFramAddr  = 0; //FRAM address for stored error codes. (2 bytes)
const uint8_t SamplingEnabledFramAddr  = 2; // FRAM address for remembering if sampling is currently enabled. (1 byte)
const uint8_t pairedSensorUnitCountAddr = 4; // FRAM address for the number of Sensor units paired with this base station (1 byte)

const uint8_t defaultSampleBufferSize_Addr = 19; // 2 bytes long, default sample buffer size for Sensor units.

const uint8_t samplePeriod_Addr = 21; // 2 bytes long
const uint8_t sampleBufferIndex_addr = 23;  // 2 bytes long: the address for the current sample buffer size for the number of samples currently stored in FRAM, before writing to SD

const uint8_t nextAlarmTime_hours_addr = 25;	// 1 byte long: address for storing the next upload time for sensor units to connect with the base station
const uint8_t nextAlarmTime_minutes_addr = 26;	// 1 Byte long
const uint8_t nextAlarmTime_seconds_addr = 27;	// 1 byte long
const uint8_t nextAlarmTime_days_addr = 28;		// 1 Byte long
const uint8_t nextAlarmTime_months_addr = 29;	// 1 Byte long
const uint8_t nextAlarmTime_years_addr = 30;	// 1 Byte long

const uint8_t upload_time_interval_sampling_addr = 31;	// 4 Bytes long, 
const uint8_t upload_time_interval_idle_addr = 35;	// 4 Bytes long

const uint8_t errCodeNumTotalAddr = 39; // address for total number of error codes stored
const uint8_t errCodeNumAddr = 40;  // current position counter for error code history
const uint8_t errCodeHistoryAddr = 41; // each error code is 2 bytes with time data of 6 bytes, thus 8 bytes long
const uint8_t errCodeMaxNum = 8;


const uint16_t framDataBufferStartAddr  = 736; //start address for sample data buffer. ~15Kbytes of scratch space for buffering
const uint16_t sensorUnitConfigBegin_addr = 16384; // start address for sensor unit configurations. ~ 49K bytes of config space. can leave ~16KB for config space if more scratch space needed etc.

uint16_t currAddrFRAM = 0;
uint16_t sampleCount = 0;
uint16_t co2SampleCount = 0;


/*============ Sensor unit config settings    ========================*/
int16_t selected_sensorUnit_Index = 0;

char text_buffer[180]; // text buffer for storing long or many strings

const uint8_t max_config_FRAM_byte_length = 128;

struct SUConfig {
	uint32_t device_id_high;
	uint32_t device_id_low;
	uint8_t SensorType;

	uint8_t network_channel;
	uint16_t network_ID;
	uint32_t network_address_high;
	uint32_t network_address_low;


	//char human_read_Label[32];
	uint32_t sample_interval_seconds;
	uint16_t buffer_size;
	uint8_t run_sampling;
	
	//uint8_t next_upload_Minutes;
	//uint8_t next_upload_Seconds;
	//uint8_t next_upload_Date;
	//uint8_t next_upload_Months;
	//uint8_t next_upload_years;
	//uint32_t placeholder_0;
	//uint32_t placeholder_1;
	//uint16_t placeholder_2;
	//uint8_t placeholder_3;

	// max 8 single channels supported
	//float gain_resolution_mV[8];
	//uint8_t channel_enabled;
	uint8_t dual_channel_mode;
	uint8_t channel_gain_index[4];
	uint8_t autoRange_mode;

	//uint32_t placeholder_4;
	//uint32_t placeholder_5;
	//uint32_t placeholder_6;
	//uint16_t placeholder_7;

};
SUConfig tempSUConfig;


/*======================================================*/

/* Interrupt Service Routine (ISR) for real time clock alarm */
void RTC_ISR() {
	sleep.interruptDisable();
	rtcTimerInterrupt = true;  // set real time clock interrupt detect flag
}


/* Interrupt Service Routine (ISR) for waking the device up*/
void menu_ISR() {
	sleep.interruptDisable();
	menu_button_flag = true;  // set real time clock interrupt detect flag
}

void back_ISR() {
	sleep.interruptDisable();
	back_button_flag = true;  // set real time clock interrupt detect flag
}

void up_ISR() {
	sleep.interruptDisable();
	up_button_flag = true;  // set real time clock interrupt detect flag
}

void down_ISR() {
	sleep.interruptDisable();
	down_button_flag = true;  // set real time clock interrupt detect flag
}



void TC5_Handler()
{
	//sleep.interruptDisable();
	TcCount16* TC = (TcCount16*) TC5; // get timer struct
	//if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
//
	//	TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
		////turnOffDisplay = true;
	//	digitalWrite(led_R_1, EXTCOM_PWM_OUTPUT);
	//	EXTCOM_PWM_OUTPUT = ~EXTCOM_PWM_OUTPUT;
	//}
	if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
		turnOffDisplay = true;
		TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
		//digitalWrite(led_R_1, EXTCOM_PWM_OUTPUT);
		//EXTCOM_PWM_OUTPUT = ~EXTCOM_PWM_OUTPUT;

	}
}






void setup() {
	// put your setup code here, to run once:

	// initialize pins to safe states
	//uint8_t temp0 = 0xff;
	//bool check = false;


	//SPI.end();

	//pinMode(38, OUTPUT);
	//digitalWrite(38, HIGH);
	//pinMode(37, OUTPUT);
	//digitalWrite(37, HIGH);
	//pinMode(36, OUTPUT);
	//digitalWrite(36, LOW);

	if (debug) {
		Serial.begin(Serial_BAUD);                                  //start serial connection

		//while (!Serial);
		Serial.println(F("I2C INIT"));
	}

	setupPins();
	if (debug) {
		Serial.println(F("Pin States Set"));
		//Serial.print("compare test: ");

		//check = (0x02 == (temp0 &= (1 << 1)));
		//Serial.println( check ) ;
	}


	beginI2cDevices();



	if (! rtc.begin()) {
		if (debug) {
			Serial.println(F("Couldn't find RTC"));
		}
		while (1);
	}

	if (rtc.lostPower()) {
		if (debug) {
			Serial.println(F("RTC lost power, lets set the time!"));
		}
		// following line sets the RTC to the date & time this sketch was compiled
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}
	rtc.writeSqwPinMode(DS3231_OFF);  // disable square wave output
	rtc.clearRTCAlarm1(true);
	rtc.clearRTCAlarm2(true);

	/* ======== Configure Powered modules ========= */
	if (debug) {
		Serial.println(F("Configuring Module power settings"));
	}
	ADC->CTRLA.bit.ENABLE = 0;
	WDT->CTRL.bit.ALWAYSON = 0;
	WDT->CTRL.bit.ENABLE = 0;
	PM->APBAMASK.reg &= ~PM_APBAMASK_RTC;
	PM->APBAMASK.reg &= ~PM_APBAMASK_WDT;
	PM->APBCMASK.reg &= ~PM_APBCMASK_ADC; // disable clock to ADC
	PM->APBCMASK.reg &= ~PM_APBCMASK_DAC; // disable clock to DAC
	PM->APBCMASK.reg &= ~PM_APBCMASK_I2S; // disable clock to I2S
	PM->APBCMASK.reg &= ~PM_APBCMASK_AC; // disable clock to AC
	PM->APBCMASK.reg &= ~PM_APBCMASK_PTC; // disable clock to peripheral touch controller

	PM->APBCMASK.reg &= ~PM_APBCMASK_AC; // disable comparator

	PM->APBCMASK.reg &= ~PM_APBCMASK_TC7; //
	PM->APBCMASK.reg &= ~PM_APBCMASK_TC6;//
	//PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;//
	PM->APBCMASK.reg &= ~PM_APBCMASK_TC4;
	//PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
	PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;//
	PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;//
	PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;//

	if (debug) {
		Serial.println(F("Attaching Button Interrupts"));
	}
	
	// Button Pins
	digitalWrite(button_menu, LOW); //
	pinMode(button_menu, INPUT);
	digitalWrite(button_back, LOW); //
	pinMode(button_back, INPUT);
	digitalWrite(button_down, LOW); //
	pinMode(button_down, INPUT);
	digitalWrite(button_up, LOW); //
	pinMode(button_up, INPUT);
	digitalWrite(RTC_alarm_pin, LOW); //
	pinMode(RTC_alarm_pin, INPUT);

	attachInterrupt(button_menu, menu_ISR, LOW ); // setup Interrupt for menu button
	attachInterrupt(button_back, back_ISR, LOW ); // setup Interrupt for Back button
	attachInterrupt(button_down, down_ISR, LOW ); // setup Interrupt for down button
	attachInterrupt(button_up  , up_ISR  , LOW ); // setup Interrupt for up button
	attachInterrupt(RTC_alarm_pin, RTC_ISR, LOW); // setup Interrupt pin for real time clock alarm

	if ( !sleep.begin(RTC_alarm_pin) ) {
		if (debug) {
			Serial.println(F("Attaching RTC alarm Interrupt Failed!"));
		}
	}
	

		
	if ( !sleep.addExternalInterrupt(button_menu)) { // add menu button interrupt pin
		if (debug) {
			Serial.println(F("Attaching menu button Interrupt Failed!"));
		}
	}
	if ( !sleep.addExternalInterrupt(button_back)) { // add back button interrupt pin
		if (debug) {
			Serial.println(F("Attaching back button Interrupt Failed!"));
		}
	}
	if ( !sleep.addExternalInterrupt(button_down)) { // add dwon button interrupt pin
		if (debug) {
			Serial.println(F("Attaching down button Interrupt Failed!"));
		}
	}
	if ( !sleep.addExternalInterrupt(button_up)) { // add up button interrupt pin
		if (debug) {
			Serial.println(F("Attaching up button Interrupt Failed!"));
		}
	}
	//  sleep.addExternalInterrupt(button_back);    // add menu button interrupt pin
	//  sleep.addExternalInterrupt(button_down);    // add menu button interrupt pin
	//  sleep.addExternalInterrupt(button_up);    // add menu button interrupt pin
	
	
	startDisplay();
	//logo2();

	
	loadSettings(); // load in settings from FRAM after a restart



	if (debug) {
		Serial.println(F("Setup Complete!"));
	}
	Serial.print(F("free Ram: "));
	Serial.println(FreeRam());
	
	// try to configure SAMD21 chips clock sources and clock multiplexers to reduce power consumption
	configureModuleClockSources();
	configureClocks();
	
	sleep.interruptEnable();
	//testSDCardWriting();

	//fram.writeByte(pairedSensorUnitCountAddr, 0);
}



void configureClocks(){
	uint32_t temp;

	 //The DFLL clock must be requested before being configured
	 //otherwise a write access to a DFLL register can freeze the device.
	 //Errata reference: 9905
	 //Fix/Workaround:
	 //Write a zero to the DFLL ONDEMAND bit in the DFLLCTRL register before
	 //configuring the DFLL module
	
	
	GCLK->GENCTRL.bit.ID |= GCLK_GENCTRL_ID(1); // write the ID bit to be able to read from the Register
	temp = REG_GCLK_GENCTRL;
	
	//// first select the generator
	//REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
	//	GCLK_GENDIV_ID(1);							// Select Generic Clock Generator (GCLK) to generator 1
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	 
	 // now write the configuration data to it to disable it
	//temp = REG_GCLK_GENCTRL;
	temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(1); // Set to external 32KHz clock source // Select GCLK1
	temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK1
	REG_GCLK_GENCTRL =  temp;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	
	
	GCLK->GENCTRL.bit.ID |= GCLK_GENCTRL_ID(3); // write the ID bit to be able to read from the Register
	temp = REG_GCLK_GENCTRL;
	
	//// first select the generator
	//REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
	//GCLK_GENDIV_ID(3);							// Select Generic Clock Generator (GCLK) to generator 1
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	//temp = REG_GCLK_GENCTRL;
	temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(3); // Set to external 32KHz clock source // Select GCLK3
	temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK3
	REG_GCLK_GENCTRL =  temp;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	
	
	GCLK->GENCTRL.bit.ID |= GCLK_GENCTRL_ID(5); // write the ID bit to be able to read from the Register
	temp = REG_GCLK_GENCTRL;
	
	//// first select the generator
	//REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
	//GCLK_GENDIV_ID(5);							// Select Generic Clock Generator (GCLK) to generator 1
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	//temp = REG_GCLK_GENCTRL;
	temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(5); // Set to external 32KHz clock source // Select GCLK5
	temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK5
	REG_GCLK_GENCTRL =  temp;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	
	
	GCLK->GENCTRL.bit.ID |= GCLK_GENCTRL_ID(6); // write the ID bit to be able to read from the Register
	temp = REG_GCLK_GENCTRL;
	
	//// first select the generator
	//REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
	//GCLK_GENDIV_ID(6);							// Select Generic Clock Generator (GCLK) to generator 1
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	//temp = REG_GCLK_GENCTRL;
	temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(6); // Set to external 32KHz clock source // Select GCLK6
	temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK6
	REG_GCLK_GENCTRL =  temp;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	
	
	GCLK->GENCTRL.bit.ID |= GCLK_GENCTRL_ID(7); // write the ID bit to be able to read from the Register
	temp = REG_GCLK_GENCTRL;
	
	//// first select the generator
	//REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
	//GCLK_GENDIV_ID(7);							// Select Generic Clock Generator (GCLK) to generator 1
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	
	//temp = REG_GCLK_GENCTRL;
	temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(7); // Set to external 32KHz clock source // Select GCLK7
	temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK7
	REG_GCLK_GENCTRL =  temp;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	//temp = REG_GCLK_GENCTRL;
	//temp |= GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(8); // Set to external 32KHz clock source // Select GCLK8
	//temp = temp & ~ GCLK_GENCTRL_GENEN;	// disable GCLK7
	//REG_GCLK_GENCTRL =  temp;
	//while (GCLK->STATUS.bit.SYNCBUSY);
	
	/*
	
	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Improve duty cycle Set the duty cycle to 50/50 HIGH/LOW
		~GCLK_GENCTRL_GENEN |						// Enable GCLK4
		GCLK_GENCTRL_SRC_XOSC32K |				// Set the 32KHz low power clock source
		GCLK_GENCTRL_ID(8);							// Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);
	*/
	
}


void configureModuleClockSources(){
	
	//REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Improve duty cycle pwerformance
	//GCLK_GENCTRL_GENEN |							// Enable GCLK2
	//GCLK_GENCTRL_SRC_OSCULP32K |						// Set the 32KHz low power clock source
	//GCLK_GENCTRL_ID(2)	|							// Select GCLK2
	//GCLK_GENCTRL_RUNSTDBY;							// Enable running in standby/sleep
	//while (GCLK->STATUS.bit.SYNCBUSY);
	//
//
	//
	//REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |        // enable GCLK2 to EIC
	//GCLK_CLKCTRL_GEN(2) |						// Select GCLK2
	//GCLK_CLKCTRL_ID_EIC;						// Feed the GCLK2 to EIC
	//while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to RTC
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_RTC);						// Feed the GCLK4 to RTC
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to SERCOM5
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_SERCOM5_CORE);				// Feed the GCLK8 to SERCOM5
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to SERCOM1
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_SERCOM1_CORE);				// Feed the GCLK8 to SERCOM1
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to TCC0_TCC1
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_TCC0_TCC1);						// Feed the GCLK8 to TCC0_TCC1
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to TC6_TC7
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_TC6_TC7);						// Feed the GCLK8 to TC6_TC7
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to DAC
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_DAC);						// Feed the GCLK8 to DAC
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to PTC
			GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
			GCLK_CLKCTRL_ID_PTC);						// Feed the GCLK8 to PTC
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to I2S_0
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_I2S_0);						// Feed the GCLK8 to I2S_0
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to I2S_1
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_I2S_1);						// Feed the GCLK8 to I2S_1
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to AC_ANA
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_AC_ANA);						// Feed the GCLK8 to AC_ANA
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_CLKCTRL = ~GCLK_CLKCTRL_CLKEN & (         // disable GCLK8 to AC_DIG
		GCLK_CLKCTRL_GEN(7) |						// Select GCLK8
		GCLK_CLKCTRL_ID_AC_DIG);						// Feed the GCLK8 to AC_DIG
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
}


void setupPins() {

	//PORT->Group[BOARD_LED_PORT].DIRSET.reg = (1<<BOARD_LED_PIN);  // see board_driver_LED .c .h and port.h

	// Radio pins:
	digitalWrite(radio_DTR, LOW);  // DTR pin
	pinMode(radio_DTR, INPUT);
	digitalWrite(radio_RX_strength, LOW);  // RX strength pin
	pinMode(radio_RX_strength, INPUT);
	//digitalWrite(radio_reset, LOW);  // Radio reset pin set low while radio is powered down
	pinMode(radio_reset, INPUT);  // Radio reset pin set to use built in pullup
	pinMode(radio_CTS, INPUT);
	digitalWrite(radio_CTS, LOW);  // Radio CTS pin


	digitalWrite(radio_RTS, LOW); // radio RTS pin
	pinMode(radio_RTS, OUTPUT);



	// SD card pins
	digitalWrite(SD_card_detect, LOW);  // (6) SD Card detect pin
	pinMode(SD_card_detect, INPUT);
	digitalWrite(SD_CS_PIN, LOW);  // (7) SD slave select pin. Should stay low to reduce power consumption until SD card is powered on.
	pinMode(SD_CS_PIN, OUTPUT);

	// display pins
	digitalWrite(displayCS, LOW);
	pinMode(displayCS, OUTPUT);
	digitalWrite(displayCS, LOW);

	digitalWrite(displayRefresh, LOW);
	pinMode(displayRefresh, OUTPUT);

	// RTC pins
	//digitalWrite(RTC_alarm_pin, LOW); // RTC alarm pin (18)
	//pinMode(RTC_alarm_pin, INPUT);



	// Analog input pins
	//digitalWrite(VBATPIN, LOW); // battery voltage pin
	pinMode(VBATPIN, INPUT);

	// LED pins
	pinMode(led_R_1, OUTPUT);
	digitalWrite(led_R_1, LOW);
	pinMode(led_G_1, OUTPUT);
	digitalWrite(led_G_1, LOW);
	pinMode(led_R_2, OUTPUT);
	digitalWrite(led_R_2, LOW);
	pinMode(led_G_2, OUTPUT);
	digitalWrite(led_G_2, LOW);


	// power control pins:
	digitalWrite(radio_PWR_EN, LOW); // radio enable pin
	pinMode(radio_PWR_EN, OUTPUT);
	digitalWrite(sd_PWR_EN, LOW); // SD enable pin (5)
	pinMode(sd_PWR_EN, OUTPUT);
	digitalWrite(display_PWR_EN, LOW); // Display Enable pin
	pinMode(display_PWR_EN, OUTPUT);

	
}




void setPinsForSleep(){
	
	
	disablePin(radio_RTS);
	disablePin(radio_RX_strength);
	disablePin(radio_DTR);
	disablePin(radio_CTS);
	disablePin(radio_PWR_EN);
	disablePin(radio_reset);
	disablePin(display_PWR_EN);
	disablePin(displayCS);
	disablePin(SD_CS_PIN);
	disablePin(SD_card_detect);
	disablePin(sd_PWR_EN);
	disablePin(VBATPIN);
	disablePin(USB_detect);
	disablePin(led_G_1);
	disablePin(led_G_2);
	disablePin(led_R_1);
	disablePin(led_R_2);
	
	// disable I2C pins
	//disablePin(33);
	//disablePin(34);
	
	// disable UART pins
	//disablePin(0);
	//disablePin(1);
	
	// now disable unused pins
	
	disablePin(19);
	disablePin(20);
	disablePin(22);
	disablePin(24);
	disablePin(A1);
	disablePin(A2);
	disablePin(A4);
	disablePin(A5);
	disablePin(A6);
	disablePin(A7);



	
	//digitalWrite()
	
}


void disablePin(uint8_t pin){
	PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].bit.INEN = 0;	// disable pin input buffer
	PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].bit.PULLEN = 0; //disable pin pull up enable
	
	PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = PORT_DIRCLR_DIRCLR(g_APinDescription[pin].ulPin); // set pin as input
	PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = PORT_OUTCLR_OUTCLR(g_APinDescription[pin].ulPin); // Clear pin output drive strength 
	
}


// used to re-initialize all sensors that rely on I2C or SPI modules
void beginI2cDevices() {
	rtc.begin();
	rtc.enableRTC_DS3231Interrupts(true);
	rtc.writeSqwPinMode(DS3231_OFF);
	
	fram.begin(0x51);
	Wire.setClock(I2C_RTC_CLOCK);

}

void loop() {
	// put your main code here, to run repeatedly:
	
	//radioTest();
	//delay(10000);
	//uint32_t startTime = millis();
	//uint32_t endtime;
	
	digitalWrite(led_R_1, HIGH);
	digitalWrite(led_R_2, LOW);
	digitalWrite(led_G_1, LOW);
	digitalWrite(led_G_2, LOW);
	

	if(rtcTimerInterrupt == true || rtc.clearRTCAlarm1(false) == true){
		
		
		if(uploadTimeNow() == true){
			
			if(debug){
				Serial.println(F("Syncing with Sensor Units..."));
			}
			
			sensorUnitWirelessSync();
			
			setNewUploadTimeAlarm();
			if(debug){
				Serial.println(F("Syncing Done."));
			}
			
		}
		else if(debug){
			Serial.println(F("Upload time match fail."));
			setNewUploadTimeAlarm();
		}
		rtcTimerInterrupt = false;
		
	}
	
	digitalWrite(led_R_1, HIGH);
	digitalWrite(led_R_2, HIGH);
	digitalWrite(led_G_1, LOW);
	digitalWrite(led_G_2, LOW);
	


	
	// check the upload time if a operation is not pending, update alarm time if the current alarm is in the past
	if(!checkUploadTimeNow() ){ //&& waitingSamplingStart == false && waitingSamplingEnd == false
			
		if(debug){
			Serial.println(F("Checking upload time."));
		}
		
		// update upload time based on the current status of the system
		calculateNextUploadTime();
		//if(runSampling == true && waitingSamplingStart == true){
			//calculateNextUploadTime(false);	// use the next standard upload time used for when sampling is not enabled
		//}
		//else if(runSampling == true && waitingSamplingStart == false){
			//calculateNextUploadTime(false, true); // find the next an upload time the sensor unit configs have that is in the future, and use it. otherwise ca
		//}
		//else if (runSampling == false && waitingSamplingStart == true){
			//calculateNextUploadTime(false, true);
		//}
		//else if (runSampling == false && waitingSamplingStart == false){
			//calculateNextUploadTime(false); // use the next standard upload time used for when sampling is not enabled
		//}
		
		
		setNewUploadTimeAlarm();
	}
	
	
	
	digitalWrite(led_R_1, HIGH);
	digitalWrite(led_R_2, HIGH);
	digitalWrite(led_G_1, HIGH);
	digitalWrite(led_G_2, LOW);
	
	if(debug){
		Serial.println(F("Woke up yay."));
	}
	
	
	if ( menu_button_flag || back_button_flag || up_button_flag || down_button_flag) {
		if(debug){
			Serial.println(F("Checking button presses."));
		}
		digitalWrite(led_R_1, HIGH);
		digitalWrite(led_R_2, HIGH);
		digitalWrite(led_G_1, HIGH);
		digitalWrite(led_G_2, HIGH);
		scanButtons();
		//Serial.print(F("free Ram: "));
		//Serial.println(FreeRam());
		
		
		if(inMenuNum == 31 || inMenuNum == 32 || inMenuNum == 111 || inMenuNum == 112 || inMenuNum == 20 || inMenuNum == 21){
			delay(150);
		}
		
		if(menu_button_flag || back_button_flag ){
			delay(300);
		}
		
		sleep.interruptEnable();
		//endtime = millis();
		//Serial.print(F("Time taken: "));
		//Serial.println(endtime - startTime);

	}
	
	
	
	digitalWrite(led_R_1, LOW);
	digitalWrite(led_R_2, HIGH);
	digitalWrite(led_G_1, HIGH);
	digitalWrite(led_G_2, HIGH);
	
	
	//Serial.println(F("Beginning sleep."));
	beginsleep(); // sleep till alarm triggers or a button is pressed.
	
	digitalWrite(led_R_1, LOW);
	digitalWrite(led_R_2, LOW);
	digitalWrite(led_G_1, LOW);
	digitalWrite(led_G_2, LOW);

}




void beginsleep(){
	bool USBDissconected = false;
	//Serial.println(turnOffDisplay);
	if(turnOffDisplay){
		if(debug){
			Serial.println(F("Display timed out."));
		}
		displayOn = false;
		turnOffDisplay = false;
	}
	
	if(displayOn){
		if(debug){
			Serial.println(F("Setting display time out counter."));
			Serial.flush();

		}
		setDisplayTimeout();
		
		//Serial.println(F("Entering polling sleep."));
		digitalWrite(led_R_1, LOW);
		digitalWrite(led_R_2, LOW);
		digitalWrite(led_G_1, HIGH);
		digitalWrite(led_G_2, HIGH);
		//
			
		sleep.idle();
		//sleep.interruptEnable();
		while (!(menu_button_flag || back_button_flag || up_button_flag || down_button_flag || turnOffDisplay || rtcTimerInterrupt));
		{
			sleep.interruptDisable();
			sleep.interruptEnable();
			__WFI();  // Wait for next interrupt
		}
		sleep.restoreRegAfterIdle();
		
		
	}
	else{
		if(debug){
			Serial.println(F("Turning off Display."));
		}
		stopDisplay();
		digitalWrite(led_R_1, LOW);
		digitalWrite(led_R_2, LOW);
		digitalWrite(led_G_1, LOW);
		digitalWrite(led_G_2, HIGH);
			
		setPinsForSleep();
		//sleep.interruptDisable(); // clear existing interrupts. Interrupts will be enabled when in standby
		sleep.standby();
		//sleep.restoreRegAfterIdle();
		//sleep.interruptDisable();
		setupPins();
		
		//sleep.interruptEnable();
		//while (!(menu_button_flag || back_button_flag || up_button_flag || down_button_flag || turnOffDisplay || rtcTimerInterrupt));
		//setupPins();
		//digitalWrite(led_R_2, HIGH);
		//REG_PM_APBCMASK |= PM_APBCMASK_SERCOM3;
		//delay(1);
		//
		
		//digitalWrite(led_R_2, LOW);
		
	}
	//Wire.begin();
	//beginI2cDevices();
	
	if(debug){
		if(menu_button_flag){
			Serial.println(F("menu_button_flag"));
		}
		if(back_button_flag){
			Serial.println(F("back_button_flag"));
		}
		if(up_button_flag){
			Serial.println(F("up_button_flag"));
		}
		if(down_button_flag){
			Serial.println(F("down_button_flag"));
		}
		if(turnOffDisplay){
			Serial.println(F("turnOffDisplay"));
		}
		if(rtcTimerInterrupt){
			Serial.println(F("rtcTimerInterrupt"));
		}
		
		
		
	}
	
	
	
	
	//sleep.interruptDisable(); // clear existing interrupts. Interrupts will be enabled when in standby
	//sleep.standby();
	//while(!(menu_button_flag || back_button_flag || up_button_flag || down_button_flag || turnOffDisplay));
	
	//if(checkUSBConnected() && USBDissconected  ){
		//USBDevice.attach(); // Safely attach the USB
		//if (debug) {
			//Serial.begin(Serial_BAUD);                                  //start serial connection
//
		//}
	//}
	//else if(debug && checkUSBConnected()){
		//Serial.begin(Serial_BAUD);                                  //start serial connection
	//}
	//REG_PM_APBCMASK |= PM_APBCMASK_SERCOM3;
	
	

	
	//Serial.println(F("waking up!"));
	
	
}



//-------------------------------------------------------------------------------------------

/*
Highly device Dependant Code. will only work with Atmel SAMD21 micro controllers!!!!

Code originally and Modified from https://github.com/maxbader/arduino_tools
*/
void setDisplayRefreshPWM() {
	pinMode(displayRefresh, OUTPUT);
	PM->APBCMASK.reg |= PM_APBCMASK_TC3; // enable counter 3 Clock
	delay(1);
	
	REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 32KHz clock source by divisor 1: 32KHz/1=32KHz
						GCLK_GENDIV_ID(4);								// Select Generic Clock (GCLK) 2
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						GCLK_GENCTRL_GENEN |							// Enable GCLK4
						GCLK_GENCTRL_SRC_OSCULP32K |						// Set the 32KHz low power clock source
						GCLK_GENCTRL_ID(4)	|							// Select GCLK4
						GCLK_GENCTRL_RUNSTDBY;							// Enable running in standby/sleep
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	// Feed GCLK4 to TCC2 and TC3
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 and TC3
						GCLK_CLKCTRL_GEN_GCLK4 |						// Select GCLK4
						GCLK_CLKCTRL_ID_TCC2_TC3;						// Feed the GCLK4 to TCC2 and TC3
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	// Enable output to test, should be a frequency of 2Hz
	// Enable the port multiplexer for the digital pin D10
	PORT->Group[g_APinDescription[displayRefresh].ulPort].PINCFG[g_APinDescription[displayRefresh].ulPin].bit.PMUXEN = 1;
	
	// Connect the TC3 timer to the port output D10 - port pins are paired odd PMUXO and even PMUXE
	// In this case peripheral E specifies the TC3/WO[0] timer output
	PORT->Group[g_APinDescription[displayRefresh].ulPort].PMUX[g_APinDescription[displayRefresh].ulPin >> 1].reg = PORT_PMUX_PMUXO_E;
	
	REG_TC3_COUNT16_CC0 = 1024;						// Set the TC3 CC0 register as the TOP value in match frequency mode
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	 
	 
	REG_TC3_INTFLAG |= TC_INTFLAG_OVF;					// Clear the interrupt flags
	//REG_TC3_INTENSET = TC_INTENSET_OVF;				// Enable TC3 interrupts
	REG_TC3_INTENCLR = TC_INTENCLR_OVF;					// Disable TC3 interrupts
	 
	REG_TC3_CTRLA |= TC_CTRLA_PRESCALER_DIV16 |		// Set pre-scaler to 1024, 48MHz/1024 = 46.875KHz
					TC_CTRLA_WAVEGEN_MFRQ |								// Put the timer TC3 into match frequency (MFRQ) mode
					TC_CTRLA_ENABLE;									// Enable TC3
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	 

	//REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ; //sets the TC3 peripheral generic clock ID to use Clock generator zero and enables it
	//while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
//
	//// The type cast must fit with the selected timer mode
	//TcCount16* TC = (TcCount16*) TC3; // get timer struct
//
	//TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCCx
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
//
	//TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	//TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as match Frequency
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
//
	//TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;   // Set pre-scaler
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	//TC->CTRLA.reg |= TC_CTRLA_RUNSTDBY;   // allow running in standby mode
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
//
	//TC->CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC;   // allow reload on prescaler clock source
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
//
	//TC->CC[0].reg = 0x5B8E; // set the compare value CC0 register to 0x5B8E (dec 23438) to achieve a 2Hz compare event with a 1024 clock divider
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//TC->CC[1].reg = 0x5B8E;
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	//TC->CTRLBCLR.bit.DIR = 1;	// set counting direction to up
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
//
//
	//
	////TC->EVCTRL.bit.MCEO0 = 1;	
	////while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	////TC->EVCTRL.bit.MCEO1 = 0;	
	////while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	////TC->EVCTRL.bit.TCEI = 1;	
	//
	//TC->EVCTRL.reg |= TC_EVCTRL_MCEO0;	// enable compare events on CC0
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	//TC->EVCTRL.reg &= ~TC_EVCTRL_MCEO1; // disable compare events on CC1
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	//
	////TC->EVCTRL.reg |= TC_EVCTRL_EVACT_RETRIGGER; // set the counter to re trigger on an event
	//
	//TC->EVCTRL.reg |= TC_EVCTRL_TCEI; // enable timer events 
	//
	//TC->EVCTRL.reg |= TC_EVCTRL_EVACT_START;	// set the counter to start on an event
	
	
	
	
	
	
	// Interrupts
	//TC->INTENSET.reg = 0;              // disable all interrupts
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
		
	//TC->INTENSET.bit.OVF = 1;          // enable over flow interrupt
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
		
	//TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0 interrupt
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	
	//set the pin mux on pin 10 (DISPLAY refresh pin) to enabled
	
	// set pin to use peripheral function E which corresponds to timer counter 3. See table 7-1 headings for functions A to H
	//PORT->Group[g_APinDescription[displayRefresh].ulPort].PMUX[g_APinDescription[displayRefresh].ulPin].reg |= PORT_PMUX_PMUXO_E;  
	
	// enable the pin to be controlled by the selected peripheral function. (PA15)
	//PORT->Group[g_APinDescription[15].ulPort].PINCFG[g_APinDescription[15].ulPin].bit.PMUXEN = 1; 
	
	// Enable InterruptVector
	//NVIC_ClearPendingIRQ(TC3_IRQn);
	//NVIC_SetPriority(TC3_IRQn, 1);
	//NVIC_EnableIRQ(TC3_IRQn);
	
	

	// Enable TC
	//TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	//while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync




}


void pauseDisplayPWM(){
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; // disable timer counter
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	//TC->INTENSET.reg = 0;              // disable all interrupts
	
	
}

void restartDisplayPWM(){
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg |= TC_CTRLA_ENABLE; // Re-enable timer counter
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	
	
	
}

/*
Highly device Dependant Code. will only work with Atmel SAMD21 micro controllers!!!!
*/
void disableDisplayRefreshPWM() {
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; // disable timer counter
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	TC->INTENSET.reg = 0;              // disable all interrupts

	PM->APBCMASK.reg &= ~PM_APBCMASK_TC3; // disable clock to timer counter 3
	timeOnCount = 0;
	
	//Serial.println(GCLK_GENCTRL_IDC | 	GCLK_GENCTRL_SRC_DFLL48M |	GCLK_GENCTRL_ID(4), BIN);


	//// Feed GCLK4 to TCC2 and TC3
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC2_TC3;         // Disable GCLK4 to TCC2 and TC3 
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Disable GCLK4 to save idle power and disable running in standby
	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						GCLK_GENCTRL_SRC_OSCULP32K |						// Set the 32KHz  low power clock source
						GCLK_GENCTRL_ID(4);								// Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	

}


/*
Highly device Dependant Code. will only work with Atmel SAMD21 micro controllers!!!!

Code originally and Modified from https://github.com/maxbader/arduino_tools
*/
void setDisplayTimeout() {
	//digitalWrite(led_R_2, HIGH);
	//digitalWrite(led_G_2, HIGH);
	//Serial.println(F("boop"));
	//SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;	// allow external oscilator to operate in standby
	
	REG_PM_APBCMASK |= PM_APBCMASK_TC5; // enable counter 5 Clock
	delay(1);
	
	REG_GCLK_GENDIV =	GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 64: 48MHz/64=750KHz
						GCLK_GENDIV_ID(2);			  // Select Generic Clock Generator (GCLK) to generator 2
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	REG_GCLK_GENCTRL =	GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
						GCLK_GENCTRL_GENEN |		// Enable GCLK2
						GCLK_GENCTRL_SRC_OSCULP32K |	// Set the 32Khz low power clock source
						GCLK_GENCTRL_ID(2)	|		// Select GCLK5
						GCLK_GENCTRL_RUNSTDBY;		// Enable running in standby
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	// Feed GCLK2 to TC4 and TC5
	REG_GCLK_CLKCTRL =	GCLK_CLKCTRL_CLKEN |        // Enable GCLK2 to TC4 and TC5
						GCLK_CLKCTRL_GEN_GCLK2 |	// Select GCLK2
						GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_TC4_TC5_Val );	// Feed the GCLK4 to TC4 and TC5
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
	
	
	//Set the EIC to use 
	
	
	
	REG_TC5_COUNT16_CC0 = 30720;	// 60 second timeout use 30720					// Set the TC5 CC0 register as the TOP value in match frequency mode - 30720
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	REG_TC5_COUNT16_COUNT = TC_COUNT16_COUNT_COUNT(0);
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	REG_TC5_INTFLAG |= TC_INTFLAG_OVF | TC_INTFLAG_MC0;		// Clear the interrupt flags
	//REG_TC5_INTFLAG |= TC_INTFLAG_MC0;					// Clear the interrupt flags
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	//REG_TC5_INTENSET |= TC_INTENSET_OVF;
	REG_TC5_INTENCLR |= TC_INTENCLR_OVF;					// Disable TC5 overflow interrupts
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	REG_TC5_INTENSET |= TC_INTENSET_MC0 ;					// Enable TC5 interrupts on match //+ TC_INTENSET_OVF
	//Serial.print(TC_INTENSET_MC0, BIN);
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	
	
	REG_TC5_CTRLA |= TC_CTRLA_PRESCALER_DIV64 |		// Set pre-scaler to 1024, 750KHz/1024 = 732.4Hz
	TC_CTRLA_WAVEGEN_MFRQ |								// Put the timer TC3 into match frequency (MFRQ) mode
	TC_CTRLA_ENABLE;									// Enable TC5
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);			// Wait for synchronization
	
	NVIC_EnableIRQ(TC5_IRQn);
	NVIC_SetPriority(TC3_IRQn, 0x00);
	
	// Enable InterruptVector
	//NVIC_ClearPendingIRQ(TC3_IRQn);
	//NVIC_SetPriority(TC3_IRQn, 1);
	
	

}

/*
Highly device Dependant Code. will only work with Atmel SAMD21 micro controllers!!!!
*/
void disableDisplayTimeout() {
	TcCount16* TC = (TcCount16*) TC5; // get timer struct
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE; // disable timer counter
	
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	
	//REG_TC5_COUNT16_COUNT = TC_COUNT16_COUNT_COUNT(0);	// reset the counter
	
	

	TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
	//TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	TC->INTENSET.reg = 0;              // disable all interrupts

	PM->APBCMASK.reg &= ~PM_APBCMASK_TC5; // disable clock to timer counter 5
	timeOnCount = 0;
	
	
	
	// Feed GCLK7 to TC4 and TC5 to disconnect TC4 and TC5 clock source
	REG_GCLK_CLKCTRL =	GCLK_CLKCTRL_CLKEN |        // Enable GCLK2 to TC4 and TC5
					GCLK_CLKCTRL_GEN_GCLK7 |	// Select GCLK7
					GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_TC4_TC5_Val );	// Feed the GCLK7 to TC4 and TC5
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

}





bool uploadTimeNow(){
	uint16_t temp_year;
	DateTime now = rtc.now();
	temp_year = now.year();
	if(temp_year >= 2000){
		temp_year -= 2000;
	}
	uint64_t currentTime = timeToSeconds(temp_year, now.month(), now.day(), now.hour(), now.minute(), now.second());
	uint64_t downloadTime = timeToSeconds(nextAlarmTime_years, nextAlarmTime_months, nextAlarmTime_days, nextAlarmTime_hours, nextAlarmTime_minutes, nextAlarmTime_seconds);
	
	if( currentTime >= downloadTime){
		return true;
	}	
	return false;
}






void testwirelessSyncLoop(){
	menu_button_flag = false;
	back_button_flag = false;
	Serial.println(F("Beginning test run"));
	uint8_t temp = 0;
	
	rtcTimerInterrupt = true;
	
	start_radio();
	
	while (true) {
		
		if(rtcTimerInterrupt == true){
			//Serial.println(F("Running wireless sync"));
			sensorUnitWirelessSync();
			//Serial.println(F("pulsing ACK"));
			
			//addr64 = XBeeAddress64(0x00000000, BROADCAST_ADDRESS);
			//setSUAddress();
			//sendCommand(ACK_REQUEST);
			
			while(Serial1.available() == 1){
				Serial.print(Serial1.read(), HEX);
				
			}
			delay(100);
			while(Serial1.available() == 1){
				Serial.print(Serial1.read(), HEX);
				
			}
			Serial.println();
			//rtcTimerInterrupt = false;
		}
		
		if ( menu_button_flag || back_button_flag) {
			stop_radio();
			return;
		}
		
		
		//sleep.standby();
		//sleep.interruptEnable();
	}

	
	
	
}


/*

*/

void sensorUnitWirelessSync(){
	uint8_t numPairedSensors = fram.readByte(pairedSensorUnitCountAddr);
	uint8_t temp = 0;
	uint8_t receivedNumSamples;
	uint8_t expectedNumSamples;

	char sampleLayoutBuffer[30];
	uint8_t sampleDataElementCount = 0;
	
	uint8_t sensorUnitRetryList[numPairedSensors];
	uint8_t numSensorUnits_toRetry = 0;
	
	uint8_t sensorUnitFailedList[numPairedSensors];
	uint8_t numSensorUnits_FailedSync = 0;
	
	numPairedSensors = 1; // just for testing
	
	if(displayOn){
		drawSyncInProgress();
	}
	
	// update the base stations alarm time 

	calculateNextUploadTime();

	
	
	Serial.println(F("Starting Radio"));
	start_radio();
	delay(1000); // delay 1 second to allow time for sensor units to turn on
	
	// check the status and download the stored samples from all paired sensor units
	for(uint8_t numPairedIndex = 0; numPairedIndex < numPairedSensors; numPairedIndex++ ){
		Serial.println(F("Loading SU config"));
		loadSUConfig(numPairedIndex);	// load the config settings for a sensor unit
		
		Serial.println(F("Setting radio address"));
		setSUAddress();					// set the radio transmit address to the current sensor unit
		
		
		/*
		Serial.println(F("Update SU config with next upload time."));
		// set the sensor units config data to have the next upload date. The Sensor units upload time will be updated when the settings are updated.
		if(runSampling == true && waitingSamplingStart == true){
			setSensorUnitNewUploadTime(false);	// use existing upload time
		}
		else if(runSampling == true && waitingSamplingStart == false){
			setSensorUnitNewUploadTime(true);	// use upload time based on sensor unit settings
		}
		else if(runSampling == false){
			setSensorUnitNewUploadTime(false);
		}
		*/

		Serial.println(F("Ping SU unit"));
		if(pingSU(5)){	// check if there is a reply from the currently selected sensor unit before proceeding
			
			Serial.println(F("Requesting Status data"));
			request_Status();		 // retrieve status info from sensor unit
			
			
			expectedNumSamples = receivedLoad[2];
			expectedNumSamples = (expectedNumSamples << 8) + receivedLoad[3];
			
			
			Serial.println(F("Checking time data"));
			if(checkSU_DateTime() == 1){ // check if the sensor units time is close enough to the base stations time, and update it if needed
				
				updateSU_DateTime();
				for(uint8_t i = 0; i < 3; i++){
					if(waitForPacket(500) == false){
						updateSU_DateTime();
					}
					else{
						if(receivedLoad[0] == ACK){
							break;
						}
					}
				
				}
			}
			
			
			
			
			Serial.println(F("Requesting Setting data"));
			if(request_settings_check() == false){ // verify the Sensor units settings are correct, update if needed.
				Serial.println(F("Updating Setting data"));
				request_settings_update();
			}
			
			
			if( (runSampling == true || waitingSamplingEnd == true) && waitingSamplingStart == false){
				// not finished yet
				Serial.println(F("Requesting sample layout data"));
				sampleDataElementCount = requestSampleLayout(sampleLayoutBuffer); // find out the sample data value layout for use in saving data to SD card.
				Serial.println(sampleDataElementCount);
			
				Serial.println(F("Requesting sample data"));
				fram.writeUInt16(sampleBufferIndex_addr, 0); // clear FRAM data buffer count
				receivedNumSamples = request_sample(expectedNumSamples); // request stored samples from Sensor units, and store them in FRAM. Return the number of Samples in FRAM
				
				SaveDataToSD(sampleLayoutBuffer, sampleDataElementCount);
				
				if(waitingSamplingEnd == true){
					waitingSamplingEnd = false;
				}
			}
			
			if(runSampling == true && waitingSamplingStart == true){
				waitingSamplingStart = false;
			}
			
			Serial.println(F("Finished with this sensor unit for now."));
			
			
			// terminate communication with sensor unit to allow it to sleep and take samples.
			for(uint8_t i = 0; i < 3; i++){
				if(endCommunication() == true){
					break;
				}
			}
					
			//saveDataToSDCard(sampleLayoutBuffer, sampleDataElementCount); // Save the buffered data samples to the SD card in a File specific to the Sensor units Lower ID number (4 bytes)
			
		}
		else{
			// store a list of Sensor units to retry syncing with later
			Serial.println(F("Ping Failed: No Response."));
			sensorUnitRetryList[numSensorUnits_toRetry] = numPairedIndex;
			numSensorUnits_toRetry++;
		}
		
		
	}
	
	Serial.println(F("Retrying failed sensors"));
	// Now Retry the sensor units which had issues. report errors if they fail again.
	for(uint8_t numPairedIndex = 0; numPairedIndex < numSensorUnits_toRetry; numPairedIndex++ ){
		Serial.println(F("Loading SU config"));
		loadSUConfig(numPairedIndex);	// load the config settings for a sensor unit
		
		Serial.println(F("Setting radio address"));
		setSUAddress();					// set the radio transmit address to the current sensor unit
		
		//Serial.println(F("Update SU config with next upload time."));
		//// set the sensor units config data to have the next upload date. The Sensor units upload time will be updated when the settings are updated.
		//if(runSampling == true && waitingSamplingStart == true){
			//setSensorUnitNewUploadTime(false);	// use existing upload time
		//}
		//else if(runSampling == true && waitingSamplingStart == false){
			//setSensorUnitNewUploadTime(true);	// use upload time based on sensor unit settings
		//}
		//else if(runSampling == false){
			//setSensorUnitNewUploadTime(false);
		//}
		
		Serial.println(F("Ping SU unit"));
		if(pingSU(3)){	// check if there is a reply from the currently selected sensor unit before proceeding
			
			Serial.println(F("Requesting Status data"));
			request_Status();		 // retrieve status info from sensor unit
			
			
			expectedNumSamples = receivedLoad[2];
			expectedNumSamples = (expectedNumSamples << 8) + receivedLoad[3];
			
			
			Serial.println(F("Checking time data"));
			if(checkSU_DateTime() == 1){ // check if the sensor units time is close enough to the base stations time, and update it if needed
				
				updateSU_DateTime();
				for(uint8_t i = 0; i < 3; i++){
					if(waitForPacket(500) == false){
						updateSU_DateTime();
					}
					else{
						if(receivedLoad[0] == ACK){
							break;
						}
					}
					
				}
			}
			
			
			Serial.println(F("Requesting Setting data"));
			if(request_settings_check() == false){ // verify the Sensor units settings are correct, update if needed.
				Serial.println(F("Updating Setting data"));
				request_settings_update();
			}
			
			
			if( (runSampling == true || waitingSamplingEnd == true) && waitingSamplingStart == false){
				// not finished yet
				Serial.println(F("Requesting sample layout data"));
				sampleDataElementCount = requestSampleLayout(sampleLayoutBuffer); // find out the sample data value layout for use in saving data to SD card.
				Serial.println(sampleDataElementCount);
				
				Serial.println(F("Requesting sample data"));
				fram.writeUInt16(sampleBufferIndex_addr, 0); // clear FRAM data buffer count
				receivedNumSamples = request_sample(expectedNumSamples); // request stored samples from Sensor units, and store them in FRAM. Return the number of Samples in FRAM
				
				SaveDataToSD(sampleLayoutBuffer, sampleDataElementCount);
				
				if(waitingSamplingEnd == true){
					waitingSamplingEnd = false;
				}
			}
			
			if(runSampling == true && waitingSamplingStart == true){
				waitingSamplingStart = false;
			}
			
			Serial.println(F("Finished with this sensor unit for now."));
			
			
			// terminate communication with sensor unit to allow it to sleep and take samples.
			for(uint8_t i = 0; i < 3; i++){
				if(endCommunication() == true){
					break;
				}
			}
			
			//saveDataToSDCard(sampleLayoutBuffer, sampleDataElementCount); // Save the buffered data samples to the SD card in a File specific to the Sensor units Lower ID number (4 bytes)
			
		}
		else{
			// store a list of Sensor units to retry syncing with later
			Serial.println(F("Ping Failed: No Response."));
			sensorUnitFailedList[numSensorUnits_toRetry] = numPairedIndex;
			numSensorUnits_FailedSync++;
			
			//error(0x00A0); // error retrying sync with Sensor Unit
		}
		
		
	}
	
	// Cycle though and terminate the connection with the sensor units if digi mesh is being used
	//for(uint8_t numPairedIndex = 0; numPairedIndex < numPairedSensors; numPairedIndex++ ){
	//endCommunication(); // terminate communication with sensor unit to allow it to sleep and take samples.
	//}
	
	
	stop_radio();
	
	if(displayOn){
		ePaper.clearDisplay();
		redrawMenus(0);
	}
	

}



/*
	Scans for new devices using broadcast address on the default channel and network ID.
	Updates a text buffer for use in displaying the available sensor units by menu code.
	
	Returns the number of Sensor units found
*/

uint8_t scanForSensorUnits(char *text_buffer, uint16_t sizeOfTextBuffer){
	uint16_t scanTimeout = 10000; // timeout in milliseconds 
	uint32_t startTime;
	uint16_t framDataAddr = framDataBufferStartAddr;
	uint16_t XbeeAddreesCounter = 0;
	uint16_t textIndex = 0;
	XBeeAddress64 tempAddress;
	
	pairing_mode = true;
	
	start_radio();
	
	addr64 = XBeeAddress64(0x00000000, BROADCAST_ADDRESS);
	
	//setRadioNetworkCH(network_channel_PAIRING);
	//setRadioNetworkID(network_ID_PAIRING);
	//radioExitCommandMode();
	
	
	// will use the sample buffer space in FRAM to remember all the addresses received.
	Wire.setClock(I2C_FRAM_CLOCK);
	
	
	for(uint8_t i = 0; i < 4; i++){
		// send pair request check on broadcast and store all the response addresses within the allowed time.
		sendCommand(ACK_REQUEST); 
		startTime = millis();
		if(debug){
			Serial.println(F("Sending ACK_REQUEST on Broadcast..."));
		}
	
		//Wait for responses from any sensor units and save the address of the sensor unit which sent the response.
		while( startTime + (scanTimeout/3) > millis()){
		
			if(receivedPacket(scanTimeout/4, &tempAddress) == true){
				if(verifyReceivedPacketChecksum()){
					if(debug){
						Serial.print(F("Found Broadcast Response! "));
						Serial.print(tempAddress.getMsb(), HEX);
						Serial.println(tempAddress.getLsb(), HEX);
					}
					
					addAddressToBuffer(&framDataAddr, &tempAddress, &XbeeAddreesCounter);
					
				}
			}
		
		}
	}
	
	// now that the addresses of Sensor units in paring mode have been found, retrieve their details to build a human readable string.
	framDataAddr = framDataBufferStartAddr;
	for(uint8_t i = 0; i < XbeeAddreesCounter; i++){
		addr64 = XBeeAddress64(fram.readUInt32(framDataAddr), fram.readUInt32(framDataAddr + sizeof(uint32_t)) );
		
		
		if(requestSUInfoHigh()){
			
			textIndex += sensorTypeToChar(receivedLoad[13], &text_buffer[textIndex]);
			

			integerToHexChar(&text_buffer[textIndex], &receivedLoad[5], 8 );
			textIndex += 17;
			
			
		}
		else{
			text_buffer[textIndex++] = 'E';
			text_buffer[textIndex++] = 'r';
			text_buffer[textIndex++] = 'r';
			text_buffer[textIndex++] = 'o';
			text_buffer[textIndex++] = 'r';
			text_buffer[textIndex++] = '\0';
		}
		
		
		
		framDataAddr += sizeof(uint64_t);
	}
	
	if(debug){
		Serial.println(text_buffer);
	}
	
	stop_radio();
	Wire.setClock(I2C_RTC_CLOCK);
	
	pairing_mode = false;
	
	return XbeeAddreesCounter;
	
}



void addAddressToBuffer(uint16_t *framDataAddr, XBeeAddress64 *tempAddress, uint16_t *numAddressesStored){
	bool existingAddress = false;
	
	if(*framDataAddr + 4 < sensorUnitConfigBegin_addr){
		Serial.println(*numAddressesStored);
		for(uint8_t i = 0; i < *numAddressesStored; i++){
			if(fram.readUInt32(framDataBufferStartAddr + i * sizeof(uint64_t)) == tempAddress->getMsb() 
				&& fram.readUInt32(framDataBufferStartAddr + i * sizeof(uint64_t) + sizeof(uint32_t)) == tempAddress->getLsb()){
				existingAddress = true;
				break;
			}
		}
		
		if(existingAddress == false){
			if(debug){
				Serial.println(F("writing Address to FRAM"));
			}
			fram.writeUInt32(*framDataAddr , tempAddress->getMsb());
			fram.writeUInt32(*framDataAddr + sizeof(uint32_t), tempAddress->getLsb());
			*numAddressesStored += 1;	// increment number of stored addresses
			*framDataAddr += sizeof(uint64_t); // increment the FRAM address
			
		}
	}
}



//bool waitForBroadcastResponse(uint16_t timeout, XBeeAddress64 *response_address){
	//
	//if (!xbee.readPacket(timeout)) {
		////Serial.println(F("Read Packet Timeout"));
		//return false;
	//}
	//if (xbee.getResponse().isAvailable()) {
		//// got something
		////Serial.println(F("Read Packet got something"));
//
		//if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			//xbee.getResponse().getZBRxResponse(radio_rx);
			//savePacketToRAM(radio_rx.getData(), radio_rx.getDataLength());
			//
			//if(receivedLoad[0] == ACK){
				//*response_address = radio_rx.getRemoteAddress64();
				//return true;
			//}
			//else{
				//return false;
			//}
		//}
	//}
	//return false;
//}









void calculateNextUploadTime(){
	
	uint8_t numPairedSensors = fram.readByte(pairedSensorUnitCountAddr);
	uint16_t addr ;
	uint8_t sample_interval_addr_offset = 20;
	uint8_t upload_time_addr_offset = 27;
	uint8_t buffer_count_addr_offset = 24;
	
	uint64_t temp_time_1;
	uint64_t temp_time_2;
	uint64_t currentTime;
	
	uint32_t temp32;
	uint32_t nextUploadTime = 0;
	uint32_t nextUploadTimeSeconds = 0;
	uint16_t nextUploadTimeMinutes = 0;
	uint8_t nextUploadTimeHours = 0;
	uint8_t nextUploadTimedays = 0;
	uint8_t nextUploadTimeMonths = 0;
	uint8_t nextUploadTimeyears = 0;
	
	DateTime now = rtc.now();
	
	
	if( (runSampling == true && waitingSamplingStart == false) || (waitingSamplingEnd == true && runSampling == false) ){

		nextUploadTime = getSecondsToNextIdleUpload(now, upload_time_interval_sampling);
		
	}
	else{
		nextUploadTime = getSecondsToNextIdleUpload( now, upload_time_interval_idle);
	}
	
	
		
	nextUploadTimeSeconds = nextUploadTime + now.second();
	nextUploadTimeMinutes = now.minute();
	nextUploadTimeHours = now.hour();
	nextUploadTimedays = now.day();
	nextUploadTimeMonths = now.month();
	nextUploadTimeyears = now.year();
	
	
	if(now.year() >= 2000){
		nextUploadTimeyears = now.year() - 2000;
	}
	else{
		nextUploadTimeyears = now.year();
	}
		
		
		
	while(nextUploadTimeSeconds > 59){
		nextUploadTimeMinutes += 1;
		nextUploadTimeSeconds -= 60;
	}
		

	while(nextUploadTimeMinutes > 59){
		nextUploadTimeHours += 1;
		nextUploadTimeMinutes -= 60;
	}
		
		
		
	while(nextUploadTimeHours > 23){
		nextUploadTimedays += 1;
		nextUploadTimeHours -= 24;
	}
		
	if(nextUploadTimeMonths < 8 && nextUploadTimeMonths % 2 == 0){
			
		if(nextUploadTimeMonths == 2 && nextUploadTimeyears % 4 != 0)	{
				
			while(nextUploadTimedays > 28){
					
				nextUploadTimeMonths += 1;
				nextUploadTimedays -= 28;
			}
		}
		else{
			while(nextUploadTimedays > 29){
				nextUploadTimeMonths += 1;
				nextUploadTimedays -= 29;
			}
		}
	}
		
	else if(nextUploadTimeMonths < 8 && nextUploadTimeMonths % 2 != 0)		{
		while(nextUploadTimedays > 31)		{
			nextUploadTimeMonths += 1;
			nextUploadTimedays -= 31;
		}
	}

	else if(nextUploadTimeMonths > 7 && nextUploadTimeMonths % 2 == 0){
		while(nextUploadTimedays > 31)		{
			nextUploadTimeMonths += 1;
			nextUploadTimedays -= 31;
		}
	}
	else{
			
		while(nextUploadTimedays > 30)		{
			nextUploadTimeMonths += 1;
			nextUploadTimedays -= 30;
		}
	}
		
	while(nextUploadTimeMonths > 12){
		nextUploadTimeyears += 1;
		nextUploadTimeMonths -= 12;
	}
	
	
	
	
	
	//if(!updateIfInPast){
		//temp_time_1 = timeToSeconds(nextAlarmTime_years, nextAlarmTime_months, nextAlarmTime_days, nextAlarmTime_hours, nextAlarmTime_minutes, nextAlarmTime_seconds);
		//temp_time_2 = timeToSeconds(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
		//temp_time_3 = timeToSeconds(nextUploadTimeyears, nextUploadTimeMonths, nextUploadTimedays, nextUploadTimeHours, nextUploadTimeMinutes,nextUploadTimeSeconds );
			//
		//if(temp_time_3 < temp_time_1){
			//// if the newly calculated time is sooner than the previous time, use the sooner time.
			//nextAlarmTime_seconds = nextUploadTimeSeconds;
			//nextAlarmTime_minutes = nextUploadTimeMinutes;
			//nextAlarmTime_hours = nextUploadTimeHours;
			//nextAlarmTime_days = nextUploadTimedays;
			//nextAlarmTime_months = nextUploadTimeMonths;
			//nextAlarmTime_years = nextUploadTimeyears;
		//}
		//else if(temp_time_1 > temp_time_2){
			//// alarm time is in the future, so does not need updating
			//return;
		//}
			//
	//}
	//else{
		nextAlarmTime_seconds = nextUploadTimeSeconds;
		nextAlarmTime_minutes = nextUploadTimeMinutes;
		nextAlarmTime_hours = nextUploadTimeHours;
		nextAlarmTime_days = nextUploadTimedays;
		nextAlarmTime_months = nextUploadTimeMonths;
		nextAlarmTime_years = nextUploadTimeyears;
	//}
	
	if(debug){
		Serial.print(F("Current time is: "));
		Serial.print(now.hour());
		Serial.print(':');
		Serial.print(now.minute());
		Serial.print(':');
		Serial.print(now.second());
		Serial.print(' ');
		Serial.print(now.day());
		Serial.print('/');
		Serial.print(now.month());
		Serial.print('/');
		Serial.println(now.year() - 2000);
	
		
		
		Serial.print(F("Next upload time is: "));
		Serial.print(nextAlarmTime_hours);
		Serial.print(':');
		Serial.print(nextAlarmTime_minutes);
		Serial.print(':');
		Serial.print(nextAlarmTime_seconds);
		Serial.print(' ');
		Serial.print(nextAlarmTime_days);
		Serial.print('/');
		Serial.print(nextAlarmTime_months);
		Serial.print('/');
		Serial.println(nextAlarmTime_years);
		
		Serial.print(F("total wait time is: "));
		Serial.println(nextUploadTime);
	
	}
	
	// update FRAM copy of upload time

	fram.writeByte(nextAlarmTime_hours_addr, nextAlarmTime_hours);
	fram.writeByte(nextAlarmTime_minutes_addr, nextAlarmTime_minutes);
	fram.writeByte(nextAlarmTime_seconds_addr, nextAlarmTime_seconds);
	fram.writeByte(nextAlarmTime_days_addr, nextAlarmTime_days);
	fram.writeByte(nextAlarmTime_months_addr, nextAlarmTime_months);
	fram.writeByte(nextAlarmTime_years_addr, nextAlarmTime_years);

	
	
	
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
	
	if(temp >= 2000){
		temp -=2000;
	}
	currentTime = timeToSeconds(temp, now.month(), now.day(), now.hour(), now.minute(), now.second());
	newTime = currentTime;
	while(newTime % idleWakeupUploadInterval != 0){
		newTime +=1;
	}
	
	if( (uint32_t)( newTime - currentTime) == 0){
		return (uint32_t)( newTime - currentTime) + wakeupUploadInterval; // catch if the time to next upload is zero, which shouldn't be valid and will cause issues with sensor units.
	}
	else{
		return (uint32_t)( newTime - currentTime);
	}
}




void updateSensorUnitSampleTimeToDefault(){
	
	tempSUConfig.sample_interval_seconds = 60*(uint16_t)defaultSampleRateMin;
	tempSUConfig.sample_interval_seconds += 3600*(uint16_t)defaultSampleRateHour;
	
}


/*
	this method calculates a new upload time for the current sensor unit config that is loaded based off of its sample rate and buffer size.

void setSensorUnitNewUploadTime(bool useSensorUnitSettings){
	DateTime now = rtc.now();

		
	uint64_t temp_time_1;
	uint64_t temp_time_2;
	uint64_t temp_time_3;
	
	uint32_t temp32;
	uint32_t nextUploadTime = 0;
	
	uint32_t nextUploadTimeSeconds = 0;
	uint16_t nextUploadTimeMinutes = 0;
	uint8_t nextUploadTimeHours = 0;
	uint8_t nextUploadTimedays = 0;
	uint8_t nextUploadTimeMonths = 0;
	uint8_t nextUploadTimeyears = 0;
	
	//if(checkUploadTimeNow() && runSampling == 0){
	//	calculateNextUploadTime(useSensorUnitSettings);
	//}
	
	if(useSensorUnitSettings == true){
		
		// use the sensor unit settings to find an appropriate upload time
		temp32 = tempSUConfig.sample_interval_seconds;
		temp32 = temp32 * tempSUConfig.buffer_size;
	
	
		nextUploadTime = temp32;
	
	

	
		nextUploadTimeSeconds = nextUploadTime + now.second();
		nextUploadTimeMinutes = now.minute();
		nextUploadTimeHours = now.hour();
		nextUploadTimedays = now.day();
		nextUploadTimeMonths = now.month();
		nextUploadTimeyears = now.year();
	
		if(now.year() >= 2000){
			nextUploadTimeyears = now.year() - 2000;
		}
		else{
			nextUploadTimeyears = now.year();
		}
	
	
	
		while(nextUploadTimeSeconds > 59){
			nextUploadTimeMinutes += 1;
			nextUploadTimeSeconds -= 60;
		}
	

		while(nextUploadTimeMinutes > 59){
			nextUploadTimeHours += 1;
			nextUploadTimeMinutes -= 60;
		}
	
	
	
		while(nextUploadTimeHours > 23){
			nextUploadTimedays += 1;
			nextUploadTimeHours -= 24;
		}
	
		if(nextUploadTimeMonths < 8 && nextUploadTimeMonths % 2 == 0){
		
			if(nextUploadTimeMonths == 2 && nextUploadTimeyears % 4 != 0)	{
			
				while(nextUploadTimedays > 28){
				
					nextUploadTimeMonths += 1;
					nextUploadTimedays -= 28;
				}
			}
			else{
				while(nextUploadTimedays > 29){
					nextUploadTimeMonths += 1;
					nextUploadTimedays -= 29;
				}
			}
		}
	
		else if(nextUploadTimeMonths < 8 && nextUploadTimeMonths % 2 != 0)		{
			while(nextUploadTimedays > 31)		{
				nextUploadTimeMonths += 1;
				nextUploadTimedays -= 31;
			}
		}

		else if(nextUploadTimeMonths > 7 && nextUploadTimeMonths % 2 == 0){
			while(nextUploadTimedays > 31)		{
				nextUploadTimeMonths += 1;
				nextUploadTimedays -= 31;
			}
		}
		else{
		
			while(nextUploadTimedays > 30)		{
				nextUploadTimeMonths += 1;
				nextUploadTimedays -= 30;
			}
		}
	
		while(nextUploadTimeMonths > 12){
			nextUploadTimeyears += 1;
			nextUploadTimeMonths -= 12;
		}
		
		// use the newly calculated time
		tempSUConfig.next_upload_Hour = nextUploadTimeHours;
		tempSUConfig.next_upload_Minutes = nextUploadTimeMinutes;
		tempSUConfig.next_upload_Seconds = nextUploadTimeSeconds;
		tempSUConfig.next_upload_Date = nextUploadTimedays;
		tempSUConfig.next_upload_Months = nextUploadTimeMonths;
		tempSUConfig.next_upload_years = nextUploadTimeyears;
		
	}
	else{
		
		// use the existing upload time 
	tempSUConfig.next_upload_Hour = nextAlarmTime_hours;
	tempSUConfig.next_upload_Minutes = nextAlarmTime_minutes;
	tempSUConfig.next_upload_Seconds = nextAlarmTime_seconds;
	tempSUConfig.next_upload_Date = nextAlarmTime_days;
	tempSUConfig.next_upload_Months = nextAlarmTime_months;
	tempSUConfig.next_upload_years = nextAlarmTime_years;

	
	
	saveSUConfigToFram();
	
	
	
	
	Serial.print(F("upload time for this Sensor Unit: "));
	Serial.print(tempSUConfig.next_upload_Hour);
	Serial.print(':');
	Serial.print(tempSUConfig.next_upload_Minutes);
	Serial.print(':');
	Serial.print(tempSUConfig.next_upload_Seconds);
	Serial.print(space);
	Serial.print(tempSUConfig.next_upload_Date);
	Serial.print('/');
	Serial.print(tempSUConfig.next_upload_Months);
	Serial.print('/');
	Serial.println(tempSUConfig.next_upload_years);
	
}
*/


void setNewUploadTimeAlarm(){
	rtc.clearRTCAlarm1(true);
	rtc.clearRTCAlarm2(true);
	
	//DateTime alarmtime(nextAlarmTime_years + 2000, nextAlarmTime_months, nextAlarmTime_days, nextAlarmTime_hours, nextAlarmTime_minutes, nextAlarmTime_seconds);
	rtc.setAlarm1Time(nextAlarmTime_seconds, nextAlarmTime_minutes, nextAlarmTime_hours, nextAlarmTime_days, false);
	
}



/*
	this method returns true if the current alarm time is in the future, false otherwise
*/
bool checkUploadTimeNow(){
	uint64_t currentAlarmTime = timeToSeconds(nextAlarmTime_years, nextAlarmTime_months, nextAlarmTime_days, nextAlarmTime_hours, nextAlarmTime_minutes, nextAlarmTime_seconds);
	uint64_t currentTime;
	uint16_t temp;
	
	//delay(2000);
	DateTime now = rtc.now();
	temp = now.year();
	if(temp >= 2000){
		temp -=2000;
	}
	currentTime = timeToSeconds(temp, now.month(), now.day(), now.hour(), now.minute(), now.second());
	
	if(currentTime < currentAlarmTime){
		return true;
	}
	else{
		return false;
	}
}




/*
============ Radio Code ==============================

/*

*/
void start_radio() {
	pinMode(radio_PWR_EN, OUTPUT);
	pinMode(radio_RTS, OUTPUT);
	pinMode(radio_CTS, INPUT);

	digitalWrite(radio_PWR_EN, HIGH);
	digitalWrite(radio_RTS, LOW);


	//  #define radio_DTR 10
	//#define radio_RX_strength 5
	//#define radio_reset 13
	//#define radio_CTS 8
	//#define radio_RTS 12
	//#define radio_PWR_EN 4
	//#define radio_sleep_status 6
	////#define radio_ss 17
	//#define radio_SPI_ATTN 9
	
	memset(receivedLoad, 0, sizeof(receivedLoad));
	


	Serial1.begin(RADIO_BAUD_RATE);
	xbee.setSerial(Serial1);

	// pause till Radio is able to receive data
	while (digitalRead(radio_CTS) == 1) {
	}
	delay(100);

	if (pairing_mode == true){
		setRadioNetworkCH(network_channel_PAIRING);
		setRadioNetworkID(network_ID_PAIRING);
	}
	else{
		setRadioNetworkID(network_ID_default);
		setRadioNetworkCH(network_channel_default);
	}
	
	radioExitCommandMode();
	

	//fram.writeUInt32(baseStation_address64_high, 0x0013A200);
	//fram.writeUInt32(baseStation_address64_low, 0x415B5F2B);

	//base_station_address_high = fram.readUInt32(baseStation_address64_high);
	//base_station_address_low  = fram.readUInt32(baseStation_address64_low);
	//addr64 = XBeeAddress64(base_station_address_high, base_station_address_low);
	//zbTx = ZBTxRequest(addr64, payload, sizeof(payload));

}



/*

*/
void stop_radio() {
	pinMode(radio_PWR_EN, OUTPUT);
	pinMode(radio_RTS, INPUT);
	pinMode(radio_CTS, INPUT);

	Serial1.end();

	digitalWrite(radio_PWR_EN, LOW);
	digitalWrite(radio_RTS, LOW);


	//  #define radio_DTR 10
	//#define radio_RX_strength 5
	//#define radio_reset 13
	//#define radio_CTS 8
	//#define radio_RTS 12
	//#define radio_PWR_EN 4
	//#define radio_sleep_status 6
	////#define radio_ss 17
	//#define radio_SPI_ATTN 9

}




/*
*	This Method sets the current XBee radio address to the address stored in the
*	SUConfig struct. the struct should be preloaded with the current config before
*	using this method.
*/
void setSUAddress(){
	
	addr64 = XBeeAddress64(tempSUConfig.network_address_high, tempSUConfig.network_address_low);
	Serial.print(F("Setting radio address to: "));
	Serial.print(tempSUConfig.network_address_high, HEX);
	Serial.print(space);
	Serial.println(tempSUConfig.network_address_low, HEX);
}




/*

*/
void sendCommand(uint8_t cmd) {
	//uint8_t cmd_payload[4];
	//Serial.print(F("Sending command: "));
	//Serial.println(cmd, HEX);
	payload[0] = cmd; // add command to payload
	payload[1] = 0;  // add data length to payload
	payload[2] = cmd; // add checksum byte. (sum of all bytes)
	
	transmitPayload(3);
}



/*

*/
void sendCommand(uint8_t cmd, uint8_t *data, uint8_t datalength) {
	uint8_t cmd_payload[4];
	uint8_t checksum;
	//Serial.print(F("Sending command: "));
	//Serial.println(cmd, HEX);
	payload[0] = cmd; // add command to payload
	checksum = cmd;
	payload[1] = datalength;  // add data length to payload
	checksum += payload[1];
	
	for(uint8_t i = 0; i < datalength; i++){
		payload[2 + i] = data[i];
		checksum += data[i];
	}
	
	
	
	payload[2 + datalength] = checksum; // add checksum byte. (sum of all bytes)
	
	for(uint8_t i = 0; i < datalength +3; i++){
		Serial.print(payload[i], HEX);
		Serial.print(' ');
	}
	Serial.println();
	
	
	transmitPayload(datalength + 3);
}



void transmitPayload(uint8_t payloadSizeBytes){
	Serial.print(F("Sending Command: "));
	Serial.println(payload[0], HEX);
	
	
	zbTx = ZBTxRequest(addr64, payload, payloadSizeBytes);
	//zbTx.setFrameId(0); //does not increase data transfer speed
	while (digitalRead(radio_CTS) == 1);
	xbee.send(zbTx);
	//Serial.println(freeRam());
}


/*
this method sends AT commands to set the XBee into coordinator mode
*/
bool setRadioAsCoordinator() {
	uint8_t cmd[] = {'C', 'E'};
	uint8_t cmdValue = 0x01;

	return sendATCommand(cmd, &cmdValue, 1);
	
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
	
	receivedPacket(1000); // wait for a response
	
	// check if the status code byte of the command returned OK (0x00)
	if ( receivedLoad[2] == 0x00) {
		return true;
	}
	else {
		return false;
	}
}




bool waitForPacket(){
	return waitForPacket(2000);
}


bool waitForPacket(uint16_t timeout_length){
	uint32_t startTime = 0;
	//uint32_t timeout_length = 30000;
	uint8_t previous_cmd = 0;
	
	startTime = millis();
	while(startTime + timeout_length > millis()){
		if(receivedPacket(200)){
			Serial.println(F("Received a packet!"));
			if(verifyReceivedPacketChecksum()){
				return true;
			}
		}
		
		
	}
	return false;
	
}



bool receivedPacket() {
	//XBeeAddress64 temp = NULL;
	return receivedPacket(500, NULL);
}


bool receivedPacket(uint16_t timeout) {
	//XBeeAddress64 temp = NULL;
	return receivedPacket(timeout, NULL);
}




/*

*/
bool receivedPacket(uint16_t timeout,  XBeeAddress64 *response_address) {
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
			processReceivedATCommand(AtCMDres);
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
		}
		else {
			// not something we were expecting
			return false;
		}
	}
	else if (xbee.getResponse().isError()) {
		//Error reading packet.
		errorCode = xbee.getResponse().getErrorCode();
		//log_error(RADIO_ERROR, errorCode );

		return false;

	}
	return false;
}


//bool receivedPacket() {
	//return receivedPacket(500);
//}
///*
//
//*/
//bool receivedPacket(uint16_t timeout) {
	////bool recieved_command = false;
//
	//if (!xbee.readPacket(timeout)) {
		////Serial.println(F("Read Packet Timeout"));
		//return false;
	//}
	//if (xbee.getResponse().isAvailable()) {
		//// got something
		////Serial.println(F("Read Packet got something"));
//
		//if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			//xbee.getResponse().getZBRxResponse(radio_rx);
			//savePacketToRAM(radio_rx.getData(), radio_rx.getDataLength());
			//errorCode = 0xFF; //clear error code
			//return true;
		//}
		//else if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
			//xbee.getResponse().getAtCommandResponse(AtCMDres);
			//// the local XBee sends this response if an AT command was sent to it
			//processReceivedATCommand(AtCMDres);
			//return true;
		//}
		//else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
			////Serial.println(F("Read Packet modem status response"));
			//xbee.getResponse().getModemStatusResponse(msr);
			//// the local XBee sends this response on certain events, like association/dissociation
//
			//if (msr.getStatus() == ASSOCIATED) {
				//errorCode = 0xFF; //clear error code
				//return false;
			//}
			//else if (msr.getStatus() == DISASSOCIATED) {
			////        // this is awful.. flash led to show our discontent
				//Serial.println(F("DISASSOCIATED"));
			////        log_error(RADIO_ERROR, msr.getStatus());
			////        return false;
			//}
			//else {
				//// this is awful..
				////log_error(RADIO_MODEM_STATUS, msr.getStatus() );
				//return false;
			//}
		//}
		//else {
			//// not something we were expecting
			//return false;
		//}
	//} 
	//else if (xbee.getResponse().isError()) {
		//Serial.println(F("Read Packet Error"));
		////Error reading packet.
		//errorCode = xbee.getResponse().getErrorCode();
		////log_error(RADIO_ERROR, errorCode );
//
		//return false;
//
	//}
	//return false;
//}



uint8_t processReceivedATCommand(AtCommandResponse cmdResponse) {
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
void savePacketToRAM(uint8_t *data, uint8_t data_length) {
	if ( data_length < maxPacketSize) { // check to ensure the data length does not exceed the maximum index of the buffer array. (assuming array size is defined by maxPacketSize)
		for (uint8_t i = 0; i < data_length; i++) {
			receivedLoad[i] = data[i];
		}
		receivedLoad[data_length] = '\0'; // add delimiter to the end
	}
	receivedDataLength = data_length;
}



void saveATResponseToRAM(uint8_t *command, uint8_t status, uint8_t *value, uint8_t valueLength){
	receivedLoad[0] = command[0];
	receivedLoad[1] = command[1];
	receivedLoad[2] = status;
	receivedLoad[3] = valueLength;
	if ( (valueLength + 4) < maxPacketSize) { // check to ensure the data length does not exceed the maximum index of the buffer array. (assuming array size is defined by maxPacketSize)
		for (uint8_t i = 0; i < valueLength; i++) {
			receivedLoad[i + 4] = value[i];
		}
		receivedLoad[valueLength] = '\0'; // add delimiter to the end
	}
	receivedDataLength = valueLength;

}




/*

*/
bool verifyReceivedPacketChecksum() {
	uint8_t checksum = 0;
	uint8_t datalength = receivedLoad[1];


	//  Serial.print(F("Data length: "));
	//  Serial.println(datalength);

	for (uint8_t i = 0; i < (datalength + 2) ; i++) { // use offset of 2 byte to include the command and length bytes, but not the checksum
		checksum += receivedLoad[i];
		//    Serial.print(receivedLoad[i], HEX);
	}

	//Serial.println();
	//Serial.print(F("Calculated checksum: "));
	//Serial.println(checksum, HEX);
	//Serial.print(F("Received checksum: "));
	//Serial.println(receivedLoad[datalength + 2], HEX);


	if ( receivedLoad[datalength + 2] == checksum) { // use offset of 1 byte to get the checksum byte
		//Serial.print(F("Checksum Pass"));
		Serial.print(F("Received Command: "));
		Serial.println(receivedLoad[0], HEX);
		return true;
	}
	else {
		Serial.println(F("Checksum Fail"));
		return false;
	}

}







bool pingSU(uint8_t retry){
	
	payload[0] = ACK_REQUEST;
	payload[1] = 0;
	payload[2] = ACK_REQUEST; // checksum byte
	
	transmitPayload(3);	// transmit three bytes
	
	if(waitForPacket()){
		if(receivedLoad[0] == ACK){
			return true;
		}
		else if(receivedLoad[0] == NACK && retry >= 1){
			return pingSU(retry - 1);
			
		}
		
	}
	else if( retry >= 1){
		return pingSU(retry - 1);
	}
	
	return false;
}








/*

*/
int8_t request_sample(uint16_t expectedNumSamples) {
	bool run_true = true;
	uint8_t count = 0;
	uint8_t errorCount = 0;
	uint32_t startTime = 0;
	uint32_t endTime = 0;
	uint32_t timeout;
	//uint16_t expectedNumSamples;
	uint16_t currentSampleIndex = 0;
	uint16_t temp16;
	uint8_t buffer[4];

	
	Serial.print(F("Expected number of samples = "));
	Serial.println(expectedNumSamples);

	while (expectedNumSamples >= currentSampleIndex && errorCount < 5) {

		sendCommand(SAMPLE_TRANSMIT_REQUEST_NEXT);

		
		if (waitForPacket()) {
				
			if ( receivedLoad[0] == SAMPLE_TRANSMIT_DATA) {
				temp16 = receivedLoad[2];
				temp16 = (temp16 << 8) + receivedLoad[3];
					
					
				if(temp16 == currentSampleIndex){
					saveDataSampleToFRAM();
					
					sendCommand(ACK);
					currentSampleIndex += 1;
					count += 1;
				}
				else{
					if(debug){
						Serial.print(F("Received sample index: "));
						Serial.print(temp16);
						Serial.print(F(", Expected: "));
						Serial.println(currentSampleIndex);
							
							
					}
					buffer[0] = (currentSampleIndex >> 8);
					buffer[1] = (currentSampleIndex );
						
					while (receivedPacket(500)); // clear out all old data from radio buffers
						
					sendCommand(SAMPLE_TRANSMIT_REQUEST_INDEX, buffer, 2);
				}
				//Serial.println(F("SAMPLE_TRANSMIT_DATA"));
				//printLoad();
			}
			else if ( receivedLoad[0] == SAMPLE_END_OF_DATA) {
				sendCommand(ACK);
				//delay(50);
				//sendCommand(END_COMMUNICATION);
				//Serial.println(F("SAMPLE_END_OF_DATA"));
				break;
					
			}
				
		}
			
		else {
			//sendCommand(NACK);
			//Serial.println(F("Verify NACK"));
			errorCount++;
		}

	}

	
	

	return count;
}



/*
	Save received data into FRAM buffer for writing to SD card later
	*/

bool saveDataSampleToFRAM(){
	uint8_t data_start_offset = 4;
	uint16_t sample_index = fram.readUInt16(sampleBufferIndex_addr);
	uint16_t addr = framDataBufferStartAddr + sample_index * maxSampleByteLength;
	
	Wire.setClock(I2C_FRAM_CLOCK);
	
	if(sample_index < maxSampleBuffersize){
		for(uint8_t i = 0; i < maxSampleByteLength ; i++){
			fram.writeByte(addr, receivedLoad[i + data_start_offset]);
			//Serial.print(receivedLoad[i + data_start_offset]);
			//Serial.print(space);
			addr += 1;
		}
		
		fram.writeUInt16(sampleBufferIndex_addr, sample_index +1);
		return true;
	}
	
	Wire.setClock(I2C_RTC_CLOCK);
	
	return false;

}




/*
Data type layout is used to inform the base station of which and what order of bytes are
used for each variable type in a data sample when broken down into the wireless transmission payload.

Data type layout does not include date/time as all 6 values are uint8_t, and also does not include checksum as it is always the last byte of the packet and a uint8_t (1 byte).

 Data type    | char symbol representation
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
int8_t requestSampleLayout(char *buffer){
	bool receivedData = false;
	uint16_t timeout = 500;
	uint8_t errorCount = 0;
	int8_t sampleLength = 0; 
	
	sendCommand(DATA_SAMPLE_LAYOUT_REQUEST);
	
	while (errorCount < 3 && receivedData == false) {
		if (waitForPacket(timeout) == true) {
			if ( receivedLoad[0] == DATA_SAMPLE_LAYOUT) {
				receivedData = true;
				break;
				
			}
			else{
				sendCommand(DATA_SAMPLE_LAYOUT_REQUEST);
				errorCount++;
			}
		}
		else {
			sendCommand(DATA_SAMPLE_LAYOUT_REQUEST);
			errorCount++;
		}
	}
	
	
	for(uint8_t i=0; i < receivedLoad[1] ; i++){
		if(receivedLoad[i+2] == 'E'){
			return sampleLength;
		}
		
		buffer[i] = receivedLoad[i+2];
		sampleLength ++;
		
	}
	
	
	return -1;
	
}




/*

*/
bool request_settings_check() {
	uint8_t errorCount = 0;
	uint32_t timeout;
	
	sendCommand(SU_SENSOR_SETTING_REQUEST);
	while (errorCount < 5) {

		if (waitForPacket(500) == true) {
			//Serial.println(receivedLoad[0],HEX);
			if ( receivedLoad[0] == SU_SENSOR_SETTING_DATA) {
				break;
			}
			else{
				sendCommand(SU_SENSOR_SETTING_REQUEST);
				errorCount++;
				
			}

		}
		else {
			sendCommand(SU_SENSOR_SETTING_REQUEST);
			errorCount++;
		}

	}
	if(errorCount >= 3){
		return false;
	}
	
	if (checkSU_settings_valid()) {
		
		return true;
	}
	else{
		return false;
	}

	

	

}


/*

*/
bool request_settings_update() {
	uint8_t errorCount = 0;
	uint8_t checksum = 0;

	payload[0] = SU_SENSOR_SETTING_UPDATE;
	payload[1] = 15;
	
	//if (tempSUConfig.SensorType == 1) {
		//payload[1] = 19;
	//}
	//else {
		//
	//}
	
	payload[2] = (tempSUConfig.buffer_size >> 8) & 0xff;
	payload[3] = tempSUConfig.buffer_size & 0xff;

	payload[4] = (tempSUConfig.sample_interval_seconds >> 24) & 0xff;
	payload[5] = (tempSUConfig.sample_interval_seconds >> 16) & 0xff;
	payload[6] = (tempSUConfig.sample_interval_seconds >> 8) & 0xff;
	payload[7] = tempSUConfig.sample_interval_seconds & 0xff;



	payload[8] = (upload_time_interval_sampling >> 24) & 0xff;
	payload[9] = (upload_time_interval_sampling >> 16) & 0xff;
	payload[10] = (upload_time_interval_sampling >> 8) & 0xff;
	payload[11] = upload_time_interval_sampling & 0xff;

	payload[12] = (upload_time_interval_idle >> 24) & 0xff;
	payload[13] = (upload_time_interval_idle >> 16) & 0xff;
	payload[14] = (upload_time_interval_idle >> 8) & 0xff;
	payload[15] = upload_time_interval_idle & 0xff;


	if(runSampling == true){
		payload[16] = 1;
	}
	else{
		payload[16] = 0;
	}
	//payload[14] = tempSUConfig.run_sampling;

	//if (tempSUConfig.SensorType == 1) {
		//payload[17] = tempSUConfig.dual_channel_mode;
		//payload[18] = tempSUConfig.autoRange_mode;
		//payload[19] = tempSUConfig.channel_gain_index[0];
		//payload[18] = tempSUConfig.channel_gain_index[1];
		//payload[19] = tempSUConfig.channel_gain_index[2];
		//payload[20] = tempSUConfig.channel_gain_index[3];
	//}

	for (uint8_t i = 0; i < payload[1] + 2 ; i++) {
		checksum += payload[i];
		//Serial.print(payload[i],HEX);
	}
	payload[17] = checksum;


	transmitPayload(payload[1] + 3);

	// Acknowledge return check
	
	while (errorCount < 3) {

		if (waitForPacket(500) == true) {
			//Serial.println(receivedLoad[0],HEX);
			if ( receivedLoad[0] == ACK) {
				return true;
			}
			else{
				transmitPayload(payload[1] + 3);
				errorCount++;
			
			}

		}
		else {
			transmitPayload(payload[1] + 3);
			errorCount++;
		}

	}

	return false;

}






bool checkSU_settings_valid() {
	bool data_match = true;
	uint16_t tempUint16 = 0;
	uint32_t tempUint32 = 0;


	tempUint16 = ( receivedLoad[2]  );	// check buffer size match
	tempUint16 = ( tempUint16 << 8) + (receivedLoad[3] );
	
	if ( tempUint16 != tempSUConfig.buffer_size) {
		data_match = false;
	}
	
	tempUint32 = receivedLoad[4];						// check sample interval match
	tempUint32 = (tempUint32 << 8) + receivedLoad[5] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[6] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[7];
	  
	if ( tempUint16 != tempSUConfig.sample_interval_seconds) {
		data_match = false;
	}
	
	// Check next upload to base station date/time match
	
	tempUint32 = receivedLoad[8];						// check sampling interval upload time
	tempUint32 = (tempUint32 << 8) + receivedLoad[9] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[10] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[11];
	
	if ( tempUint32 != upload_time_interval_sampling) {
		data_match = false;
	}
	

	tempUint32 = receivedLoad[12];						// check idle interval upload time
	tempUint32 = (tempUint32 << 8) + receivedLoad[13] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[14] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[15];
	
	if ( tempUint32 != upload_time_interval_idle) {
		data_match = false;
	}
	
	
	Serial.println();

	//if ( receivedLoad[14] != tempSUConfig.next_upload_years) {
	//data_match = false;
	//}

	if ( receivedLoad[16] != tempSUConfig.run_sampling) {
		data_match = false;
	}

	//// if the sensor unit is an ADC type with additional settings, check them
	//if (tempSUConfig.SensorType == 1 && receivedLoad[1] > 13) {
		//if ( receivedLoad[17] != tempSUConfig.dual_channel_mode) {
			//data_match = false;
		//}
		//if ( receivedLoad[18] != tempSUConfig.autoRange_mode) {
			//data_match = false;
		//}
		//if ( receivedLoad[19] != tempSUConfig.channel_gain_index[0]) {
			//data_match = false;
		//}
		//if ( receivedLoad[20] != tempSUConfig.channel_gain_index[1]) {
			//data_match = false;
		//}
		//if ( receivedLoad[21] != tempSUConfig.channel_gain_index[2]) {
			//data_match = false;
		//}
		//if ( receivedLoad[22] != tempSUConfig.channel_gain_index[3]) {
			//data_match = false;
		//}
	//}
	printSUsettings();
	return data_match;
}


void printSUsettings(){
	uint16_t tempUint16 = 0;
	uint32_t tempUint32 = 0;
	
	
	tempUint16 = receivedLoad[2];
	tempUint16 = (tempUint16 << 8) + receivedLoad[3] ;	// check buffer size match
	Serial.print(tempUint16);
	Serial.print(space);

	tempUint32 = receivedLoad[4];						// check sample interval match
	tempUint32 = (tempUint32 << 8) + receivedLoad[5] ;
	tempUint32 = (tempUint32 << 8) + receivedLoad[6] ; 
	tempUint32 = (tempUint32 << 8) + receivedLoad[7];  
	Serial.print(tempUint32);
	Serial.print(space);
	
	for(uint8_t i = 8; i < 20; i++){
		Serial.print(receivedLoad[i]);
		Serial.print(space);
	}
	Serial.println();
	
	Serial.print(tempSUConfig.buffer_size);
	Serial.print(space);
	Serial.print(tempSUConfig.sample_interval_seconds);
	Serial.print(space);
	Serial.print(upload_time_interval_sampling);
	Serial.print(space);
	Serial.print(upload_time_interval_idle);
	Serial.print(space);
	Serial.println(tempSUConfig.run_sampling);
	
	
	
}




bool pairWithSensor(uint8_t FRAMaddressIndex){
	uint16_t framDataAddr = framDataBufferStartAddr + sizeof(uint64_t) * FRAMaddressIndex;
	
	pairing_mode = true;
	disableDisplayTimeout();
	start_radio();
	
	if(framDataAddr >= framDataBufferStartAddr && framDataAddr + sizeof(uint64_t) < sensorUnitConfigBegin_addr){
		Serial.println(FRAMaddressIndex);
		
		
		addr64 = XBeeAddress64(fram.readUInt32(framDataAddr), fram.readUInt32(framDataAddr + sizeof(uint32_t)) );
		if(debug){
			Serial.println(F("Setting Sensor Unit pairing settings"));
		}
		
		// retrieve sensor unit settings and set the upload time before saving configureation data to FRAM
		
		if(!setSUPairingInfo()){
			if(debug){
				Serial.println(F("Setting Sensor Unit pairing settings Failed!!!"));
			}
			return false;
		}
		
		
		if(debug){
			Serial.println(F("Setting Sensor Unit upload time"));
		}
		
		// update the sensor units upload time that is stored on the base station
		calculateNextUploadTime();	
		setNewUploadTimeAlarm();
		//scan for an existing configuration in FRAM and save the new upload time
		saveSUConfigToFram();
		
		
		
		if(debug){
			Serial.println(F("Requesting Sensor Unit status"));
		}
		
		
		//request_Status();		 // retrieve status info from sensor unit
		if(!request_Status()){
			if(debug){
				Serial.println(F("Status request Failed!!!"));
			}
			return false;
		}
		
		if(debug){
			Serial.println(F("Checking time data"));
		}
		
		if(checkSU_DateTime() == 1){ // check if the sensor units time is close enough to the base stations time, and update it if needed
			if(debug){
				Serial.println(F("Updating Time"));
			}
			updateSU_DateTime();
			for(uint8_t i = 0; i < 3; i++){
				if(waitForPacket(500) == false){
					updateSU_DateTime();
				}
				else{
					if(receivedLoad[0] == ACK){
						break;
					}
				}
				
			}
		}
		
		if(debug){
			Serial.println(F("Requesting Setting data"));
		}
		if(request_settings_check() == false){ // verify the Sensor units settings are correct, update if needed.
			Serial.println(F("Updating Setting data"));
			request_settings_update();
		}
		
		
		// terminate communication with sensor unit to allow it to sleep and take samples.
		for(uint8_t i = 0; i < 3; i++){
			if(endCommunication() == true){
				break;
			}
		}
		
		
			
	}
	
	stop_radio();
	setDisplayTimeout();
	pairing_mode = false;
	return true;
	
}


bool setSUPairingInfo(){
	uint8_t errorCount = 0;
	uint8_t checksum = 0;
	uint8_t cmd_high[] = {'S', 'H'};
	uint8_t cmd_low[] = {'S', 'L'};
	uint8_t pairing_packet_length = 13;
	bool havePacket = false;

	
	payload[0] = PAIR_REQUEST;
	payload[1] = pairing_packet_length;

	payload[2] = network_channel_default;
	
	payload[3] = (network_ID_default >> 8) & 0xff;
	payload[4] = network_ID_default & 0xff;
	
	Serial.println(F("Retrieving radio address..."));
	// send the AT command 'SH' to retrieve the high address of the radio module, and load it into the payload
	sendATCommand(cmd_high, NULL, 0);
	receivedPacket();
	if(receivedLoad[0] == cmd_high[0] && receivedLoad[1] == cmd_high[1] ){
		for (int i = 0; i < receivedLoad[3]; i++) {
			payload[5 + i] = receivedLoad[i + 4] ;
			Serial.print(receivedLoad[i + 4], HEX);
			Serial.print(' ');
		}
	}
		
		
	// send the AT command 'SL' to retrieve the low address of the radio module, and load it into the payload
	sendATCommand(cmd_low, NULL, 0);
	//receivedPacket();
	if(receivedLoad[0] == cmd_low[0] && receivedLoad[1] == cmd_low[1] ){
		for (int i = 0; i < receivedLoad[3]; i++) {
			payload[9 + i] = receivedLoad[i + 4] ;
			Serial.print(receivedLoad[i + 4], HEX);
			Serial.print(' ');
		}
	}
	radioExitCommandMode();
	Serial.println(F("Exited command mode.."));
	
	

	for (uint8_t i = 0; i < pairing_packet_length + 2 ; i++) {
		checksum += payload[i];
		//Serial.print(payload[i],HEX);
	}
	
	payload[pairing_packet_length + 2] = checksum;

	transmitPayload(pairing_packet_length + 3);

	// Acknowledge return check
	
	while (errorCount < 3 && havePacket == false) {
		if (waitForPacket(500) == true) {
			//Serial.println(receivedLoad[0],HEX);
			if ( receivedLoad[0] == ACK) {
				Serial.println(F("Received Pairing ACK"));

				return importSUConfig();
				//return true;

			}
			else{
				transmitPayload(pairing_packet_length + 3);
				errorCount++;
			}
		}
		else {
			transmitPayload(payload[1] + 3);
			errorCount++;
		}
	}
	

	return false;

}




bool importSUConfig(){

	uint16_t addr = sensorUnitConfigBegin_addr;
	uint8_t configSize = max_config_FRAM_byte_length;
	uint8_t num_configs = 	fram.readByte(pairedSensorUnitCountAddr);
	
	
	if(requestSUInfoHigh()){
		tempSUConfig.network_channel         =  receivedLoad[2];
		tempSUConfig.network_ID              = receivedLoad[3];
		tempSUConfig.network_ID              = (tempSUConfig.network_ID << 8) + receivedLoad[4];
		
		tempSUConfig.network_address_high    = receivedLoad[5];
		tempSUConfig.network_address_high    = (tempSUConfig.network_address_high << 8) + receivedLoad[6];
		tempSUConfig.network_address_high    = (tempSUConfig.network_address_high << 8) + receivedLoad[7];
		tempSUConfig.network_address_high    = (tempSUConfig.network_address_high << 8) + receivedLoad[8];
		
		tempSUConfig.network_address_low    = receivedLoad[9];
		tempSUConfig.network_address_low    = (tempSUConfig.network_address_low << 8) + receivedLoad[10];
		tempSUConfig.network_address_low    = (tempSUConfig.network_address_low << 8) + receivedLoad[11];
		tempSUConfig.network_address_low    = (tempSUConfig.network_address_low << 8) + receivedLoad[12];
		
		tempSUConfig.device_id_high          = tempSUConfig.network_address_high;	// for now, they can be the same
		tempSUConfig.device_id_low           = tempSUConfig.network_address_low;
		
		tempSUConfig.SensorType              = receivedLoad[13];
		
	}
	else if(debug){
		Serial.println(F("Failed to receive SUInfoHigh packet!"));
		return false;
	}
	
	
	if(requestSUInfoLow()){
		
		tempSUConfig.buffer_size             = receivedLoad[2];
		tempSUConfig.buffer_size             = (tempSUConfig.buffer_size << 8) + receivedLoad[3];
		
		//tempSUConfig.max_buffer_size             = receivedLoad[4];
		//tempSUConfig.max_buffer_size             = (tempSUConfig.max_buffer_size << 8) + receivedLoad[5];
		
		tempSUConfig.sample_interval_seconds    = receivedLoad[6];
		tempSUConfig.sample_interval_seconds    = (tempSUConfig.sample_interval_seconds << 8) + receivedLoad[7];
		tempSUConfig.sample_interval_seconds    = (tempSUConfig.sample_interval_seconds << 8) + receivedLoad[8];
		tempSUConfig.sample_interval_seconds    = (tempSUConfig.sample_interval_seconds << 8) + receivedLoad[9];
		

		
		//tempSUConfig.next_upload_Hour        = receivedLoad[10];
		//tempSUConfig.next_upload_Minutes     = receivedLoad[11];
		//tempSUConfig.next_upload_Seconds     = receivedLoad[12];
		//tempSUConfig.next_upload_Date        = receivedLoad[13];
		//tempSUConfig.next_upload_Months      = receivedLoad[14];
		//tempSUConfig.next_upload_years       = receivedLoad[15];
		
		tempSUConfig.run_sampling             = 0;
		
	}
	else if(debug){
		Serial.println(F("Failed to receive SUInfoLow packet!"));
		return false;
	}
	


	tempSUConfig.dual_channel_mode       = 0;
	memset(tempSUConfig.channel_gain_index, 0, sizeof(tempSUConfig.channel_gain_index));
	tempSUConfig.autoRange_mode           = 0;
	
	
	
	// update Sensor unit settings to the base stations current settings
	updateSensorUnitSampleTimeToDefault();
	tempSUConfig.buffer_size = uploadInterval_bufferLength;
	
		////scan for an existing configuration in FRAM and save
	//saveSUConfigToFram();
	
	return true;
}
	



/*

*/
bool request_Status() {
	uint8_t errorCount = 0;
	uint32_t timeout;
	//delay(1500);
	//memset(&receivedLoad[0], 0, maxPacketSize);
	
	
	
	
	sendCommand(STATUS_REQUEST);
	while (errorCount < 3) {

		if (waitForPacket(500) == true) {
			//Serial.println(receivedLoad[0],HEX);
			if ( receivedLoad[0] == STATUS_DATA) {
				return true;
			}
			else{
				sendCommand(STATUS_REQUEST);
				errorCount++;
				
			}

		}
		else {
			sendCommand(STATUS_REQUEST);
			errorCount++;
		}

	}

	return false;



}


void printStatus() {
	//uint8_t packet_length = receivedLoad[1];
	//uint8_t index = 0;
	//float tempfloat;
	uint16_t tempUint;
	//int16_t tempInt;

	Serial.print(receivedLoad[0], HEX);
	Serial.print(space);
	Serial.print(receivedLoad[1], HEX);
	Serial.print(space);

	Serial.print(receivedLoad[2], HEX);
	Serial.print(receivedLoad[3], HEX);
	Serial.print(space);
	//memcpy(&tempUint, &receivedLoad[2], sizeof(uint16_t));
	tempUint = receivedLoad[2] ;
	tempUint = (tempUint << 8) + receivedLoad[3];
	Serial.print(tempUint);
	Serial.print(space);
	for (uint8_t index = 3; index < 12; index++) {
		Serial.print(receivedLoad[index]);
		Serial.print(space);
	}
	Serial.println();

}





/*
This method returns true if the time on the Sensor units RTC needs updating.

The sensor units Status must be called before running this method.
*/
int checkSU_DateTime(){
	bool dateTime_needs_updating = false;
	uint8_t tollerance_Seconds = 1;
	DateTime now = rtc.now();
	
	if(receivedLoad[0] != STATUS_DATA ){
		if(debug){
			Serial.println(F("Sensor Unit STATUS_DATA needs to be loaded in first!"));
		}
		return -1; // status data is not in the received load buffer, thus time cannot be compared.
	}


	if (now.hour() != receivedLoad[4]) {
		dateTime_needs_updating = true;
	}
	if (now.minute() != receivedLoad[5]) {
		dateTime_needs_updating = true;
	}
	
	// allow +/- 5 seconds tolerance
	if ( (now.second() >= receivedLoad[6] + tollerance_Seconds) && (now.second() <= receivedLoad[6] - tollerance_Seconds) ) {
		dateTime_needs_updating = true;
		Serial.println(now.second());
		Serial.println(receivedLoad[6]);
	}
	
	if (now.day() != receivedLoad[7]) {
		dateTime_needs_updating = true;
	}
	if (now.month() != receivedLoad[8]) {
		dateTime_needs_updating = true;
	}
	if(now.year() >= 2000){
		
		if ( (now.year() - 2000) != receivedLoad[9]) {
			dateTime_needs_updating = true;
		}
	}
	else{
		if ( now.year() != receivedLoad[9]) {
			dateTime_needs_updating = true;
		}
	}
	
	
	if(dateTime_needs_updating){
		if(debug){
			Serial.println(F("Time on device Needs updating!"));
		}
		return 1;
	}
	else{
		if(debug){
			Serial.println(F("Time on device is correct!"));
		}
		return 0;
	}
	
}




void updateSU_DateTime(){
	uint8_t errorCount = 0;
	uint8_t checksum = 0;
	uint16_t tempUint16 = 0;
	uint8_t dateTime_packet_length = 6;

	DateTime now = rtc.now();


	payload[0] = UPDATE_DATE_TIME_INFO;
	payload[1] = dateTime_packet_length;
	payload[2] = now.hour();
	payload[3] = now.minute();
	payload[4] = now.second();
	payload[5] = now.day();
	payload[6] = now.month();
	if(now.year() >= 2000){
		payload[7] = (now.year() - 2000) & 0xFF; //truncate year to ensure it fits into one byte
	}
	else{
		payload[7] = now.year();
	}


	Serial.println(' ');
	// Calculate payload checksum
	for ( uint8_t i = 0;  i < dateTime_packet_length + 2; i++) {
		checksum += payload[i];
		Serial.print(payload[i], HEX);
		Serial.print(' ');
	}
	Serial.println(' ');
	payload[dateTime_packet_length + 2] = checksum;


	transmitPayload(dateTime_packet_length + 3);

}






bool requestSUInfoHigh(){
	bool havePacket = false;
	uint16_t timeout = 500;
	uint8_t errorCount = 0;
		
	sendCommand(SU_INFO_HIGH_REQUEST);

	// Acknowledge return check
	
	while (errorCount < 3 && havePacket == false) {
		if (waitForPacket(timeout) == true) {
			if ( receivedLoad[0] == SU_INFO_HIGH_DATA) {
				return true;
				
			}
			else{
				sendCommand(SU_INFO_HIGH_REQUEST);
				errorCount++;
			}
		}
		else {
			sendCommand(SU_INFO_HIGH_REQUEST);
			errorCount++;
		}
	}
	return false;
}



bool requestSUInfoLow(){
	bool havePacket = false;
	uint16_t timeout = 500;
	uint8_t errorCount = 0;
	
	sendCommand(SU_INFO_LOW_REQUEST);

	// Acknowledge return check
	
	while (errorCount < 3 && havePacket == false) {
		if (waitForPacket(timeout) == true) {
			if ( receivedLoad[0] == SU_INFO_LOW_DATA) {
				return true;
			}
			else{
				sendCommand(SU_INFO_LOW_REQUEST);
				errorCount++;
			}
		}
		else {
			sendCommand(SU_INFO_LOW_REQUEST);
			errorCount++;
		}
	}
	return false;
}




/*

*/
bool endCommunication(){
	sendCommand(END_COMMUNICATION);
	if(waitForPacket(1200)){
		if(receivedLoad[0] == NACK){
			return false;
		}
		else if(receivedLoad[0] == END_COMMUNICATION_ACK){
			return true;
		}
	}

	return false;
}


void printLoad() {
	uint8_t packet_length = receivedLoad[1];
	//uint8_t index = 0;
	//float tempfloat;
	//uint16_t tempUint;
	//int16_t tempInt;

	Serial.print(receivedLoad[0], HEX);
	Serial.print(space);
	Serial.print(receivedLoad[1], HEX);
	Serial.print(space);
	
	Serial.print(receivedLoad[2]);
	Serial.print(space);
	Serial.print(receivedLoad[3]);
	Serial.print(space);
	for(uint8_t i = 4; i < packet_length; i++){
		Serial.print(receivedLoad[i]);
		Serial.print(' ');
	}

	
	
	//for (uint8_t index = 2; index < 8; index++) {
	//Serial.print(receivedLoad[index]);
	//Serial.print(space);
	//}
	//for (uint8_t index = 8; index < 20; index += 4) {
	//memcpy(&tempfloat, &receivedLoad[index], sizeof(float));
	//Serial.print(tempfloat, 2);
	//Serial.print(space);
	//}
	//
	//for (uint8_t index = 20; index < 28; index += 2) {
	//memcpy(&tempUint, &receivedLoad[index], sizeof(uint16_t));
	//Serial.print(tempUint);
	//Serial.print(space);
	//}
	//for (uint8_t index = 28; index < 32; index += 2) {
	//memcpy(&tempInt, &receivedLoad[index], sizeof(int16_t));
	//Serial.print(tempInt);
	//Serial.print(space);
	//}


	Serial.println();

}





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=



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
	
	for(uint8_t i = 1; i < month; ++i){
		monthInDays += pgm_read_byte(daysInMonth + i - 1);
	}
	if( month > 2 && years % 4 == 0){
		monthInDays += 1;

	}
	
	timeInDays += (years )*365 + monthInDays + (years + 3) / 4 ; // the (years + 3) / 4 - 1 part is exploiting integer divisions which round to zero
	
	timeInSeconds = seconds;
	timeInSeconds += (uint32_t)minute*60;
	timeInSeconds += (uint32_t)hour*3600;
	timeInSeconds += (uint32_t)(day-1)*86400;
	
	timeInSeconds += timeInDays*86400;
	
	return timeInSeconds;
}






void removeSensorUnit(uint16_t sensorUnit_ID) {
	Wire.setClock(I2C_FRAM_CLOCK);
	uint8_t num_sensors_paired = fram.readByte(pairedSensorUnitCountAddr);
	uint16_t toRemove_Addr = sensorUnitConfigBegin_addr + max_config_FRAM_byte_length * sensorUnit_ID;
	uint16_t toShift_Addr = sensorUnitConfigBegin_addr + max_config_FRAM_byte_length * (num_sensors_paired - 1); // - 1 for zero index offset
	uint16_t byte_count = 0;

	uint32_t temp_uint32 = 0;
	uint16_t temp_uint16 = 0;
	uint8_t temp_uint8 = 0;
	
	

	if (debug) {
		Serial.print("Removing config at address ");
		Serial.print(toRemove_Addr);
		Serial.print(" and replacing with config at address ");
		Serial.println(toShift_Addr);
	}

	// remove sensor unit config data by shifting the last config file to the same position the config to be erased is in, IF there are more than one connected Sensor units
	if ( num_sensors_paired > 1) {

		/*
		struct variable sizes do not matter here, so read and writes not matching the
		size of each variable in the config struct do not matter as a fixed block of data is being copied
		*/
		if ( max_config_FRAM_byte_length % 4 == 0) { // if the config size is divisible by 4 bytes, perform 4 byte block read and writes
			if (debug) {
				Serial.println("Performing 4 Byte R/W...");
			}
			for ( int i = 0; i < max_config_FRAM_byte_length / 4; i++) {
				temp_uint32 = fram.readUInt32(toShift_Addr + byte_count);

				fram.writeUInt32(toRemove_Addr + byte_count, temp_uint32);
				byte_count += 4;
			}
		}
		else if ( max_config_FRAM_byte_length % 2 == 0) {
			if (debug) {
				Serial.println("Performing 2 Byte R/W...");
			}
			for ( int i = 0; i < max_config_FRAM_byte_length / 2; i++) {
				temp_uint16 = fram.readUInt16(toShift_Addr + byte_count);
				fram.writeUInt16(toRemove_Addr + byte_count, temp_uint16);
				byte_count += 2;
			}
		}
		else {
			if (debug) {
				Serial.println("Performing 1 Byte R/W...");
			}
			for ( int i = 0; i < max_config_FRAM_byte_length ; i++) {
				temp_uint8 = fram.readByte(toShift_Addr + byte_count);
				fram.writeByte(toRemove_Addr + byte_count, temp_uint8);
				byte_count += 1;
			}
		}

	}


	/*
	if there is only one paired sensor, just subtracting the connected sensor count by one will be sufficient
	*/
	if(num_sensors_paired >= 1){
		num_sensors_paired -= 1;
	}
	
	
	fram.writeByte(pairedSensorUnitCountAddr, num_sensors_paired);

	if (debug) {
		Serial.print("Sensor unit ");
		Serial.print(sensorUnit_ID);
		Serial.println(" removed.");
	}
	
	Wire.setClock(I2C_RTC_CLOCK);

}






uint8_t getSensorTypeText(uint16_t SensorSettingIndex, char * charBuffer) {
	uint16_t addr = sensorUnitConfigBegin_addr + SensorSettingIndex * max_config_FRAM_byte_length + 4 + 4; // add two x 4 to account for the offset

	tempSUConfig.SensorType = fram.readByte(addr);
	return sensorTypeToChar(tempSUConfig.SensorType, charBuffer);
	
}




uint8_t sensorTypeToChar(uint8_t sensorType, char * charBuffer){
	uint8_t stringLength;
	switch (sensorType) {
		case 1:
		charBuffer[0] = 'A';
		charBuffer[1] = 'D';
		charBuffer[2] = 'C';
		charBuffer[3] = ':';
		charBuffer[4] = space;
		charBuffer[5] = '\0';
		stringLength = 5;
		break;

		case 2:
		charBuffer[0] = 'C';
		charBuffer[1] = 'O';
		charBuffer[2] = '2';
		charBuffer[3] = ':';
		charBuffer[4] = space;
		charBuffer[5] = '\0';
		stringLength = 5;
		break;

		case 3:
		charBuffer[0] = 'T';
		charBuffer[1] = 'E';
		charBuffer[2] = 'M';
		charBuffer[3] = 'P';
		charBuffer[4] = '/';
		charBuffer[5] = 'R';
		charBuffer[6] = 'H';
		charBuffer[7] = ':';
		charBuffer[8] = space;
		charBuffer[9] = '\0';
		stringLength = 9;
		break;

		default:
		charBuffer[0] = '-';
		charBuffer[1] = '-';
		charBuffer[2] = '-';
		charBuffer[3] = ':';
		charBuffer[4] = space;
		charBuffer[5] = '\0';
		stringLength = 5;
		break;
	}
	return stringLength;
}


void deviceID_toText( char * charBuffer, uint16_t SensorSettingIndex ) {
	uint16_t addr = sensorUnitConfigBegin_addr + SensorSettingIndex * max_config_FRAM_byte_length + 12;
	//uint8_t int_buffer[4];
	uint8_t temp;
	uint32_t deviceID[2];
	uint8_t int_buffer[8];
	
	
	
	
	Wire.setClock(I2C_FRAM_CLOCK);
	// read in the 8 bytes for the device ID
	
	//fram.readBlock(addr, &deviceID[0], 8);
	
	deviceID[0] = fram.readUInt32(addr);
	deviceID[1] = fram.readUInt32(addr +4);
	memcpy(&int_buffer[0], &deviceID[0], sizeof(uint32_t));	// bit order is reversed, compared to expected order, when copied to byte arrays
	memcpy(&int_buffer[4], &deviceID[1], sizeof(uint32_t));
	
	Wire.setClock(I2C_RTC_CLOCK);

	
	//tempSUConfig.device_id_high = fram.readUInt32(addr);
	//addr += 4;
	//tempSUConfig.device_id_low = fram.readUInt32(addr);
	
	//memcpy(deviceID[0], tempSUConfig.device_id_high, sizeof(uint32_t));
	
	//Serial.print(F("ID: "));
	//Serial.print(tempSUConfig.device_id_high, HEX);
	//Serial.print(F(" "));
	//Serial.println(tempSUConfig.device_id_low, HEX);
	
	
	integerToHexChar(&charBuffer[0], &int_buffer[0], 4, false); // bit order is reversed when copied to byte arrays
	integerToHexChar(&charBuffer[8], &int_buffer[4], 4, false);
	
	//for (index = 0; index < 8; index++) {
		//charBuffer[index] = ((tempSUConfig.device_id_high >> (28 - index * 4)) & 0x0F) + 0x30; // bit shift the device id, mask off everything apart from lowest 4 bits and add 0x30 to get HEX value
		//if (charBuffer[index] > 0x39) {
			//charBuffer[index] += 0x07;
		//}
//
	//}


	
	//for (index = 8; index < 16; index++) {
		//charBuffer[index] = ((tempSUConfig.device_id_low >> (28 - (index - 8) * 4)) & 0x0F) + 0x30; // bit shift the device id, mask off everything apart from lowest 4 bits and add 0x30 to get HEX value
		//if (charBuffer[index] > 0x39) {
			//charBuffer[index] += 0x07;
		//}
//
	//}
	

	charBuffer[16] = '\0'; // add null char to end off array

}

void integerToHexChar(char * charBuffer, uint8_t *value, uint8_t bytelength){
	
	integerToHexChar(charBuffer, value, bytelength,  true);
}



void integerToHexChar(char * charBuffer, uint8_t *value, uint8_t bytelength, bool direction){
	uint16_t count = 0;
	if(direction == true){
		for (uint8_t i = 0; i < bytelength; i++) {
			charBuffer[count] = ( ( value[i] >> 4)  & 0x0F) + 0x30; // bit shift the byte to get the top 4 bits, and add 0x30 to get the high HEX value
		
			if (charBuffer[count] > 0x39) {
				charBuffer[count] += 0x07;
			}
			count++;
		
			charBuffer[count] = (  value[i]  & 0x0F) + 0x30; // bit and byte to get the bottom 4 bits, and add 0x30 to get the low HEX value
		
			if (charBuffer[count] > 0x39) {
				charBuffer[count] += 0x07;
			}
			count++;
			
		}
		charBuffer[count] = '\0';
	}
	else{
		count = 0;
		for (int16_t i = bytelength -1; i >= 0; i--) {
			
			charBuffer[count] = ( ( value[i] >> 4)  & 0x0F) + 0x30; // bit shift the byte to get the top 4 bits, and add 0x30 to get the high HEX value
			
			if (charBuffer[count] > 0x39) {
				charBuffer[count] += 0x07;
			}
			
			count++;
			
			charBuffer[count] = (  value[i]  & 0x0F) + 0x30; // bit and byte to get the bottom 4 bits, and add 0x30 to get the low HEX value
			
			if (charBuffer[count] > 0x39) {
				charBuffer[count] += 0x07;
			}
			
			count++;
			
		}
		
	}
	
}



/*
Button Control methods - Warning! long amounts of code! ----------------------------------------
*/






uint8_t scanButtons() {
	//uint32_t startTime = millis();
	//uint8_t gpio;
	//bool buttonPressed = false;
	//uint8_t interruptReg = 0;

	uint8_t button_pressed = B00000000; //lower 4 bits are used to define the buttons that are pressed.  bit 3 is Menu, bit 2 is Back, bit 1 is up, bit 0 is Down


	//beginI2cDevices();


	if (debug) {
		Serial.println(F("Checking Valid Button Presses"));
	}


	if ( menu_button_flag || back_button_flag || up_button_flag || down_button_flag) {

		if (  menu_button_flag || back_button_flag ) { // add button delay for menu and back buttons, to help prevent rapid menu traversing
			if ( (uint16_t(lastButtonPress + 200) > millis())  ) {
				if (debug) {
					Serial.println(F("Button press too quick!"));
				}
				return 0x00;
			}
		}
		//    if (  menu_button_flag || back_button_flag ) { // add button delay for menu and back buttons, to help prevent rapid menu traversing
		//      if ( (uint16_t(lastButtonPress + 25) > millis())  ) {
		//        if (debug) {
		//          Serial.println("Button press too quick!");
		//        }
		//        return 0x00;
		//      }
		//    }


		//Serial.println(interruptReg, BIN);
		//if (debug) {
			//
			//Serial.print(F("Display State"));
			//Serial.println(displayOn);
		//}
		
		

		if ( !displayOn) {
			if (debug) {
				Serial.println(F("Starting Display"));
			}
			startDisplay();
			delay(100); // wait for displays boost converter to power up
			//logo();
			inMenuNum = 0;

			//ePaper.refresh();
			displayOn = true;
			
			menu_button_flag = false;
			back_button_flag = false;
			up_button_flag = false;
			down_button_flag = false;
			return 0x00;
		}

		//ePaper.begin();

		//if (debug) {
		//Serial.println("Running button Pressed methods");
		//}

		if ( menu_button_flag ) {
			if (debug) {
				Serial.println(F("Menu button"));
			}
			button_pressed |= B00001000;
			displayOn = true;
			menu_button_flag = false;
			redrawMenus(button_pressed);
		}
		else if (back_button_flag) {
			if (debug) {
				Serial.println(F("back button"));
			}
			displayOn = true;
			button_pressed |= B00000100;
			back_button_flag = false;
			redrawMenus(button_pressed);
		}

		else if (up_button_flag ) {
			if (debug) {
				Serial.println(F("up button"));
			}
			displayOn = true;
			up_button_flag = false;
			button_pressed |= B00000010;
			redrawMenus(button_pressed);

		}

		else if (down_button_flag) {
			if (debug) {
				Serial.println(F("down button"));
			}
			displayOn = true;
			down_button_flag = false;
			button_pressed |= B00000001;
			redrawMenus(button_pressed);
		}



		lastButtonPress = millis();

		//buttonPressedContinuous = true;
		if (debug) {
			Serial.println("button Pressed");
		}

		menu_button_flag = false;
		back_button_flag = false;
		up_button_flag = false;
		down_button_flag = false;


		//}

		//  if (debug) {
		//
		//    Serial.print("menu Level : ");
		//    Serial.println(inMenuNum);
		//    Serial.print("prev menu : ");
		//    Serial.println(prevMenuNum);
		//    Serial.print("cursor Pos : ");
		//    Serial.println(cursorPos);
		//    Serial.print("max cursor : ");
		//    Serial.println(maxCursor);
		//    Serial.print("Sampling : ");
		//    Serial.println(runSampling);
		//  }

		//  while (digitalRead(wakeButton) == 0) {
		//    gpioExpander.readGPIO();
		//  }

	}
	return button_pressed;
}


//------------------------
void updateCursor(uint8_t button_mask) {

	if ( button_mask == B00000010) { // up button pressed
		if (cursorPos > 1) {
			cursorPos = cursorPos - 1;
		}
		else if ( cursorPos == 1 ) {
			cursorPos = maxCursor;
		}
	}
	else if ( button_mask == B00000001) { // Down button pressed
		if (cursorPos < maxCursor) {
			cursorPos = cursorPos + 1;
		}
		else if ( cursorPos == maxCursor ) {
			cursorPos = 1;
		}
	}

}

//-------------------------------------------------------------------------------

void startDisplay(){
	pinMode(display_PWR_EN, OUTPUT);
	digitalWrite(display_PWR_EN, HIGH); // Display Enable pin
	delay(100);
	
	Serial.println(F("Starting SPI1 port"));
	if (debug) {
		Serial.println(F("Starting SPI1 port"));
	}
	SPI1.begin();
	digitalWrite(displayCS, HIGH);
	
	if (debug) {
		Serial.println(F("Starting Display"));
	}
	ePaper.begin();
	if (debug) {
		Serial.println(F("Clearing Display"));
	}
	
	ePaper.clearDisplay();
	
	if (debug) {
		Serial.println(F("Drawing logo"));
	}
	logo2();
	if (debug) {
		Serial.println(F("Refreshing Display"));
	}
	ePaper.refresh();

	
	setDisplayRefreshPWM();
	 displayOn = true;
}


void stopDisplay(){
	ePaper.clearDisplay();
	ePaper.refresh();
	digitalWrite(displayCS, LOW);
	SPI1.end();
	disableDisplayTimeout();
	disableDisplayRefreshPWM();
	
	digitalWrite(displayRefresh, LOW);
	digitalWrite(display_PWR_EN, LOW);
	digitalWrite(display_MOSI, LOW);
	displayOn = false;
	turnOffDisplay = false;
}

//-------------------------------------------------------------------------------

void redrawMenus(uint8_t button_mask) {

	disableDisplayRefreshPWM();
	switch (inMenuNum) {

		case 0:
		if (button_mask == B00001000) {
			cursorPos = 1;
			draw_Base_Menu(uint8_t(0));
		}
		break;

		// ---- Base menus ------
		case 1:
		draw_Base_Menu(button_mask);
		break;


		// ---- display graph data menus menus ------


		// ---- sampling settings menus ------

		case 20:
		draw_Start_Menu(button_mask);
		break;

		case 21:
		draw_Stop_Menu(button_mask);
		break;

		case 30:
		draw_SampleSettings_Menu(button_mask);
		break;

		case 31:
		draw_ChangeDefaultSampleRate_Menu(button_mask);
		break;

		case 32:
		draw_ChangeUploadRate_Menu(button_mask);
		break;



		// ---- Paring menus ---------------
		case 40:
		draw_Pair_Menu(button_mask);
		break;

		case 41:
		draw_AddSensor_Menu(button_mask);
		break;

		case 42:
		draw_PairedSensors_Menu(button_mask);
		break;

		case 43:
		draw_PairedSensor_Sub_Menu(button_mask);
		break;
		
		case 44:
		draw_scannedSensors_Menu(button_mask);
		break;
		
		

		case 50:
		draw_PairedSensorLabels_Menu(button_mask);
		break;

		case 51:
		draw_PairedSensorSettings_Menu(button_mask);
		break;

		case 52:
		draw_RemoveSensor_Menu(button_mask);
		break;


		// ---- General Settings menus ------

		case 100:
		draw_GenSettings_Menu(button_mask);
		break;

		case 110:
		draw_TimeSetting_Menu(button_mask);
		break;

		case 111:
		draw_TimeSubSettingDate_Menu(button_mask);
		break;

		case 112:
		draw_TimeSubSettingTime_Menu(button_mask);
		break;

		case 120:
		draw_FactoryReset_Menu(button_mask);
		break;

		case 130:
		draw_DataSettings_Menu(button_mask);
		break;

		case 131:
		draw_ClearSDCard_Menu(button_mask);
		break;

		case 132:
		draw_FormatSDCard_Menu(button_mask);
		break;

		// error history menu
		case 140:
		draw_ErrorHistory_Menu(button_mask);
		break;

		case 141:
		draw_RadioBypass_Menu(button_mask, false);
		break;

		case 142:
		draw_RequestSample_Menu(button_mask, false);
		break;

		default:
		logo();
		inMenuNum = 0;
		break;

	}
	if (inMenuNum != 0) {
		updateStatusBar();
	}
	if (debug) {
		Serial.println(inMenuNum);
		Serial.println(button_mask);
	}
	ePaper.refresh();
	setDisplayRefreshPWM();
}




/*
Display Control methods - Warning! long amounts of code! =============================================
*/




//=========================================================

void logo() {

	ePaper.clearDisplay();

	ePaper.fillTriangle(135, 135, 135, 35, 265, 35, BLACK);
	ePaper.drawLine(265, 35, 265, 140, BLACK);
	ePaper.drawLine(135, 35, 265, 140, BLACK);


	ePaper.setCursor(142, 145);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.setTextSize(4);
	ePaper.print("BRANZ");
	ePaper.setCursor(147, 195);
	ePaper.setTextSize(2);
	ePaper.print("B.A.C.O.N");
	ePaper.setCursor(180, 215);
	ePaper.setTextSize(1);
	ePaper.print("ver ");
	ePaper.print(DeviceVersionNumber);


	updateStatusBar();
	ePaper.fillRect(35, 17, 37, 240, WHITE); // side bar line

	//  if (!batteryGood()) { //!batteryGood()
	//    ePaper.setCursor(165, 5);
	//    ePaper.setTextColor( WHITE, BLACK);
	//    ePaper.fillRect(162, 2, 78, 13, BLACK);
	//    ePaper.print(F("LOW BATTERY!"));
	//
	//  }


	//ePaper.refresh();
}

//=========================================================

void logo2() {

	//ePaper.clearDisplay();

	ePaper.fillTriangle(135, 135, 135, 35, 265, 35, BLACK);
	ePaper.drawLine(265, 35, 265, 140, BLACK);
	ePaper.drawLine(135, 35, 265, 140, BLACK);


	ePaper.setCursor(142, 145);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.setTextSize(4);
	ePaper.print("BRANZ");
	ePaper.setCursor(147, 195);
	ePaper.setTextSize(2);
	ePaper.print("B.A.C.O.N");
	ePaper.setCursor(180, 215);
	ePaper.setTextSize(1);
	ePaper.print("ver ");
	ePaper.print(DeviceVersionNumber);

	updateStatusBar();
	ePaper.fillRect(35, 17, 37, 240, WHITE); // side bar line


	//  if (!batteryGood()) { //!batteryGood()
	//    ePaper.setCursor(165, 5);
	//    ePaper.setTextColor( WHITE, BLACK);
	//    ePaper.fillRect(162, 2, 78, 13, BLACK);
	//    ePaper.print(F("LOW BATTERY!"));
	//
	//  }

	ePaper.refresh();
}

//=========================================================



void updateStatusBar() {
	uint16_t temp;
	ePaper.setTextSize(1);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.setCursor(10, 5);
	if (runSampling) {
		ePaper.print(F("Sampling Enabled"));
	}
	ePaper.drawLine(0, 16, 400, 16, BLACK); // top bar line
	ePaper.drawLine(36, 17, 36, 240, BLACK); // side bar line

	//DS3234.begin(DS3234_CS_PIN, SPI_DIVISOR_GLOBAL);
	DateTime now = rtc.now();

	ePaper.setCursor(295, 5);

	//print time with zero padding
	temp = now.hour();

	if (temp < 10) {
		ePaper.print(zero);
	}
	ePaper.print(temp);

	ePaper.print(':');
	temp = now.minute();
	if (temp < 10) {
		ePaper.print(zero);
	}
	ePaper.print(temp);

	//print date with zero padding
	ePaper.print(space);
	ePaper.print(space);

	temp = now.day();
	if (temp < 10) {
		ePaper.print(zero);
	}
	ePaper.print(temp);
	ePaper.print('/');
	temp = now.month();
	if (temp < 10) {
		ePaper.print(zero);
	}
	ePaper.print(temp);
	ePaper.print('/');
	//ePaper.print(20);
	temp = now.year();
	if(temp >= 2000){
		temp -= 2000;
	}

	if (temp < 10) {
		ePaper.print(zero);
	}
	ePaper.print(temp);


	if (!batteryGood()) { //!batteryGood()
		ePaper.setCursor(205, 5);
		ePaper.setTextColor( WHITE, BLACK);
		ePaper.fillRect(202, 2, 78, 13, BLACK);
		ePaper.print(F("LOW BATTERY!"));
		ePaper.setTextColor( BLACK, WHITE);

	}

}


//=========================================================

void drawMenuBox( const char * charList, uint8_t posVertIndex, int16_t boxWidths, int16_t posLeftAdjust, int16_t posYAdjust, bool invertColor, int16_t charXoffset = 0, bool clearBox = true) {
	uint8_t offsetY = uint8_t(70 + posYAdjust);
	uint8_t offsetX = uint8_t( 37 + posLeftAdjust );
	uint8_t yIncrement = 25;


	uint8_t charOffsetY = 5;
	int16_t charOffsetX = 15 + charXoffset;
	ePaper.setTextSize(2);


	if (clearBox) {
		if ( invertColor) {
			ePaper.fillRect(offsetX, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, BLACK);
			ePaper.setCursor(offsetX + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
			ePaper.setTextColor(WHITE, BLACK);

			ePaper.print(charList);
		}
		else {
			ePaper.fillRect(offsetX, offsetY + (yIncrement ) * posVertIndex - 1 , boxWidths, yIncrement, WHITE);
			ePaper.drawRect(offsetX, offsetY + (yIncrement - 1) * posVertIndex - 1, boxWidths, yIncrement, BLACK);
			ePaper.setCursor(offsetX + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
			ePaper.setTextColor(BLACK, WHITE);
			ePaper.print(charList);
		}
	}
	else {
		ePaper.fillRect(offsetX, offsetY + (yIncrement ) * posVertIndex - 1 , boxWidths, yIncrement, WHITE);
	}

}

//=========================================================

void drawList( const char ** charList, uint8_t num_list, uint8_t cursorPos, uint16_t textBoxWidth, int16_t xAdjust, int16_t yAdjust = 0, int16_t charAdjustX = 0) {

	for (int i = 0; i < num_list; i++) {
		if ( i == (cursorPos - 1)) {
			drawMenuBox(charList[i], i, textBoxWidth, xAdjust, yAdjust, true, charAdjustX);
		}
		else {
			drawMenuBox(charList[i], i, textBoxWidth, xAdjust, yAdjust, false, charAdjustX);
		}
	}

}


//=========================================================

void drawSettingChangeBox(const char * charList , uint8_t posHorizIndex, uint8_t posVertIndex,
int16_t boxWidths, int16_t posLeftAdjust, bool invertColor) {

	uint8_t offsetX = uint8_t( 41 + posLeftAdjust );
	uint8_t offsetY = 80;
	uint8_t yIncrement = 25;
	uint8_t xIncrement = 5;

	uint8_t charOffsetY = 5;
	uint8_t charOffsetX = 15;
	ePaper.setTextSize(2);


	if ( invertColor) {
		ePaper.fillRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, BLACK);
		ePaper.setCursor(offsetX + (xIncrement - 1)* posHorizIndex + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
		ePaper.setTextColor(WHITE, BLACK);

		ePaper.print(charList);
	}
	else {
		ePaper.fillRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex , boxWidths, yIncrement, WHITE);
		//ePaper.fillRect(offsetX, offsetY + (yIncrement ) * posVertIndex - 1 , boxWidths, yIncrement, WHITE);
		ePaper.drawRect(offsetX + (xIncrement - 1)* posHorizIndex, offsetY + (yIncrement - 1) * posVertIndex - 1, boxWidths, yIncrement, BLACK);
		ePaper.setCursor(offsetX + (xIncrement - 1)* posHorizIndex + charOffsetX, offsetY + charOffsetY + (yIncrement - 1) * (posVertIndex ) );
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(charList);
	}


}


//=========================================================

void draw_Base_Menu(uint8_t button_mask) { //, bool refresh
	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;

	maxCursor = 4;


	const char menuText_1[] = "Start Sampling";
	const char menuText_2[] = "Stop Sampling";

	const char menuText_3[] = "Sampling Settings";
	const char menuText_4[] = "Pair Sensors";
	const char menuText_5[] = "General Settings";

	prevMenuNum = 1;
	//if (inMenuNum != 1) {
	inMenuNum = 1;    // update current menu value

	//}

	//else { // functions to perform depending on button pressed in this menu

	switch (button_mask) {

		case B00001000: // menu button pressed
		if ( cursorPos == 1) {
			cursorPos = 2;

			if (!runSampling) {
				draw_Start_Menu(uint8_t(0));
				//runSampling = true;
				//          clearRTCAlarm();
				//          DS3234.update();
				//          nextAlarmTime_mainDevice_minutes = DS3234.minute() + 1;
				//          nextAlarmTime_mainDevice_hours = DS3234.hour();
				//          DS3234.enableAlarmInterrupt(true, false);
				//          incrementAlarmTime();
			}
			else {
				draw_Stop_Menu(uint8_t(0));
			}



			return;
		}
		else if (cursorPos == 2) { // back button pressed
			cursorPos = 1;
			//buttonPressedContinuous = false;
			//loopButtonPress = false;

			draw_SampleSettings_Menu(uint8_t(0));
			return;
		}
		else if ( cursorPos == 3) { // Up button pressed
			cursorPos = 1;

			draw_Pair_Menu(uint8_t(0));
			return;

		}
		else if ( cursorPos == 4) { // down button pressed
			cursorPos = 1;

			draw_GenSettings_Menu(uint8_t(0));
			return;

		}
		break;

		case B00000100: // back button pressed

		inMenuNum = 0;
		cursorPos = 1;
		maxCursor = 0;
		logo();
		return;
		break;

		case B00000010: // up button pressed
		refresh = false;
		updateCursor(button_mask);
		break;

		case B00000001: // down button pressed
		refresh = false;
		updateCursor(button_mask);
		break;

		default:
		refresh = true;
		break;
	}
	//}


	// setup display if this menu is not currently being displayed (eg, menu or back button pressed)
	if ( refresh == true) {
		ePaper.clearDisplay();
		//ePaper.fillRect(1, 1, 400, 240, WHITE);

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Menu"));
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	if (!runSampling) {
		textList[0] = &menuText_1[0];
	}
	else {
		textList[0] = &menuText_2[0];
	}
	textList[1] = &menuText_3[0];
	textList[2] = &menuText_4[0];
	textList[3] = &menuText_5[0];

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);


}


//=========================================================

void draw_Start_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 0;
	int16_t charPadding = 120;
	bool refresh = true;

	maxCursor = 2;

	const char yes[] = "Yes";
	const char no[] = "No";


	if (inMenuNum != 20) {
		inMenuNum = 20;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if ( cursorPos == 1) {
				runSampling = true;
				
				waitingSamplingEnd = false;
				waitingSamplingStart = true;
				
				fram.writeByte(SamplingEnabledFramAddr, runSampling);
				//          DS3234.enableAlarmInterrupt(false, false);  // enables interrupt from square wave pin on RTC to trigger on alarm event
				//          DS3234.alarm1(true);
				//          DS3234.alarm2(true);
				//          if ( checkCO2ConnectionType() != 0) { // if CO2 Sensor is present
				//            disableCO2();
				//          }
				//          //clearRTCAlarm();
				
				
				cursorPos = 1;

				draw_Base_Menu(uint8_t(0));
				return;
			}
			else if ( cursorPos == 2) {
				runSampling = false;
				cursorPos = 1;

				draw_Base_Menu(uint8_t(0));
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_Base_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;

		}
	}

	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &yes[0];
	textList[1] = &no[0];



	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Start Sampling"));
		ePaper.print('?');
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust , charPadding);


}

void draw_Stop_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 0;
	int16_t charPadding = 120;
	bool refresh = true;

	maxCursor = 2;

	const char yes[] = "Yes";
	const char no[] = "No";


	if (inMenuNum != 21) {
		inMenuNum = 21;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if ( cursorPos == 1) {
				runSampling = false;
				waitingSamplingEnd = true;
				waitingSamplingStart = false;

				fram.writeByte(SamplingEnabledFramAddr, runSampling);
				//          DS3234.enableAlarmInterrupt(false, false);  // enables interrupt from square wave pin on RTC to trigger on alarm event
				//          DS3234.alarm1(true);
				//          DS3234.alarm2(true);
				//          if ( checkCO2ConnectionType() != 0) { // if CO2 Sensor is present
				//            disableCO2();
				//          }
				//          //clearRTCAlarm();
				cursorPos = 1;

				draw_Base_Menu(uint8_t(0));
				return;
			}
			else if ( cursorPos == 2) {
				runSampling = true;
				cursorPos = 1;
				draw_Base_Menu(uint8_t(0));
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 1;
			draw_Base_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;

		}
	}

	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &yes[0];
	textList[1] = &no[0];

	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Stop Sampling"));
		ePaper.print('?');
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, charPadding);


}

//====================================================================================================================

//=========================================================

void draw_SampleSettings_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;

	maxCursor = 2;

	const char menuText_1[] = "Sampling Interval";
	const char menuText_2[] = "Upload Interval";


	prevMenuNum = 30;

	if (inMenuNum != 30) {
		inMenuNum = 30;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed

			if ( cursorPos == 1) {
				cursorPos = 1;
				// load in current values to temp variables
				temp1 = defaultSampleRateHour;
				temp2 = defaultSampleRateMin;

				draw_ChangeDefaultSampleRate_Menu(uint8_t(0));
				return;
			}
			else if ( cursorPos == 2) {
				cursorPos = 1;

				temp1 = upload_time_interval_sampling / 60;
				
				draw_ChangeUploadRate_Menu(uint8_t(0));
				return;
			}

			break;

			case B00000100: // back button pressed
			cursorPos = 2;
			draw_Base_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;

		}
	}




	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];


	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Sampling Settings"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);


}

//====================================================================================================================



void draw_ChangeDefaultSampleRate_Menu(uint8_t button_mask) {

	int16_t boxwidth = 60;
	int16_t leftAdjust = 92;
	bool refresh = true;

	char numBuffer[5];
	memset(numBuffer, '\0', sizeof(numBuffer));
	maxCursor = 2;

	//if (inMenuNum != 31) {
	inMenuNum = 31;    // update current menu value


	//else { // functions to perform depending on button pressed in this menu

	switch (button_mask) {

		case B00001000: // menu button pressed
		if ( cursorPos == 1) {
			cursorPos = 2;
			refresh = false;
		}
		else if ( cursorPos == 2) {
			cursorPos = 1;
			// save variables entered
			defaultSampleRateHour = temp1;
			defaultSampleRateMin = temp2;
			fram.writeByte(samplePeriod_Addr, defaultSampleRateHour);
			fram.writeByte(samplePeriod_Addr + 1 , defaultSampleRateMin);
			temp1 = 0;
			temp2 = 0;
			draw_SampleSettings_Menu(uint8_t(0)); // return to menu
			return;
		}
		break;

		case B00000100: // back button pressed
		if ( cursorPos == 1) {
			cursorPos = 1;
			draw_SampleSettings_Menu(uint8_t(0));
			return;
		}
		else {
			cursorPos = 1;
			refresh = false;
		}
		break;

		case B00000010:               // up button pressed
		if (cursorPos == 1) {       //sample rate hour
			if (temp1 < 24) {
				temp1 = temp1 + 1;
			}
			else if (temp2 == 0 && temp1 == 24) {
				temp1 = 0;
				temp2 = 1;
			}
			else {
				temp1 = 0;
			}
		}
		else if (cursorPos == 2) {  //sample rate minute
			if (temp2 < 59) {
				temp2 = temp2 + 1;
			}
			else if (temp1 > 0) {
				temp2 = 0;
			}
			else {
				temp2 = 1;
			}
		}
		refresh = false;

		break;

		case B00000001: // down button pressed
		if (cursorPos == 1) {
			if (temp1 > 0 && temp2 > 1) { //sample rate hour
				temp1 = temp1 - 1;
			}
			else if (temp1 > 1 && temp2 >= 0) {
				temp1 = temp1 - 1;
			}
			else if ( temp2 == 0 && temp1 == 1) {
				temp1 = 0;
				temp2 = 1;
			}
			else {
				temp1 = 24;
			}
		}

		else if (cursorPos == 2) {
			if (temp2 > 1 && temp1 != 0) {  //sample rate min
				temp2 = temp2 - 1;
			}
			else if (temp1 == 0 && temp2 > 1) {
				temp2 = temp2 - 1;
			}
			else if (temp1 > 0 && temp2 == 1) {
				temp2 = 0;
			}
			else if (temp1 > 0 && temp2 == 0) {
				temp2 = 59;
			}
			else if (temp2 == 1) {
				temp2 = 59;
			}
		}
		refresh = false;
		break;

		default:
		refresh = true;

		break;


	}


	if ( refresh ) {
		Serial.print("refresh: ");
		Serial.println(refresh);
		ePaper.clearDisplay();

		ePaper.setCursor(55, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);

		ePaper.print(F("Default Sample Rate"));


		ePaper.setTextSize(2);
		ePaper.setCursor(140, 85 );
		ePaper.print(F("Hour"));
		ePaper.setCursor(215, 85 );
		ePaper.print(F("Min"));

		ePaper.setTextSize(1);
		ePaper.setCursor(120, 150 );
		ePaper.print(F("Sets the default sample rate for"));
		ePaper.setCursor(120, 160 );
		ePaper.print(F("sensor units and samples will be"));
		ePaper.setCursor(120, 170 );
		ePaper.print(F("taken after this amount of time has passed"));

	}


	// draw menu boxes on display with box highlighted if cursor index matches
	if ( cursorPos == 1) {  // hours box
		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust , true);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust + boxwidth + 11, false);
	}
	else if ( cursorPos == 2) { // minutes box
		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust , false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust + boxwidth + 11, true);
	}

}

//====================================================================================================================

void draw_ChangeUploadRate_Menu(uint8_t button_mask) {

	int16_t boxwidth = 80;
	int16_t leftAdjust = 110;
	bool refresh = true;

	char numBuffer[5];
	memset(numBuffer, '\0', sizeof(numBuffer));

	maxCursor = 1;

	delay(150);


	if (inMenuNum != 32) {
		inMenuNum = 32;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed

			upload_time_interval_sampling = temp1 * 60;
			fram.writeUInt16(upload_time_interval_sampling_addr, upload_time_interval_sampling);
			
			temp1 = 0;
			cursorPos = 2;
			draw_SampleSettings_Menu(0);
			return;
			//break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_SampleSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;

			temp1 += samplingUploadTimeIncrement;

			if (temp1 > maxSamplingUploadTime) {
				temp1 = minimumSamplingUploadTime;
			}

			break;

			case B00000001: // down button pressed
			refresh = false;
			temp1 -= samplingUploadTimeIncrement;

			if (temp1 < minimumSamplingUploadTime) {
				temp1 = maxSamplingUploadTime;
			}

			break;

			default:
			refresh = true;

			break;
		}
	}


	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(2);
		ePaper.setTextColor(BLACK, WHITE);

		ePaper.print(F("Upload Interval"));

		ePaper.setTextSize(2);
		ePaper.setCursor(120, 85 );
		ePaper.print(F("Minutes"));

		ePaper.setTextSize(1);
		ePaper.setCursor(80, 150 );
		ePaper.print(F("Data will be downloaded from Sensor"));
		ePaper.setCursor(80, 160 );
		ePaper.print(F("units after this amount of time has"));
		ePaper.setCursor(80, 170 );
		ePaper.print(F("passed."));

	}


	// draw menu boxes on display with box highlighted if cursor index matches
	//drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

	// draw menu boxes on display with box highlighted if cursor index matches
	if ( cursorPos == 1) {  // hours box
		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust , true);
		//memset(numBuffer, '\0', sizeof(numBuffer));
		//drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust + boxwidth + 11, false);
	}
}

//=================== Pairing menus =================================================================================================
void draw_Pair_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;


	maxCursor = 2;

	const char menuText_1[] = "Add Sensor";
	const char menuText_2[] = "Paired Sensors";


	if (inMenuNum != 40) {
		inMenuNum = 40;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos == 1) {
				draw_AddSensor_Menu(uint8_t(0));
				return;
			}
			else if (cursorPos == 2) {
				cursorPos = 1;
				draw_PairedSensors_Menu(uint8_t(0));
				return;
			}

			break;

			case B00000100: // back button pressed
			cursorPos = 3;
			draw_Base_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}

	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Pair Sensor Menu"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

}

//------------------------------------------------------------

void draw_AddSensor_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;
	

	maxCursor = 2;

	const char menuText_1[] = "Scan";
	const char menuText_2[] = "Cancel";


	if (inMenuNum != 41) {
		inMenuNum = 41;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
				if(cursorPos == 1){
					drawScanninginProgress();
					temp1 = scanForSensorUnits(&text_buffer[0], sizeof(text_buffer));
					draw_scannedSensors_Menu(0);
					return;
				}
				else if( cursorPos == 2){
					cursorPos = 1;
					draw_Pair_Menu(uint8_t(0));
					return;
				}
				
			break;

			case B00000100: // back button pressed
				cursorPos = 1;
				draw_Pair_Menu(uint8_t(0));
				return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];
	
	


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Pair Sensor"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);




}





//------------------------------------------------------------

void draw_PairedSensors_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 130;
	bool refresh = true;
	char menuText_buffer[32];
	char serial_text_buffer[17];
	uint8_t textIndex = 0;

	const char menuText_1[] = "Back";

	uint8_t maxItemsPerPage = 6;
	uint8_t pageIndex = 0; // 0 is first page
	uint8_t startIndex = 0;
	uint8_t stopIndex = 0;
	//uint16_t startTime = 0;
	//uint16_t endTime = 0;
	uint8_t num_sensors_paired = fram.readByte(pairedSensorUnitCountAddr);
	maxCursor = num_sensors_paired;




	if (inMenuNum != 42) {
		inMenuNum = 42;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if ( num_sensors_paired > 0) {
				selected_sensorUnit_Index = cursorPos - 1; // store current cursor pos as the selected sensor unit ID/index in FRAM
				cursorPos = 1;
				draw_PairedSensor_Sub_Menu(uint8_t(0));
				return;
			}
			else {
				cursorPos = 2;
				draw_Pair_Menu(uint8_t(0));
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 2;
			draw_Pair_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}


	if (cursorPos > 0) {
		pageIndex = (cursorPos - 1) / maxItemsPerPage;
	}
	else {
		pageIndex = (cursorPos) / maxItemsPerPage;
	}

	startIndex = pageIndex * maxItemsPerPage;
	stopIndex = startIndex + maxItemsPerPage;

	//  if(debug){
	//    Serial.print("pageIndex: ");
	//    Serial.println(pageIndex);
	//    Serial.print("startIndex: ");
	//    Serial.println(startIndex);
	//    Serial.print("cursorPos: ");
	//    Serial.println(cursorPos);
	//  }


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Paired Sensors"));
	}


	if ( num_sensors_paired > 0) {
		for (int i = startIndex; i < stopIndex; i++) {
			if ( i < num_sensors_paired) {

				//retrieve human readable label of sensor units
				//getSensorNameList(i);
				
				
				textIndex = getSensorTypeText(i, &text_buffer[0]); // get human readable sensor unit type
				
				deviceID_toText(&text_buffer[textIndex], i); // get human readable sensor unit ID/serial in HEX
				//textIndex += 16;
				//Serial.println(serial_text_buffer);
				//concatCharArray(&text_buffer[0], 32, &menuText_buffer[0] , &serial_text_buffer[0]);
				textList[0] = &text_buffer[0];

				if ( i == (cursorPos - 1)) {
					drawMenuBox(textList[0], i - startIndex, boxwidth, leftAdjust, 0, true);
				}
				else {
					drawMenuBox(textList[0], i - startIndex, boxwidth, leftAdjust, 0, false);
				}
			}
			else {
				drawMenuBox(textList[0], i - startIndex, boxwidth, leftAdjust, 0, false, 0, false);
			}
		}
	}
	else {
		ePaper.setCursor(60, 80 );
		ePaper.setTextSize(2);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("No Sensor Units Paired"));


		textList[0] = &menuText_1[0];
		drawList(textList, 1, 1, boxwidth, leftAdjust, yAdjust);
	}


	//  endTime = millis();
	//  Serial.print("render time: ");
	//  Serial.print(endTime - startTime);
	//  Serial.println(" ms");
	//  Serial.print("render time per item: ");
	//  Serial.print((endTime - startTime) / (stopIndex - startIndex) );
	//  Serial.println(" ms");


}

//------------------------------------------------------------

void draw_PairedSensor_Sub_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;
	//char menuText_buffer[32];

	const char menuText_1[] = "-----";
	const char menuText_2[] = "Sensor Unit Settings";
	const char menuText_3[] = "Un-Pair Sensor Unit";



	maxCursor = 1;

	if (inMenuNum != 43) {
		inMenuNum = 43;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			//if (cursorPos == 1) {
				//draw_PairedSensorLabels_Menu(uint8_t(0));
				//return;
			//}
			//else if ( cursorPos == 2) {
				//cursorPos = 1;
				//draw_PairedSensorSettings_Menu(uint8_t(0));
				//return;
			//}
			//else 
			if ( cursorPos == 1) {
				cursorPos = 2;
				draw_RemoveSensor_Menu(uint8_t(0));
				return;
			}
			return;
			break;

			case B00000100: // back button pressed
			cursorPos = selected_sensorUnit_Index + 1;
			draw_PairedSensors_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}

	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_3[0];
	//textList[0] = &menuText_1[0];
	//textList[1] = &menuText_2[0];
	//textList[2] = &menuText_3[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Paired Sensors"));
	}




	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);



}

//------------------------------------------------------------

void draw_scannedSensors_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 130;
	bool refresh = true;
	
	uint8_t maxItemsPerPage = 6;
	uint8_t pageIndex = 0; // 0 is first page
	uint8_t startIndex = 0;
	uint8_t stopIndex = 0;
	uint8_t count = 0;
	uint8_t num_sensors_found = temp1;

	maxCursor = num_sensors_found;
	if( maxCursor == 0){
		maxCursor =1;
	}

	const char menuText_1[] = "Back";
	//const char menuText_2[] = "Cancel";


	if (inMenuNum != 44) {
		inMenuNum = 44;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
				if(num_sensors_found == 0 ){
					cursorPos = 1;
					draw_AddSensor_Menu(uint8_t(0));
					return;
				}
				else{
					drawPairingInProgress();
					if(pairWithSensor(cursorPos - 1)){
						drawPairingSuccess();
					}
					else{
						drawPairingFail();
					}
					
					
				}
				
			break;

			case B00000100: // back button pressed
				cursorPos = 1;
				draw_Pair_Menu(uint8_t(0));
				return;
			break;

			case B00000010: // up button pressed
				refresh = false;
				updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
				refresh = false;
				updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}
	
	if (cursorPos > 0) {
		pageIndex = (cursorPos - 1) / maxItemsPerPage;
	}
	else {
		pageIndex = (cursorPos) / maxItemsPerPage;
	}

	startIndex = pageIndex * maxItemsPerPage;
	stopIndex = startIndex + maxItemsPerPage;
	
	Serial.print(F("Refresh Display: "));
	Serial.println(refresh);
	


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Scan For Sensors"));
	}
	
	Serial.print(F("Found number of Sensor units: "));
	Serial.println(num_sensors_found);
	
	// draw menu boxes on display with box highlighted if cursor index matches
	if ( num_sensors_found > 0) {
		
		// assign the starting memory addresses of the text labels to the TextList pointer to pointers by scanning the text buffer for null characters as string delimiters
		textList[count++] = &text_buffer[0];
		for(uint8_t i = 1; i < sizeof(text_buffer) -1 ; i++ ){
			if(text_buffer[i] != '\0' && text_buffer[i - 1] == '\0'){
				textList[count++] = &text_buffer[i];
			}
			if(count == num_sensors_found){
				break;
			}
			
		}
		
		for (int i = startIndex; i < stopIndex; i++) {
			if ( i < num_sensors_found) {

				if ( i == (cursorPos - 1)) {
					drawMenuBox(textList[i], i - startIndex, boxwidth, leftAdjust, 0, true);
				}
				else {
					drawMenuBox(textList[i], i - startIndex, boxwidth, leftAdjust, 0, false);
				}
			}
			else {
				drawMenuBox(textList[i], i - startIndex, boxwidth, leftAdjust, 0, false, 0, false);
			}
		}
	}
	else {
		ePaper.setCursor(60, 80 );
		ePaper.setTextSize(2);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("No Sensor Units Found"));
		cursorPos = 1;

		textList[0] = &menuText_1[0];
		drawList(textList, 1, 1, boxwidth, leftAdjust, yAdjust);
	}




}


//------------------------------------------------------------

void draw_PairedSensorLabels_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;


	maxCursor = 2;

	const char menuText_1[] = "Blank";
	const char menuText_2[] = "Blank";


	if (inMenuNum != 50) {
		inMenuNum = 50;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed


			break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_PairedSensor_Sub_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Assign Label"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);




}



//------------------------------------------------------------

void draw_PairedSensorSettings_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;

	maxCursor = 2;

	const char menuText_1[] = "Blank";
	const char menuText_2[] = "Blank";


	if (inMenuNum != 51) {
		inMenuNum = 51;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed


			break;

			case B00000100: // back button pressed
			cursorPos = 2;

			draw_PairedSensor_Sub_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Sensor Settings"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);



}

//------------------------------------------------------------

void draw_RemoveSensor_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;
	uint8_t num_sensors_paired = fram.readByte(pairedSensorUnitCountAddr);

	maxCursor = 2;

	const char menuText_1[] = "Delete";
	const char menuText_2[] = "Cancel";



	if (inMenuNum != 52) {
		inMenuNum = 52;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if ( cursorPos == 1) {
				if (selected_sensorUnit_Index >= 0 && selected_sensorUnit_Index < num_sensors_paired) { // only remove the config file if a valid id is given

					cursorPos = selected_sensorUnit_Index;   // set cursor position to prevent displaying from the start of the sensor list
					if (cursorPos > num_sensors_paired) {
						cursorPos = num_sensors_paired;
					}
					else if ( cursorPos == 0) {
						cursorPos = 1;
					}

					removeSensorUnit(selected_sensorUnit_Index);   // remove/delete the config settings for the selected sensor from FRAM
					selected_sensorUnit_Index = -1;                // update selected sensor id to an invalid one
					draw_PairedSensors_Menu(uint8_t(0));
				}
				else {
					cursorPos = selected_sensorUnit_Index + 1;
					draw_PairedSensors_Menu(uint8_t(0));
				}
				return;
			}

			else { // cancel was selected
				cursorPos = 3;
				draw_PairedSensor_Sub_Menu(uint8_t(0));
				return;
			}

			break;

			case B00000100: // back button pressed
			cursorPos = 3;

			draw_PairedSensor_Sub_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Un-Pair Sensors"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);

}

//====================================================================================================================




//====================================================================================================================



void draw_GenSettings_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;


	maxCursor = 6;

	const char menuText_1[] = "Time Settings";
	const char menuText_2[] = "Factory Reset";
	const char menuText_3[] = "Data Settings";
	const char menuText_4[] = "Error History";
	const char menuText_5[] = "Radio USB Bypass";
	const char menuText_6[] = "Request Sample";



	if (inMenuNum != 100) {
		inMenuNum = 100;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos == 1) {
				draw_TimeSetting_Menu(uint8_t(0));
				return;
			}
			else if (cursorPos == 2) {
				cursorPos = 2;
				draw_FactoryReset_Menu(uint8_t(0));
				return;
			}
			else if (cursorPos == 3) {
				cursorPos = 1;
				draw_DataSettings_Menu(uint8_t(0));
				return;
			}
			else if (cursorPos == 4) {
				cursorPos = 1;
				draw_ErrorHistory_Menu(uint8_t(0));
				return;
			}
			else if (cursorPos == 5) {
				cursorPos = 1;
				draw_RadioBypass_Menu(0, true);
				return;
			}
			else if (cursorPos == 6) {
				cursorPos = 1;
				draw_RequestSample_Menu(0, true);
				return;
			}


			break;

			case B00000100: // back button pressed
			cursorPos = 4;

			draw_Base_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}



	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	textList[1] = &menuText_2[0];
	textList[2] = &menuText_3[0];
	textList[3] = &menuText_4[0];
	textList[4] = &menuText_5[0];
	textList[5] = &menuText_6[0];

	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("General Settings"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);




}



//====================================================================================================================

void draw_TimeSetting_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;


	maxCursor = 2;

	const char subMenuTime_SettingDate_Text[] = "Date";
	const char subMenuTime_SettingTime_Text[] = "Time";



	if (inMenuNum != 110) {
		inMenuNum = 110;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos == 1) {
				DateTime now = rtc.now();
				temp1 = now.day();
				temp2 = now.month();
				temp3 = now.year();
				if(temp3 >= 2000){
					temp3 -= 2000;
				}
				draw_TimeSubSettingDate_Menu(0);
				return;
			}
			else if (cursorPos == 2) {
				DateTime now = rtc.now();
				cursorPos = 1;
				temp1 = now.hour();
				temp2 = now.minute();
				temp3 = now.second();
				draw_TimeSubSettingTime_Menu(0);
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_GenSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}



	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &subMenuTime_SettingDate_Text[0];
	textList[1] = &subMenuTime_SettingTime_Text[0];


	if ( refresh ) {
		ePaper.clearDisplay();
		// draw title
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Time Settings"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);




}


//====================================================================================================================

void draw_TimeSubSettingDate_Menu(uint8_t button_mask) {

	int16_t boxwidth = 65;
	int16_t leftAdjust = 80;
	int16_t offset = 45;
	char numBuffer[5];
	memset(numBuffer, '\0', sizeof(numBuffer));
	maxCursor = 3;

	bool refresh = true;


	if (inMenuNum != 111) {
		inMenuNum = 111;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos < 3) {
				cursorPos += 1;
			}
			else if (cursorPos == 3) {
				cursorPos = 1;
				DateTime now = rtc.now();
				rtc.adjust(DateTime(temp3 , temp2, temp1, now.hour(), now.minute(), now.second()));
				temp1 = 0;
				temp2 = 0;
				temp3 = 0;
				draw_TimeSetting_Menu(0);
				return;
			}
			break;

			case B00000100: // back button pressed
			if (cursorPos == 1) {
				cursorPos = 1;
				temp1 = 0;
				temp2 = 0;
				temp3 = 0;
				draw_TimeSetting_Menu(0);
				return;
			}
			else if ( cursorPos > 1) {
				cursorPos -= 1;
			}
			break;

			case B00000010: // up button pressed
			refresh = false;

			if (cursorPos == 1) {
				if (temp1 < 31) {
					temp1 = temp1 + 1;
				}
				else {
					temp1 = 1;
				}
			}
			else if (cursorPos == 2) {
				if (temp2 < 12) {
					temp2 = temp2 + 1;
				}
				else {
					temp2 = 1;
				}
			}
			else if (cursorPos == 3) {
				if ( temp3 < 2099) {
					temp3 = temp3 + 1;
				}
				else {
					temp3 = 2000;
				}
			}
			break;

			case B00000001: // Down button pressed
			refresh = false;
			if (cursorPos == 1) {
				// day
				if (temp1 > 1) {
					temp1 = temp1 - 1;
				}
				else {
					temp1 = 31;
				}
			}
			else if (cursorPos == 2) {
				// Month
				if (temp2 > 1) {
					temp2 = temp2 - 1;
				}
				else {
					temp2 = 12;
				}
			}
			else if (cursorPos == 3) {
				//year
				if ( temp3 > 2000) {
					temp3 = temp3 - 1;
				}
				else {
					temp3 = 2099;
				}
			}
			break;

			default:
			refresh = true;

			break;
		}
	}


	if ( refresh ) {
		ePaper.clearDisplay();
		// draw title
		ePaper.setCursor(120, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Set "));
		ePaper.print(F("Date"));
		// draw labels
		ePaper.setTextSize(2);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.setCursor(90, 85 );
		ePaper.print(F("Day"));
		ePaper.setCursor(165, 85 );
		ePaper.print(F("Month"));
		ePaper.setCursor(255, 85 );
		ePaper.print(F("Year"));
	}


	// draw menu boxes on display with box highlighted if cursor index matches
	if ( cursorPos == 1) {

		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, true);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2 + offset , false);
	}
	else if ( cursorPos == 2) {

		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1 + offset, true);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2 + offset, false);
	}
	else if ( cursorPos == 3) {
		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10), 1, 1, boxwidth, leftAdjust * 1 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10), 2, 1, boxwidth, leftAdjust * 2 + offset, true);
	}





}

//====================================================================================================================

void draw_TimeSubSettingTime_Menu(uint8_t button_mask) {

	int16_t boxwidth = 60;
	int16_t leftAdjust = 80;
	int16_t offset = 45;
	bool refresh = true;


	char numBuffer[5];
	memset(numBuffer, '\0', sizeof(numBuffer));
	maxCursor = 3;



	if (inMenuNum != 112) {
		inMenuNum = 112;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if ( cursorPos < 3) {
				cursorPos += 1;
			}
			else if ( cursorPos == 3) {
				cursorPos = 2;
				DateTime now = rtc.now();
				rtc.adjust(DateTime(now.year() , now.month(), now.day(), temp1, temp2, temp3));
				temp1 = 0;
				temp2 = 0;
				temp3 = 0;
				draw_TimeSetting_Menu(0);
				return;
			}
			break;

			case B00000100: // back button pressed
			if (cursorPos == 1) {
				cursorPos = 2;
				temp1 = 0;
				temp2 = 0;
				temp3 = 0;
				draw_TimeSetting_Menu(0);
				return;
			}
			else if (cursorPos > 1) {
				cursorPos -= 1;
			}
			cursorPos = 1;

			draw_TimeSetting_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			//updateCursor(button_mask);

			if (cursorPos == 1) {
				//hour
				if (temp1 < 23) {
					temp1 = temp1 + 1;
				}
				else {
					temp1 = 0;
				}
			}
			else if (cursorPos == 2) {
				//minutes
				if ( temp2 < 59) {
					temp2 = temp2 + 1;
				}
				else {
					temp2 = 0;
				}
			}
			else if (cursorPos == 3) {
				//sec
				if (temp3 < 59) {
					temp3 = temp3 + 1;
				}
				else {
					temp3 = 0;
				}
			}
			break;

			case B00000001: // Down button pressed
			refresh = false;
			//updateCursor(button_mask);

			if (cursorPos == 1) {
				//hour
				if (temp1 > 0) {
					temp1 = temp1 - 1;
				}
				else {
					temp1 = 23;
				}
			}
			else if (cursorPos == 2) {
				//minutes
				if ( temp2 > 0) {
					temp2 = temp2 - 1;
				}
				else {
					temp2 = 59;
				}
			}
			else if (cursorPos == 3) {
				//sec
				if (temp3 > 0) {
					temp3 = temp3 - 1;
				}
				else {
					temp3 = 59;
				}
			}
			break;

			default:
			refresh = true;
			break;
		}
	}



	if ( refresh ) {
		ePaper.clearDisplay();
		// draw title
		ePaper.setCursor(120, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Set "));
		ePaper.print(F("Time"));
		//draw labels
		ePaper.setTextSize(2);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.setCursor(90, 85 );
		ePaper.print(F("Hour"));
		ePaper.setCursor(165, 85 );
		ePaper.print(F("Min"));
		ePaper.setCursor(255, 85 );
		ePaper.print(F("Sec"));
	}




	// draw menu boxes on display with box highlighted if cursor index matches
	if ( cursorPos == 1) {

		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, true);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 + offset , false);
	}
	else if ( cursorPos == 2) {

		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1 + offset, true);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 + offset , false);
	}
	else if ( cursorPos == 3) {
		drawSettingChangeBox(itoa(temp1, numBuffer, 10), 0, 1, boxwidth, leftAdjust * 0 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp2, numBuffer, 10) , 1, 1, boxwidth, leftAdjust * 1 + offset, false);
		memset(numBuffer, '\0', sizeof(numBuffer));
		drawSettingChangeBox(itoa(temp3, numBuffer, 10) , 2, 1, boxwidth, leftAdjust * 2 + offset, true);
	}




}

//====================================================================================================================

void draw_FactoryReset_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 80;
	bool refresh = true;


	maxCursor = 2;

	const char yes[] = "Yes";
	const char no[] = "No";



	if (inMenuNum != 120) {
		inMenuNum = 120;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if(cursorPos == 1){
				
				factoryReset();
				return;
			}
			else{
				cursorPos = 2;
				draw_GenSettings_Menu(0);
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 2;

			draw_GenSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;
			break;
		}
	}


	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &yes[0];
	textList[1] = &no[0];

	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Factory Reset"));
		ePaper.print('?');

		ePaper.setTextSize(2);
		ePaper.setCursor(65, 70 );
		ePaper.print(F("Warning: this cannot be"));
		ePaper.setCursor(65, 85 );
		ePaper.print(F("undone."));
		
		ePaper.setCursor(65, 110 );
		ePaper.print(F("Device will restart once"));
		ePaper.setCursor(65, 125 );
		ePaper.print(F("complete."));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, 150);



}

//---------------------------------------------------------------------

void draw_DataSettings_Menu(uint8_t button_mask) {
	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	bool refresh = true;


	const char menuText_1[] = "Clear SD Card";
	//const char sampleBuffer[] = "Set Sample Buffer Size";
	//const char menuText_2[] = "Format SD Card";

	maxCursor = 1;



	if (inMenuNum != 130) {
		inMenuNum = 130;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos == 1) {
				cursorPos = 2;
				draw_ClearSDCard_Menu( uint8_t(0));
				return;
			}
			else if (cursorPos == 2) {
				cursorPos = 2;
				draw_FormatSDCard_Menu( uint8_t(0));
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 3;

			draw_GenSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}




	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &menuText_1[0];
	//textList[1] = &menuText_2[0];


	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Data Settings"));

	}

	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust);




}


//====================================================================================================================

void draw_ClearSDCard_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 50;
	bool refresh = true;


	maxCursor = 2;

	const char yes[] = "Yes";
	const char no[] = "No";



	if (inMenuNum != 131) {
		inMenuNum = 131;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			if (cursorPos == 1) {
				draw_PleaseWait();
				if (wipeSDCard()) {
					drawDataClearedMessage();

				}
				else {
					drawDataClearFailedMessage();
				}
				cursorPos = 1;
				draw_DataSettings_Menu(uint8_t(0));
				return;
			}
			else {
				cursorPos = 1;
				draw_DataSettings_Menu(uint8_t(0));
				return;
			}
			break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_DataSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}




	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &yes[0];
	textList[1] = &no[0];

	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Clear Data On SD"));
		ePaper.print('?');

		ePaper.setTextSize(2);
		ePaper.setCursor(65, 70 );
		ePaper.print(F("Warning: this cannot be"));
		ePaper.setCursor(155, 85 );
		ePaper.print(F("undone."));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust);



}


//====================================================================================================================

void draw_FormatSDCard_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 50;
	int16_t charPadding = 120;
	bool refresh = true;

	maxCursor = 2;

	const char yes[] = "Yes";
	const char no[] = "No";


	if (inMenuNum != 132) {
		inMenuNum = 132;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			break;

			case B00000100: // back button pressed
			cursorPos = 1;

			draw_DataSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}




	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &yes[0];
	textList[1] = &no[0];

	if ( refresh ) {
		ePaper.clearDisplay();

		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Format SD Card"));
		ePaper.print('?');

		ePaper.setTextSize(2);
		ePaper.setCursor(65, 70 );
		ePaper.print(F("Warning: this cannot be"));
		ePaper.setCursor(155, 85 );
		ePaper.print(F("undone."));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, charPadding);




}

//====================================================================================================================


void draw_ErrorHistory_Menu(uint8_t button_mask) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 130;
	int16_t charXPadding = 0;
	uint8_t errCountNumposition = 0;
	uint8_t errCountNumMax = 0;
	uint8_t count = 0;
	uint16_t temp;

	bool refresh = true;


	const char backText[] = "Back";

	maxCursor = 1;


	if (inMenuNum != 140) {
		inMenuNum = 140;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			cursorPos = 4;

			draw_GenSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000100: // back button pressed
			cursorPos = 4;

			draw_GenSettings_Menu(uint8_t(0));
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}




	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &backText[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Error History"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, charXPadding);

	errCountNumposition = fram.readByte(errCodeNumAddr ); // get error count number position
	errCountNumMax = fram.readByte(errCodeNumTotalAddr ); // get max number of error count stored

	// sanity check to prevent reading from FRAM adresses not used for error codes
	if ( errCountNumposition > errCodeMaxNum) {
		errCountNumposition = errCodeMaxNum;
	}

	ePaper.setTextSize(1);
	ePaper.setTextColor(BLACK, WHITE);


	if (errCountNumMax != 0) {
		if (errCountNumposition > 0) {

			// error history may not start at '0' since the history is not shifted allong in FRAM, instead a position counter is incremented and the oldest value is overwritten
			for (uint8_t i = errCountNumposition; i < errCountNumMax; i++) {
				ePaper.setCursor(90, 70 + (i - errCountNumposition) * 13 );

				ePaper.print(F("Err: "));
				temp = fram.readUInt16(errCodeHistoryAddr + i * 8 );
				if ( temp < 10) { // padding
					ePaper.print('0');
				}
				if ( temp < 100) {
					ePaper.print('0');
				}
				if ( temp < 1000) {
					ePaper.print('0');
				}
				ePaper.print(temp , HEX); // print error code
				ePaper.print(space);
				ePaper.print(space);
				ePaper.print(F("Time: "));

				for ( uint8_t c = 0; c < 5; c++) {
					temp1 = fram.readByte(errCodeHistoryAddr + i * 8 + 2 + c);
					if ( temp1 < 10) {
						ePaper.print(zero);
					}
					ePaper.print(temp1);
					if (c < 2) {
						ePaper.print(':');
					}
					else if (c == 2) {
						ePaper.print(space);
					}
					else if ( c > 2) {
						ePaper.print('/');
					}

				}

				//print year
				ePaper.print(20);
				temp1 = fram.readByte(errCodeHistoryAddr + i * 8 + 7);
				if ( temp1 < 10) {
					ePaper.print(zero);
				}
				ePaper.print(temp1);

			}
		}
		count = errCountNumMax - errCountNumposition;
		for (uint8_t i = 0; i < errCountNumposition; i++) {
			ePaper.setCursor(90, 70 + (count + i) * 13 );
			ePaper.print(F("Err: "));
			temp = fram.readUInt16(errCodeHistoryAddr + (count + i) * 8 );
			if ( temp < 10) { // padding
				ePaper.print('0');
			}
			if ( temp < 100) {
				ePaper.print('0');
			}
			if ( temp < 1000) {
				ePaper.print('0');
			}
			ePaper.print(temp , HEX); // print error code
			ePaper.print(space);
			ePaper.print(space);
			ePaper.print(F("Time: "));

			for ( uint8_t c = 0; c < 5; c++) {
				temp1 = fram.readByte(errCodeHistoryAddr + (count + i) * 8 + 2 + c);
				if ( temp1 < 10) {
					ePaper.print(zero);
				}
				ePaper.print(temp1);
				if (c < 2) {
					ePaper.print(':');
				}
				else if (c == 2) {
					ePaper.print(space);
				}
				else if ( c > 2) {
					ePaper.print('/');
				}

			}

			//print year
			ePaper.print(20);
			temp1 = fram.readByte(errCodeHistoryAddr + (count + i) * 8 + 7);
			if ( temp1 < 10) {
				ePaper.print(zero);
			}
			ePaper.print(temp1);

		}
	}

	else {
		ePaper.setCursor(90, 70);
		ePaper.print(F("No errors in History"));
	}


}

void draw_RequestSample_Menu(uint8_t button_mask, bool run_true) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 130;
	int16_t charXPadding = 0;
	//uint8_t errCountNumposition = 0;
	//uint8_t errCountNumMax = 0;
	//uint8_t count = 0;
	//uint16_t temp;

	bool refresh = true;


	const char backText[] = "Back";

	maxCursor = 1;


	if (inMenuNum != 142) {
		inMenuNum = 142;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			cursorPos = 6;

			draw_GenSettings_Menu(0);
			return;
			break;

			case B00000100: // back button pressed
			cursorPos = 6;

			draw_GenSettings_Menu(0);
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			//updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			//updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}





	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &backText[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Request Sample"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, charXPadding);

	if (run_true) {
		ePaper.refresh();
		setDisplayRefreshPWM();
		sleep.interruptEnable();
		delay(500);
		request_sample_loop();
	}

}

void request_sample_loop() {
	menu_button_flag = false;
	back_button_flag = false;
	sensorUnitWirelessSync();
	//while (true) {
		//
		//
		//if ( menu_button_flag || back_button_flag) {
			//break;
		//}
	//}

}




void draw_RadioBypass_Menu(uint8_t button_mask, bool run_true) {

	int16_t boxwidth = 369;
	int16_t leftAdjust = 0;
	int16_t yAdjust = 130;
	int16_t charXPadding = 0;
	//uint8_t errCountNumposition = 0;
	//uint8_t errCountNumMax = 0;
	//uint8_t count = 0;
	//uint16_t temp;

	bool refresh = true;


	const char backText[] = "Back";

	maxCursor = 1;


	if (inMenuNum != 141) {
		inMenuNum = 141;    // update current menu value
	}

	else { // functions to perform depending on button pressed in this menu

		switch (button_mask) {

			case B00001000: // menu button pressed
			cursorPos = 5;

			draw_GenSettings_Menu(0);
			return;
			break;

			case B00000100: // back button pressed
			cursorPos = 5;

			draw_GenSettings_Menu(0);
			return;
			break;

			case B00000010: // up button pressed
			refresh = false;
			//updateCursor(button_mask);
			break;

			case B00000001: // down button pressed
			refresh = false;
			//updateCursor(button_mask);
			break;

			default:
			refresh = true;

			break;
		}
	}





	// assign the starting memory addresses of the text labels to the TextList pointer to pointers.
	textList[0] = &backText[0];


	// setup display if this menu is not currently being displayed
	if ( refresh ) {
		ePaper.clearDisplay();
		ePaper.setCursor(60, 30 );
		ePaper.setTextSize(3);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.print(F("Radio USB Bypass"));
	}

	// draw menu boxes on display with box highlighted if cursor index matches
	drawList(textList, maxCursor, cursorPos, boxwidth, leftAdjust, yAdjust, charXPadding);

	if (run_true) {
		ePaper.refresh();
		setDisplayRefreshPWM();
		sleep.interruptEnable();
		delay(500);
		radio_bypass_loop();
	}

}



void radio_bypass_loop() {
	uint32_t baud = 9600;
	uint32_t brkStart = 0;
	uint16_t brkLength = 0;
	uint8_t dtr = 0;
	uint8_t rts = 0;
	menu_button_flag = false;
	back_button_flag = false;
	
	start_radio();

	if (!debug) {
		Serial.begin(0);
		// Initialize using an arbitrary default baud rate

	}
	Serial.begin(0);
	Serial1.begin(baud); // hardware serial connected to radio
	pinMode(radio_DTR, INPUT);
	pinMode(radio_reset, INPUT);
	pinMode(radio_RTS, INPUT);
	digitalWrite(radio_DTR, LOW); // set low to allow pin to be floating or pull down
	digitalWrite(radio_reset, LOW);
	digitalWrite(radio_RTS, LOW);
	while (true) {

		if ( menu_button_flag || back_button_flag) {
			stop_radio();
			break;
		}

		// Forward the DTR and RTS signals. Use OUTPUT LOW to pull low and
		// INPUT to leave the pin floating. This needs a pullup on the other
		// side, but also works when the other device is 3.3V
		//dtr = SerialUSB.dtr() ? OUTPUT : INPUT;
		//if (dtr == INPUT) {
		//pinMode(radio_DTR, INPUT_PULLUP);
		//}
		//else if (dtr == OUTPUT) {
		//pinMode(radio_DTR, OUTPUT);
		//digitalWrite(radio_DTR, LOW);
		//}
		//
		//rts = SerialUSB.rts() ? OUTPUT : INPUT;
		//if (rts == INPUT) {
		//pinMode(radio_DTR, INPUT_PULLUP);
		//}
		//else if (rts == OUTPUT) {
		//pinMode(radio_DTR, OUTPUT);
		//digitalWrite(radio_DTR, LOW);
		//}

		pinMode(radio_DTR, SerialUSB.dtr() ? OUTPUT : INPUT);
		pinMode(radio_RTS, SerialUSB.rts() ? OUTPUT : INPUT);

		// Process break requests. 0xffff means to start a break, 0 to end it
		// and any other value to start a break that lasts that many ms.
		int32_t brk = SerialUSB.readBreak();
		if (brk > 0) {
			Serial1.flush();
			setBreak(true);

			// If the break has a defined length, keep track of it so it can be
			// disabled at the right time
			if (brk != 0xffff) {
				brkLength = brk;
				brkStart = micros();
			}
			} else if (brk == 0 || (brkLength > 0 && (micros() - brkStart) >= 1000L * brkLength)) {
			// If a timed break was running and its time expires, or if the
			// break is explicitly canceled through a request with time = 0,
			// end the break
			setBreak(false);
			brkLength = 0;
		}

		// If the requested baudrate changed on SerialUSB, update it on Serial1
		// too (but not while a break condition is ongoing, since calling
		// Serial1.begin() will end the break condition
		if (!inBreak() && SerialUSB.baud() != baud) {
			baud = SerialUSB.baud();

			// Set up the TX pin as OUTPUT HIGH, so that when the UART is
			// disabled, the pin remains high
			pinMode(1, OUTPUT);
			digitalWrite(1, HIGH);

			Serial1.end();
			Serial1.begin(baud);
		}

		// Forward data between SerialUSB and Serial
		if (!inBreak()) {
			// Only write to Serial when TX is enabled, to prevent lockup
			if (SerialUSB.available())
			Serial1.write(SerialUSB.read());
		}
		if (Serial1.available())
		SerialUSB.write(Serial1.read());

	}
	
	
}

void setBreak(bool enable) {
	if (enable) {
		// If a break was requested, force the TX pin low and then disable the
		// UART TX side so the digitalWrite actually takes effect
		pinMode(1, OUTPUT);
		digitalWrite(1, LOW);
		SERCOM0->USART.CTRLB.bit.TXEN = 0;
		} else {
		// End a break by enabling the UART TX again.
		SERCOM0->USART.CTRLB.bit.TXEN = 1;
	}
}

bool inBreak() {
	// We're in a break when the UART TX is disabled
	return (SERCOM0->USART.CTRLB.bit.TXEN ) == 0;
}







//================ Display notification messages ============================================

void drawDataClearedMessage() {
	ePaper.fillRect(95, 75, 200, 70, WHITE);
	ePaper.drawRect(95, 75, 200, 70, BLACK);
	ePaper.drawRect(96, 76, 198, 68, BLACK);
	ePaper.setCursor(120, 95);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Data Cleared!"));
	ePaper.refresh();
	//ePaper.refresh();
	delay(2000);


}

void draw_PleaseWait() {
	ePaper.fillRect(95, 75, 200, 70, WHITE);
	ePaper.drawRect(95, 75, 200, 70, BLACK);
	ePaper.drawRect(96, 76, 198, 68, BLACK);
	ePaper.setCursor(115, 95);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Please Wait..."));
	ePaper.refresh();
}

void drawDataClearFailedMessage() {
	ePaper.fillRect(95, 75, 200, 70, WHITE);
	ePaper.drawRect(95, 75, 200, 70, BLACK);
	ePaper.drawRect(96, 76, 198, 68, BLACK);
	ePaper.setCursor(90, 90);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("!Data Clear Failed!"));

	ePaper.refresh();
	//ePaper.refresh();
	delay(5000);
}

void drawSamplingStart() {
	//ePaper.clearDisplay();
	ePaper.fillRect(90, 60, 220, 70, WHITE);
	ePaper.drawRect(90, 60, 220, 70, BLACK);
	ePaper.drawRect(91, 61, 218, 68, BLACK);
	ePaper.setCursor(100, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Sampling Started!"));
	//delay(1);
	ePaper.refresh();
	//ePaper.refresh();
	delay(2000);
}

void drawSyncInProgress() {

	ePaper.fillRect(95, 60, 200, 110, WHITE);
	ePaper.drawRect(95, 60, 200, 110, BLACK);
	ePaper.drawRect(96, 61, 198, 108, BLACK);
	ePaper.setCursor(115, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Syncing In"));
	ePaper.setCursor(140, 100);
	ePaper.print(F("Progress."));
	ePaper.setCursor(120, 130);
	ePaper.print(F("Please Wait.."));

	ePaper.refresh();
	//ePaper.refresh();
	//delay(1000);
}



void drawScanninginProgress(){
	ePaper.fillRect(75, 60, 240, 110, WHITE);
	ePaper.drawRect(75, 60, 240, 110, BLACK);
	ePaper.drawRect(76, 61, 238, 108, BLACK);
	ePaper.setCursor(115, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Scanning for"));
	ePaper.setCursor(115, 100);
	ePaper.print(F("Sensor Units."));
	ePaper.setCursor(120, 130);
	ePaper.print(F("Please Wait.."));

	ePaper.refresh();
	
	
}

void drawPairingInProgress() {

	ePaper.fillRect(95, 60, 200, 110, WHITE);
	ePaper.drawRect(95, 60, 200, 110, BLACK);
	ePaper.drawRect(96, 61, 198, 108, BLACK);
	ePaper.setCursor(115, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Pairing..."));
	ePaper.setCursor(115, 130);
	ePaper.print(F("Please Wait.."));

	ePaper.refresh();
	//ePaper.refresh();
	//delay(1000);
}

void drawPairingSuccess() {

	ePaper.fillRect(95, 60, 200, 110, WHITE);
	ePaper.drawRect(95, 60, 200, 110, BLACK);
	ePaper.drawRect(96, 61, 198, 108, BLACK);
	ePaper.setCursor(115, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Pairing..."));
	ePaper.setCursor(115, 130);
	ePaper.print(F("Success!"));

	ePaper.refresh();
	//ePaper.refresh();
	delay(3000);
}

void drawPairingFail() {

	ePaper.fillRect(95, 60, 200, 110, WHITE);
	ePaper.drawRect(95, 60, 200, 110, BLACK);
	ePaper.drawRect(96, 61, 198, 108, BLACK);
	ePaper.setCursor(115, 80);
	ePaper.setTextSize(2);
	ePaper.setTextColor(BLACK, WHITE);
	ePaper.print(F("Pairing Failed!"));
	ePaper.setCursor(115, 130);
	ePaper.print(F("Please try again."));

	ePaper.refresh();
	
	if(debug){
		Serial.println(F("Pairing failed!"));
	}
	//ePaper.refresh();
	delay(3000);
}


/*
-----------------------------------------------------------------------------------
*/




bool batteryGood() {
	float measuredvbat = 0;

	PM->APBCMASK.reg |= 0x00010000; // enable the ADC Clock
	ADC->CTRLA.bit.ENABLE = 1; // Enable ADC

	for (int i = 0; i < 3 ; i++) {
		measuredvbat += analogRead(VBATPIN);
	}

	ADC->CTRLA.bit.ENABLE = 0;       // disable ADC
	PM->APBCMASK.reg &= ~0x00010000; // disable the ADC Clock

	measuredvbat /= 3; //apply average

	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	measuredvbat -= 0.07; //add offset to measurement to get better value aof actual voltage
	//  ePaper.setTextSize(1);
	//  ePaper.setTextColor(BLACK, WHITE);
	//  ePaper.setCursor(170, 220);
	//
	//  ePaper.print("VBat: " );
	//  ePaper.println(measuredvbat);
	if (measuredvbat < 3.55) { // low battery voltage is assumed to be 3.55v for lithium ion battery
		return false;
	}
	else {
		return true;
	}
}

//======================================================================



void error(uint16_t errorCode) {
	uint16_t err;
	uint8_t ledCount = 1;
	uint8_t errCountNum = fram.readByte(errCodeNumAddr) ; // get error code position
	uint8_t errCountNumtotal;

	// error history total for while the device is on

	err = fram.readUInt16(0);
	err |= errorCode;
	fram.writeUInt16(0, err); //store error in FRAM for later checking
	if (debug) {
		printError();
	}
	if (displayOn) {
		ePaper.setTextSize(1);
		ePaper.setTextColor(BLACK, WHITE);
		ePaper.setCursor(135, 5);
		ePaper.print(F("ERR: "));
		ePaper.print(errorCode, HEX);
		ePaper.refresh();
	}

	// error history over all time
	errCountNumtotal = fram.readByte(errCodeNumTotalAddr ); // get max number of error count stored

	// write error code history to FRAM
	if (errCountNum > errCodeMaxNum) {
		errCountNum = 0;
	}
	DateTime now = rtc.now();

	fram.writeUInt16(errCodeHistoryAddr + errCountNum * 8 , errorCode); //store error in FRAM history for later checking
	fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 2, now.hour()); //store timestamps for error in FRAM for later checking
	fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 3, now.minute());
	fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 4, now.second());
	fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 5, now.day());
	fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 6, now.month());
	if(now.year() >= 2000){
		fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 7, now.year()-2000);
	}
	else{
		fram.writeByte(errCodeHistoryAddr + errCountNum * 8 + 7, now.year());
	}
	
	fram.writeByte(errCodeNumAddr, errCountNum + 1); // update error count number
	if (debug) {
		Serial.print(F("Err code FRAMwrite address: "));
		Serial.println(errCodeHistoryAddr + errCountNum * 8 + 7);
	}

	// increment the total number of errors stored in FRAM counter
	if (errCountNumtotal <= errCodeMaxNum) {
		fram.writeByte(errCodeNumTotalAddr,  errCountNum + 1);
	}


	if (debug) {
		Serial.print(F("error history: "));

		if (errCountNumtotal > 0) {
			for (uint8_t i = errCountNum; i < errCountNumtotal; i++) {
				Serial.print(F(" Err: "));
				Serial.print(fram.readUInt16(errCodeHistoryAddr + i * 8 ), HEX); // print error code
				Serial.print(space);
				Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 2)); // print time stamps
				Serial.print(':');
				Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 3));
				Serial.print(':');
				Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 4));
				Serial.print(space);
				Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 5));
				Serial.print('/');
				Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 6));
				Serial.print('/');
				Serial.println(fram.readByte(errCodeHistoryAddr + i * 8 + 7));

			}
		}
		for (uint8_t i = 0; i < errCountNum; i++) {
			Serial.print(F(" Err: "));
			Serial.print(fram.readUInt16(errCodeHistoryAddr + i * 8 ), HEX); // print error code
			Serial.print(space);
			Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 2)); // print time stamps
			Serial.print(':');
			Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 3));
			Serial.print(':');
			Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 4));
			Serial.print(space);
			Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 5));
			Serial.print('/');
			Serial.print(fram.readByte(errCodeHistoryAddr + i * 8 + 6));
			Serial.print('/');
			Serial.println(fram.readByte(errCodeHistoryAddr + i * 8 + 7));

		}

	}




	if (!displayOn) {
		//err = errorCode; // reuse err for finding number of times to flash led
		while (!(errorCode & 1)) {
			errorCode = errorCode >> 1; //bit shift errorCode untill the least significant bit is 1
			ledCount++;
		}

		digitalWrite(PWRLED, HIGH); // pulse LED to show start of error status flashing
		delay(800);
		digitalWrite(PWRLED, LOW);
		delay(500);
		pwrLED(ledCount); // flash led x number of times to indicate error

		//double up Led flashes to catch error easier
		delay(1000);
		digitalWrite(PWRLED, HIGH); // pulse LED to show start of error status flashing
		delay(800);
		digitalWrite(PWRLED, LOW);
		delay(500);
		pwrLED(ledCount); // flash led x number of times to indicate error
	}

	//  if(errorCode > 0x00FF){ // if the error is a hardware critical error, the devices has failed and is put to sleep
	//    fail();
	//  }


}

//======================================================================

void fail() {
	// go to sleep until device gets reset
	//cli(); // disable interrupts - this should prevent device from waking
	//EIMSK &= (0 << INT0);
	//startSleep();

}

//======================================================================
void printError() {

	Serial.print(F("ERR: "));
	Serial.println(fram.readUInt16(0), HEX);


}


void pwrLED(uint8_t flashAmount) {

	for (int i = 0; i < flashAmount; i++) {
		digitalWrite(PWRLED, HIGH);
		delay(10);
		digitalWrite(PWRLED, LOW);
		delay(380);
	}

}




void loadSettings(){
	
	
	
	runSampling = fram.readByte(SamplingEnabledFramAddr);
	
	uploadInterval_bufferLength = fram.readUInt16(defaultSampleBufferSize_Addr);
	
	if(uploadInterval_bufferLength < minimumSamplingUploadTime){
		// set default value if a factory reset occurred.
		uploadInterval_bufferLength = minimumSamplingUploadTime;
	}
	
	defaultSampleRateHour = fram.readByte(samplePeriod_Addr);
	defaultSampleRateMin = fram.readByte(samplePeriod_Addr + 1);
	if(defaultSampleRateHour == 0 && defaultSampleRateMin == 0){
		// set default value if a factory reset occurred.
		defaultSampleRateMin = 5;
	}
	upload_time_interval_sampling = fram.readUInt32(upload_time_interval_sampling_addr);
	
	if(upload_time_interval_sampling == 0 ){
		// set default value if a factory reset occurred.
		upload_time_interval_sampling = 600;
		fram.writeUInt16(upload_time_interval_sampling_addr, upload_time_interval_sampling);
	}
	
	upload_time_interval_idle = fram.readUInt32(upload_time_interval_idle_addr);
	
	if(upload_time_interval_idle == 0 ){
		// set default value if a factory reset occurred.
		upload_time_interval_idle = 300;
		fram.writeUInt16(upload_time_interval_idle_addr, upload_time_interval_idle);
	}
	
	
	nextAlarmTime_hours = fram.readByte(nextAlarmTime_hours_addr);
	nextAlarmTime_minutes = fram.readByte(nextAlarmTime_minutes_addr);
	nextAlarmTime_seconds = fram.readByte(nextAlarmTime_seconds_addr);
	nextAlarmTime_days = fram.readByte(nextAlarmTime_days_addr);
	nextAlarmTime_months = fram.readByte(nextAlarmTime_months_addr);
	nextAlarmTime_years = fram.readByte(nextAlarmTime_years_addr);

}






/*
Loads
*/
void loadSUConfig(uint16_t SU_index) {
	uint16_t addr = sensorUnitConfigBegin_addr + max_config_FRAM_byte_length * SU_index;
	uint8_t configSize = max_config_FRAM_byte_length;
	uint8_t num_configs = 8;
	Wire.setClock(I2C_FRAM_CLOCK);

	tempSUConfig.device_id_high          = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	tempSUConfig.device_id_low           = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	tempSUConfig.SensorType              = fram.readByte(addr);
	addr += sizeof(uint8_t);

	tempSUConfig.network_channel         = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.network_ID              = fram.readUInt16(addr);
	addr += sizeof(uint16_t);
	tempSUConfig.network_address_high    = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	tempSUConfig.network_address_low     = fram.readUInt32(addr);
	addr += sizeof(uint32_t);

	tempSUConfig.sample_interval_seconds = fram.readUInt32(addr);
	addr += sizeof(uint32_t);
	tempSUConfig.buffer_size             = fram.readUInt16(addr);
	addr += sizeof(uint16_t);
	tempSUConfig.run_sampling            = fram.readByte(addr);
	addr += sizeof(uint8_t);

	//tempSUConfig.next_upload_Hour        = fram.readByte(addr);
	//addr += sizeof(uint8_t);
	//tempSUConfig.next_upload_Minutes     = fram.readByte(addr);
	//addr += sizeof(uint8_t);
	//tempSUConfig.next_upload_Seconds     = fram.readByte(addr);
	//addr += sizeof(uint8_t);
	//tempSUConfig.next_upload_Date        = fram.readByte(addr);
	//addr += sizeof(uint8_t);
	//tempSUConfig.next_upload_Months      = fram.readByte(addr);
	//addr += sizeof(uint8_t);
	//tempSUConfig.next_upload_years       = fram.readByte(addr);
	//addr += sizeof(uint8_t);

	tempSUConfig.dual_channel_mode       = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.channel_gain_index[0]      = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.channel_gain_index[1]      = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.channel_gain_index[2]      = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.channel_gain_index[3]      = fram.readByte(addr);
	addr += sizeof(uint8_t);
	tempSUConfig.autoRange_mode          = fram.readByte(addr);

	Wire.setClock(I2C_RTC_CLOCK);
}



void saveSUConfigToFram() {
	uint16_t addr = sensorUnitConfigBegin_addr;
	uint8_t configSize = max_config_FRAM_byte_length;
	uint8_t num_configs = 	fram.readByte(pairedSensorUnitCountAddr);
	
	Wire.setClock(I2C_FRAM_CLOCK);
	
	int16_t existingConfigIndex = scanForExistingConfig(tempSUConfig.network_address_high, tempSUConfig.network_address_low); // search fram for existing settings

	if(existingConfigIndex >= 0){ 
		// if a matching SUConfig exists in FRAM, update it.
		addr = sensorUnitConfigBegin_addr + configSize * existingConfigIndex;

	}
	else{
		// Otherwise, append the new config to the end
		addr = sensorUnitConfigBegin_addr + configSize * (num_configs );
		fram.writeByte(pairedSensorUnitCountAddr, num_configs + 1); // increment paired sensor count
	}
	

	fram.writeUInt32(addr, tempSUConfig.device_id_high);
	addr += 4;
	fram.writeUInt32(addr, tempSUConfig.device_id_low);  // increment here
	addr += 4;
	fram.writeByte(addr, tempSUConfig.SensorType);
	addr += 1;

	fram.writeByte(addr, tempSUConfig.network_channel);
	addr += 1;
	fram.writeUInt16(addr, tempSUConfig.network_ID);
	addr += 2;
	fram.writeUInt32(addr, tempSUConfig.network_address_high);
	//Serial.print(F("Saving Address: "));
	//Serial.println(addr);
	addr += 4;
	fram.writeUInt32(addr, tempSUConfig.network_address_low ); //increment here
	addr += 4;

	fram.writeUInt32(addr, tempSUConfig.sample_interval_seconds);
	addr += 4;
	fram.writeUInt16(addr, tempSUConfig.buffer_size);
	addr += 2;

	fram.writeByte(addr, tempSUConfig.run_sampling);
	addr += 1;
	

	//if(tempSUConfig.SensorType == 1){
		//// max 8 single channels supported
		////    for (int n = 0; n < 4; n++) {
		////      fram.writeFloat(addr, tempSUConfig.gain_resolution_mV[n]);
		////      addr += 4;
		////    }
		////    fram.writeByte(addr, tempSUConfig.channel_enabled);
		////    addr += 1;
		//fram.writeByte(addr, tempSUConfig.dual_channel_mode);
		//addr += 1;
//
		//for (int n = 0; n < 4; n++) {
			//fram.writeByte(addr, tempSUConfig.channel_gain_index[n]);
			//addr += 1;
		//}
		//fram.writeByte(addr, tempSUConfig.autoRange_mode);
		//addr += 1;
	//}
	//else{
		fram.writeUInt32(addr, 0);
		addr += 4;
		fram.writeUInt16(addr, 0);
	//}

	
	Wire.setClock(I2C_RTC_CLOCK);
}





int16_t scanForExistingConfig(uint32_t network_address_high, uint32_t network_address_low){
	uint16_t addr = sensorUnitConfigBegin_addr;
	uint8_t configSize = max_config_FRAM_byte_length;
	uint8_t num_configs = 	fram.readByte(pairedSensorUnitCountAddr);
	int16_t index = -1;
	
	
	for(uint16_t i = 0; i < num_configs; i++){
		addr = sensorUnitConfigBegin_addr + configSize * i;
		
		if(network_address_high == fram.readUInt32(addr + 12) && network_address_low == fram.readUInt32(addr + 16) ){
			index = i;
			break;
		}
	}
	return index;	
}



void concatCharArray(char * charBuffer, uint8_t buffer_length, char * charList_1, const char * charList_2) {
	uint8_t index = 0;
	for (int i = 0; i < buffer_length - 1; i++) {
		if (index < buffer_length - 1) {
			if ( charList_1[i] != '\0') {
				charBuffer[index] = charList_1[i];
				charBuffer[index + 1] = '\0';
				index++;
			}
			else {
				break;
			}
		}
	}

	for (int i = 0; i < buffer_length - 1; i++) {
		if (index < buffer_length - 1) {
			if ( charList_2[i] != '\0') {
				charBuffer[index] = charList_2[i];
				charBuffer[index + 1] = '\0';
				index++;
			}
			else {
				break;
			}
		}
	}


}



void factoryReset(){
	uint32_t endAddress = FRAM_SIZE;
	uint8_t zeros[16];
	memset(&zeros[0], (uint8_t)0, sizeof(zeros) );
	uint32_t iterations = endAddress/sizeof(zeros) ;
	
	
	// erase FRAM with 16 byte block writes (Arduino software has a 32 byte limit (artificial) on I2C)
	
	//Serial.println(endAddress);
	for(uint32_t i = 0; i < iterations ; i++) {
		//Serial.println(i*sizeof(zeros));
		fram.writeBlock(i*sizeof(zeros), zeros, sizeof(zeros));
	}
	

	NVIC_SystemReset();      // processor software reset
}



//============== SD card methods====================================================
void testSDCardWriting(){
	uint16_t addr;
	char datalayout[] = {'c', 'E'};
	tempSUConfig.device_id_high = 0x01234567;
	tempSUConfig.device_id_low	= 0x89ABCDEF;
	fram.writeUInt16(sampleBufferIndex_addr, 16);
	
	for(uint8_t i = 0; i < 16; i++){
		addr = framDataBufferStartAddr + i * maxSampleByteLength;
		fram.writeByte(addr++, 8);
		fram.writeByte(addr++, 7);
		fram.writeByte(addr++, i);
		fram.writeByte(addr++, 6);
		fram.writeByte(addr++, 12);
		fram.writeByte(addr++, 17);
		
		
		fram.writeUInt16(addr, (i+i));
		addr += sizeof(uint16_t);
		fram.writeByte(addr++, i+1);
		
	}
	
	if(SaveDataToSD(datalayout, 2)){
		Serial.println(F("Success!"));
	}
	else{
		Serial.println(F("SD card writing failed!"));
	}
	
	
}


bool SaveDataToSD(char *sampleLayoutBuffer, uint8_t sampleDataElementCount){
	uint8_t int_buffer[8];
	char fileName[22];
	double temp;
	uint8_t fileCheck;
	bool breakSampleWrite = false;
	DateTime now = rtc.now();
	Wire.setClock(I2C_FRAM_CLOCK);
	
	uint16_t sample_count = fram.readUInt16(sampleBufferIndex_addr);
	uint16_t addr;// + sample_index * maxSampleByteLength;
	
	if(debug){
		Serial.print(F("Writing Sample count: "));
		Serial.println(sample_count);
	}
	
	// get file name based on sensor unit ID number. NOTE: 8.3 file names work but longer file names cause issues such as only being unable to re open a file for writing. 8.3 file name = 8 char name . 3 char extension
	
	//memcpy(&int_buffer[0], &tempSUConfig.device_id_high, sizeof(uint32_t));	// bit order is reversed, compared to expected order, when copied to byte arrays
	memcpy(&int_buffer[0], &tempSUConfig.device_id_low, sizeof(uint32_t));
	
	integerToHexChar(&fileName[0], &int_buffer[0], 4, false); // bit order is reversed when copied to byte arrays
	//integerToHexChar(&fileName[8], &int_buffer[4], 4, false);
	fileName[8] = '.';
	fileName[9] = 'C';
	fileName[10] = 'S';
	fileName[11] = 'V';
	fileName[12] = '\0';

	//fileName[16] = '.';
	//fileName[17] = 'T';
	//fileName[18] = 'X';
	//fileName[19] = 'T';
	//fileName[20] = '\0';

	
	//Switch on power to SD card
	startSDCard(); 
	
	if(debug){
		Serial.println(F("Initializing SD card."));
	}

	if(!initialiseSDCard()){ // attempt to initialize SD card failed
		stopSDCard(); //Switch off power to SD card
		if(debug){
			Serial.println(F("SD card initialization failed!"));
		}
		return false;
		
	}
	
	if(debug){
		Serial.print(F("Opening file: "));
		Serial.println(fileName);
	}
	
	// open file on SD card. create the file if it does not exist. Throw error and return if failed.
	
	if ( !txtFile.open(fileName, O_RDWR   | O_CREAT | O_APPEND ) ) { //O_AT_END
		if(debug){
			Serial.println(F("SD card file could not be opened/created!"));
		}
		error(0x0010);
		stopSDCard(); //Switch off power to SD card
		return false;
	}
	
	//txtFile.seekEnd();
	
	
	for(uint16_t index = 0; index < sample_count; index++){
		breakSampleWrite = false;
		Serial.print(index);
		
		addr = framDataBufferStartAddr + index * maxSampleByteLength;
		
		txtFile.print(fram.readByte(addr));	// save hour
		addr += 1;
		txtFile.write(':');
		txtFile.print(fram.readByte(addr));	// save minute
		addr += 1;
		txtFile.write(':');
		txtFile.print(fram.readByte(addr));	// save seconds
		addr += 1;
		txtFile.write(commar);
		txtFile.print(fram.readByte(addr));	// save day
		addr += 1;
		txtFile.write('/');
		txtFile.print(fram.readByte(addr));	// save month
		addr += 1;
		txtFile.write('/');
		txtFile.print(fram.readByte(addr));	// save year
		addr += 1;
		txtFile.write(commar);
		
		// now save the data samples using the provided encoding information
		for(uint8_t i =0; i < sampleDataElementCount; i++ ){
			if(breakSampleWrite == true){
				break;
			}
			
			
			switch(sampleLayoutBuffer[i]){
				
				case 'a':
					txtFile.write( (char)fram.readByte(addr) );
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print((char)fram.readByte(addr));
					Serial.print(commar);
					
					addr += 1;
					break;
				
				case 'b':
					txtFile.print(fram.readByte(addr));
					txtFile.write(commar);

					Serial.print(sampleLayoutBuffer[i]);
					Serial.print(fram.readByte(addr));
					Serial.print(commar);
					
					addr += sizeof(uint8_t);
					break;
				
				case 'c':
					txtFile.print(fram.readUInt16(addr));
					txtFile.write(commar);

					Serial.print(sampleLayoutBuffer[i]);
					Serial.print(fram.readUInt16(addr));
					Serial.print(commar);
					
					addr += sizeof(uint16_t);
					break;
					
				case 'd':
					txtFile.print(fram.readUInt32(addr));
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print(fram.readUInt32(addr));
					Serial.print(commar);
					
					addr += sizeof(uint32_t);
					break;
				
				case 'e':
					txtFile.print( (int8_t)fram.readByte(addr) );
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print( (int8_t)fram.readByte(addr) );
					Serial.print(commar);
					
					addr += 1;
					break;
				
				case 'f':
					txtFile.print( (int16_t)fram.readUInt16(addr));
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print( (int16_t)fram.readUInt16(addr));
					Serial.print(commar);
					
					addr += sizeof(int16_t);
					break;
				
				case 'g':
					txtFile.print( (int32_t)fram.readUInt32(addr));
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print( (int32_t)fram.readUInt32(addr));
					Serial.print(commar);
					
					addr += sizeof(int32_t);
					break;
				
				case 'F':
					txtFile.print( fram.readFloat(addr), 6);
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print( fram.readFloat(addr), 6);
					Serial.print(commar);
					
					addr += sizeof(float);
					
					break;
				
				case 'D':
					
					txtFile.print( fram.readDouble(addr), 10);
					txtFile.write(commar);
					
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print( fram.readDouble(addr), 10);
					Serial.print(commar);
					
					addr += sizeof(double);
					break;
					
				
				case 'E':
					breakSampleWrite = true;
					Serial.print(sampleLayoutBuffer[i]);
					Serial.print(space);
					break;
					
				default:
					break;
				
			}
		}
		Serial.println();
		
		txtFile.print(fram.readByte(addr));	// save the sample data checksum to the SD card.
		addr += 1;
		txtFile.println(commar);
		
		
		
	}
	
	txtFile.timestamp(T_ACCESS, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() );
	
	txtFile.sync();
	txtFile.close();
	
	
	
	Wire.setClock(I2C_RTC_CLOCK);
	stopSDCard();
	return true;
	
}

bool wipeSDCard() {
	bool initFlag = false;
	int beginAttempts = 0;
	//bool wipe_success = false;
	startSDCard(); //Switch on power to SD card, NOTE N-CH MOSFET
	delay(500);

	checkSDCard();

	while (!initFlag && beginAttempts < 5) { //, SPI_HALF_SPEED

		if (debug) {
			Serial.print(F("SD INIT FAIL: Try "));
			Serial.println(beginAttempts + 1);
		}
		if ( SD.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
			initFlag = true;
		}
		beginAttempts++;
		delay(100);
	}
	if ( !initFlag) {
		error(0x4000);

	}

	// no error so continue with wiping

	if ( !SD.wipe()) {
		digitalWrite(sd_PWR_EN, LOW);
		delay(250);
		digitalWrite(SD_CS_PIN, LOW);
		return false;
	}

	initFlag = false;
	beginAttempts = 0;

	// now reinitialize the card to check it works

	while (!initFlag && beginAttempts < 5) { //, SPI_HALF_SPEED

		if (debug) {
			Serial.print(F("SD INIT FAIL: Try "));
			Serial.println(beginAttempts + 1);
		}
		if ( SD.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
			initFlag = true;
		}
		beginAttempts++;
		delay(100);
	}
	if ( !initFlag) {
		error(0x4000);
		stopSDCard();
		return false;
	}

	else {
		stopSDCard();
		return true;
	}

}


// Method that checks if SD card is present and prevents further execution of code until one is present
void checkSDCard() {
	uint8_t tryCount = 0;
	int sdCardPresent;

	pinMode(SD_card_detect, INPUT);
	sdCardPresent = digitalRead(SD_card_detect);

	while (sdCardPresent == 1) {
		if (tryCount >= 2) {
			if (debug) {
				Serial.println(F(" SD card not found, Device Halted"));
				Serial.flush();
			}
			error(0x0004);
			//pwrLED(4);
			//startSleep(); // sleep until card is detected - woken from sleep by rtc

			//clearRTCAlarm();
		}

		//digitalWrite(sd_PWR_EN, HIGH); //
		sdCardPresent = digitalRead(SD_card_detect);
		if (sdCardPresent == 0) {
			delay(3000);
			if (debug) {
				Serial.println(F(" SD card found, Device Resuming"));
			}

			//digitalWrite(PWR_SW_SENS, HIGH); // Switch on power to Sensors

		}
		tryCount++;
	}
}


bool initialiseSDCard() {
	uint8_t beginAttempts = 0;
	bool initFlag = false;


	checkSDCard(); // check if card is present before proceeding

	while (!initFlag && beginAttempts < 5) { //, SPI_HALF_SPEED

		if (debug) {
			Serial.print(F("SD INIT FAIL: Try "));
			Serial.println(beginAttempts + 1);
		}
		if ( SD.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
			initFlag = true;
			break;
		}
		beginAttempts++;
		delay(100);
	}
	if ( !initFlag ) {
		
		error(0x4000);
		return false;
	}

	//if (!SD.exists(file)) {
		//if (debug) {
			//Serial.print(F("cannot find "));
			//Serial.write(file);
			//Serial.println(F(" file on SD card!"));
		//}
		//return false;
	//}
	return true;
}



void startSDCard(){
	digitalWrite(sd_PWR_EN, HIGH);
	SPI.begin();
	pinMode(SD_CS_PIN, OUTPUT);
	digitalWrite(SD_CS_PIN, HIGH);
	
	
}




void stopSDCard(){
	pinMode(SD_CS_PIN, OUTPUT);
	
	SPI.end();

	digitalWrite(sd_PWR_EN, LOW);
	digitalWrite(SD_MOSI, LOW);
	digitalWrite(SD_CS_PIN, LOW);
	//delay(250);
}




bool checkUSBConnected() {
	pinMode(USB_detect, INPUT);
	digitalRead(USB_detect);
	return (digitalRead(USB_detect)) == 1;
}



uint32_t FreeRam() {
	uint32_t stackTop;
	uint32_t heapTop;

	// current position of the stack.
	stackTop = (uint32_t) &stackTop;

	// current position of heap.
	void* hTop = malloc(1);
	heapTop = (uint32_t) hTop;
	free(hTop);

	// The difference is the free, available ram.
	return stackTop - heapTop;
}



