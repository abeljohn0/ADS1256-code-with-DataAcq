// Build example: gcc ads1256.c -std=c99 -o ads1256 -lbcm2835

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835.h>
#include <glib.h>
#include <librdkafka/rdkafka.h>
#include "DAC8532.h"

#include "common.c"

#define ARR_SIZE(arr) ( sizeof((arr)) / sizeof((arr[0])) )


//#include <ads1256.h>

/*
	***************************
	** PART 0 - enumerations **
	***************************
	Enumerations:
		- PGA   - programmable gain amplifier (PGA) settings
		- DRATE - data rate of programmable filter settings
		- REG   - register control adresses
		- CMD   - commands for controlling operation of ADS1256
		- AIN   - input analog channels
		- bool  - boolean True, False
*/

// Set custom data types that are 8, 16 and 32 bits long.
#define uint8_t  unsigned char  	// 1 byte
#define uint16_t unsigned short 	// 2 bytes
#define uint32_t unsigned long  	// 4 bytes
//#define uint64_t unsigned long long // 8 bytes

//  Set the Programmable gain amplifier (PGA).
//	PGA Provides more resolution when measuring smaller input signals.
//	Set the PGA to the highest possible setting.
enum
{
	PGA_GAIN1	= 0, // Input voltage range: +- 5 V
	PGA_GAIN2	= 1, // Input voltage range: +- 2.5 V
	PGA_GAIN4	= 2, // Input voltage range: +- 1.25 V
	PGA_GAIN8	= 3, // Input voltage range: +- 0.625 V
	PGA_GAIN16	= 4, // Input voltage range: +- 0.3125 V
	PGA_GAIN32	= 5, // Input voltage range: +- 0.15625 V
	PGA_GAIN64	= 6  // Input voltage range: +- 0.078125 V
};

//  Set a data rate of a programmable filter (programmable averager).
//	Programmable from 30,000 to 2.5 samples per second (SPS).
//	Setting the data rate to high value results in smaller resolution of the data.
enum
{
	DRATE_30000 = 0xF0, 
	DRATE_15000 = 0xE0,
	DRATE_7500  = 0xD0,
	DRATE_3750  = 0xC0,
	DRATE_2000  = 0xB0,
	DRATE_1000  = 0xA1,
	DRATE_500   = 0x92,
	DRATE_100   = 0x82,
	DRATE_60    = 0x72,
	DRATE_50    = 0x63,
	DRATE_30    = 0x53,
	DRATE_25    = 0x43,
	DRATE_15    = 0x33,
	DRATE_10    = 0x20,
	DRATE_5     = 0x13,
	DRATE_2d5   = 0x03
};

//  Set of registers.
//	The operation of the ADS1256 is controlled through a set of registers. 
//	Collectively, the registers contain all the information needed to configure 
//	data rate, multiplexer settings, PGA setting, calibration, etc.
enum
{
	REG_STATUS = 0,	 // Register adress: 00h, Reset value: x1H
	REG_MUX    = 1,  // Register adress: 01h, Reset value: 01H
	REG_ADCON  = 2,  // Register adress: 02h, Reset value: 20H
	REG_DRATE  = 3,  // Register adress: 03h, Reset value: F0H
	REG_IO     = 4,  // Register adress: 04h, Reset value: E0H
	REG_OFC0   = 5,  // Register adress: 05h, Reset value: xxH
	REG_OFC1   = 6,  // Register adress: 06h, Reset value: xxH
	REG_OFC2   = 7,  // Register adress: 07h, Reset value: xxH
	REG_FSC0   = 8,  // Register adress: 08h, Reset value: xxH
	REG_FSC1   = 9,  // Register adress: 09h, Reset value: xxH
	REG_FSC2   = 10, // Register adress: 0Ah, Reset value: xxH
};

//  This commands control the operation of the ADS1256. 
//	All of the commands are stand-alone except for the register reads and writes 
//	(RREG, WREG) which require a second command byte plus data.
//	CS must stay low (CS_0()) during the entire command sequence.
enum
{
	CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode
	CMD_RDATA    = 0x01, // Read Data
	CMD_RDATAC   = 0x03, // Read Data Continuously
	CMD_SDATAC   = 0x0F, // Stop Read Data Continuously
	CMD_RREG     = 0x10, // Read from REG - 1st command byte: 0001rrrr 
						 //					2nd command byte: 0000nnnn
	CMD_WREG     = 0x50, // Write to REG  - 1st command byte: 0001rrrr
						 //					2nd command byte: 0000nnnn
						 // r = starting reg address, n = number of reg addresses
	CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration
	CMD_SYSOCAL  = 0xF3, // System Offset Calibration
	CMD_SYSGCAL  = 0xF4, // System Gain Calibration
	CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion
	CMD_STANDBY  = 0xFD, // Begin Standby Mode
	CMD_RESET    = 0xFE, // Reset to Power-Up Values
};

// Input analog channels.
enum
{
	AIN0   = 0, //Binary value: 0000 0000
	AIN1   = 1, //Binary value: 0000 0001
	AIN2   = 2, //Binary value: 0000 0010
	AIN3   = 3, //Binary value: 0000 0011
	AIN4   = 4, //Binary value: 0000 0100
	AIN5   = 5, //Binary value: 0000 0101
	AIN6   = 6, //Binary value: 0000 0110
	AIN7   = 7, //Binary value: 0000 0111
	AINCOM = 8  //Binary value: 0000 1000
};

// Boolean values.
typedef enum
{
	False = 0,
	True  = 1,
} bool;

struct cmplx {
	int r;
	int i;
};

/*
	*******************************
	** PART 1 - serial interface **
	*******************************
	Functions:
		- CS_1()
		- CS_0()
		- RST_1()
		- RST_0()
		- DRDY_LOW()
		- delayus()
		- send8bit()
		- recieve8bit()
		- waitDRDY()
		- initializeSPI()
		- endSPI()
*/

// DRDY (ads1256 data ready output) - used as status signal to indicate when 
// conversion data is ready to be read.  
// Low  - new data avaliable, high - 24 bits are read or new data is being updated.
#define  DRDY		RPI_GPIO_P1_11
// RST (ADS1256 reset output)
#define  RST 		RPI_GPIO_P1_12
// SPICS (ADS1256 chip select) - allows individual selection of a ADS1256 device 
// when multiple devices share the serial bus. 
// Low - for the duration of the serial communication, high - serial interface is reset 
// and DOUT enters high impedance state.
#define	 SPICS		RPI_GPIO_P1_15
// DIN (data input) - send data to ADS1256. When SCLK goes from low to high.
#define  DIN 		RPI_GPIO_P1_19
// DOUT (data output) - read data from ADS1256. When SCLK goes from high to low.
#define  DOUT 		RPI_GPIO_P1_21
// SCLK (serial clock) - used to clock data on DIN and DOUT pins into and out of ADS1256.
// If not using external clock, ignore it.
#define  SCLK 		RPI_GPIO_P1_23
// Set SPICS to high (DOUT goes high).
#define  CS_1()  	bcm2835_gpio_write(SPICS, HIGH)
// Set SPICS to low (for serial communication).
#define  CS_0()  	bcm2835_gpio_write(SPICS, LOW)
// Set RST to high.
#define  RST_1() 	bcm2835_gpio_write(RST, HIGH)
// Set RST to low.
#define  RST_0() 	bcm2835_gpio_write(RST, LOW)
// Returns True if DRDY is low.
#define  DRDY_LOW()	bcm2835_gpio_lev(DRDY)==0


// Delay in microseconds.
void delayus(uint64_t microseconds)
{
	bcm2835_delayMicroseconds(microseconds);
}

// Send 8 bit value over serial interface (SPI).
void send8bit(uint8_t data)
{
	bcm2835_spi_transfer(data);
}

// Recieve 8 bit value over serial interface (SPI).
uint8_t recieve8bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

// Wait until DRDY is low.
void waitDRDY(void)
{
	while(!DRDY_LOW()){
		continue;
	}
}

// Initialize SPI, call every time at the start of the program.
// Returns 1 if succesfull!
uint8_t initializeSPI()
{
	if (!bcm2835_init())
	    return -1;
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
	// Spi clock divider: 250Mhz / 256 = 0.97 Mhz ~ between 4 to 10 * 1/freq.clkin.
	// Divider 128 is already more than 4 * 1/freq.clckin so it is not apropriate for usage.
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
	bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP); // Set SPICS pin to output
	bcm2835_gpio_write(SPICS, HIGH);
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);  // Set DRDY pin to input
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP); 
	return 1;
}

// End SPI, call every time at the end of the program.
void endSPI()
{
	bcm2835_spi_end();
	bcm2835_close();
}


/*
	*****************************
	** PART 2 - ads1256 driver **
	*****************************
	Functions:
		- readByteFromReg()
		- writeByteToReg()
		- writeCMD()
		- readChipID()
		- setSEChannel()
		- setDIFFChannel()
		- setPGA()
		- setDataRate()
		- readData()
		- getValSEChannel()
		- getValDIFFChannel()
		- scanSEChannels()
		- scanDIFFChannels()
		- scanSEChannelContinuous()
		- scanDIFFChannelContinuous()
*/

void resetADS()
{
	CS_0();
	send8bit(CMD_RESET);
	CS_1();
}

// Read 1 byte from register address registerID. 
// This could be modified to read any number of bytes from register!	
uint8_t readByteFromReg(uint8_t registerID)
{
	CS_0();
	send8bit(CMD_RREG | registerID); // 1st byte: address of the first register to read
	send8bit(0x00); 				 // 2nd byte: number of bytes to read = 1.
	
	delayus(7); 	// min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec
	uint8_t read = recieve8bit();
	CS_1();
	return read;
}

// Write value (1 byte) to register address registerID.
// This could be modified to write any number of bytes to register!
void writeByteToReg(uint8_t registerID, uint8_t value)
{
	CS_0();
	send8bit(CMD_WREG | registerID); // 1st byte: address of the first register to write
	send8bit(0x00); 				 // 2nd byte: number of bytes to write = 1.
	send8bit(value);				 // 3rd byte: value to write to register
	CS_1();
}

// Send standalone commands to register.
uint8_t writeCMD(uint8_t command)
{
	CS_0();
	send8bit(command);
	CS_1();
}

// Set the internal buffer (True - enable, False - disable).
uint8_t setBuffer(bool val)
{
	CS_0();
	send8bit(CMD_WREG | REG_STATUS);
	send8bit((0 << 3) | (1 << 2) | (val << 1));
	CS_1();
}

// Get data from STATUS register - chip ID information.
uint8_t readChipID(void)
{
	waitDRDY();
	uint8_t id = readByteFromReg(REG_STATUS);
	return (id >> 4); // Only bits 7,6,5,4 are the ones to read (only in REG_STATUS) - return shifted value!
}

// Write to MUX register - set channel to read from in single-ended mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setSEChannel(uint8_t channel)
{
	writeByteToReg(REG_MUX, channel << 4 | 1 << 3); // xxxx1000 - AINp = channel, AINn = AINCOM
}

// Write to MUX register - set channel to read from in differential mode.
// Bits 7,6,5,4 determine the positive input channel (AINp).
// Bits 3,2,1,0 determine the negative input channel (AINn).
void setDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	writeByteToReg(REG_MUX, positiveCh << 4 | negativeCh); // xxxx1000 - AINp = positiveCh, AINn = negativeCh
}

// Write to A/D control register - set programmable gain amplifier (PGA).
// CLKOUT and sensor detect options are turned off in this case.
void setPGA(uint8_t pga)
{
	writeByteToReg(REG_ADCON, pga); // 00000xxx -> xxx = pga 
}

// Write to A/D data rate register - set data rate.
void setDataRate(uint8_t drate)
{
	writeByteToReg(REG_DRATE, drate);
}

// Read 24 bit value from ADS1256. Issue this command after DRDY goes low to read s single
// conversion result. Allows reading data from multiple different channels and in 
// single-ended and differential analog input.
int32_t readData(void)
{
	uint32_t read = 0;
	uint8_t buffer[3];
	waitDRDY();
	CS_0();

	send8bit(CMD_RDATA);
	delayus(7); // min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec

	buffer[0] = recieve8bit();
	buffer[1] = recieve8bit();
	buffer[2] = recieve8bit();
	// DRDY goes high here

	// construct 24 bit value
	read =  ((uint32_t)buffer[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buffer[1] << 8);
	read |= buffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;
	}

	CS_1();

	return (int32_t)read;
}

// Get one single-ended analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getValSEChannel(uint8_t channel)
{
	int32_t read = 0;
/*	setSEChannel(channel); // MUX command
	delayus(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);    // SYNC command
	delayus(3);
	writeCMD(CMD_WAKEUP);  // WAKEUP command
	delayus(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
*/
	read = readData();
	return read;
}

// Get one differential analog input value by issuing command to input multiplexer.
// It reads a value from previous conversion!
// DRDY needs to be low!
int32_t getValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	int32_t read = 0;
	setDIFFChannel(positiveCh, negativeCh);
	delayus(3); // min delayus: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);
	delayus(3);
	writeCMD(CMD_WAKEUP);
	delayus(1); // min delayus: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData();
	return read;
}

// Get one single-ended analog input value from input channels you set (min 1, max 8).
void scanSEChannels(uint8_t channels[], uint8_t numOfChannels, int n, double values[][n], int curSample)
{
	/*for (int i = 0; i < numOfChannels; ++i){
		waitDRDY();
		values[i][curSample] = (double)getValSEChannel(channels[i])/1670000;
	}*/
	waitDRDY();
	values[0][curSample] = (double)readData()/1670000;
}

// Get one differential analog input value from input channels you set (min 1, max 4).
void scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, uint32_t *values)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY();
		values[i] = getValDIFFChannel(positiveChs[i], negativeChs[i]);
	}
}

// Continuously acquire analog data from one single-ended analog input.
// Allows sampling of one single-ended input channel up to 30,000 SPS.
void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 8;

	// Set single-ended analog input channel.
	waitDRDY();
	setSEChannel(channel);
	delayus(1);
	send8bit(CMD_SYNC);
	delayus(3);
	send8bit(CMD_WAKEUP);
	delayus(3);
	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

	// Start reading data
	currentTime [numOfMeasure];
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] = (uint32_t)read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		//printf("%8ld\n", (long)values[i]);
		delayus(del);
	}
	
	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}

// Continuously acquire analog data from one differential analog input.
// Allows sampling of one differential input channel up to 30,000 SPS.
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 8;

	// Set differential analog input channel.
	setDIFFChannel(positiveCh, negativeCh);
	delayus(del);

	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	delayus(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

	// Start reading data.
	currentTime [numOfMeasure];	
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = recieve8bit();
		buffer[1] = recieve8bit();
		buffer[2] = recieve8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] =(uint32_t) read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		delayus(del);
	}

	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}

static void dr_msg_cb (rd_kafka_t *kafka_handle,
                       const rd_kafka_message_t *rkmessage,
                       void *opaque) {
    if (rkmessage->err) {
        g_error("Message delivery failed: %s", rd_kafka_err2str(rkmessage->err));
    }
}


/*
	********************************
	** PART 4 - Examples of usage **
	********************************
*/

int main(int argc, char *argv[]){

	// Initialization and AD configuration
	if (!initializeSPI()) return 1;
	resetADS();
	waitDRDY();
	setDataRate(240);
	delayus(3);
	setBuffer(True);
	delayus(3);
	setPGA(PGA_GAIN1);
	delayus(7);
	uint8_t regVal = readByteFromReg(REG_DRATE);
	printf("new data rate: %u\n", regVal);


	if (argc < 2){
		printf("Usage: %s <number of measurements>\n", argv[0]);
		return 1;
	}
	
    	rd_kafka_t *producer;
    	rd_kafka_conf_t *conf;
    	char errstr[512];

    // Parse the command line.
    	if (argc != 3) {
        	g_error("Usage: %s <config.ini>", argv[0]);
        	return 1;
    	}

    	// Parse the configuration.
    	// See https://github.com/edenhill/librdkafka/blob/master/CONFIGURATION.md
    	const char *config_file = argv[2];

    	g_autoptr(GError) error = NULL;
    	g_autoptr(GKeyFile) key_file = g_key_file_new();
    	if (!g_key_file_load_from_file (key_file, config_file, G_KEY_FILE_NONE, &error)) {
        	g_error ("Error loading config file: %s", error->message);
        	return 1;
    	}

    // Load the relevant configuration sections.
    	conf = rd_kafka_conf_new();
    	load_config_group(conf, key_file, "default");

	// Install a delivery-error callback.
	rd_kafka_conf_set_dr_msg_cb(conf, dr_msg_cb);

    // Create the Producer instance.
	producer = rd_kafka_new(RD_KAFKA_PRODUCER, conf, errstr, sizeof(errstr));
        if (!producer) {
        	g_error("Failed to create new producer: %s", errstr);
        	return 1;
    	}

    // Configuration object is now owned, and freed, by the rd_kafka_t instance.
    	conf = NULL;
   // initialize the topic. 1st step in producing data.
	const char *topic = "sensordata";


	/////////////////////////////////
	// Single-ended input channels //
	/////////////////////////////////

	//Creating files to house data//
//	FILE *fptr;
//	int fileCount = 1;
	char measurement[100];
	// num of measurements, vector of channels, 

	clock_t start_SE, end_SE;
	//int num_ch_SE = 8;
	const int num_ch_SE = 1;
	const int num_measure_SE = atoi(argv[1]);
	double values_SE[num_ch_SE][num_measure_SE];
	uint32_t time_SE[num_measure_SE];
	//uint8_t  channels_SE [8] = {AIN0, AIN1, AIN2, AIN3, AIN4, AIN5, AIN6, AIN7};
	uint8_t channels_SE [1] = {AIN2}; // TESTING
	start_SE = clock();
	setSEChannel(channels_SE[0]); // MUX command
	delayus(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);    // SYNC command
	delayus(3);
	writeCMD(CMD_WAKEUP);  // WAKEUP command
	delayus(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	// Fourier Transform Declarations
	int size = 20000*sizeof(char);
	char* string_value = malloc(size);
	const char *key =  "new data";
        size_t key_len = strlen(key);
	int str_len = 0;
	for (int i = 1; i < num_measure_SE + 1; ++i){
		scanSEChannels(channels_SE, num_ch_SE, num_measure_SE, values_SE, i - 1);
		time_SE[i - 1] = clock() - start_SE;
		// removed the zero from the first part of the string.
		snprintf(measurement, 32, "%f %ld\n", values_SE[0][i - 1], time_SE[i - 1]);
		int measure_len = strlen(measurement);
		int new_len = str_len + measure_len;
		if (size <= new_len) {
			string_value = realloc(string_value, size*=2);
			// printf("size being realloced %i, number of iterations is %i", size, i);
		}
		memcpy(&string_value[str_len], measurement, measure_len + 1);
		str_len = new_len;
		
		// float v_out = values_SE[0][i-1]*5.0/0x7fffff;
		// DAC8532_Out_Voltage(channel_B, v_out);
        	// DAC8532_Out_Voltage(channel_A, (3.3 - v_out));

		
		if (!(i % 10000)) {
		        size_t value_len = strlen(string_value);

			rd_kafka_resp_err_t err;

        		err = rd_kafka_producev(producer,
                                RD_KAFKA_V_TOPIC(topic),
                                RD_KAFKA_V_MSGFLAGS(RD_KAFKA_MSG_F_COPY),
                                RD_KAFKA_V_KEY((void*)key, key_len),
                                RD_KAFKA_V_VALUE((void*)string_value, value_len),
                                RD_KAFKA_V_OPAQUE(NULL),
                                RD_KAFKA_V_END);
        		rd_kafka_poll(producer, 0);
			str_len = 0;
		}
/*        	if (err) {
            		g_error("Failed to produce to topic %s: %s", topic, rd_kafka_err2str(err));
            		return 1;
        	} else {
            		g_message("Produced event to topic %s: key = %12s value = %12s", topic, key, value);
        	}
*/






		/*if (!(i % 11000)) {
			fclose(fptr);
			snprintf(newPath, 32, "/home/pi/shared/data_file%i.txt", fileCount);
			fptr = fopen(newPath, "w");
			fileCount++;
		}*/
		//fprintf(tptr, "%ld ", time_SE[i]);
//		cx_in[i].r = values_SE[0][i];
//		cx_in[i].i = values_SE[0][i];
//		kiss_fft(cfg, cx_in, cx_out);
	//printf("%i ||", i+1);
	/*	printf("%i ", i+1);
		for (int ch = 0; ch < num_ch_SE; ++ch)
		{
			//printf(" %fV ||", (double)values_SE[ch]/1670000);

		}
		printf("\n");
	*/

	}
	end_SE = clock();
	free(string_value);
//	kiss_fft_free(cfg);
	/*for (int i = 0; i < num_measure_SE; ++i) {

//		printf(" %f %i\n", cx_out[i].r, time_SE[i]);
	}*/
//	fclose(fptr);
//	fclose(tptr);
	/////////////////////////////////
	// Differential input channels //
	/////////////////////////////////
/*
	clock_t start_DIFF, end_DIFF;
	//int num_ch_DIFF = 4;
	int num_ch_DIFF = 1;
	int num_measure_DIFF = atoi(argv[1]);
	uint32_t values_DIFF [num_ch_DIFF];
	//uint8_t  posChannels [4] = {AIN0, AIN2, AIN4, AIN6};
	//uint8_t  negChannels [4] = {AIN1, AIN3, AIN5, AIN7};
	uint8_t  posChannels [1] = {AIN2}; // TESTING
	uint8_t  negChannels [1] = {AINCOM}; // TESTING

	start_DIFF = clock();
	for (int i = 0; i < num_measure_DIFF; ++i){
		scanDIFFChannels(posChannels, negChannels, num_ch_DIFF, values_DIFF);
		//printf("%i ||", i+1);
		printf("%i ", i+1);
		for (int ch = 0; ch < num_ch_DIFF; ++ch)
		{
			//printf(" %fV ||", (double)values_DIFF[ch]/1670000);
			printf(" %f %i", (double)values_DIFF[ch]/1670000, clock() - start_DIFF);
		}
		printf("\n");
	}
	end_DIFF = clock();

	/////////////////////////////////////////
	// Single-ended input, continuous mode //
	/////////////////////////////////////////

	clock_t start_SE_CONT, end_SE_CONT;

	int num_measure_SE_CONT = atoi(argv[1])*30; // 30x measurements because it works with much higher sample rate

	uint32_t values_SE_CONT [num_measure_SE_CONT];

	uint32_t time_SE_CONT [num_measure_SE_CONT];

	start_SE_CONT = clock();

	scanSEChannelContinuous(AIN2, num_measure_SE_CONT, values_SE_CONT, time_SE_CONT);
	end_SE_CONT = clock();
	//Creating files to house data//
	FILE *fptr;
	FILE *tptr;
	char data_file[25];
	scanf("%s", data_file);
	char time_file[25];
	scanf("%s", time_file);
	fptr = fopen(data_file, "w");
	tptr = fopen(time_file, "w");
	for (int i = 0; i < num_measure_SE_CONT; ++i){
		//printf("%i || %fV\n", i+1, (float)values_SE_CONT[i]/1670000);
	//	printf("%i %f %li\n", i+1, (float)values_SE_CONT[i]/1670000, time_SE_CONT[i]);
		fprintf(fptr, "%f\n",  (float)values_SE_CONT[i]/1670000);
		fprintf(tptr, "%ld\n", time_SE_CONT[i]);
	}
	fclose(fptr);
	fclose(tptr);
	/////////////////////////////////////////
	// Differential input, continuous mode //
	/////////////////////////////////////////

	clock_t start_DIFF_CONT, end_DIFF_CONT;
	int num_measure_DIFF_CONT = atoi(argv[1])*30; // 30x measurements because it works with much higher sample rate
	uint32_t values_DIFF_CONT [num_measure_DIFF_CONT];
	uint32_t time_DIFF_CONT [num_measure_DIFF_CONT];
	start_DIFF_CONT = clock();
	scanDIFFChannelContinuous(AIN2, AINCOM, num_measure_DIFF_CONT, values_DIFF_CONT, time_DIFF_CONT);
	end_DIFF_CONT = clock();
	for (int i = 0; i < num_measure_DIFF_CONT; ++i){
		//printf("%i || %fV\n", i+1, (float)values_DIFF_CONT[i]/1670000);
		printf("%i %f %i\n", i+1, (float)values_DIFF_CONT[i]/1670000, time_DIFF_CONT[i]);
	}
*/


    // Block until the messages are all sent.
	g_message("Flushing final messages..");
   	rd_kafka_flush(producer, 10 * 1000);

    	if (rd_kafka_outq_len(producer) > 0) {
        	g_error("%d message(s) were not delivered", rd_kafka_outq_len(producer));
    	}

    	g_message("%d events were produced to topic %s.", num_measure_SE, topic);

    	rd_kafka_destroy(producer);

	printf("Time for %i single-ended measurements on %i channels is %d microseconds (%5.1f SPS/channel).\n", num_measure_SE, num_ch_SE, end_SE - start_SE, (double)(num_measure_SE)/(end_SE - start_SE)*1e6);
//	printf("Time for %i differential measurements on %i channels is %d microseconds (%5.1f SPS/channel).\n", num_measure_DIFF, num_ch_DIFF, end_DIFF - start_DIFF, (double)num_measure_DIFF/(end_DIFF - start_DIFF)*1e6);
//	printf("Time for %i single-ended measurements in continuous mode is %d microseconds (%5.1f SPS).\n", num_measure_SE_CONT, end_SE_CONT - start_SE_CONT, (double)num_measure_SE_CONT/(end_SE_CONT - start_SE_CONT)*1e6);
//	printf("Time for %i differential measurements in continuous mode is %d microseconds (%5.1f SPS).\n", num_measure_DIFF_CONT, end_DIFF_CONT - start_DIFF_CONT, (double)num_measure_DIFF_CONT/(end_DIFF_CONT - start_DIFF_CONT)*1e6);


	endSPI();
	return 0;
}
