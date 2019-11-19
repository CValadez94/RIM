#include <FlexCAN.h>        // !!! Do not use FlexCAN that comes with teensy program. Must used
                            // FlexCAN_Library_CollinK. Replace the default FlexCAN with that one.
#include <kinetis_flexcan.h>
#include <stdlib.h>
#include <Metro.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
#include <SPI.h>
#include <SdFat.h>
#include <TimeLib.h>
#include <genieArduino.h>

/** Remote Input Module
*   by Cristian Valadez
*   Version 1.0	October 2019
*/ 

/** The RIM is configurable to receive analog/digital, I2C and SPI inputs. Data is converted and
*   broadcasted via CAN bus to ECU and possibly other moduels. Refer to FSAE CAN protocol.
*   Main usage:
*   - MEMS sensor BNO055 data is sent to ECU
*   - All available data is sent to XBEE module through serial1 port
*   - Select data is sent to 4D data display through serial2 port
*     (code and setup for display courtesy of Corey Judkins)
*
*   The RIMCONFIG.txt file is used to configure the RIM. File must exist in SD and
*   be properly setup, else defaults will be used.
*
*   MODE
*   0 = Release MODE; don't display messages on port, logging enabled
*   1 = Logging MODE; display only debug messages, logging enabled
*   2 = Sniffer MODE; display all CAN and debug messages, logging disabled
*   3 = Debug MODE; display only debug messages, logging disabled
*/

// Global variables and constructors
bool _log = false;              // For logging logic
bool _log_old = false;
bool _dbLog1 = false;           // For debouncing log enable switch
bool _dbLog2 = false;           // For debouncing log enable switch
bool _ECTWarn = false;          // ECT Warning Lamp
bool _ECTWarn_old = false;
bool _EOPWarn = false;          // EOP Warning Lamp
bool _EOPWarn_old = false;
bool _fileOpen = false;         // For logging logic
static CAN_message_t txmsg, RIMmsg;     // CAN messages
static CAN_message_t bufmsg[2]; // Buffered messages
const int ledPin = 13;          // For LED
#define logSS 2                 // For logging start/stop digital input
const size_t LINE_MAX = 75;     // Maximum line length from config file
char line[LINE_MAX];            // Line that will be pulled from config file
const uint8_t chipSelect = 10;  // For SD card
int inc = 0;                    // Counter
int _mapAnalog[12] = {			// Analog pin mapping, see datasheet
  0, 2, 3, 8, 9, 10, 26, 27, 28, 29, 30, 31};
int8_t preAnIn_10Hz[13];        // Store which analog inputs to be logged at 10Hz
int8_t preAnIn_50Hz[13];        // Store which analog inputs to be logged at 50Hz
int16_t buffer_v[12] = {	    // Temporary 2 byte buffer vector size 12
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t buffer_XBEE[17] = {	    // XBEE buffer, see CAN EL129 UIC protocol
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t buffer = 0;            // 2 byte buffer
uint32_t longfileName;          // File name, unsigned 32 bits
String fileName;                // Log file name
char _fileName[13];             // Log file name (char array)
const uint8_t _blockSize = 20;  // Number of lines to store in memory before it is sent to SD card
String bufferSD[_blockSize];    // Buffer of log stored in memory that gets sent to SD card
uint8_t _blockCnt = 0;          // Line counter
bool configRead = false;		// Config file read status
bool configRead_old = configRead;
bool sdWriteError = false;      // SD card write error status
bool sdWriteError_old = sdWriteError;
const int displayReset = A4;    // For display TODO: Necessary? For now use pin A4 which is used for
                                // SDA (BNO055). Will need to change this after when BNO055 is used
double voltage = 0;             // Battery voltage
double speed = 0;               // Wheel based vehicle speed
double rpm = 0;                 // Engine speed
double ect = 0;                 // Engine coolant Temperature
double eop = 0;                 // Engine oil pressure
uint8_t gear = 0;               // Gear estimate
double tps = 0;                 // Throttle position
double iat = 0;                 // Intake air temperature

// Objects
Metro t50Hz = Metro(20);				// 20 ms = 50 Hz
Metro log_timer = Metro(100);				// 100 ms = 10 Hz
Metro disp_timer = Metro(200);			// Display transmit delay timer
Metro XBEE_timer = Metro(220);			// XBEE transmit delay timer
Adafruit_BNO055 bno = Adafruit_BNO055(55);	// BNO055 object
imu::Vector < 3 > linearAccelData;      // BNO055 linear acceleration data
int16_t xLinAccel, yLinAccel, zLinAccel;// BNO055 x,y,z linear acceleration data
SdFat sd;
SdFile configFile;
SdFile file;
Genie genie;

// Struct to hold configuration of each analog channel
typedef struct {
  bool en;
  uint8_t freq;
  uint8_t resBits;
  uint32_t canId;
  uint8_t canIdStart;
} AnalogSettings;

// Default parameters
uint8_t MODE = 3;
bool EN_XBEE = true;
bool EN_BNO055 = true;
bool EN_Display = true;
AnalogSettings analogSettings[12] = {{false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0},
  {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0},
  {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0},
  {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0 }, {false, 10, 8, 0x5AE99, 0},
  {false, 10, 8, 0x5AE99, 0},
};

unsigned long xbeeTime, displayTime;
uint16_t CANRXcounter = 0;

// CAN listener class - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class ListenerClass: public CANListener {
  public:
    void printFrame(CAN_message_t &frame, int mailbox);
    void logCANFrameRX(CAN_message_t &frame, int mailbox);
    void sendFrameXBee(CAN_message_t &frame, int mailbox);
    void sendFrameDisplay(CAN_message_t &frame, int mailbox);
    bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};

bool ListenerClass::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller) {
    if (frame.id == 0x5AE01){
        bufmsg[0] = frame;
        CANRXcounter++;
    } else if (frame.id == 0x5AE02){
        bufmsg[1] = frame;
        CANRXcounter++;
    }
  return true;
}

/**
    Log the CAN frame that was received by the RIM. Build the string starting with the CAN ID,
    then following the data separated by commas
*/
void ListenerClass::logCANFrameRX(CAN_message_t &frame, int mailbox) {
  bufferSD[_blockCnt] = String(millis());
  bufferSD[_blockCnt] += ',';
  bufferSD[_blockCnt] += String(frame.id);
  for (int c = 0; c < frame.len; c++) {
    bufferSD[_blockCnt] += ',';
    bufferSD[_blockCnt] += String(frame.buf[c]);
  }
  //bufferSD[_blockCnt] += '\r';        // Don't need. Serial1.println already provides the \r
  _blockCnt++;                      // Increment global counter
}

void ListenerClass::printFrame(CAN_message_t &frame, int mailbox) {
  Serial.print("_log: "); Serial.print(_log); Serial.print(" Mailbox: ");
  Serial.print(mailbox);
  Serial.print(" "); Serial.print(millis());
  Serial.print(" ID: ");
  Serial.print(frame.id, HEX);
  Serial.print(" Data: ");
  for (int c = 0; c < frame.len; c++) {
    Serial.print(frame.buf[c], HEX);
    Serial.write(' ');
  }
  Serial.write('\n');
}

/**
	Store the frame received in the XBEE buffer. Refer to CAN EL129 UIC protocol
	for mapping. The frame needs to be sorted and data placed into the correct
	array location. This is done to facilitate live streaming in software MakerPlot
*/
void ListenerClass::sendFrameXBee(CAN_message_t &frame, int mailbox) {
  if (frame.id == 0x5AE01) {
    for (uint8_t i = 0; i < 8; i++) {
      buffer_XBEE[i] = frame.buf[i];
    }
  } else if (frame.id == 0x5AE02) {
	  for (uint8_t i = 0; i < 6; i++) {
		  buffer_XBEE[i+8] = frame.buf[i];
	  }
  }
}

/**
 * Get the required signals and update the respective variables. Serial data is actually sent in the
 * loop(), or else serial would be overloaded since CAN coming in too fast. Also the conversions are
 * done when the serial is sent since the variabel may get overriden many times before it is actually
 * sent.
 * 
 * TODO: Investigate, the display probably could do the processing after it gets the raw data
 * from serial. This will alleviate RIM CPU usage.
 */
void ListenerClass::sendFrameDisplay(CAN_message_t &frame, int mailbox) {
    // Refer to CAN EL129 UIC protocol
    if (frame.id == 0x5AE01) {
        rpm = frame.buf[0]*255 + frame.buf[1];
        iat = frame.buf[2];
        ect = frame.buf[4];
        speed = frame.buf[5];
        voltage = frame.buf[7];
    } else if (frame.id == 0x5AE02) {
        eop = frame.buf[3];
        gear = frame.buf[5];
    }
}

ListenerClass listenerclass;        // CAN listener object

void setup(void) {
    delay(2000);
  // Initialize the digital pins
  pinMode(ledPin, OUTPUT);
  pinMode(logSS, INPUT);          // Digital input that starts/stops logging
  pinMode(logSS, INPUT_PULLUP);   // Setup as pullup; Open = 1, Grounded = 0

  // Initialize serial port if in a debugging MODE
  if (MODE) {               // True if > 0
    digitalWrite(ledPin, HIGH);
    Serial.begin(38400);     // Start serial if in debug MODE
    delay(1000);
  }

  // Initialize SD card
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    dprintln("Could not initialize SD card");
    // Default configuration is used!!
    // configRead remains false
  } else {
    dprintln("SD card initialized");
    /*  Get logger preferences from RIMCONFIG.txt
     * If a line starts with '#', it is skipped. If line does not start with '#', it is
     * parsed and the corresponding configuration default is overriden. */
    if (!configFile.open("RIMCONFIG.TXT", O_READ)) {
      dprintln("Could not open or find file");
    } else {
      size_t n;
      int equalsInd = -1;
      String key;
      String value;
      while ((n = configFile.fgets(line, sizeof(line))) > 0) {
        // Parse lines
        if (line[0] != '#') {           // Skip line if it starts with #
          equalsInd = -1;             // Reset the equals sign index
          for (uint8_t i = 0; i < sizeof(line); i++) {
            if (line[i] == '=') {
              equalsInd = i;      // Found the '='
              i = sizeof(line);   // End the search for '='
            }
          }
          if (equalsInd != -1) {      // -1 indicates '=' not found.. skip
            String lineStr = (String)line;          // Convert char[] to string
            key = lineStr.substring(0, equalsInd);     // Get key
            value = lineStr.substring(equalsInd + 1, n - 1); // Get value

            // Adjust the configurations based on key and value
            adjustConfiguration(key, value);
          }
        }
      }
      configFile.close();       // Close file
      configRead = true;        // Update status

      // Print each configuration; for debugging
      dprintln("** FINAL CONFIG VALUES **");
	  dprint("MODE: "); dprintln(MODE);
      dprint("EN_XBEE: "); dprintln(EN_XBEE);
      dprint("EN_BNO055: "); dprintln(EN_BNO055);
	  dprint("EN_Display: "); dprintln(EN_Display);
      for (int i = 0; i < 12; i++) {
        dprint("A"); dprint((i + 1)); dprint(": ");
        dprint(analogSettings[i].en); dprint(", ");
        dprint(analogSettings[i].freq); dprint(", ");
        dprint(analogSettings[i].resBits); dprint(", ");
        dprint(analogSettings[i].canId); dprint(", ");
        dprintln(analogSettings[i].canIdStart);
      }
      dprintln("** ** ** ** ** ** ** **");
    }
  }

  // Initialize the display
  if (EN_Display) {
	  dprint("Initializing display ... ");
      Serial3.begin(38400);
      delay(1000);
      genie.Begin(Serial3);
      delay(50);
      // TODO: NOTE: Commented out the display reset since we need that pin for BNO055
      /*pinMode(displayReset, OUTPUT);       // For display reset
      digitalWrite(displayReset, 0);       // Reset display
      delay(100);
      digitalWrite(displayReset, 1);       // Remove reset trigger*/
      delay(3500);                      // Let display startup after reset (important)
	  dprintln("done.");
	  dprint("Checking if display is even connected...");
	  if (checkDisplayStatus()){
		  dprintln("nope. Disabling display.");
		  EN_Display = false;
	  } else {
		  dprintln("yes it is.");
	  }	  
  } else { dprintln("Display not initialized");}
  
  // Initialize the XBEE
  if (EN_XBEE) {
      // TODO: Check if a faster clock time would be better
    Serial1.begin(38400);	// Start serial for XBee
    delay(1000);
    dprintln("Communication to XBee initialized");
  } else {
    dprintln("XBee disabled");
  }

  // Initialize the BNO055 sensor
  if (EN_BNO055) {
    if (!bno.begin()) {
      EN_BNO055 = false;      // Disable it
      delay(1000);
      dprintln("No BNO055 detected");
    } else {
      dprintln("BNO055 initialized");
      delay(1000);
      setBNO055Calibration();		// Set calibration
      delay(2000);
      bno.setExtCrystalUse(true);
      // TODO: Allow user to choose op mode?
      //bno.setMode(bno.OPERATION_MODE_ACCONLY);      // Set mode to OPERATION_MODE_ACCONLY
      // Default to NDOF mode (fusion) so that we can access computed Linear Acceleration Data
      xLinAccel = -32768;         // Initialize to bogus values, easier to debug
      yLinAccel = -32768;
      zLinAccel = -32768;
    }
  } else {
    dprintln("BNO055 sensor disabled");
  }

  // Initialize CAN bus
  Can0.begin(500000);               // Start CANbus
  Can0.attachObj(&listenerclass);   // Attach object to CANbus
  CAN_filter_t allPassFilter;       // Create an all pass filter using extended bit
  allPassFilter.id = 0;
  allPassFilter.ext = 1;            // Crucial
  allPassFilter.rtr = 0;

  // Default filter on first 4 mailboxes; all pass filter on rest
  for (uint8_t filterNum = 4; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }
  for (uint8_t filterNum = 0; filterNum < 16; filterNum++) {
    listenerclass.attachMBHandler(filterNum);
  }
  dprintln("CAN communication initialized");

  // Setup tx CAN message
  txmsg.ext = 1;		// Extended
  txmsg.len = 8;		// Default data length is 8 bytes
  
  // Setup RIM heartbeat message
    RIMmsg.ext = 1;
    RIMmsg.len = 8;
    RIMmsg.id = 0x5AE13;
    RIMmsg.buf[0] = 0x0000;
    RIMmsg.buf[1] = 0x0000;
    RIMmsg.buf[2] = 0x0000;
    RIMmsg.buf[3] = 0x0000;
    RIMmsg.buf[4] = 0x0000;
    RIMmsg.buf[5] = 0x0000;
    RIMmsg.buf[6] = 0x0000;
    RIMmsg.buf[7] = 0x0000;
	
	// Update configRead signal on RIM heartbeat message
    if (configRead) {
      RIMmsg.buf[0] = RIMmsg.buf[0] || B00000010;   // Set bit
    } else {
      RIMmsg.buf[0] = RIMmsg.buf[0] && B11111101;   // Clear bit
    }

  // Pre process analog inputs
  uint8_t j = 0;                      // Temporary counter j
  uint8_t k = 0;                      // Temporary counter k
  for (uint8_t i = 0; i < 12; i++) {
    if (analogSettings[i].en) {
      if (analogSettings[i].freq == 50) {
        preAnIn_50Hz[j] = i; // Store analog # so we correctly process it during 50Hz timer
        j += 1;                         // Increment counter
      } else {
        preAnIn_10Hz[k] = i; // Store analog # so we correctly process it during 10Hz timer
        k += 1;                         // Increment counter
      }
    }
  }
  preAnIn_50Hz[j] = -1;           // Add 'end of array' marker to next element
  preAnIn_10Hz[k] = -1;           // Add 'end of array' marker to next element
  
  // Make sure status indicators are initially all false
  genie.WriteObject(GENIE_OBJ_USER_LED,0,0);
  genie.WriteObject(GENIE_OBJ_USER_LED,1,0);
  genie.WriteObject(GENIE_OBJ_USER_LED,2,0);
  genie.WriteObject(GENIE_OBJ_USER_LED,3,0);
  configRead_old = false;       // Initialize the last known values
  _log_old = false;
  
  dprintln("** ** ** ** ** ** ** **");
  t50Hz.reset();			// Resets timers
  log_timer.reset();
  XBEE_timer.reset();
  disp_timer.reset();
}

void loop(void) {
  if (t50Hz.check()) {  // 50 Hz timer
    processAnalogInputs(preAnIn_50Hz); // Get and transmit analog input data at 50Hz
    if (EN_BNO055) {        // Get and transmit data if BNO055 is enabled
      processBNO055_accOnly();
    }
    
    if (_log && (_blockCnt >= _blockSize)) {    // Should only ever read =, but just in case check >=
      // Force data to SD and update directory entry to avoid data loss
      if (!file.sync() || file.getWriteError()) {
        dprintln("SYNC ERROR!!!!!!!!!");        // Sync error
        sdWriteError = true;    // SD card write error
        if (_fileOpen) {		// Close the file if it was created just to be sure
          file.close();
          _fileOpen = false;
        }
        // Try to initialize SD again
        if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
          dprintln("Tried to re-initialize SD card but failed...");
        } else {
          dprintln("SD card has been re-initialized.");
          sdWriteError = false;    // SD card write error reset
        }
      } else {          // No sync/write errors, proceed to send next buffer to SD card
          for (int i = 0; i < _blockSize; i++) {
              file.println(bufferSD[i]);
          }
          _blockCnt = 0;                  // Reset counter
        }
    }
  }

  if (log_timer.check()) {     // 10 Hz timer
      CANRXcounter = 0;     // Reset counter
      
    processAnalogInputs(preAnIn_10Hz); // Get and transmit analog input data at 10Hz

    // Check digital input for logging start/stop
	if (digitalReadFast(logSS)) {     // Extremely fast, using bitmasking
        _dbLog1 = true;        // Instantaneous switch state
	} else {_dbLog1 = false;}  // Instantaneoud switch state

	if (_dbLog1 && _dbLog2) {_log = _dbLog2;}   // Debounced (2 10Hz cycles = 200ms) log state
	_dbLog2 = _dbLog1;         // Delayed switch state
	_log = _dbLog2;            // Update

    // Log if in appropriate mode and log start switch enabled
    if (_log && (MODE < 2)) {
      if (!_fileOpen) {           // If not already, create file name; open file
        // TODO: Figure out how to create file with longer name to avoid this HEX stuff
        // File naming algorithm
        // Ex: 1212153020 = December 12, 15:30::20 (483FFCAB in hex)
        // This number is converted to hex since max FAT32 file naming size = 8.3
        longfileName = month() * 100000000 + day() * 1000000 + // Month and day
                       hour() * 10000 + minute() * 100 + second(); // Hour, minute and second

        // Convert to hex and create file name, pad with 0 if necessary
        if (month() > 2) { // March produces 8 chars while anything before produces 7 chars
          fileName = String(longfileName, HEX) + ".csv";
        } else {
          fileName = '0' + String(longfileName, HEX) + ".csv";
        }

        // Convert to char array
        fileName.toCharArray(_fileName, 13);
        // Create file on SD card
        if (!file.open(_fileName, O_CREAT | O_WRITE | O_EXCL)) {
          dprintln(("SD create file error"));   // File wasn't created
          sdWriteError = true;		            // SD card write error flag
          // Try to initialize SD again
          if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
            dprintln("Tried to re-initialize SD card but failed...");
          } else {
            dprintln("SD card has been re-initialized.");
            sdWriteError = false;               // Reset error flag
          }
        } else {
          dprint("File created. Logging to: "); dprintln(fileName);
          _fileOpen = true;
        }
      }
    } else {
      // Close file if it was opened
      if (_fileOpen) {
        file.close();
        _fileOpen = false;
        dprintln("File closed.");
        dprintln("** ** ** ** ** ** ** **");
      }
    }

    // Update the RIM heartbeat message
    if (_log) {
      RIMmsg.buf[0] = RIMmsg.buf[0] || B00000001;   // Set bit
    } else {
      RIMmsg.buf[0] = RIMmsg.buf[0] && B11111110;   // Clear bit
    }
    if (sdWriteError) {
      RIMmsg.buf[0] = RIMmsg.buf[0] || B00000100;   // Set bit
    } else {
      RIMmsg.buf[0] = RIMmsg.buf[0] && B11111011;   // Clear bit
    }

    Can0.write(RIMmsg);                             // Transmit RIM heartbeat on CAN bus
    if (_log && _fileOpen) {                        // Finally LOG here
		logCANFrameTX(bufmsg[0]);
		logCANFrameTX(bufmsg[1]);
		//logCANFrameTX(RIMmsg);   // Don't really care to log this
	}
  }
  
  if (XBEE_timer.check()) {
	  if (EN_XBEE){
        transmitXBEE();                  // Transmit data to XBEE now
    }    
  }
  
  if (disp_timer.check()) {
	  if (EN_Display) transmitDisplay();                // Transmit data to display now
  }
}

/**
	Transmit buffered data to XBEE through the hardware serial port.
*/
void transmitXBEE() {
    Serial1.print((bufmsg[0].buf[0]*255)+bufmsg[0].buf[1]);
  for (uint8_t i = 1; i < 7; i++) {
    Serial1.print(','); Serial1.print(bufmsg[0].buf[i+1]);
  }
  
  
    Serial1.print(','); Serial1.print(bufmsg[1].buf[3]);
  
  
	Serial1.print(','); Serial1.print(RIMmsg.buf[0]&B00000001);
	Serial1.print(','); Serial1.print(RIMmsg.buf[0]&B00000010>>1);
  Serial1.print('\r');
}

/**
 * Transmit the data to the display. Conversions are required before sending.
 * TODO: See if display can do the conversions instead to reduce RIM CPU usage.
 */ 
void transmitDisplay() {
    // See CAN EL129 UIC protocol for conversions
    // Refer to CAN EL129 UIC protocol
    // TODO: fix rounding errors in a fast way
    rpm = double(bufmsg[0].buf[0]*255 + bufmsg[0].buf[1]) * 0.3906241/100; // RPM, scaled down by 100
    tps = double(bufmsg[0].buf[2])*0.39062;             // TPS [%]
    iat = double(bufmsg[0].buf[3])*1.8 + 32;            // IAT [F]
    ect = double(bufmsg[0].buf[4])*1.8 + 32;            // ECT [F]
    speed = double(bufmsg[0].buf[5]);                   // Speed [MPH]
    voltage = double(bufmsg[0].buf[7]);                 // Battery voltage [V]
    eop = double(bufmsg[1].buf[3])*0.580151;            // EOP [PSIg]
    //gear = bufmsg[1].buf[5];                   // Estimated gear
    
    // Check thresholds
    if (eop <= 5) _EOPWarn = true;
    else _EOPWarn = false;
    if (ect >= 245) _ECTWarn = true;
    else _ECTWarn = false;
    
    // Write to dashboard display gauges
    genie.WriteObject(GENIE_OBJ_GAUGE,0,rpm);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,0,tps);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,1,voltage*6.28477); // Scaled up by 100
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,2,rpm);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,3,ect);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,4,eop);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,5,iat);
    
    // Speed optomization: Slow changing values updated based on last known value
	if (_log_old != _log) {        // Only update if different from last known value
		if (_log) genie.WriteObject(GENIE_OBJ_USER_LED,1,1);
        else genie.WriteObject(GENIE_OBJ_USER_LED,1,0);
        _log_old = _log;
	}
	
	if (configRead_old != configRead) {
		if (configRead) genie.WriteObject(GENIE_OBJ_USER_LED,0,1);
		else genie.WriteObject(GENIE_OBJ_USER_LED,0,0);
		configRead_old = configRead;
	}
	
	if (_ECTWarn_old != _ECTWarn) {        // Engine Coolant Temperature Warning Lamp     
        if (_ECTWarn) genie.WriteObject(GENIE_OBJ_USER_LED,3,1);
        else genie.WriteObject(GENIE_OBJ_USER_LED,3,0);
        _ECTWarn_old = _ECTWarn;
    }
    
	if (_EOPWarn_old != _EOPWarn) {        // Engine Oil Pressure Warning Lamp
        if (_EOPWarn) genie.WriteObject(GENIE_OBJ_USER_LED,2,1);
        else genie.WriteObject(GENIE_OBJ_USER_LED,2,0);
        _EOPWarn_old = _EOPWarn;
    }
}

void processAnalogInputs(int8_t ary[13]) {
  for (int i = 0; i < 13; i++) {
    if (ary[i] != -1) {
      uint8_t resolution = analogSettings[ary[i]].resBits;
      uint8_t startByte = analogSettings[ary[i]].canIdStart;
      analogReadResolution(resolution);             // Set analog read resolution
      txmsg.id = analogSettings[ary[i]].canId;      // Set CAN ID
      if (resolution <= 8) {                        // Only 1 byte required
        // TODO: Replace analogRead with a faster version
        txmsg.buf[startByte] = analogRead(_mapAnalog[ary[i]]);
      } else if (resolution <= 16) {                // 2 bytes required
        // TODO: Replace analogRead with a faster version
        buffer = analogRead(_mapAnalog[ary[i]]);
        txmsg.buf[startByte] = buffer >> 8;         // High byte
        txmsg.buf[startByte + 1] = buffer & 0x00FF; // Low byte
      } else {
        // More bytes not supported right now
      }
      Can0.write(txmsg);            // Send data on bus
      if (_log && _fileOpen) logCANFrameTX(txmsg);  // Log frame we are transmitting
      //if (MODE == 2) printCANFrameTX(txmsg);      // Print frame we are transmitting
      //if (EN_XBEE) sendInternalFrameXBee(txmsg);	// Store in the XBEE buffer
    } else {
      break;        // Stop processing when 'end of array' (-1) is found
    }
  }
}

/**
 * Get accelerometer data only; fom the BNO055 sensor
 **/
void processBNO055_accOnly(void) {
    // Get BNO055 vector raw data (int16). It is in 1 mG = 1 LSB
    linearAccelData = bno.getRawVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    xLinAccel = linearAccelData.x();
    yLinAccel = linearAccelData.y();
    zLinAccel = linearAccelData.z();
    
    // TODO: Should we still get the temp data from BNO055 when pulling acceleration
    //       data only?
    
    // Create the CAN message
    txmsg.id = 0x5AE07;   // Accelerometer (X, Y, Z)
    txmsg.buf[0] = xLinAccel >> 8;
    txmsg.buf[1] = xLinAccel & 0x00FF;
    txmsg.buf[2] = yLinAccel >> 8;
    txmsg.buf[3] = yLinAccel & 0x00FF;
    txmsg.buf[4] = zLinAccel >> 8;
    txmsg.buf[5] = zLinAccel & 0x00FF;
	txmsg.buf[6] = 0;
	txmsg.buf[7] = 0;
    Can0.write(txmsg);    // Send on CAN bus*/
    dprint("X: "); dprint(xLinAccel);
    dprint(" mG  Y: "); dprint(yLinAccel);
    dprint(" mG  Z: "); dprint(zLinAccel); dprintln(" mG");
    if (_log && _fileOpen) { logCANFrameTX(txmsg);}     // Store frame in logging buffer
    // TODO: Do we want to send this data to XBEE? I think nah
    
}

void processBNO055(void) {
  // Get BNO055 vector data
  imu::Vector < 3 > euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector < 3 > accel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector < 3 > magne = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector < 3 > gyros = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  buffer_v[0] = euler.x();
  buffer_v[1] = euler.y();
  buffer_v[2] = euler.z();
  buffer_v[3] = accel.x();
  buffer_v[4] = accel.y();
  buffer_v[5] = accel.z();
  buffer_v[6] = magne.x();
  buffer_v[7] = magne.y();
  buffer_v[8] = magne.z();
  buffer_v[9] = gyros.x();
  buffer_v[10] = gyros.y();
  buffer_v[11] = gyros.z();

  txmsg.id = 0x5AE06;   // Temperature, Orientation (X, Y, Z)
  txmsg.buf[0] = bno.getTemp();
  txmsg.buf[1] = buffer_v[0] >> 8;
  txmsg.buf[2] = buffer_v[0] & 0x00FF;
  txmsg.buf[3] = buffer_v[1] >> 8;
  txmsg.buf[4] = buffer_v[1] & 0x00FF;
  txmsg.buf[5] = buffer_v[2] >> 8;
  txmsg.buf[6] = buffer_v[2] & 0x00FF;
  Can0.write(txmsg);    // Send on CAN bus
  if (EN_XBEE) {
    sendInternalFrameXBee(txmsg);  // Send to XBee
  }

  txmsg.id = 0x5AE07;   // Accelerometer (X, Y, Z), Magnetometer (X)
  txmsg.buf[0] = buffer_v[3] >> 8;
  txmsg.buf[1] = buffer_v[3] & 0x00FF;
  txmsg.buf[2] = buffer_v[4] >> 8;
  txmsg.buf[3] = buffer_v[4] & 0x00FF;
  txmsg.buf[4] = buffer_v[5] >> 8;
  txmsg.buf[5] = buffer_v[5] & 0x00FF;
  txmsg.buf[6] = buffer_v[6] >> 8;
  txmsg.buf[7] = buffer_v[6] & 0x00FF;
  Can0.write(txmsg);    // Send on CAN bus
  if (EN_XBEE) {
    sendInternalFrameXBee(txmsg);  // Send to XBee
  }

  txmsg.id = 0x5AE08;   // Magnetometer (Y, Z), Gyroscope (X, Y)
  txmsg.buf[0] = buffer_v[7] >> 8;
  txmsg.buf[1] = buffer_v[7] & 0x00FF;
  txmsg.buf[2] = buffer_v[8] >> 8;
  txmsg.buf[3] = buffer_v[8] & 0x00FF;
  txmsg.buf[4] = buffer_v[9] >> 8;
  txmsg.buf[5] = buffer_v[9] & 0x00FF;
  txmsg.buf[6] = buffer_v[10] >> 8;
  txmsg.buf[7] = buffer_v[10] & 0x00FF;
  Can0.write(txmsg);    // Send on CAN bus
  if (EN_XBEE) {
    sendInternalFrameXBee(txmsg);  // Send to XBee
  }

  txmsg.id = 0x5AE09;   // Gyroscope (Z)
  txmsg.buf[0] = buffer_v[11] >> 8;
  txmsg.buf[1] = buffer_v[11] & 0x00FF;
  Can0.write(txmsg);    // Send on CAN bus
  if (EN_XBEE) {
    sendInternalFrameXBee(txmsg);  // Send to XBee
  }
}

void setBNO055Calibration(void) {
  adafruit_bno055_offsets_t calibrationData;
  calibrationData.accel_offset_x = 17;
  calibrationData.accel_offset_y = -61;
  calibrationData.accel_offset_z = -6;
  calibrationData.mag_offset_x = -1;
  calibrationData.mag_offset_y = -1;
  calibrationData.mag_offset_z = -1;
  calibrationData.gyro_offset_x = -6;
  calibrationData.gyro_offset_y = -1;
  calibrationData.gyro_offset_z = -1;
  calibrationData.accel_radius = 1000;
  calibrationData.mag_radius = 721;
  bno.setSensorOffsets(calibrationData);
}

void sendInternalFrameXBee(CAN_message_t &frame) {
	if (frame.id == 0x5AE13) {
		uint8_t tmp = frame.buf[0];
		for (uint8_t i = 0; i<3; i++){
			buffer_XBEE[i+14] = (tmp>>i) & 0x1;
		}
	}
}


bool checkDisplayStatus() {
	// Note there is a timeout function (TIMEOUT = 1000ms). This was causing the entire program to run
	// extremely slow. Should check if a timeout happens, just disable display since it probably is not connected
	// return false if no issues, true if there is a timeout
	unsigned long time = millis();
    genie.WriteObject(0x0B,0,rpm*0.3906241 / 100);   // RPM scaled down for gauge
	if ((millis() - time) >= 1000) { return true;}
	
	time = millis();
    genie.WriteObject(0x10,0,ect*1.8 + 32);           // Converted to degrees F
	if ((millis() - time) >= 1000) { return true;}
    //delay(10);
	
	time = millis();
    genie.WriteObject(0x07,0,speed);                 // Already in mph
	if ((millis() - time) >= 1000) { return true;}
	
	time = millis();
    genie.WriteObject(0x0F,0,voltage*0.0628477 * 100); // Voltage scaled up for gauge
	if ((millis() - time) >= 1000) { return true;}
	
    //delay(10);
	
	time = millis();
    genie.WriteObject(0x10,1,eop*0.580151);           // Converted to PSI
	if ((millis() - time) >= 1000) { return true;}
	
	time = millis();
    genie.WriteObject(0x0F,1,gear);                     // Already an integer representing gear #
	if ((millis() - time) >= 1000) { return true;}
	
	return false;
}

/**
     Based on key and value extracted from the configuration file edited by user,
     the configuration will be updated from the default. First the key should be
     compared against every possible configurable item, once it there is a match,
     the value will be parsed and the configuration of the key will be updated accordingly.
     @param key, value Strings separated by the "=" in the configuration file
*/
void adjustConfiguration(String key, String value) {
  if (key.equalsIgnoreCase("MODE")) {
    MODE = (uint8_t) value.toInt();			// Convert to uint8_t
  } else if (key.equalsIgnoreCase("EN_XBEE")) {
    EN_XBEE = string2Boolean(value);
  } else if (key.equalsIgnoreCase("EN_BNO055")) {
    EN_BNO055 = string2Boolean(value);
  } else if (key.equalsIgnoreCase("EN_Display")) {
	EN_Display = string2Boolean(value);
  } else {                    // Analog Settings
    String analogValues[5];
    int valueStart = 0;        // Hold index of start of value
    int valueEnd = 0;          // Hold index of end of value
    for (int i = 0; i < 5; i++) {
      valueEnd = value.indexOf(',', valueStart);
      analogValues[i] = value.substring(valueStart, valueEnd);
      valueStart = valueEnd + 1;
    }

    // Go through each analog option to see if there is a match
    if (key.equalsIgnoreCase("A1")) {
      analogSettings[0].en = string2Boolean(analogValues[0]);
      analogSettings[0].freq = string2Freq(analogValues[1]);
      analogSettings[0].resBits = string2Resolution(analogValues[2]);
      analogSettings[0].canId = hexString2CanId(analogValues[3]);
      analogSettings[0].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A2")) {
      analogSettings[1].en = string2Boolean(analogValues[0]);
      analogSettings[1].freq = string2Freq(analogValues[1]);
      analogSettings[1].resBits = string2Resolution(analogValues[2]);
      analogSettings[1].canId = hexString2CanId(analogValues[3]);
      analogSettings[1].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A3")) {
      analogSettings[2].en = string2Boolean(analogValues[0]);
      analogSettings[2].freq = string2Freq(analogValues[1]);
      analogSettings[2].resBits = string2Resolution(analogValues[2]);
      analogSettings[2].canId = hexString2CanId(analogValues[3]);
      analogSettings[2].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A4")) {
      analogSettings[3].en = string2Boolean(analogValues[0]);
      analogSettings[3].freq = string2Freq(analogValues[1]);
      analogSettings[3].resBits = string2Resolution(analogValues[2]);
      analogSettings[3].canId = hexString2CanId(analogValues[3]);
      analogSettings[3].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A5")) {
      analogSettings[4].en = string2Boolean(analogValues[0]);
      analogSettings[4].freq = string2Freq(analogValues[1]);
      analogSettings[4].resBits = string2Resolution(analogValues[2]);
      analogSettings[4].canId = hexString2CanId(analogValues[3]);
      analogSettings[4].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A6")) {
      analogSettings[5].en = string2Boolean(analogValues[0]);
      analogSettings[5].freq = string2Freq(analogValues[1]);
      analogSettings[5].resBits = string2Resolution(analogValues[2]);
      analogSettings[5].canId = hexString2CanId(analogValues[3]);
      analogSettings[5].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A7")) {
      analogSettings[6].en = string2Boolean(analogValues[0]);
      analogSettings[6].freq = string2Freq(analogValues[1]);
      analogSettings[6].resBits = string2Resolution(analogValues[2]);
      analogSettings[6].canId = hexString2CanId(analogValues[3]);
      analogSettings[6].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A8")) {
      analogSettings[7].en = string2Boolean(analogValues[0]);
      analogSettings[7].freq = string2Freq(analogValues[1]);
      analogSettings[7].resBits = string2Resolution(analogValues[2]);
      analogSettings[7].canId = hexString2CanId(analogValues[3]);
      analogSettings[7].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A9")) {
      analogSettings[8].en = string2Boolean(analogValues[0]);
      analogSettings[8].freq = string2Freq(analogValues[1]);
      analogSettings[8].resBits = string2Resolution(analogValues[2]);
      analogSettings[8].canId = hexString2CanId(analogValues[3]);
      analogSettings[8].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A10")) {
      analogSettings[9].en = string2Boolean(analogValues[0]);
      analogSettings[9].freq = string2Freq(analogValues[1]);
      analogSettings[9].resBits = string2Resolution(analogValues[2]);
      analogSettings[9].canId = hexString2CanId(analogValues[3]);
      analogSettings[9].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A11")) {
      analogSettings[10].en = string2Boolean(analogValues[0]);
      analogSettings[10].freq = string2Freq(analogValues[1]);
      analogSettings[10].resBits = string2Resolution(analogValues[2]);
      analogSettings[10].canId = hexString2CanId(analogValues[3]);
      analogSettings[10].canIdStart = analogValues[4].toInt();
    } else if (key.equalsIgnoreCase("A12")) {
      analogSettings[11].en = string2Boolean(analogValues[0]);
      analogSettings[11].freq = string2Freq(analogValues[1]);
      analogSettings[11].resBits = string2Resolution(analogValues[2]);
      analogSettings[11].canId = hexString2CanId(analogValues[3]);
      analogSettings[11].canIdStart = analogValues[4].toInt();
    }
  }
}

/**
    Converts the string "true" or "false" to boolean
    @param str "true" or "false"
    @return the boolean representation of the string
*/
bool string2Boolean(String str) {
  if (str.equalsIgnoreCase("true")) {
    return true;
  } else return false;    // Default is false in case user entered garbage
}

/**
        Converts the string to a supported polling frequency. At this time only 10 and 50Hz allowed
        @param str input value which should be an integer
        @return Either uint8_t 10 or 50
*/
uint8_t string2Freq(String str) {
  if (str.toInt() > 10) {
    return 50;
  } else return 10;       // Default is 10Hz
}

/**
    Convert string to a supported resolution
    @param str String representing resolution of analog channel; cannot be 0 or > 16
    @return Resolution of analog channel
*/
uint8_t string2Resolution(String str) {
  uint8_t res = str.toInt() ;
  if (res == 0) {             // toInt() returns 0 if it could not convert
    return 1;
  } else if (res > 16) {      // Max supported resolution as of now is 16 bits
    return 16;
  } else return res;          // Correct input inside supported range
}

/**
    Convert a string in hex format to an unsigned integer
    @param str String representing the CAN Id in hex, starting with '0x'
    @return unsigned integer representing CAN Id
*/
uint32_t hexString2CanId(String str) {
  uint8_t len = sizeof(str);
  char buffer[len];
  str.toCharArray(buffer, len);       // Convert String to char array
  return strtoul(buffer, NULL, 16);   // Convert to unsigned long int in base 16
}

/**
    Log a CAN frame
*/
void logCANFrameTX(CAN_message_t &frame) {
    if (_blockCnt < _blockSize){        // Overflow catch. This also means overlfow data would be lost
        bufferSD[_blockCnt] = String(millis());
        bufferSD[_blockCnt] += ',';
        bufferSD[_blockCnt] += String(frame.id);
        for (int c = 0; c < frame.len; c++) {
            bufferSD[_blockCnt] += ',';
            bufferSD[_blockCnt] += String(frame.buf[c]);
        }
        //bufferSD[_blockCnt] += '\r';    // Don't need. Serial1.println already provides the \r
        _blockCnt++;                      // Increment global counter
    }
}

void printCANFrameTX(CAN_message_t &frame) {
  Serial.print(millis());
  Serial.print(" ID: ");
  Serial.print(frame.id, HEX);
  Serial.print(" Data: ");
  for (int c = 0; c < frame.len; c++) {
    Serial.print(frame.buf[c], HEX);
    Serial.write(' ');
  }
  Serial.write('\n');
}

void dprintln(String str) {
  // Only print to console if not in Release mode
  if (MODE > 0) {
    Serial.println(str);
  }
}

void dprint(String str) {
  // Only print to console if not in Release mode
  if (MODE > 0) {
    Serial.print(str);
  }
}
