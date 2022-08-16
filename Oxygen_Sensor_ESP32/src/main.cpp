#include <Arduino.h>

/* Oxygen sensor arduino program

1. Read from oxygen sensor model number: AA428-210 O2 CiTiceL (0mV for 0% oxygen, around 9-13mV for 21% Oxygen)
  a. Reading is done through i2c protocol (address 0x48) from ads1115 for two channels.
  two oxygen sensors are connected at once. Data is sent continously at 64 samples per second
  a1. Every time that a sample is ready a trigger line is triggered and the ads1115 is read
  b. I am switching the reading for the two channels
  c. programmalable gain amplifier is set to 16 (giving maximum range of plus minus 0.265 mV)
  d. I will read the raw values and calibrate towards the raw values (adc counts). I can also convert the values to mV.

  I am using ADS1115_WE by Wolfgan Ewald version 1.4.1
  Arduino IDE 2.0.0-rc9.1

2. Display
3. Modbus

*/

#include <SPI.h>
#include <Wire.h>
#include <TimeLib.h>
#include <EEPROM.h>
//#include "ssd1306.h"
//#include "ssd1306_console.h"
#include<ADS1115_WE.h> 
//#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
//#include <ArduinoModbus.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//TwoWire Wire2(PB11, PB10);
//HardwareSerial Serial1(PA10, PA9);
//HardwareSerial Serial2(PA3, PA2);

#define I2C_ADDRESS 0x48
//TwoWire Wire1(PB7, PB6);

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

// Global variables
int interruptPin = 18;
volatile bool convReady = false;
volatile bool debug = true;
const int display_delay_time = 10; // in ms

// ----------------------------------------------------- Draw buttons
  const int NUM_BUTTONS = 4;
  const int BOX_HEIGHT = 13;
  const int BOX_RADIUS = 5;
  const int BOX_SPACING = 5;
  //const int BOX_TEXT_X_OFFSET = 3;
  const int BOX_TEXT_Y_OFFSET = 3;

  struct button_box {
    int x;
    int y;
    int width;
    char text[5];
  };
  struct button_box boxes[NUM_BUTTONS];

// Interrupt routine that is called when a sample is ready and notifies that a sample is ready by setting
// convReady flag to true
void convReadyAlert(){
   convReady = true;
}

// Global serial printing
void SerialPrint(String myString) {
  if(debug) {
    //Serial2.println(myString);
  }
}

int32_t combined(uint16_t low, uint16_t high)
{
  return (static_cast<uint32_t>(high) << 16) + static_cast<uint32_t>(low);
}

/*
void update_time_from_modbus() {

  uint16_t first_date_num, second_date_num;
  time_t num_seconds_since_start;

  // Update date and time from raspberry pi time base
  first_date_num = ModbusRTUServer.holdingRegisterRead(40003);
  second_date_num = ModbusRTUServer.holdingRegisterRead(40004);
  num_seconds_since_start = combined(first_date_num, second_date_num);
  //Serial2.println(num_seconds_since_start);
  setTime(num_seconds_since_start);

}

*/

void WriteValueToScreen(int16_t ch1_value, int16_t ch2_value) {
      //display.clearDisplay();
      display.setTextSize(2);             // Normal 1:1 pixel scale
      
      //display.setTextColor(WHITE);        // Draw white text
      display.setCursor(0, 16);
      display.print("O_1:" + String(ch1_value));
      display.setCursor(0,34);
      display.print("O_2:" + String(ch2_value));
}

void WriteButtonsToScreen() {
  
  for (int i=0; i < NUM_BUTTONS; i++) {
    display.drawRoundRect(i*(display.width())/NUM_BUTTONS, // x pos
                    display.height()-BOX_HEIGHT,              // y pos
                    (display.width()-BOX_SPACING)/NUM_BUTTONS ,   // width rect
                    BOX_HEIGHT,                               // Height rect
                    BOX_RADIUS,                                // Radius
                    SSD1306_WHITE);                   // Color rect
    
    boxes[i].x = i*(display.width())/NUM_BUTTONS;
    boxes[i].y = display.height()-BOX_HEIGHT;
    boxes[i].width = (display.width()-BOX_SPACING)/NUM_BUTTONS;
    
  }
}

void WriteTextInButtons() {
  // --------------------------------------------------  Text in BOXES
  strcpy(boxes[0].text, "Cal");
  strcpy(boxes[1].text, "Info");
  strcpy(boxes[2].text, "");
  strcpy(boxes[3].text, ""); 

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  uint16_t x1, y1, w1, h1;
  for (int i=0; i < NUM_BUTTONS; i++) {
    // get text size
    /*
    display.getTextBounds(boxes[i].text, 
                          boxes[i].x,
                          boxes[i].y + BOX_TEXT_Y_OFFSET,
                           &x1,&y1,&w1,&h1);
    Serial.println(strlen(boxes[i].text));
    */
    int x_pos = (boxes[i].width - (strlen(boxes[i].text) * 6))/2; // find offset to place text in box center
    display.setCursor(boxes[i].x + x_pos,boxes[i].y + BOX_TEXT_Y_OFFSET); // Start at top-left corner
    display.print(boxes[i].text);
  }
  
}

void WriteMessageWarnings() {
  // ----------------------------------------------------- Draw Warning/Messages
  //display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  char message_warning[25];
  display.setCursor(0,0);
  strcpy(message_warning, "Not Calibrated!");
  strcpy(message_warning, "Calibration Expired!");
  strcpy(message_warning, "WiFi Not Connected!");
  strcpy(message_warning, "SCADA Not Connected!");
  strcpy(message_warning, "Next Cal: XXX months");

  display.print(message_warning);
  
}

void setup() {


  // set time
  //setTime(10,9,0,8,8,2022);
  //EEPROM.put(0, now());
  //time_t data;
  //EEPROM.get(0, data);
  //setTime(data);
  
  /*
  // Initialize modbus 
  if (!ModbusRTUServer.begin(2, 9600)) {
    Serial2.println("Failed to start Modbus RTU Server!");
    while (1);
  }
*/
 // configure a single coil at address 0x00
  //ModbusRTUServer.configureCoils(0x00, 1);
  //ModbusRTUServer.configureHoldingRegisters(40001, 4);
  
  // update time for modbus
  //update_time_from_modbus();

  // Initializing SSD1306 display
   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    SerialPrint("display failed");
    for(;;);
  } // Address 0x3D for 128x64
  display.clearDisplay();
  
  //Wire.setClock(100000);
  //Wire.begin();
  //Wire2.setClock(100000);
  //Wire2.begin();
  
  Serial2.begin(9600, SERIAL_8N1);
  Serial2.println(now());

  //SerialPrint("Defining interrupt pin with input pullup");
  pinMode(interruptPin, INPUT_PULLUP);
  //pinMode(PA1, OUTPUT);

  // init the adc, check if there is i2c communication with the ADC
  if(!adc.init()){
    //Serial2.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_0256); // ADS1115_RANGE_0256  ->  +/- 256 mV
  //adc.setVoltageRange_mV(ADS1115_RANGE_6144); // ADS1115_RANGE_0256  ->  +/- 256 mV
  //SerialPrint("Setting adc to 0.256 mV range");
  //SerialPrint("Successful? " + String(adc.getVoltageRange_mV()==256));

  adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1);
  adc.setAlertPinToConversionReady(); //needed for this sketch
  //SerialPrint("ADS1115_ASSERT_AFTER_1  -> after 1 conversion");

  adc.setConvRate(ADS1115_64_SPS); //comment line/change parameter to change SPS
  //SerialPrint("Setting sampling rate to 64 samples per second (lowest noise with highest sampling rate)");
  //SerialPrint("Successful? "  + String(adc.getConvRate()==96));
  
  adc.setMeasureMode(ADS1115_CONTINUOUS);
  //SerialPrint("Setting ADS to send data continously, streaming mode");

  attachInterrupt(digitalPinToInterrupt(interruptPin), convReadyAlert, FALLING);
  //SerialPrint("attaching the interrupt function to a pin number" + String(interruptPin));

  adc.setCompareChannels(ADS1115_COMP_0_1); //comment line/change parameter to change channels
  //SerialPrint("Start with Channel 1");
  
  //SerialPrint("Setup is done");
  Serial2.println("Setup is done!");
}

void loop() {
  
  int16_t value = 0;
  static int counter = 0;
  static bool toggle = false;
  static int ch1_value = 0;
  static int ch2_value = 0;
  const int delay_time = 10; // in ms
  char str_number[6];
  char date_string[100];
  uint16_t voltage_range;

  // poll for Modbus RTU requests
  //ModbusRTUServer.poll();

  // read the current value of the coil
  // int coilValue = ModbusRTUServer.coilRead(0x00);
  
  // if a conversion is ready then read the value from the ADS1115 under certain conditions
  if(convReady) {
    
    counter++;
    convReady = false;

    if (counter%5==4) { // after x samples write variable
      //delay(8); // delay for 8 ms
      //digitalWrite(PA1, LOW);
      value = adc.getRawResult();
      if (toggle) {
        adc.setCompareChannels(ADS1115_COMP_0_1); // differential value between channel1 - channel0
        
        ch2_value = value;
        //ModbusRTUServer.holdingRegisterWrite(40002,ch2_value);
        //digitalWrite(PA1, HIGH);

        //digitalWrite(PA1, LOW);
        
      } else {
        adc.setCompareChannels(ADS1115_COMP_2_3); // differential value between channel2 - channel3
        ch1_value = value;
        //ModbusRTUServer.holdingRegisterWrite(40001,ch1_value);
        
        
        //digitalWrite(PA1, HIGH);
        
        //digitalWrite(PA1, LOW);
      }

      
      display.clearDisplay();
      WriteValueToScreen(ch1_value, ch2_value);
      WriteButtonsToScreen();
      WriteMessageWarnings();
      WriteTextInButtons();
      display.display();
      toggle = !toggle;
      
    }
  }
  
  if(counter % 10 == 9) {
    //update_time_from_modbus();
    //ssd1306_setFixedFont(ssd1306xled_font5x7);
    sprintf(date_string, "%02d/%02d/%04d %02d:%02d:%02d",day(), month(), year(), hour(), minute(), second());
    //ssd1306_printFixedN(4, 0, date_string, STYLE_NORMAL, 0);
  }
}


