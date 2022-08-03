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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AltSoftSerial.h>
#include<ADS1115_WE.h> 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
AltSoftSerial altSerial;

// Global variables
int interruptPin = 3;
volatile bool convReady = false;
volatile bool debug = true;

// Interrupt routine that is called when a sample is ready and notifies that a sample is ready by setting
// convReady flag to true
void convReadyAlert(){
   convReady = true;
}

// Global serial printing
void SerialPrint(String myString) {
  if(debug) {
    altSerial.println(myString);
  }
}

void setup() {
  
  if(debug) {
    //Serial.begin(9600);
    altSerial.begin(9600);
  }
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    SerialPrint("display failed");
    for(;;);
  } // Address 0x3D for 128x64

  display.clearDisplay();
  display.drawPixel(20, 20, WHITE);
  display.display();

  pinMode(interruptPin, INPUT_PULLUP);
  //SerialPrint("Defining interrupt pin with input pullup");

  // init the adc, check if there is i2c communication with the ADC
  if(!adc.init()){
  //  SerialPrint("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_0256); // ADS1115_RANGE_0256  ->  +/- 256 mV
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


  
  SerialPrint("Setup is done");

}

void loop() {
  
  int16_t value = 0;
  static int counter = 0;
  static bool toggle = false;
  static int ch1_value = 0;
  static int ch2_value = 0;

  // if a conversion is ready then read the value from the ADS1115 under certain conditions
  if(convReady) {
    value = adc.getRawResult();
    counter++;
    convReady = false;

    if (counter==8) { // after 8 samples write variable
      if (toggle) {
        adc.setCompareChannels(ADS1115_COMP_0_1); // differential value between channel1 - channel0
        SerialPrint("CH2: " + String(value));
        ch2_value = value;
      } else {
        adc.setCompareChannels(ADS1115_COMP_2_3); // differential value between channel2 - channel3
        SerialPrint("CH1: " + String(value));
        ch1_value = value;
      }

      WriteValueToScreen(ch1_value, ch2_value);
      toggle = !toggle;
         
      counter = 0;    
    }

  }
}


void WriteValueToScreen(int16_t ch1_value, int16_t ch2_value) {
      display.clearDisplay();
      display.setTextSize(2);             // Normal 1:1 pixel scale
      display.setTextColor(WHITE);        // Draw white text
      display.setCursor(10,20);             // Start at top-left corner
      display.println("CH1:" + String(ch1_value));
      display.setCursor(10,37);             // Start at top-left corner
      display.println("CH2:" + String(ch2_value));
      display.display();

}

