/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution

 Originally from: 
 https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/installing-ble-library
 Specifically:
 https://github.com/adafruit/Adafruit_BluefruitLE_nRF51/blob/master/examples/bleuart_cmdmode/bleuart_cmdmode.ino
 These are also very helpful resources for getting setup and started:
 https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/software-resources
 
 jaron42 Modifications made to allow BLE-PixMaker to program the color
 sequence of the Adafruit Bicolor Led via BLE. Feel free to modify and
 have fun!!!
*********************************************************************/
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

// includes for matrix control
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
//Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Matrix for congtrol of the 8x8 panel
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();
bool isDisplayUpdateNeeded;

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 };
    
static const uint8_t PROGMEM
  background_bmp[] =
  { B11000011,
    B10111101,
    B01011010,
    B01111110,
    B01011010,
    B10111101,
    B11000011,
    B00111100 };

// Note 42 is roughly the most we could eventually store bit packed in memory
static uint8_t red_bmp[42][8] = {0};
static uint8_t green_bmp[42][8] = {0};
static uint8_t yellow_bmp[42][8] = {0};

static const uint8_t PROGMEM LOOP_TICK = 10;
uint8_t imageCount = 0;
uint8_t currentImageIndex = 0;
uint32_t imageDelay = 100;
uint32_t currentImageInterval = 0;
uint32_t imageRepeats = 0;
uint32_t currentImageRepeats = 0;
int bytesRead = 0;
bool waitingForCommand = true;
    
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  matrix.begin(0x70);  // pass in the address
  int serialAttempts = 0;
  while (!Serial && serialAttempts < 50) {
    //delay(10); // give it a few seconds if not go without it
    serialAttempts++;
      matrix.clear();      // clear display
      matrix.drawPixel(3, 4, LED_GREEN); // Show waiting for BLE connection  
      matrix.writeDisplay();
      delay(100); 
      matrix.clear();      // clear display
      matrix.drawPixel(4, 3, LED_YELLOW); // Show waiting for BLE connection  
      matrix.writeDisplay();
      delay(100); 
  }
  if(Serial){
    Serial.begin(115200);
    // NOTE must leave this splash in per the license!!!
    Serial.println(F("Adafruit Bluefruit Command Mode Example"));
    Serial.println(F("---------------------------------------"));

    /* Initialise the module */
    Serial.print(F("Initialising the Bluefruit LE module: "));
  }
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if(Serial){
    Serial.println( F("OK!") );
  }
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    
    if(Serial){
      Serial.println(F("Performing a factory reset: "));
    }
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  if(Serial){
    Serial.println("Requesting Bluefruit info:");
  }
  /* Print Bluefruit information */
  ble.info();

  
  if(Serial){
    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    Serial.println(F("Then Enter characters to send to Bluefruit"));
    Serial.println();
  }
  ble.verbose(false);  // debug info is a little annoying after this point!
  isDisplayUpdateNeeded = true;

}


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for connection */
  if(imageCount<=0 || waitingForCommand){
    // wait to get a new program
    while (! ble.isConnected()) { 
      matrix.clear();      // clear display
      matrix.drawPixel(3, 3, LED_RED); // Show waiting for BLE connection  
      matrix.writeDisplay();
      delay(100); 
      matrix.clear();      // clear display
      matrix.drawPixel(4, 4, LED_RED); // Show waiting for BLE connection  
      matrix.writeDisplay();
      delay(100); 
    }
  }
  if (isDisplayUpdateNeeded) {
     matrix.clear();
     if(imageCount>0){
       matrix.drawBitmap(0, 0, red_bmp[currentImageIndex], 8, 8, LED_RED);
       matrix.drawBitmap(0, 0, green_bmp[currentImageIndex], 8, 8, LED_GREEN);
       matrix.drawBitmap(0, 0, yellow_bmp[currentImageIndex], 8, 8, LED_YELLOW);
       matrix.writeDisplay();
     }
     isDisplayUpdateNeeded = false;
  }
  
  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  bytesRead = ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
  }
  else {
    if (Serial){
      Serial.println(F("Bytes Read:"));
      Serial.println(bytesRead);
      Serial.println(F("Some data other than OK"));
      Serial.println(ble.buffer);
    }
    if (13 == bytesRead ){
    //&&
//      (strcmp(ble.buffer[0], "C") == 0) &&
//      (strcmp(ble.buffer[3], "D") == 0) &&
//      (strcmp(ble.buffer[8], "R") == 0)) {
      if(Serial){
        Serial.println(F("New Command Found"));
      }
      imageCount = 1; //todo
      imageCount = atoi(&ble.buffer[1]); // TODO something better than atoi
      if(Serial){
        Serial.println(F("New Image Count:"));
        Serial.println(imageCount);
      }
      currentImageIndex = 0; //reset
      imageDelay = atoi(&ble.buffer[4]); // TODO something better than atoi
      if(Serial){
        Serial.println(F("New Image Delay:"));
        Serial.println(imageDelay);
      }
      currentImageInterval = 0; //reset
      imageRepeats = atoi(&ble.buffer[9]); // TODO something better than atoi
      if(Serial){
        Serial.println(F("New Image Repeats:"));
        Serial.println(imageRepeats);
      }
      ble.waitForOK(); // why is this needed? Example only did it after read
      currentImageRepeats = 0; //reset

      uint8_t newRedVal = 0;
      uint8_t newGreenVal = 0;
      uint8_t newYellowVal = 0;
      // Need to read in the rest of the image data
      int row = 0;
      int imageNumber = 0;
      for(int eightPixelBlockExpected = 0; eightPixelBlockExpected < imageCount * 8; eightPixelBlockExpected++){
        ble.println("AT+BLEUARTRX");
        bytesRead = ble.readline();
        if (Serial){
          Serial.println(F("Bytes Read:"));
          Serial.println(bytesRead);
        }
        int retry = 0;
        int maxRetry = 8;
        while (strcmp(ble.buffer, "OK") == 0) {
          // no data error not enough image data was sent over
          bytesRead = 0;
          ble.println("AT+BLEUARTRX");
          bytesRead = ble.readline();
          if (Serial){
            Serial.println(F("Bytes Read:"));
            Serial.println(bytesRead);
          }
          if(retry>maxRetry){
            if (Serial){
              Serial.println(F("Image Data not found"));
            }
            return;
          }
          retry++;
        }
        if (Serial){
          Serial.println(F("Bytes Read:"));
          Serial.println(bytesRead);
          Serial.println(F("Some data other than OK"));
          Serial.println(ble.buffer);
        }
        if(8==bytesRead){
          newRedVal = 0;
          newGreenVal = 0;
          newYellowVal = 0;
          for(int j=0;j<8;j++){
            switch(ble.buffer[j]){
              case 'R':
                newRedVal = newRedVal + (1<<(7-j));
                break;
              case 'G':
                newGreenVal = newGreenVal + (1<<(7-j));
                break;
              case 'Y':
                newYellowVal = newYellowVal + (1<<(7-j));
                break;
            }
          }    
          ble.waitForOK(); // why is this needed? Example only did it after read
          red_bmp[imageNumber][row] = newRedVal;
          green_bmp[imageNumber][row] = newGreenVal;
          yellow_bmp[imageNumber][row] = newYellowVal;
          row++;
          if(row>=8){
            row = 0;
            imageNumber++;
          }
        }
        else{
          if (Serial){
            Serial.println(F("Image Data message of 8 bytes expected"));
          }
          return; // todo validate error transition
        }
      } // end for each 8 pixel block
      isDisplayUpdateNeeded = true;
      waitingForCommand = false;      
      if(Serial){
        Serial.println(F("New image(s) should load"));
      }
    }
  } // end else not OK read in
  bytesRead = 0; // reset for next loop
  if(!waitingForCommand){
    delay(LOOP_TICK); 
    currentImageInterval += LOOP_TICK;
    if(currentImageInterval > imageDelay){
      currentImageInterval = 0; //reset
      isDisplayUpdateNeeded = true;
      currentImageIndex++;
      if(currentImageIndex >= imageCount){
        currentImageIndex=0; //reset
        currentImageRepeats++;
        if(currentImageRepeats>imageRepeats){
          //time to stop until next command
          waitingForCommand = true;
        }
      }
    }
  }
  return;
}
