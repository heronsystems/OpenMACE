// This program contains both the FastLED and XBee libraries
// Attempt to send a package containing a color over the XBee connection to control the color of the LED's

#include <FastLED.h>
#include <XBee.h>
#include <SoftwareSerial.h>

/*****************************************
// FastLED setup and definitions   */

// How many leds in your strip?
#define NUM_LEDS 16

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 3
#define CLOCK_PIN 13

#define  LED_ID 7

// Define the array of leds
CRGB leds[NUM_LEDS];


/******************************************
XBee setup   */

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

// Define NewSoftSerial TX/RX pins
// Connect Arduino pin 8 to TX of usb-serial device
uint8_t ssRX = 8;
// Connect Arduino pin 9 to RX of usb-serial device
uint8_t ssTX = 9;
// Remember to connect all devices to a common Ground: XBee, Arduino and USB-Serial device
SoftwareSerial nss(ssRX, ssTX);

byte R;
byte G;
byte B;

void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);

xbee.setSerial(Serial);
nss.begin(9600);

Serial.print("STarting up...");


FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

for (int i=0;i<NUM_LEDS;i++){
  
 leds[i].setRGB(255,255,255);
  FastLED.show();
}
}

void loop() {
  // XBee Loop

  xbee.readPacket();
    
    if (xbee.getResponse().isAvailable()) {
      // got something
           
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
        
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
      
        Serial.println("Got an rx packet!");

               
            
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            // the sender got an ACK
            Serial.println("packet acknowledged");
        } else {
          Serial.println("packet not acknowledged");
        }
        
        Serial.print("checksum is ");
        Serial.println(rx.getChecksum(), HEX);

        Serial.print("packet length is ");
        Serial.println(rx.getPacketLength(), DEC);
        
         for (int i = 0; i < rx.getDataLength(); i++) {
          Serial.print("payload [");
          Serial.print(i, DEC);
          Serial.print("] is ");
          Serial.println(rx.getData()[i]);
        }
        
       for (int i = 0; i < xbee.getResponse().getFrameDataLength(); i++) {
       // Serial.print("frame data [");
        //Serial.print(i, DEC);
        //Serial.print("] is ");
        //Serial.println(xbee.getResponse().getFrameData()[i], HEX);
      }
      }
    } else if (xbee.getResponse().isError()) {
      Serial.print("error code:");
      Serial.println(xbee.getResponse().getErrorCode());
    }

// FastLED Loop

// Set Colors directly from RGB hex values received 
//  Hex Color Code:  0xRRGGBB  (RGB values are 0-255)


/*for (int i=0; i<NUM_LEDS;i++) {
  leds[i] =  rx.getData()[i];
}*/

//R = (rx.getData()[0]|)
//long color = (R<<4) | (G<<2) | (B);
//long color = (rx.getData()[0]<<5 | rx.getData()[1]<<4 | rx.getData()[2]<<3 |rx.getData()[3]<<2 |rx.getData()[4]<<1 |rx.getData()[5] );
//Serial.println(color, HEX);
/*
for (int i=0;i<NUM_LEDS;i++){
  
 leds[i].setRGB(rx.getData()[3*i], rx.getData()[3*i+1], rx.getData()[3*i+2]);
  FastLED.show();
}*/

/*
  delay(500);
  for (int i=0;i<NUM_LEDS;i++){
  // Now turn the LED off, then pause
  leds[i] = CRGB::Black;
  FastLED.show();
  
}
*/
//delay(500);

}
