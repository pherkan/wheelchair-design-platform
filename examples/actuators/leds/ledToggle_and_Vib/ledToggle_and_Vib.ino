#include <Adafruit_NeoPixel.h> // Necessary Library include

#define LED_PIN1 2 // Defining the pin of the arduino that sends the data stream.
#define LED_PIN2 7
#define VIB_PIN 10

Adafruit_NeoPixel LED_controller1 = Adafruit_NeoPixel( 1, LED_PIN1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel LED_controller2 = Adafruit_NeoPixel( 1, LED_PIN2, NEO_RGB + NEO_KHZ800);

int i = 127;
uint8_t R = 0, G = 0, B = 0; // Unsigned integer with 8 bits
uint32_t counter = 0; // 32 bits unsigned integer, we only need 24 to go through all the colors

bool left_red = false;
bool right_red = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Set serial to 9600 baud
  pinMode(VIB_PIN, OUTPUT);
  LED_controller1.begin(); // We're starting up the library
  LED_controller2.begin(); // We're starting up the library

  LED_controller1.setPixelColor( 0, 0x008000);
  LED_controller2.setPixelColor( 0, 0x008000);
  // Red = 0xFF0000 Green = 0x008000
}

void loop() {

  LED_controller1.show(); // Sending updated pixel color to the hardware
  LED_controller2.show(); // Sending updated pixel color to the hardware

  if (Serial.available() > 0 ) {
    int command = Serial.read();
//  int inByte = Serial.read();

    switch (command) {
      //backward
      case '0' :
        digitalWrite(VIB_PIN, HIGH);
        delay(1000);
//        analogWrite(VIB_PIN, 153);
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //forward
      case '1' :
        digitalWrite(VIB_PIN, HIGH);
        delay(1000);
//        analogWrite(VIB_PIN, 153);
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //right
      case '2' :
        digitalWrite(VIB_PIN, HIGH);
        delay(1000);
//        analogWrite(VIB_PIN, 153);
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //left
      case '3' :
        digitalWrite(VIB_PIN, HIGH);
        delay(1000);
//        analogWrite(VIB_PIN, 153);
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      default:
//        digitalWrite(VIB_PIN, LOW);
//        delay(1000);
        LED_controller1.setPixelColor( 0, 0xFFFFFF);
        LED_controller2.setPixelColor( 0, 0xFFFFFF);
        LED_controller1.show();
        LED_controller2.show();
      }
    }
}
