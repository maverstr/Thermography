#include <Wire.h> // I2C library, required for MLX90614
#include "SparkFunMLX90614.h" // SparkFunMLX90614 Arduino library

IRTherm therm; // Create an IRTherm object to interact with throughout


float startingTime;
float currentTime;

int counter;

String lastValue;
String value;
int16_t reg;

uint8_t err;

float tmin;
float tmax;

void setup()
{

  counter = 0;
  tmax = 320;
  tmin = 290;
  Serial.begin(500000); // Initialize Serial to log output
  therm.begin(); // Initialize thermal IR sensor
  //therm.setUnit(TEMP_K);
  err = therm.writeEEPROM(0x20, 0x7D00); 
  err = therm.writeEEPROM(0x21, 0x728D); 
  err = therm.writeEEPROM(0x25, 0xFFFF9C74); 


/*
eeprom dump
FFFF9993
62E3
201
FFFFF71C
FFFFFFFF
FFFF9F75
619E
6234
652B
6BB2


*/
  
  //err = therm.setPWMControl(0b111);
  //err = therm.setConfigRegister(0b1001111101110101);
  //err = therm.setMax(tmax); //ok
  //err = therm.setMin(tmin); //ok
  //therm.readRange(); //ok
  startingTime = millis();
}

void loop()
{ 

    // Call therm.read() to read object and ambient temperatures from the sensor.
    if (therm.read()) // On success, read() will return 1, on fail 0.
    {
     // Use the object() and ambient() functions to grab the object and ambient
     // temperatures.
     // They'll be floats, calculated out to the unit you set with setUnit().
     value = String(therm.object(), 2);
     Serial.print("Object: " + value);
     Serial.println("C");
     Serial.print("Ambient: " + String(therm.ambient(), 2));
     Serial.println("C");
     Serial.println();
    }
    if (value != lastValue) {
     counter++;
     currentTime = millis();
     lastValue = value;
     Serial.println("fps : " + String((counter / (currentTime - startingTime)) * 1000));
    }

  /*
    therm.I2CReadWord(0x20, &reg);
    Serial.println(reg);

    therm.I2CReadWord(0x25, &reg);
    Serial.println(reg, BIN);
  */

  therm.I2CReadWord(0x20, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x21, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x22, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x23, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x24, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x25, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x26, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x27, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x28, &reg);
  Serial.println(reg, HEX);
  therm.I2CReadWord(0x29, &reg);
  Serial.println(reg, HEX);
  Serial.println("---------------------");



}

