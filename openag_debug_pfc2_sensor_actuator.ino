// The relay board is using the arduino gpio pins to power the
// onboard controller so the power and ground pins should always be
// in this state whenever the device is on. Doing this made wiring
// much cleaner and let us use less components. There is a 20 pin 
// cable connecting pins 26-45 on the arduino to the GPIO 
// and POWER / GND pins on the 16-channel relay block. There are 16
// signal lines, 2 power (5V) lines, and 2 ground lines. 

const int relayPower1 = 44;
const int relayPower2 = 45;
const int relayGround1 = 26;
const int relayGround2 = 27;


// These are the pinouts that show the linkage between the arduino
// GPIO signals, corresponding relay, and device controlled.

const int heater1 = 43; // K1
const int redLED = 42; // K2
const int whiteLED = 41; // K3
const int blueLED = 40; // K4
const int chamberFan = 39; // K5
const int circulationPump = 38; // K6
const int aerationPump = 37; // K7
const int airFlush = 36; // K8
const int heater2 = 35; // K9
const int chillerPump = 34; // K10
const int chillerFan = 33; // K11
const int chillerCompressor = 10; // pulse generator
const int pump1 = 32; // K12
const int pump2 = 31; // K13
const int pump3 = 30; // K14
const int pump4 = 29; // K15
const int pump5 = 28; // K16
const int chillerDriver = 10; // need to add a pulldown resistor 

// Here are the sensor connections. Note: he am2315, mhz16, atlas 
// ec, and atlas ph are on the i2c bus. 
const int lowLevelSensor = 3; // lle102000
const int highLevelSensor = 4; // lle102000
const int waterTemperatureSensor = 5; // ds18b20

// Include sensor libraries
#include <Wire.h>
#include "Adafruit_AM2315.h"
#include "NDIR_I2C.H"

// Setup sensor classes
Adafruit_AM2315 am2315;
NDIR_I2C mhz16(77);


// Runs once
void setup() {
  // Setup power for relay board (uses GPIO pins to power the board's controller)
  pinMode(relayPower1, OUTPUT);
  digitalWrite(relayPower1, HIGH);
  pinMode(relayPower2, OUTPUT);
  digitalWrite(relayPower2, HIGH);  
  pinMode(relayGround1, OUTPUT);
  digitalWrite(relayGround1, LOW);  
  pinMode(relayGround2, OUTPUT);
  digitalWrite(relayGround2, LOW);

  // Setup pin modes
  pinMode(lowLevelSensor, INPUT);
  pinMode(highLevelSensor, INPUT);

  // Setup serial port
  Serial.begin(9600);

  // Setup actuator states, note: relay board is active low
  digitalWrite(heater1, HIGH);

  // Setup air temperature & humidity sensor
  if (!am2315.begin()) {
     Serial.println("AM2315 Sensor not found");
  }

  // Setup co2 sensor
  mhz16.power_on();
  if (!mhz16.begin()) {
    Serial.println("MHZ16 Sensor not found");
  }

}

// Runs repeatedly
void loop() {
  // Read low level sensor
  Serial.print("Low Level Sensor: "); 
  if (digitalRead(lowLevelSensor)) Serial.println("Above Water");
  else Serial.println("Below Water");

  // Read high level sensor
  Serial.print("High Level Sensor: "); 
  if (digitalRead(highLevelSensor)) Serial.println("Above Water");
  else Serial.println("Below Water");
  
  // Read air temperature & humidity sensor
  Serial.print("Air Temperature: "); 
  Serial.println(am2315.readTemperature());
  Serial.print("Humidity: "); 
  Serial.println(am2315.readHumidity());

  // Read co2 sensor
  mhz16.measure();
  Serial.print("CO2: ");
  Serial.println(mhz16.ppm);

  

  // End of loop
  Serial.println();
  delay(1000); 
  }
