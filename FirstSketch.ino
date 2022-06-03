
#include <Adafruit_BME680.h>
#include <Adafruit_LIS3DH.h>
# include <Notecard.h>

// #include <SerLCD.h>
// #include <Wire.h>

// #define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD address 0x72

// int cycles = 0;

// #include <bme68x_defs.h>
// #include <bme68x.h>

// Notecard address 0x17

// LED_BUILTIN - Arduino
// USER_BTN - ST
volatile int button_flag = 0; // volatile means read from memory every time
volatile int last_pressed = 0;
static const int DEBOUNCE_MS = 200;
int button_count = 0;
int prev_button_count = 0;

#define SEALEVELPRESSURE_HPA (1013.25)

#define productUID "com.blues.pniedringhaus:first_swan_project"

Adafruit_LIS3DH lis = Adafruit_LIS3DH(); // address 0x18

Adafruit_BME680 bme; // I2C address 0x77

Notecard notecard;

// double click button function
void button_isr() {
  // don't do anything here, everything's stopped
  int now = millis();
  // double click
  if(now - last_pressed < 1000){
    // debounce
    if(now - last_pressed > DEBOUNCE_MS){
      button_flag = 1;
      last_pressed = 0;
    }
  } else { // store b/c no dbl click
    last_pressed = now;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);    // sets the digital pin as LED_BUILTIN mapped by the Swan support package as output (output under our control)
  pinMode(USER_BTN, INPUT_PULLUP); // set to input, which is under external control (i.e. someone else pushes the button and we read it), pullup so it has a default state
  attachInterrupt(digitalPinToInterrupt(USER_BTN), button_isr, FALLING); // compulsory digitalPinToInterrupt, interrupt to user_btn fires off button_isr, whenever button's released (could also be falling)

  // Wire.begin();

  // initialize usb on swan for logs 
  Serial.begin(115200); // opens serial port, sets data rate to 115200 bps
  // until you start reading Serial, the program will not move from this point
  while(!Serial);

  notecard.begin();
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", productUID);
  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequest(req);


  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }

  //Send the reset command to the display - this forces the cursor to return to the beginning of the display
  // Wire.beginTransmission(DISPLAY_ADDRESS1);
  // Wire.write('|'); //Put LCD into setting mode
  // Wire.write('-'); //Send clear display command
  // Wire.endTransmission();
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);  // sets the digital pin LED_BUILTIN mapped by the Swan support package as off

  if(button_flag){
    button_flag = 0;
    prev_button_count = button_count;
    button_count = button_count + 1;
    digitalWrite(LED_BUILTIN, HIGH); // sets the digital pin LED_BUILTIN mapped by the Swan support package as on
    Serial.print("Button count = ");
    Serial.print(button_count);
    Serial.println();
    delay(1000);            // waits for a second
  }

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  // delay(2000);

  lis.read(); // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x);
  Serial.print("  \tY:  "); Serial.print(lis.y);
  Serial.print("  \tZ:  "); Serial.print(lis.z);

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.println();

  // cycles++;
  // i2cSendValue(cycles);

  // air.qo notes
  {
    J *req = notecard.newRequest("note.add");
    if (req != NULL) {
      JAddStringToObject(req, "file", "air.qo");
      J *body = JAddObjectToObject(req, "body");
      if(body) {
        JAddNumberToObject(body, "temp", bme.temperature);
        JAddNumberToObject(body, "humidity", bme.humidity);
        JAddNumberToObject(body, "pressure", bme.pressure);
        JAddNumberToObject(body, "gas", bme.gas_resistance);
      }
      notecard.sendRequest(req); // send note to notecard
    }
  }

  // motion.qo accelerometer notes 
  {
    J *req = notecard.newRequest("note.add");
    if (req != NULL) {
      JAddStringToObject(req, "file", "motion.qo");
      J *body = JAddObjectToObject(req, "body");
      if(body) {
        JAddNumberToObject(body, "X", event.acceleration.x);
        JAddNumberToObject(body, "Y", event.acceleration.y);
        JAddNumberToObject(body, "Z", event.acceleration.z);
      }
      notecard.sendRequest(req); // send note to notecard
    }
  }

  // conditional button double tap notes
  {
    if(button_count){
      J *req = notecard.newRequest("note.add");
      if (req != NULL && button_count != prev_button_count) {
        JAddStringToObject(req, "file", "button.qo");
        J *body = JAddObjectToObject(req, "body");
        if(body) {
          Serial.print("Sending button count ");
          prev_button_count = button_count;
          JAddNumberToObject(body, "button_count", button_count);
        }
        notecard.sendRequest(req); // send note to notecard
      }
    }
  }

  notecard.sendRequest(notecard.newRequest("hub.sync")); // send notes to notehub
  delay(10000);
}

//Given a number, i2cSendValue chops up an integer into four values and sends them out over I2C
// void i2cSendValue(int value)
// {
//   Wire.beginTransmission(DISPLAY_ADDRESS1); // transmit to device #1

//   Wire.write('|'); //Put LCD into setting mode
//   Wire.write('-'); //Send clear display command

//   Wire.print("Cycles: ");
//   Wire.print(value);

//   Wire.endTransmission(); //Stop I2C transmission
// }