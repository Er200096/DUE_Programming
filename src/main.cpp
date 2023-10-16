#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include <Arduino.h>

//BME680 Setup
Adafruit_BME680 bme;

//BH1750 Setup
BH1750 lightMeter(0x23);

//SDI-12 Setup
#define DIRO 7

String command;
int deviceAddress = 0;
String deviceIdentification = "allccccccccmmmmmmvvvxxx";


class sensor{
  public:
    int address = 0;

    virtual float get_raw_value(); // Gets the raw sensor_value
    virtual void setup(); // Used to initialise the sensors

    int get_address(){ // Default address is gong to be zero for both sensors (Refer to address value)
      return address;
    }

    void set_address(int new_address){ // This will be used to set the address of the sensors in change of address commands
      address = new_address;
      Serial.println("New address Set");
    }
};

class BME_Sensor : public sensor{
  public:
  float get_raw_value() override{
    bme.performReading();
    return (bme.readTemperature()); 
  }

  void setup() override{
    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
    // Set the temperature, pressure and humidity oversampling (NEED TO SET LATER)
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setPressureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
  } 
};

class BH_Sensor : public sensor{
  public:
    float get_raw_value() override{
      return (lightMeter.readLightLevel());// Note that this is an unsigned 16bit - might need to parse if code not working 
    }

    void setup() override{
        Wire.begin();
        lightMeter.begin();
    }
};

BME_Sensor bme_sensor;
BH_Sensor bh_sensor;

void SDI12Send(String message) {
  Serial.print("message: "); Serial.println(message);
  
  digitalWrite(DIRO, LOW);
  delay(100);
  Serial1.print(message + String("\r\n"));
  Serial1.flush();    //wait for print to finish
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
  //secondruntoken = 0;
}


void SDI12Receive(String input) {
  //convert device address to string
  String address = String(deviceAddress);
  //Determines if the command is addressed for this device
  if (String(input.charAt(0)) == address) {  
    //Repond to Start Measurement command "aTEST!"   **Notice: Not correctly implemented, this only demostrates command and usuage of I2C sensors
    if ((String(input.charAt(1)) == "T") && (String(input.charAt(2)) == "E")&& (String(input.charAt(3)) == "S")&& (String(input.charAt(4)) == "T") ) {

      SDI12Send(String(bme_sensor.get_raw_value()));
      SDI12Send(String(bh_sensor.get_raw_value()));
      Serial.println("Responding to TEST command");
    }
  }  
}



void setup() {
  //Arduino IDE Serial Monitor
  Serial.begin(9600);

  // BME_Sensor bme_sensor;
  bme_sensor.setup();
  bh_sensor.setup();

  // ================ BME680 ================
  // if (!bme.begin(0x76)) {
  //   Serial.println("Could not find a valid BME680 sensor, check wiring!");
  //   while (1);
  // }
    // Set the temperature, pressure and humidity oversampling
  // bme.setTemperatureOversampling(BME680_OS_8X);
  // bme.setPressureOversampling(BME680_OS_8X);
  // bme.setHumidityOversampling(BME680_OS_2X);


  // ================ BH1750 ================
  // Wire.begin();
  // lightMeter.begin();


  // ================ SDI-12 ================
  Serial1.begin(1200, SERIAL_7E1);  //SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);               //DIRO Pin

  //HIGH to Receive from SDI-12
  digitalWrite(DIRO, HIGH);
}

void loop() {
  int byte;
  // Serial.println("I AM RUNING");
  //Receive SDI-12 over UART and then print to Serial Monitor
  if(Serial1.available()) {
    byte = Serial1.read();        //Reads incoming communication in bytes
    Serial.println(byte);
    if (byte == 33) {             //If byte is command terminator (!)
      SDI12Receive(command);
      Serial.println(command);
      command = "";               //reset command string
    } else {
      if (byte != 0) {            //do not add start bit (0)
      command += char(byte);      //append byte to command string
      }
    }
  }
}


