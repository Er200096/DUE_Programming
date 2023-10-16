#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include <List.hpp>

//BME680 Setup
Adafruit_BME680 bme;

//BH1750 Setup
BH1750 lightMeter(0x23);

//SDI-12 Setup
#define DIRO 7

String command;
int command_address;
int target_address;


String deviceIdentification = "allccccccccmmmmmmvvvxxx";

void prase_command();

class sensor{
  public:
    int address; // Will be used to identify the sensors involved

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


class SDI12_device { 
  // This class is the main class that that has a bunch of sensors attached to it
  // and will handle sensor initialization
  public:
  List<sensor>sensor_list;
  int deviceAddress = 0; // The default device address

  int get_device_address() {
     return deviceAddress; 
     }

  void set_device_address(int set_deviceAddress) { 
    deviceAddress = set_deviceAddress; 
    }
    
    void attach_sensor(sensor sensor){
      sensor_list.add(sensor);
      //sensors.push_back(sensor);
    }

  String Read_Sensors() {
    String output;
    for (int i = 0; i < sensor_list.getSize(); i++) {
      output = String(sensor_list.getValue(i).get_raw_value()) + String("\n");
    }
  }


    void initialize_attached_sensors(){
    for (int i = 0; i < sensor_list.getSize(); i++) {
        sensor_list.getValue(i).setup();
      }
    }
};



class BME_Sensor : public sensor{
  public:
  int address = 1; // Default SDI-12 address
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
  int address = 2; // Default SDI-12 address
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

SDI12_device this_device;

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


  if (input.length() == 3){
    command = String(input.charAt(1));
    target_address = int(input.charAt(2)-'0');
  }
  else if(input.length() == 4){
    command = String(String(input.charAt(1))+String(input.charAt(2)));
    target_address = int(input.charAt(3)-'0');
  }
  else{
    command = String(input.charAt(0));
  }

  Serial.println(String("Command Breakdown:\n CMD: "+ command+" target: "+ String(target_address)));
  // prase_command(command);
}


void prase_command(String command,int device_address = 0,int sensor_address = 0, int value_set = 0){ // Cannot use a switch in cpp 😥
  if(command.equals("?")){
    SDI12Send(String(device_address));
  }
  if(command.equals("A")){
    device_address = value_set;
    //  change address Query Command
  }
  if(command.equals("AS")){
    // Change Sensor Address for ur 

  }
  if(command.equals("M")){
    //start Measuring da boi
  }
  if(command.equals("D") ){
    // Send data command
}
  if(command.equals("I")){
    //Send Identication data
  }
}

void setup() {
  //Arduino IDE Serial Monitor
  Serial.begin(9600);



// Sensor Setup
  this_device.attach_sensor(bme_sensor);
  this_device.attach_sensor(bh_sensor);
  this_device.initialize_attached_sensors();


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


