#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include <Array.h>
#include <LinkedList.h>
// #include "SuperWatchDog.h"

//BME680 Setup
Adafruit_BME680 bme;

//BH1750 Setup
BH1750 lightMeter(0x23);

//SDI-12 Setup
#define DIRO 7
#define Num_Of_Sensors 3
#define watchdog_time 3000 // 3 seconds


String command;
String Raw_command;
int command_address;
int target_address;
int value_sent;
int device_address_to;

bool sensors_init_request;


/**********************************************
 *       watchdog Setup 🐕‍🦺                   *
 **********************************************/
void watchdogSetup(void) {} // to enable the watchdog service 


/**********************************************
 *       ✨Class declarations here ✨        *
 **********************************************/

class sensor{
  public:
    virtual float get_raw_value(); // Gets the raw sensor_value
    virtual void setup_sensor(); // Used to initialise the sensors
    virtual int get_sensor_address(); //
    virtual String get_sensor_name();
};

class BME_Sensor : public sensor{
  String name = "Pressure sensor (BME680)";

  public:
  int address; // the declerastion for the 
  float get_raw_value() override{
    bme.performReading();
    return (bme.readTemperature()); 
  }
   
  int get_sensor_address() override{
    return address;
  }

  void setup_sensor() override{
    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
    // Set the temperature, pressure and humidity oversampling (NEED TO SET LATER)
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setPressureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
  } 
   String get_sensor_name() override{
    return name;
  }
};


class BME_temprature : public BME_Sensor{
  int address = 2;
  String name = "Temperature Sensor";
  float get_raw_value() override{
    bme.performReading();
    return (bme.readTemperature()); 
  }
  
  int get_sensor_address() override{
    return address;
  }

  String get_sensor_name() override{
    return name;
  }
};

class BME_pressure : public BME_Sensor{
  int address = 3;
  String name = "Pressure Sensor";
  float get_raw_value() override{
    bme.performReading();
    return (bme.readPressure()); 
  }
  
  int get_sensor_address() override{
    return address;
  }

    String get_sensor_name() override{
      return name;
    }
};

class BME_altitude : public BME_Sensor{
  String name = "Altitude Sensor";
  int address = 4;
  float get_raw_value() override{
    bme.performReading();
    return (bme.readAltitude(0)); 
  }
  
  int get_sensor_address() override{
    return address;
  }

  String get_sensor_name() override{
    return name;
  }
};

class BME_humidty : public BME_Sensor{
  int address = 5;
  String name = "Humidty Sensor";
  float get_raw_value() override{
    bme.performReading();
    return (bme.readHumidity()); 
  }  
  
  int get_sensor_address() override{
    return address;
  }
 
  String get_sensor_name() override{
    return name;
  }
  
};


class BH_Sensor : public sensor{
  public:
  int address = 1; // Default SDI-12 address
  String name = "Light sensor (BH1750)";
    float get_raw_value() override{
      return (lightMeter.readLightLevel());// Note that this is an unsigned 16bit - might need to parse if code not working 
    }

    int get_sensor_address() override{
      return address;
    }

    void setup_sensor() override{
        Wire.begin();
        lightMeter.begin();
    }

  String get_sensor_name() override{
    return name;
  }
};


class SDI12_device { 
  // This class is the main class that that has a bunch of sensors attached to it
  // and will handle sensor initialization
  public:

    LinkedList<sensor*> sensor_list; // List of pointers to sensors

  int deviceAddress = 0; // The default device address


  int get_device_address() {
     return deviceAddress; 
     }
  
    String sensor_ID = String(String(get_device_address()) + "14ENG20009103218929xxx...xx<CR><LF>");


  void set_device_address(int set_deviceAddress) { 
    deviceAddress = set_deviceAddress; 
    }
    
    void attach_sensor(sensor* sensor){
      sensor_list.add(sensor);
      //sensors.push_back(sensor);
    }

    String Read_a_Sensor(int deviceAddress){
      if(sensor_list.size() >= deviceAddress){
        return String(sensor_list.get(deviceAddress-1)->get_raw_value());
        } 
      else{
      return (String("-"));
      }
    }

  String Read_Sensors() {
    String output;
    watchdogReset();
    for (int i = 0; i < sensor_list.size(); i++) {
      output += String(sensor_list.get(i)->get_raw_value()) + String("\n");
    }
    return output;
  }

    void initialize_attached_sensors(){
    watchdogReset();
    for (int i = 0; i <= sensor_list.size()-1; i++) { // Should be -1
      Serial.println("Sensor initialized ");
        sensor_list.get(i)->setup_sensor();
      }
    }

    String pretty_print_sensor_values(String TIME){ // USE THE RTC value for this
      String output;
      for (int i = 0; i < sensor_list.size(); i++) {
        output += String(sensor_list.get(i)->get_sensor_name()) +" : " + String(sensor_list.get(i)->get_raw_value()) + String("\n");
    }
    return output; // Note To This is basically the output you wanna use to write to the sensor output file
    }
};

/**********************************************
 *       Class initialisation🔥               *
 **********************************************/

SDI12_device* this_device = new SDI12_device();

BME_altitude* altitude_sensor = new BME_altitude();
BME_humidty* humidity_sensor = new BME_humidty();
BME_temprature* temperature_sensor = new BME_temprature();
BH_Sensor* bh_sensor = new BH_Sensor();

/**********************************************
 *       SD1-12 Function declaration 📱        *
 **********************************************/

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

void prase_command(String this_command, int device_address = 0,int value_set = 0,int sensor_address = 0){ // Cannot use a string switch in cpp  😥
  if(this_command.equals("?")){
    SDI12Send(String(this_device->deviceAddress));
  }
  else if(device_address != this_device->deviceAddress){
    SDI12Send("Invalid device address");
  }
  else{
    if(this_command.equals("A")){

        this_device->set_device_address(value_set);
        SDI12Send(String(this_device->deviceAddress));
    }

    if(this_command.equals("M")){ // Start scatter plot GUI with readings
      
      if(!sensors_init_request){
        
        // this_device->attach_sensor(altitude_sensor);  Im having issues with getting a proper reading with the altitude sensor :(
        this_device->attach_sensor(humidity_sensor);
        this_device->attach_sensor(temperature_sensor);
        this_device->attach_sensor(altitude_sensor);
        this_device->attach_sensor(bh_sensor);
        this_device->initialize_attached_sensors();

        SDI12Send(String(this_device->deviceAddress) + "001" + String(this_device->sensor_list.size()));

        sensors_init_request = true;
      }
      else{
          Serial.println("Sensors already initialized");
      }
    }

    if(command.equals("D") ){
      switch(value_set){
        case 0: // Display all readings 
        Serial.println("Reading Sensors");
        SDI12Send(this_device->Read_Sensors());
        break;

        default:
        SDI12Send(this_device->Read_a_Sensor(value_set));
        break;
      }
  }
    if(command.equals("I")){
      SDI12Send(this_device->sensor_ID); 
    }
  }
}

int SDI12Receive(String input) {
  watchdogEnable(watchdog_time);

  device_address_to = int(input.charAt(0)-'0');

  switch(input.length()){
    case 2:
    
      command = String(input.charAt(1)); 
    case 3:
      command = String(input.charAt(1));
      value_sent = int(input.charAt(2)-'0');
    break;
  
    case 4:
      command = String(String(input.charAt(1))+String(input.charAt(2)));
      value_sent = int(input.charAt(3)-'0');
    break;

    default:
      command = String(input.charAt(0)); 
    break;
  }
    Serial.println(String("Command Breakdown:\n CMD: "+ command+" target: "+ String(value_sent)));
    prase_command(command, device_address_to, value_sent); 
    return 1;
}

/**********************************************
 *       Default Arduino Setup function       *
 **********************************************/


void setup() {
  //Arduino IDE Serial Monitors
  Serial.begin(9600);
  Serial.println("SETUP");
  // this_device.initialize_attached_sensors();

  Serial1.begin(1200, SERIAL_7E1);  //SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);               //DIRO Pin

  //HIGH to Receive from SDI-12
  digitalWrite(DIRO, HIGH);
}

/**********************************************
 *       Default Arduino Main function       *
 **********************************************/

void loop() {
  int byte;
  //Receive SDI-12 over UART and then print to Serial Monitor
  watchdogReset();

  if(Serial1.available()) {
    byte = Serial1.read();        //Reads incoming communication in bytes
    // Serial.println(byte);
    if (byte == 33) {             //If byte is command terminator (!)
      SDI12Receive(Raw_command);
      // Serial.println(command);
      Raw_command = "";               //reset command string
    } else {
      if (byte != 0) {            //do not add start bit (0)
      Raw_command += char(byte);      //append byte to command string
      }
    }
  }
}