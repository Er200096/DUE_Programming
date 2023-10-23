#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include <LinkedList.h>
#include <OneWire.h>
#include <Scheduler.h>

// #include "SuperWatchDog.h"

//BME680 Setup
Adafruit_BME680 bme;

//BH1750 Setup
BH1750 lightMeter(0x23);

//SDI-12 Setup
#define DIRO 7
#define Num_Of_Sensors 3
#define watchdog_time 3000 // 3 seconds
#define BUTTON_PIN 2


String command;
String Raw_command;
int command_address;
int target_address;
int value_sent;
int device_address_to;

volatile  bool sensors_init_request;

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h> // allowing to communicate with SPI libraries
#include <Scheduler.h> // allows to run multiple functions simoultaneously


#define TFT_CS    10 // essential definitions for TFT display
#define TFT_RST   6
#define TFT_DC    7
#define TFT_SCLK 13  
#define TFT_MOSI 11   
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#include <DueTimer.h>
#include <SdFat.h>
#include <RTClib.h>

RTC_DS1307 rtc;
SdFs sd;
FsFile file;
int BUZZER = 6;

// Pin Assignments
const uint8_t SD_CS_PIN = A3;
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN = 13;
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi) 


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
    return (bme.readAltitude(1013.25)); 
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

OneWire ds(8);


byte addr[] = {0x2D, 0x4B , 0xFF , 0x67 , 0x40 , 0x0 , 0x0 , 0xB4}; //insert the Address of EEPROM value into here after you run the searchFunction



class SDI12_device {
  public:
    LinkedList<sensor*> sensor_list;
    int deviceAddress = 0;
  
    int get_device_address() {
      return deviceAddress;
    }
  
    String sensor_ID = String(String(get_device_address()) + "14ENG20009103218929");
  
    void set_device_address(int set_deviceAddress) {
      deviceAddress = set_deviceAddress;
    }
  
    void attach_sensor(sensor* sensor) {
      sensor_list.add(sensor); // add sensor to device by adding pointer to sensor list
    }
  
    String Read_a_Sensor(int deviceAddress) {
      if (sensor_list.size() >= deviceAddress) {
        return String(sensor_list.get(deviceAddress - 1)->get_raw_value());
      }
      else {
        return (String("-"));   // return "-" if device address is out of bounds
      }
    }
  
    String Read_Sensors() {
      String output;
      output += String(get_device_address());
      for (int i = 0; i < sensor_list.size(); i++) {
        output +=  " : " + String(sensor_list.get(i)->get_raw_value());
      }
      return output;
    }
  
    void initialize_attached_sensors() {
      for (int i = 0; i < sensor_list.size(); i++) {
        sensor_list.get(i)->setup_sensor();
      }
    }
  
    // format sensor data for SD card storage
    String pretty_print_sensor_values() {
      String output;
      for (int i = 0; i < sensor_list.size(); i++) {
          String sensorName = sensor_list.get(i)->get_sensor_name();
          float rawValue = sensor_list.get(i)->get_raw_value();
          String unit = "";

          // Add units based on the sensor type
          if (sensorName.equals("Pressure sensor (BME680)")) {
              unit = " hPa";
          } else if (sensorName.equals("Light sensor (BH1750)")) {
              unit = " lux";
          } else if (sensorName.equals("Temperature sensor (BME680)")) {
              unit = "°C";
          } else if (sensorName.equals("Humidity sensor (BME680)")) {
              unit = "%";
          } else if (sensorName.equals("Altitude sensor (BME680)")) {
              unit = " meters";
          }
          // Format sensor value with appropriate unit
          output += sensorName + " : " + String(rawValue, 2) + unit + "\n";
      }
      return output;
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
 *       SD1-12 Function declaration 📱       *
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
 *       GUI SETUP                           *
 **********************************************/

void reset_screen(){ // Use this to reset the screen back to black
  tft.fillScreen(ST77XX_BLACK);
}

int buttonPin = 2; // referring to button 1 on the board
int buttonState = 0; // the state of the button before an action
int oldButtonState = LOW; // allows for a toggle function


void drawtext(const String &text, uint16_t color, float x, float y, int size) { // function to allow for easy displaying words to screen

  tft.setCursor(x, y); // position on screen
  tft.setTextColor(color); // colour of text
  tft.setTextWrap(true); // allows it to write to next line when reaching end of display
  tft.setTextSize(size); // size of text
  
  tft.println(text); 
}

  void CoolMenu(){ // function for the menu which appears in the beginning
    drawtext("Welcome!", ST77XX_YELLOW, 38, 30, 2); 
    drawtext("Please Press Button 1 to    Toggle to a Different            Sensor",ST77XX_WHITE, 10, 70, 1);

    delay(4000);

    tft.fillScreen(ST77XX_BLACK); //filling screen in black
  }
 

int xPos = 0; // position of X

int graph_type = 1; //types of graphs starts from 0



void drawGraph(int type) { // funciton needed to draw the graph

  int value = (this_device->Read_a_Sensor(1)).toInt(); // This would be: this_device -> get_a_sensor(type);
  String name  = "Sensor: " + String(type); // This would be this_device->get_sensor_name();
  drawtext(name,ST77XX_WHITE,52,10,1); // displaying the name of the on-screen graph
  int yPos = map(value, 0, 1023, tft.height() - 1, 0);  // y value starts at 0 - Value for sensor
  tft.drawPixel(xPos, yPos, ST77XX_GREEN); // drawing the line
  xPos++; // adds one to the next positon for x values

  if (xPos >= tft.width()) { // funciton to ensure the code moves from left to right 
    xPos = 0;
    tft.fillScreen(ST77XX_BLACK);
    Serial.println(type);
  }
}


void SDI12_command_center() {
    if (Serial1.available()) {
        byte incomingByte = Serial1.read();  // Read incoming communication in bytes
        if (incomingByte == 33) {  // If byte is command terminator (!)
            SDI12Receive(Raw_command);
            Raw_command = "";  // Reset command string
        } else {
            if (incomingByte != 0) {  // Do not add start bit (0)
                Raw_command += char(incomingByte);  // Append byte to command string
            }
        }
    }
    yield();
}

 // TIME

void WriteSD(FsFile file, String sensorData) {
  Serial.println("---   Saving To File   ---");       // Serial message to show the function is called
  file.open("dataLogger.txt", O_WRITE | O_APPEND);    // Open the file to write at the end of the script
  file.println(sensorData);                           // Write the passed parameter sensorData into the file
  file.close();
}

void ReadSD(FsFile file) {
  Serial.println("--- Reading From file! ---");       // Serial message to show the function is called
  file.open("dataLogger.txt", O_RDWR);                // Open the file to read/write
  file.seek(0);                                       // Set the pointer to char 0
  String contents = file.readString();                // Obtain file contents as a string via readString()
  Serial.println(contents);                           // Print the contents string to the serial
  file.close();
}

String GetTime() {
  DateTime now = rtc.now();
  int hours = now.hour();
  int minutes = now.minute();
  int seconds = now.second();

  String currentTime = "Current time (RTC) : " + String(hours) + ":" + String(minutes) + ":" + String(seconds); 
  return currentTime;         // return formatted string of RTC time 
}

void buzzBuzzer() {
  digitalWrite(BUZZER, HIGH);         // Turn on the buzzer
  delay(500);                         // Buzz for 500 ms
  digitalWrite(BUZZER, LOW);          // Turn off the buzzer
  delay(500);                         // Pause for 500 ms
}

void log_data(){
  if(sensors_init_request){
    Serial.println(this_device->pretty_print_sensor_values());
    String currentTime = GetTime();
     String Sensor_Values = this_device->pretty_print_sensor_values();     // send sensor data to be formatted using pretty_print_sensor_values()
    WriteSD(file, currentTime);
    WriteSD(file, Sensor_Values);
    Serial.println("Hello, data has been logged");

  }
}


/**********************************************
 *       Default Arduino Setup function       *
 **********************************************/
void next_graph(){
  if(sensors_init_request){
    Serial.println("hey I was pressed");
    Serial.println(this_device->Read_a_Sensor(1));
    reset_screen();
    graph_type++;
  }
}


void setup() {
  //Arduino IDE Serial Monitors
  Serial.begin(9600);
  Serial.println("SETUP");
  // this_device.initialize_attached_sensors();
  pinMode(buttonPin, INPUT_PULLUP);

    tft.initR(INITR_BLACKTAB);      // initiaises the TFT display
    tft.setRotation(3); //Ensures the rotation is correct
    tft.fillScreen(ST77XX_BLACK);

    // CoolMenu();
    Serial.println("GUI INITIALISATION COMPLETE"); //graphical user interface

    Serial1.begin(1200, SERIAL_7E1);  //SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
    pinMode(DIRO, OUTPUT);               //DIRO Pin

  //HIGH to Receive from SDI-12
    digitalWrite(DIRO, HIGH);

    // attachInterrupt(BUTTON_PIN,next_graph,RISING); 
    Scheduler.startLoop(SDI12_command_center);
    

  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD card initialization failed!");     // check if SD card is initialized
    sd.initErrorHalt();
    while (1);
  }
    if (!file.open("dataLogger.txt", O_RDWR | O_CREAT)) {
      sd.errorHalt(F("File open failed!"));       // check if file is opened
  }

  file.close(); // Release file

}

/**********************************************
 *       Default Arduino Main function       *
 **********************************************/
unsigned long previousMillis;

void loop() {
  int byte;
  //Receive SDI-12 over UART and then print to Serial Monitor
  watchdogReset();
  delay(1000);  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 2000) {
    previousMillis = currentMillis;
    log_data();
    // Code to execute every 1 second
  }

  if(sensors_init_request){
  // drawGraph(graph_type); 
  }
}