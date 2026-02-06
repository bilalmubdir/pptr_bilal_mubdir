#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <HX711_ADC.h>

//////// Network Hotspot Settings ////////////////////////////////
const char* ssid = "Access Point name"; // WiFi SSID
const char* password = "password";        // WiFi password
// UDP Configuration
WiFiUDP udp;   // UDP Server
const unsigned int udpPort = 4210;  // UDP port
char packetBuffer[255]; // Buffer for received packets
IPAddress remoteIP;    // variable for ip address
unsigned int remotePort;  // variable for port
/////////////////////////////////////////////////////////////////


//////// Loadcells Sensors ////////////////////////////////
const int HX711_dout_1 = D5; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = D0; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = D6; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = D0; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = D7; //mcu > HX711 no 2 dout pin
const int HX711_sck_3 = D0; //mcu > HX711 no 2 sck pin

HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
const int calVal_eepromAdress_3 = 8; // eeprom adress for calibration value load cell 2 (4 bytes)

unsigned long t = 0;
////////////////////////////////////////////////////////////////

////// Pulse Measurement //////////////////////////////////////
volatile unsigned long lastPulseTime = 0; 
volatile unsigned long pulseInterval = 0;
volatile float frequency = 0.0;
volatile float lastFrequency = 0.0;
volatile float RPM = 0.0;
const int PulsesSensPin = D4; 
////////////////////////////////////////////////////////////////


///// Power Measurement ///////////////////////////////////////
#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
/////////////////////////////////////////////////////////////

///// ESC Settings //////////////////////////////////////////////////////
Servo ESC;  // create servo object
/////////////////////////////////////////////////////////////////////////

/// Variables /////////////////////////
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
unsigned long stabilizingtime = 5000; // tare preciscion can be improved by adding a few seconds of stabilizing time
boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
byte loadcell_1_rdy = 0;
byte loadcell_2_rdy = 0;
byte loadcell_3_rdy = 0;
String process = "";
String option = "";
//float torque1,torque2,thrust;
String forces = "0.0,0.0,0.0";

///// Display Settings ///////////////////////////////////////
int tm, displayTime = 200;
/////////////////////////////////////////////////////////////

///// Loadcells Sensors //////////////////////////
float calibrationValue_1 = 1500.0; // Right torque sensor calibration scale factor
float calibrationValue_2 = 1500.0; // Left torque sensor calibration scale factor
float calibrationValue_3 = 2159.35; //  Thrust sensor calibration scale factor
/////////////////////////////////////////////////////////////


/////// Pulse Measuring Function /////////////////////////////////////////////////
void IRAM_ATTR countPulse() {
    unsigned long currentTime = micros();
    
    if (lastPulseTime > 0) {  // Ignore first pulse
        unsigned long newInterval = currentTime - lastPulseTime;
        // Ignore spurious pulses with extremely short intervals (e.g., noise)
        float tmp = 1.0 / (newInterval / 1e6);  // Convert µs to seconds
        float diff = abs(tmp - lastFrequency);
        if (newInterval > 500 && diff<=10.0) {  // Minimum interval = 500 µs (Max ~2000 Hz)
            pulseInterval = newInterval;
            frequency = tmp; //1.0 / (pulseInterval / 1e6);  // Convert µs to seconds
            lastFrequency =  frequency;    
        }
    }

    lastPulseTime = currentTime;
}
/////////////////////////////////////////////////////////////////////////////////////




void setup() {
  Serial.begin(115200);
  ESC.attach(D3, 1000, 2000);   /// set the min/max time for the ESC signal
  ESC.write(0);  // Wait for input
  wifi_setup();    // set up the network and start the server
  pinMode(PulsesSensPin, INPUT_PULLUP);  // pulses port
  Serial.println("Wait until all sensors initialized ....");
  ////// Power Measurement ///////////////////////////////////////
  Wire.begin();
  Wire.setClock(20000L);
  if(!ina226.init()){
    Serial.println("Failed to init INA226. Check your wiring.");
    while(1){}
  }
  ina226.setAverage(AVERAGE_256);  // number of samples to be averaged
  ina226.setConversionTime(CONV_TIME_1100); /// convertion time
  ina226.setResistorRange(0.01,20); // choose resistor 5 mOhm and gain range up to 10 A
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  ina226.setCorrectionFactor(1.03); // correction factor used for calibration
  //////////////////////////////////////////////////////////////////////////////////////////////


  ////// Loadcells Sensors //////////////////////////////////////////////////////////////////
  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  attachInterrupt(digitalPinToInterrupt(PulsesSensPin), countPulse, FALLING);  // Trigger on rising edge
  Serial.println("Initialization is complete");
}


void loop() {
   ///////////////// Loadcells Sensors ///////////////////
   static boolean newDataReady = 0;
   // check for new data/start next conversion:
    if (LoadCell_1.update()) newDataReady = true;
    LoadCell_2.update();
    LoadCell_3.update();
    if ((newDataReady)) {
      float torque1 = LoadCell_1.getData();
      float torque2 = LoadCell_2.getData();
      float thrust = LoadCell_3.getData();
      newDataReady = 0;
      forces = String(torque1) + "," + String(torque2) + "," + String(thrust) ;
    ////////////////////////////////////////////////////////////
  int packetSize = udp.parsePacket();
  if (packetSize) {
     udp.read(packetBuffer, sizeof(packetBuffer));
     packetBuffer[packetSize] = '\0';   // Null terminate the string
     String payload(packetBuffer);
     process = getValue(payload,',', 0);  
     ///// set the motor only  
     if (process=="CM1"){
        option = getValue(payload,',', 1);
        ESC.write(option.toInt());
     }
      ///// set the motor and read sensors OR reading sensors only
     if (process == "CM2"||process == "CM3") {
        // active only when motor is controlled
        if (process == "CM2"){
          option = getValue(payload,',', 1);
          ESC.write(option.toInt());
        }
        remoteIP = udp.remoteIP();     // get the remote ip address
        remotePort = udp.remotePort(); // get the remote udp port
    
        /// Power Measurement Section //////////////////////////////
        shuntVoltage_mV = ina226.getShuntVoltage_mV()*1000;

          // Convert sensor values to a string
          String response = String(frequency) + "," +
                            String(forces) + "," +
                            String(shuntVoltage_mV)+"," +
                            String(busVoltage_V);
        // Send response
        udp.beginPacket(remoteIP, remotePort);
        udp.print(response);
        udp.endPacket();
      }

      if (process=="CM4"){
        Serial.println("Calibrate ESC");
         option = getValue(payload,',', 1);
         ESC_calibration(option.toInt());
      }
      if (process=="CM5"){
         Serial.println("Tare Thrust and Torque Sensors");
         loadcells_tare();
         Serial.println("Tare process completed");
      }
      if (process=="CM6"){
         Serial.println("Safely switching off the motor");
         for (int kk=ESC.read()-1;kk>-1;--kk){
          ESC.write(kk);
          delay(100);
          Serial.println(kk);
         }
         Serial.println("motor is OFF now");
      }
      if (process=="CM7"){
         Serial.println("Safely starting up the motor");
         option = getValue(payload,',', 1);
         for (int kk=ESC.read()+1;kk<option.toInt()+1;++kk){
          ESC.write(kk);
          delay(100);
          Serial.println(kk);
         }
         Serial.println("motor is ON now");
      }
      if (process=="CM8"){
          remoteIP = udp.remoteIP();     // get the remote ip address
          remotePort = udp.remotePort(); // get the remote udp port
          udp.beginPacket(remoteIP, remotePort);
          udp.print("Okay");
          udp.endPacket();
          Serial.println("CM8");
      }   
  }

}



/// ESC Calibration Function ///////////////////////////////////////////////////////
void ESC_calibration(byte pulseSize){
  if (pulseSize==1){
     Serial.println("Now writing maximum output.");
     Serial.println("Turn on power source, then wait 2 seconds and press any key.");
     ESC.write(180);  // Wait for input
  }else{
   ESC.write(0);
    Serial.println("motor is ready now");
  }
}
////////////////////////////////////////////////////////////////////////////////////



////// Loadcells Sensors ////////////////////////////////////
// receive command 't' to initiate tare operation:
void loadcells_tare(){
  LoadCell_1.tareNoDelay();
  LoadCell_2.tareNoDelay();
  LoadCell_3.tareNoDelay();
    //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
//    Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
//    Serial.println("Tare load cell 2 complete");
  }
  if (LoadCell_3.getTareStatus() == true) {
//    Serial.println("Tare load cell 3 complete");
  }
}
///////////////////////////////////////////////////////////



//// Setting Up the Network function ////////
void wifi_setup(){
  // Connect to WiFi
  WiFi.softAP(ssid, password);
  Serial.println(" ");
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start UDP Server
  udp.begin(udpPort);
  Serial.print("Listening on UDP port ");
  Serial.println(udpPort);
}
/////////////////////////////////////////////


/// Parsing data function //////////////////////////////
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<= maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        if (i == maxIndex){
          strIndex[1] = i+1;
        }
        else {strIndex[1] = i;}
    }
  }
  if (found>index){
    return data.substring(strIndex[0], strIndex[1]);
  }
  else { return "";} 
}
////////////////////////////////////////////////////////////
