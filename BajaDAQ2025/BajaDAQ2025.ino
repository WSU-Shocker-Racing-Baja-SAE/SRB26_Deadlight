//Wiring for Thermocouple Amp (MAX31855)
//Connect Vin to 5V
//Connect GND to GND on ESP32
//Connect DO to GPIO 37
//Connect CS to GPIO 38
//Connect CLK to GPIO 41 (alternatively GPIO 40, 41, 20, 19, or 18)

//Wiring for GPS (PTMK3339)
// Connect VIN to 5V
// Connect GND to Ground
// Connect GPS RX (data into GPS) to Digital 18
// Connect GPS TX (data out from GPS) to Digital 17

//Wiring for Thermistor
//In series with a 10k ohm resistor
//5V to one end of thermistor
//GND to end of resistor
//GPIO 1 to where resistor and thermistor connect

//Wiring for Gyro/Accel (LSM6DSOX+LIS3MDL)
//VIN to ESP32S3 5V
//GND to ESP32S3 GND
//SCL to GPIO 5
//SDA to GPIO 4

//Wiring for display
//Backlight to GPIO 7   //These are set in userSetup.h in the TFT_eSPI library
//MISO not used
//MOSI to GPIO 11
//SCLK to GPIO 12
//CS to GPIO 10
//DC to GPIO 9
//RST to 3V
//VCC to 3V3
//GND to GND

//Wiring for SD card Breakout
//VIN to 5V
//GND to GND
//CLK to
//DO to
//DI to
//CS to 

//Wiring for Battery Monitoring
//Battery positive to 30kohm resistor
//30kohm resistor to 10kohm resitor and GPIO 6 (19)
//10kohm resistor to GND and battery negative

//

#include <TFT_eSPI.h>
#include "SD.h"
#include "FS.h"
#include "PubSubClient.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <ezButton.h>
#include "Shocker_Racing_Logo.h"

TFT_eSPI tft = TFT_eSPI();

Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

//const int CSpin = 10;
//String dataString;
//File sensorData;

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
String timestamp;

#define I2C_SDA 4
#define I2C_SCL 5

int ThermistorPin = 1;
float R1 = 10000;
float Tf = 0;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int cvt_temp=300;

#define MAXDO 37
#define MAXCS 38
#define MAXCLK 41
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

float velocity = 0.0;
float acc = 0.0;
float gps_speed = 0.0;
float dt = 0.2;
float speed_mph = 0;

float X[2] = { 0.0, 0.0 };
float P[2][2] = { { 1, 0 }, { 0, 1 } };
float Q[2][2] = { { 0.01, 0 }, { 0, 0.05 } };
float R = 0.05;
float K[2];

double tempF = 0;
int engine_temp=300;

int xpos;
int ypos;
int xpos2;
int ypos2;
int xpos3;
int ypos3;
int xpos4;
int ypos4;
int xpos5;
int textSize;

#define batteryPin 6
const int R3=33800;  //~33k
const int R4=10000;   //~10k
float batt_volt=0.0;
float old_batt_volt=0.0;
int i=1;
int low =0;

#define REASSIGN_PINS
int sck = 12;
int miso = 13;
int mosi = 11;
int cs = 10;

float last_log=0;
float log_interval=1000;   //1 second

#define DEBOUNCE_TIME 110
ezButton button(21);

enum LapState { DISARMED, ARMED, BASE_LAP };
LapState currentLapState = DISARMED;

struct GeoPoint {
  float lat;
  float lon;
};
const int MAX_BASE_POINTS = 300; 
GeoPoint baseline[MAX_BASE_POINTS];
int pointCount = 0;

float startLat = 0.0, startLon = 0.0;
float minLat = 90.0, maxLat = -90.0, minLon = 180.0, maxLon = -180.0;

unsigned long lapStartTime = 0;
unsigned long lastLapTime = 0;
unsigned long bestLapTime = 4294967295; 
bool inZone = false;
const float TRIGGER_RADIUS_SQ = 0.0000000225; // ~15 meter radius geofence

unsigned long lastPointSave = 0;
unsigned long displayTimer = 0;

/*void createDir(fs::FS &fs, const char *path) {
  fs.mkdir(path);
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  file.println(message);
  file.close();
  Serial.println("Data wrote");
}*/

void setup() {
  tft.init();
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  tft.setRotation(1);
  
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 480, 320, Shocker_Racing_Logo);
  delay(1000);

  //pinMode(21, INPUT_PULLUP);
  //button.setDebounceTime(DEBOUNCE_TIME);

  analogSetAttenuation(ADC_11db);
  pinMode(batteryPin, INPUT);

  //pinMode(cs, OUTPUT);
  //digitalWrite(cs, HIGH);
  //SPI.begin(sck, miso, mosi, cs);
  //SD.begin(cs);
  //createDir(SD,"/sensorData");

  //while (!Serial) delay(10);

  bool lsm6ds_success, lis3mdl_success;
  lsm6ds_success = lsm6ds.begin_I2C(0x6B);
  lis3mdl_success = lis3mdl.begin_I2C(0x1C);
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(10);
    }
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true);
 
  Serial1.begin(9600, SERIAL_8N1, 17, 18);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

  while (!thermocouple.begin()) delay(10);

  tft.fillScreen(TFT_WHITE);
  tft.setTextFont(4);
  tft.setTextColor(TFT_BLACK);
  xpos = 180;
  ypos = 165 + tft.fontHeight();
  textSize = 2;
  tft.setTextSize(textSize);
  tft.drawString("MPH", xpos, ypos);

  textSize = 1;
  tft.setTextSize(textSize);
  tft.setTextColor(TFT_DARKGREEN);
  xpos2 = 10;
  ypos2 = 10;
  tft.drawString("Engine", xpos2, ypos2);
  xpos3 = xpos2+tft.textWidth("Engine")+270;
  ypos3=ypos2;
  xpos2+=50;
  ypos2+=tft.fontHeight();

  tft.setTextColor(TFT_RED);
  tft.setCursor(xpos3,ypos3);
  tft.print("Gearcase");
  ypos3+=tft.fontHeight();

  xpos4=10;
  ypos4=250;
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(xpos4,ypos4);
  tft.print("Battery");
  ypos4+=tft.fontHeight();
}

void loop() {
  //Serial.print("GPS Fix: ");
  //Serial.println(GPS.fix);
  //Serial.print("Satellites: ");
  //Serial.println(GPS.satellites);

  //############### Accelerometer & Gyroscope ####################
  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  /*acc = accel.acceleration.x;
  X[0] = X[0] + (acc * dt);
  X[1] = acc;

  P[0][0] += Q[0][0];
  P[1][1] += Q[1][1];*/

  //###################### Thermistor #############################
  float old_Tf = Tf;
  float Vo = analogRead(ThermistorPin);
  if (Vo < 1023) {
    float R2 = R1 * ((1023.0 / (float)Vo) - 1.0);
    float logR2 = log(R2);
    float T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    float Tc = T - 273.15;
    Tf = (Tc * 9.0) / 5.0 + 32.0;
  }
  else if(Vo>=1023){
    Tf=585;
  }

  //########################## GPS ###############################
  float old_speed = speed_mph;
  while (Serial1.available()) {
    GPS.read();
  }

  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      timestamp = String(GPS.year + 2000) + "-" +
                  (GPS.month < 10 ? "0" : "") + String(GPS.month) + "-" + 
                  (GPS.day < 10 ? "0" : "") + String(GPS.day) + " " +
                  ((GPS.hour) < 10 ? "0" : "") + String(GPS.hour) + ":" +
                  (GPS.minute < 10 ? "0" : "") + String(GPS.minute) + ":" +
                  (GPS.seconds < 10 ? "0" : "") + String(GPS.seconds);

      float gps_speed = GPS.speed * 0.51444;

      if (gps_speed<0.3){
        speed_mph=0;
        /*X[0]=0;
        X[1]=0;
        P[0][0]=1;
        P[1][1]=1;*/
      } else {
        /*K[0] = P[0][0] / (P[0][0] + R);
        K[1] = P[1][1] / (P[1][1] + R);

        X[0] = X[0] + K[0] * (gps_speed - X[0]);
        X[1] = X[1] + K[1] * (gps_speed - X[1]);

        P[0][0] *= (1 - K[0]);
        P[1][1] *= (1 - K[1]);

        speed_mph = (X[0]) * 2.237;*/
        speed_mph=gps_speed*2.237;
      }
    }
  }

  //###################### Thermocouple ###########################
  double old_temp = tempF;
  tempF = thermocouple.readFahrenheit();
  if (isnan(tempF)) tempF=0.0;

  //#################### Battery Voltage ##########################
  old_batt_volt = batt_volt;
  float voltOut=analogReadMilliVolts(batteryPin);
  batt_volt=((voltOut*(R3+R4))/R4)/1000;
  //Serial.println(voltOut);

  //#################### Save Data to CSV #########################
  /*if (millis()-last_log>=log_interval){
    last_log=millis();
    dataString = timestamp + "," +
                    String(GPS.latitudeDegrees, 4) + "," + 
                    String(GPS.longitudeDegrees, 4) + "," +
                    String(GPS.altitude) + "," +
                    String(accel.acceleration.x,4) + "," + 
                    String(accel.acceleration.y,4) + "," + 
                    String(accel.acceleration.z,4) + "," +
                    String(gyro.gyro.x,4) + "," + 
                    String(gyro.gyro.y,4) + "," + 
                    String(gyro.gyro.z,4) + "," +
                    String(mag.magnetic.x,4) + "," + 
                    String(mag.magnetic.y,4) + "," + 
                    String(mag.magnetic.z,4) + "," +
                    String(Tf,4) + "," + 
                    String(speed_mph,4) + "," +
                    String(tempF,4) + "," +
                    String(batt_volt,4);
    writeFile(SD, "/sensorData/data.csv", dataString.c_str());
  }
  */
  //######################## Display ###########################
  tft.setTextFont(4);
  if (String(speed_mph,0) != String(old_speed,0)) {
    textSize = 4;
    tft.setTextSize(textSize);
    tft.setTextColor(TFT_BLACK);
    int xpos1 = 200;
    int ypos1 = 100;
    int speed_length = tft.textWidth(String("00"));
    tft.fillRect(xpos1, ypos1, speed_length, tft.fontHeight()-15, TFT_WHITE);
    tft.setCursor(xpos1, ypos1);
    if (speed_mph<1){
      tft.print(String(0));
    }
    else{
      tft.print(String((speed_mph+0.5), 0));
    }
  }
  if (String(tempF) != String(old_temp)) {
    textSize = 2;
    tft.setTextSize(textSize);
    if (tempF>engine_temp){
      tft.setTextColor(TFT_RED);
    }
    else {
      tft.setTextColor(TFT_DARKGREEN);
    }
    int temp_length = tft.textWidth(String(old_temp, 2));
    tft.fillRect(xpos2, ypos2, temp_length, tft.fontHeight(), TFT_WHITE);
    tft.setCursor(xpos2, ypos2);
    tft.print(String((tempF+0.5), 0));
    //tft.println(" F");
  }
  if (Tf != old_Tf) {
    if (Tf<585){
      tft.setTextSize(3);
      if (Tf>cvt_temp){
        tft.setTextColor(TFT_RED);
      }
      else {
        tft.setTextColor(TFT_DARKGREEN);
      }
      int Tf_length = tft.textWidth(String(old_Tf, 2));
      tft.fillRect(335, ypos3, Tf_length, tft.fontHeight(), TFT_WHITE);
      tft.setCursor(335, ypos3);
      tft.print(String(Tf,0));
    // tft.println(" F");
    }
    else if (Tf==585) {
      tft.setTextColor(TFT_RED);
      tft.setTextSize(2);
      int Tf_length = tft.textWidth(String(old_Tf, 2));
      tft.fillRect(320, ypos3, Tf_length, tft.fontHeight(), TFT_WHITE);
      tft.setCursor(320, ypos3);
      tft.print(">585 F");
    } 
  }

  if (batt_volt != old_batt_volt) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_BLACK);
    int batt_volt_length = tft.textWidth(String(old_batt_volt, 2));
    tft.fillRect(10, ypos4, batt_volt_length, tft.fontHeight(), TFT_WHITE);
    tft.setCursor(10, ypos4);
    tft.print(batt_volt);
    if (batt_volt<11.8){
      if(i>0){
        xpos5+=10+batt_volt_length+20;
        i=i-1;
      }
      tft.setCursor(xpos5, ypos4);
      tft.setTextColor(TFT_RED);
      tft.print("LOW");
      low=1;
    }
    else if (low==1) {
      tft.fillRect(xpos5, ypos4, tft.textWidth(String("LOW")), tft.fontHeight(), TFT_WHITE);
      low=0;
    }
  }
  //###################### Lap Timing #########################
  //button debouncing!!!
  /*if (base_lap==1 && button_pressed==1){
    //Save gps data and when car is within 5 meters of starting point end data collection
    //identify start point
    //create start zone using radius
    //save new coordinates until starting zone is reentered
    //Switch gps coordinates to cartesian and scale to fit on display
    //This will be used to create an outline of the track
    //Maybe triple press allows for baseline retake
    base_lap=0;
  }
  
  if (button_pressed==1) button=1;
  else if (button_pressed==1 && button==1) button=0;

  if (button==1 && //car has moved out of start zone)   //maybe use while loop
  {
    start_time=millis();//start lap time
    if (//car reenters the zone)
    {
      end_time=millis();
      laptime=end_time-start_time;
      
    }
  }
d
  //log lap time on SD
  //compare to best lap or if 2nd lap compare to base
  //store on board if best lap and display best lap until button is pressed again
  //Display laptime
  */

  delay(10);
}