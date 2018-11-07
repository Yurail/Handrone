// ---------------------------------------------------------------------------
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h" 
#include "Wire.h"
#include <SoftwareSerial.h>
#include <HMC5883L.h>
#include <MS561101BA.h>

// -------------Defining Magnetometer Memory Addresses & Constants------------
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07
int16_t mx, my; // Magnetic direction X & Y
float heading; // Heading direction
// ----------------------------Pin Number of Buttons--------------------------
#define bt1Pin     0x06 //UP
#define bt2Pin     0x07 //DOWN
#define bt3Pin     0x08 //RESET ANGLE
#define bt4Pin     0x09 //EMERGENCY STOP
// Initial button state
int bt1State = HIGH;
int bt2State = HIGH;
int bt3State = HIGH;
int bt4State = HIGH;
// Number of throttle count(height);
int Count = 0;
const int Step = 20;
// ------------------------------Defining Barometer Constants-----------------
#define MOVAVG_SIZE         32
#define STANDARD_PRESSURE   1013.25
MS561101BA baro = MS561101BA();
float movavg_buff[MOVAVG_SIZE];
int movavg_i = 0;
const float sea_press = 1015; // Standard sea pressure
float pressure, temp;
int altit; // Altitude
//value of throttle (0 ~ 1000) controlled by variable resistor
//value of throttle (0 ~ 1000) controlled by up/down button
int throttleVal;
// Offset between Current & Zero angle
float OFFSET[3] = {0,0,0};
// Instruction to Drone {Throttle, YAW, ROLL, PITCH}
int instruction[4] = {0,0,0,0};
// Bluetooth HC-06 port number(4-TX, 5-RX in HC-06)
SoftwareSerial mySerial(4, 5);
// ------------------------------Defining IMU MPU6050 Constants---------------
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // YAW,ROLL,PITCH
int angle[3]; // Offseted angles
int anglelimit = 10; // Threshold

void setup() {
    Wire.begin(); // Start wire communications
    //TWBR = 24;
    Serial.begin(9600); // Set wire communications speed
    delay(1);
    mySerial.begin(9600); // Set bluetooth communications speed
    delay(1);
    while (Serial.available() && Serial.read()); // Check wire communications on
    
    AccelerometerSetting(); // Initialize Accelerometer
    delay(1);
    MagnetometerSetting(); // Initialize Magnetometer
    delay(1);
    //BarometerSetting(); // Initialize Barometer
    //delay(1);
    pinMode(bt1Pin,INPUT_PULLUP);
    pinMode(bt2Pin,INPUT_PULLUP);
    pinMode(bt3Pin,INPUT_PULLUP);
    pinMode(bt4Pin,INPUT_PULLUP); // Make button pins as pullup resistors

    delay(2000);

}

void loop() {
    //BarometerCalculation();
    
    MagnetometerCalculation(); // Calculate heading direction(YAW)
    
    AccelerometerCalculation(); // Calculate angles(ROLL,PITCH)

    ResetButtonControl(); // Make Current angles as Zero angles
    
    ThrottleButtonControl(); // Change Throttle values

    AngleLimitation(); // Set Threshold to angles
    
    /*
    Serial.print(temp);
    Serial.print('@');
    Serial.print(pressure);
    Serial.print('@');
    Serial.println(altit);
    */
    // print values
    
    //Serial.print(ypr[0]*180/M_PI-OFFSET[0]);
    //Serial.print(ypr[1]*180/M_PI-OFFSET[1]);
    //Serial.print(ypr[2]*180/M_PI-OFFSET[2]);
    String a = String("") + throttleVal + "&" + angle[0] + "&" + angle[1] + "&" + angle[2] + "&";
    
    Serial.println(a);
    mySerial.println(a);
    
    bt4State = digitalRead(bt4Pin);
    bt3State = digitalRead(bt3Pin);
    bt2State = digitalRead(bt2Pin);
    bt1State = digitalRead(bt1Pin);

    delay(30);
}

void AccelerometerSetting() {
    mpu.initialize();
    mpu.dmpInitialize(); // Initialize
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85); // Set Gyroscope Offset
    mpu.setZAccelOffset(1788); // Set Accelerometer Offset
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true; // Make DMP mode ready
}
void MagnetometerSetting() {
    // Let Magnetometer Calculated in Accelerometer Chip (MPU6050)
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write(0x02); 
    Wire.write(0x00);
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write(0x00);
    Wire.write(B00001000);  // 75Hz Speed
    Wire.endTransmission();
    delay(5);
    mpu.setI2CBypassEnabled(0);
    // Setting magnetometer x direction
    mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
    mpu.setSlaveEnabled(0, true);
    mpu.setSlaveWordByteSwap(0, false);
    mpu.setSlaveWriteMode(0, false);
    mpu.setSlaveWordGroupOffset(0, false);
    mpu.setSlaveDataLength(0, 2);
    // Setting magnetometer y direction
    mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
    mpu.setSlaveEnabled(1, true);
    mpu.setSlaveWordByteSwap(1, false);
    mpu.setSlaveWriteMode(1, false);
    mpu.setSlaveWordGroupOffset(1, false);
    mpu.setSlaveDataLength(1, 2);
    // Setting magnetometer z direction
    mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
    mpu.setSlaveEnabled(2, true);
    mpu.setSlaveWordByteSwap(2, false);
    mpu.setSlaveWriteMode(2, false);
    mpu.setSlaveWordGroupOffset(2, false);
    mpu.setSlaveDataLength(2, 2);
    mpu.setI2CMasterModeEnabled(1);
}
/*
void BarometerSetting() {
    baro.init(MS561101BA_ADDR_CSB_LOW);
    delay(1);
    for(int i=0; i<MOVAVG_SIZE; i++) {
      movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
    }
}

void BarometerCalculation() {
    float temperature = baro.getTemperature(MS561101BA_OSR_4096);
    if(temperature) {
      temp = temperature;
    }
    pressure = baro.getPressure(MS561101BA_OSR_4096);
    if(pressure!=NULL) {
      pushAvg(pressure);
    }
    pressure = getAvg(movavg_buff, MOVAVG_SIZE);
    pressure += 2150;
    altit = int(getAltitude(pressure,temp))/10;
}

float getAltitude(float pressure, float temp) {
  return -((pow((sea_press / pressure), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
  float sum = 0.0;
  for(int i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}
*/
void MagnetometerCalculation() {
    // Get data from External Sensor to IMU (MPU6050)
    mx=mpu.getExternalSensorWord(0);
    my=mpu.getExternalSensorWord(2);
    // Calculate Heading direction
    heading = atan2(my, mx);
}

void AccelerometerCalculation() {
    if(!dmpReady) {
      ArduinoReset(); // If Error in Accelerometer, Reset Whole Arduino
    }
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        mpu.resetFIFO(); // If Packet Buffer full, Reset
    }
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); // Get Packet Buffer
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer); // Get Quaternion from Buffer
    mpu.dmpGetGravity(&gravity, &q); // Get Gravity from Quaternion
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get angles from Quaternion & Gravity
    ypr[0] = heading; // Set Yaw angle from Magnetometer
}

void ResetButtonControl() {
    // reset angles to current values
    // Current angles becomes Zero angles
    if((bt3State!=digitalRead(bt3Pin))&&(digitalRead(bt3Pin)==LOW)) {
      // When Falling Edge, Reset angles
      // bt3State is LOW in normal, become HIGH when click button
      // At that moment, bt3State still HIGH, however pin vale becomes LOW
      OFFSET[0] = ypr[0]*180/M_PI;
      OFFSET[1] = ypr[1]*180/M_PI;
      OFFSET[2] = ypr[2]*180/M_PI;
    }
}

void ThrottleButtonControl() {
  throttleVal = Count * Step;
    if((bt4State!=digitalRead(bt4Pin))&&(digitalRead(bt4Pin)==LOW)) {
      Count=0;
    }
    else if((bt2State!=digitalRead(bt2Pin))&&(digitalRead(bt2Pin)==LOW)) {
      Count--;
    }
    else if((bt1State!=digitalRead(bt1Pin))&&(digitalRead(bt1Pin)==LOW)) {
      Count++;
    }
    
  //throttleVal = map(analogRead(A0),0,1023,0,1000); // Change throttle using potentiometer
  //if(Serial.available()) // When Serial Communications available
  //throttleVal = Serial.parseInt(); // Change throttle using Serial Monitor's input
  //Kpcontrol = map(analogRead(A1),0,1023,0,1000); // Change Kp using potentiometer 
  //Kp = float(Kpcontrol)/100; // Use value between 0~10
}

/*
void ThrottleButtonControl() {
    // Up / Down / Stop values
    if((bt4State!=digitalRead(bt4Pin))&&(digitalRead(bt4Pin)==LOW)) {
      mySerial.print('e');
      Serial.print('e');
    }
    else if((bt2State!=digitalRead(bt2Pin))&&(digitalRead(bt2Pin)==LOW)) {
      mySerial.print('d');
      Serial.print('d');
    }
    else if((bt1State!=digitalRead(bt1Pin))&&(digitalRead(bt1Pin)==LOW)) {
      mySerial.print('u');
      Serial.print('u');
    }
    else {
      mySerial.print('n');
      Serial.print('n');
    }
}
*/
void AngleLimitation(){
  // Get angle data with Offset values
  angle[0] = int(ypr[0]*180/M_PI-OFFSET[0]);
  angle[1] = int(ypr[1]*180/M_PI-OFFSET[1]);
  angle[2] = int(ypr[2]*180/M_PI-OFFSET[2]);

  for (int i=0;i<3;i++){ // Repeat 3 times for 3 axis
    if(angle[i]>3*anglelimit){ // Treat angle above anglelimit
      angle[i] = -6;
    }
    else if(angle[i]>2*anglelimit){
      angle[i] = -4;
    }
    else if(angle[i]>anglelimit){
      angle[i] = -2;
    }
    else if(angle[i]>-anglelimit){
      angle[i] = 0;
    }
    else if(angle[i]>-2*anglelimit){
      angle[i] = 2;
    }
    else if(angle[i]>-3*anglelimit){
      angle[i] = 4;
    }
    else{
      angle[i] = 6;
    }
  }
}

// Arduino Reset function
void ArduinoReset(){
    asm volatile ("jmp 0");
}
