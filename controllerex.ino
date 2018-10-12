// ---------------------------------------------------------------------------
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h" 
#include "Wire.h"
#include <SoftwareSerial.h>
#include <HMC5883L.h>
#include <MS561101BA.h>

// ------------------------------Defining Constants---------------------------
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

// ----------------------------Pin Number of Buttons--------------------------
#define bt1Pin     0x06 //UP
#define bt2Pin     0x07 //DOWN
#define bt3Pin     0x08 //RESET ANGLE
#define bt4Pin     0x09 //EMERGENCY STOP

#define MOVAVG_SIZE         32
#define STANDARD_PRESSURE   1013.25
//-----------------------------------------------------------------------------

int16_t mx, my;
float heading;

MS561101BA baro = MS561101BA();
float movavg_buff[MOVAVG_SIZE];
int movavg_i = 0;
const float sea_press = 1015;
float pressure, temp;
int altit;

//value of throttle (0 ~ 1000) controlled by variable resistor
int throttleVal;
int Kpcontrol;
float Kp;

int bt1State = HIGH;
int bt2State = HIGH;
int bt3State = HIGH;
int bt4State = HIGH;
float OFFSET[3] = {0,0,0};
int instruction[4] = {0,0,0,0};

SoftwareSerial mySerial(4, 5);

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
int angle[3];
int anglelimit = 5;

void setup() {
    Wire.begin();
    //TWBR = 24;
    Serial.begin(9600);
    delay(1);
    mySerial.begin(9600);
    delay(1);
    while (Serial.available() && Serial.read());
    
    AccelometerSetting();
    delay(1);
    MagnetometerSetting();
    delay(1);
    //BarometerSetting();
    //delay(1);
    pinMode(bt1Pin,INPUT_PULLUP);
    pinMode(bt2Pin,INPUT_PULLUP);
    pinMode(bt3Pin,INPUT_PULLUP);
    pinMode(bt4Pin,INPUT_PULLUP);

    delay(2000);

}

void loop() {
    //BarometerCalculation();
    
    MagnetometerCalculation();
    
    AccelometerCalculation();

    ResetButtonControl();
    
    ThrottleButtonControl();    

    AngleLimitation();
    
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
    
    //Serial.print(a);
    mySerial.println(a);
    
    bt4State = digitalRead(bt4Pin);
    bt3State = digitalRead(bt3Pin);
    bt2State = digitalRead(bt2Pin);
    bt1State = digitalRead(bt1Pin);

    delay(30);
}

void AccelometerSetting() {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
}
void MagnetometerSetting() {
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write(0x02); 
    Wire.write(0x00);
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
    Wire.write(0x00);
    Wire.write(B00001000);  // 75Hz
    Wire.endTransmission();
    delay(5);
    mpu.setI2CBypassEnabled(0);
    //x
    mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
    mpu.setSlaveEnabled(0, true);
    mpu.setSlaveWordByteSwap(0, false);
    mpu.setSlaveWriteMode(0, false);
    mpu.setSlaveWordGroupOffset(0, false);
    mpu.setSlaveDataLength(0, 2);
    //y
    mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
    mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
    mpu.setSlaveEnabled(1, true);
    mpu.setSlaveWordByteSwap(1, false);
    mpu.setSlaveWriteMode(1, false);
    mpu.setSlaveWordGroupOffset(1, false);
    mpu.setSlaveDataLength(1, 2);
    //z
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
    mx=mpu.getExternalSensorWord(0);
    my=mpu.getExternalSensorWord(2);
    heading = atan2(my, mx);
}

void AccelometerCalculation() {
    if(!dmpReady) {
      ArduinoReset();
    }
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        mpu.resetFIFO();
    }
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] = heading;
}

void ResetButtonControl() {
    // reset angle with current values
    if((bt4State!=digitalRead(bt3Pin))&&(digitalRead(bt3Pin)==LOW)) {
      OFFSET[0] = ypr[0]*180/M_PI;
      OFFSET[1] = ypr[1]*180/M_PI;
      OFFSET[2] = ypr[2]*180/M_PI;
    }
}

void ThrottleButtonControl() {
  //throttleVal = map(analogRead(A0),0,1023,0,1000);
  if(Serial.available())
  throttleVal = Serial.parseInt();
  //Kpcontrol = map(analogRead(A1),0,1023,0,1000);
  //Kp = float(Kpcontrol)/100;
}

/*
void ThrottleButtonControl() {
    // Up / Down / Stop values
    if((bt3State!=digitalRead(bt4Pin))&&(digitalRead(bt4Pin)==LOW)) {
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
  angle[0] = int(ypr[0]*180/M_PI-OFFSET[0]);
  angle[1] = int(ypr[1]*180/M_PI-OFFSET[1]);
  angle[2] = int(ypr[2]*180/M_PI-OFFSET[2]);
  
  for (int i=0;i<3;i++){
    if(angle[i]>anglelimit)
      angle[i] = -2;
    else if(abs(angle[i])>anglelimit)
      angle[i] = 2;
    else
      angle[i] = 0;
  }
}

void ArduinoReset(){
    asm volatile ("jmp 0");
}
