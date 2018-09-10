#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include <SoftwareSerial.h>

#define YAW   0
#define ROLL  1
#define PITCH 2
#define ANG   3
#define NUM   4

#define MotorZero 0
#define MotorTestZero 100
#define MotorArm 500
#define MotorLow 1000
#define MotorHigh 2000

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[ANG];
int angle[ANG];
//---PID---//
float elapsedTime, Time, prevTime, InitialT, FinalT;
float errors[ANG] = {0,0,0};
float pi[ANG]      = {0,0,0};
float previous_error[ANG] = {0,0,0};
float deltaErr[ANG] = {0, 0, 0};
float Kp[ANG]={0,1.5,1.5};
float Ki[ANG]={0,0,0};
float Kd[ANG]={0,0,0};
int yaw = 0;
int pitch = 0;
int roll = 0;
unsigned long pulse_length_esc1 = 1000;
unsigned long pulse_length_esc2 = 1000;
unsigned long pulse_length_esc3 = 1000;
unsigned long pulse_length_esc4 = 1000;
//---Motor---//
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int CurrentSpeed;
int Step = 20;
//---Receiver---//
int instruction[ANG];

void setup() {

    Wire.begin();
    //TWBR = 24;
    Serial.begin(9600);
    while(!Serial){}
    Serial3.begin(9600);
    while(!Serial3){}
    
    esc1.attach(4,MotorLow,MotorHigh);
    esc1.writeMicroseconds(MotorLow);
    esc2.attach(5,MotorLow,MotorHigh);
    esc2.writeMicroseconds(MotorLow);
    esc3.attach(6,MotorLow,MotorHigh);
    esc3.writeMicroseconds(MotorLow);
    esc4.attach(7,MotorLow,MotorHigh);
    esc4.writeMicroseconds(MotorLow);
    delay(1000);
    
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    
    delay(500);
    Time = millis();
}

void loop() {
  if(Serial3.available()>0){
    ReadParts();
    CurrentSpeed = floor(CurrentSpeed/Step)*Step;
    /*
    if(CurrentSpeed>0&&CurrentSpeed<Step){
      CurrentSpeed = PrevSpeed;
    }
    if((CurrentSpeed-PrevSpeed)<40&&(PrevSpeed-CurrentSpeed)<40){
      CurrentSpeed = floor(CurrentSpeed/Step)*Step;
      PrevSpeed = CurrentSpeed;
      for(int i=0;i<ANG;i++){
        Prev[i] = instruction[i];
      }
    }
    Serial.println(SpeedString);
    if(SpeedString=="u"){
      if(CurrentSpeed<MotorLow-Step)
        CurrentSpeed+=Step;
    }
    else if(SpeedString=="d"){
      if(CurrentSpeed>MotorZero+Step)
        CurrentSpeed-=Step;
    }
    else if(SpeedString=="e"){
      CurrentSpeed=MotorZero;
    }
    */
    /*
    Serial.print(CurrentSpeed);
    Serial.print("\t");
    Serial.print(instruction[YAW]);
    Serial.print("\t");
    Serial.print(instruction[PITCH]);
    Serial.print("\t");
    Serial.println(instruction[ROLL]);
    */
    }
    if(!dmpReady) {
      ArduinoReset();
    }
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= 1024) {
        mpu.resetFIFO();
    }
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    angle[YAW]=ypr[YAW]*180/M_PI-57;
    angle[PITCH]=ypr[PITCH]*180/M_PI-0.03;
    angle[ROLL]=ypr[ROLL]*180/M_PI-5.14;
    /*
    Serial.print(angle[YAW]);
    Serial.print("\t");
    Serial.print(angle[PITCH]);
    Serial.print("\t");
    Serial.println(angle[ROLL]);
    */
    prevTime = Time;
    Time = millis();
    elapsedTime = (Time-prevTime)/1000;

    pulse_length_esc1 = MotorLow+CurrentSpeed;
    pulse_length_esc2 = MotorLow+CurrentSpeed;
    pulse_length_esc3 = MotorLow+CurrentSpeed;
    pulse_length_esc4 = MotorLow+CurrentSpeed;
    
    //errors[PITCH] = angle[PITCH] - instruction[PITCH];
    //errors[ROLL] = angle[ROLL] - instruction[ROLL];
    errors[PITCH] = angle[PITCH];
    errors[ROLL] = angle[ROLL];
    
    if (CurrentSpeed > MotorTestZero) {
      if(abs(errors[PITCH])<5){
        pi[PITCH] += errors[PITCH]*Ki[PITCH];
      }
      deltaErr[PITCH] = errors[PITCH] - previous_error[PITCH];
      previous_error[PITCH] = errors[PITCH];
      pitch = (errors[PITCH] * Kp[PITCH]) + pi[PITCH] + (deltaErr[PITCH]/elapsedTime * Kd[PITCH]);
      
      if(abs(errors[ROLL])<5){
        pi[ROLL] += errors[ROLL]*Ki[ROLL];
      }
      deltaErr[ROLL] = errors[ROLL] - previous_error[ROLL];
      previous_error[ROLL] = errors[ROLL];
      roll = (errors[ROLL] * Kp[ROLL]) + pi[ROLL] + (deltaErr[ROLL]/elapsedTime * Kd[ROLL]);
      
      
      constrain(pitch,-300,300);
      constrain(roll,-300,300);
      
      pulse_length_esc1 = pulse_length_esc1+pitch+roll;
      pulse_length_esc2 = pulse_length_esc2-pitch+roll;
      pulse_length_esc3 = pulse_length_esc3+pitch-roll;
      pulse_length_esc4 = pulse_length_esc4-pitch-roll;

      pulse_length_esc2 *= 0.97;
      
      constrain(pulse_length_esc1,1100,1800);
      constrain(pulse_length_esc2,1100,1800);
      constrain(pulse_length_esc3,1100,1800);
      constrain(pulse_length_esc4,1100,1800);
      
      Serial.print(pulse_length_esc1);
      Serial.print("\t");
      Serial.print(pulse_length_esc2);
      Serial.print("\t");
      Serial.print(pulse_length_esc3);
      Serial.print("\t");
      Serial.print(pulse_length_esc4);
      Serial.print("\t");
      Serial.print(roll);
      Serial.print("\t");
      Serial.println(pitch);
      
    }
    esc1.writeMicroseconds(pulse_length_esc1);
    esc2.writeMicroseconds(pulse_length_esc2);
    esc3.writeMicroseconds(pulse_length_esc3);
    esc4.writeMicroseconds(pulse_length_esc4);
    /*
    Serial.print(pulse_length_esc1);
    Serial.print('\t');
    Serial.print(pulse_length_esc2);
    Serial.print('\t');
    Serial.print(pulse_length_esc3);
    Serial.print('\t');
    Serial.println(pulse_length_esc4);
    */
}

void ReadParts() {
    String temp = Serial3.readStringUntil('\n');
    byte from = 0 ;
    byte to = temp.indexOf('&', 0);
    CurrentSpeed = temp.substring(from, to).toInt();
    from = to + 1;
    to = temp.indexOf('&', from);
    instruction[YAW] = temp.substring(from, to).toInt();
    from = to + 1;
    to = temp.indexOf('&', from);
    instruction[PITCH] = temp.substring(from, to).toInt();
    from = to + 1;
    to = temp.indexOf('&', from);
    instruction[ROLL] = temp.substring(from, to).toInt();
}

void ArduinoReset(){
    asm volatile ("jmp 0");
}
