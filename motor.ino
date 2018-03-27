// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder
// by Nicholas Zambetti <http://www.zambetti.com>
// and James Tichenor <http://www.jamestichenor.net>

// Demonstrates use of the Wire library reading data from the
// Devantech Utrasonic Rangers SFR08 and SFR10

// Created 29 April 2006

// This example code is in the public domain.


#include <Wire.h>
#include "I2C1.h"
#include "io.h"
#include "main.h"
char* reading;
char* nvm;
char command[24];
char nRecv = 0;
//unsigned int nReading;
int ta;
int tb;
int cnt1 = 0x00;
int cnt2 = 0xFF;
int nCurrent_Pos = 0;
void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Wire.setClock(400000);
  // step 1: activate hall sensor
  writeByte(AK7375_ADDR, CONT1, 0x40);
  //TCCR1A = 0x00;                // Normal mode, just as a Timer
  //TCCR1B |= _BV(CS12);          // prescaler = CPU clock/1024
  //TCCR1B &= ~_BV(CS11);      
  //TCCR1B |= _BV(CS10);       
  //TCNT1 = 0;                     //64us per tick
  Serial.begin(115200);          // start serial communication at 115200bps
  getVersion();
}

void writeByte(int addr, int reg, char data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

char * readBytes(int addr, int reg, int nBytes) {
  char cResult[nBytes];
  int n;
  // step 1: instruct sensor to return a particular echo reading
  Wire.beginTransmission(addr); // transmit to device 0xF
  Wire.write(reg);      // sets register pointer to AD_POS1 register (0x84)
  Wire.endTransmission();      // stop transmitting
  // step 2: initialize reading values
  Wire.requestFrom(addr, nBytes);    // request 2 bytes from slave device 0xF
  // step 3: send sensor values
  while (Wire.available()) { // if two bytes were received
    for (n = 0; n < nBytes; n++)
      cResult[n] = Wire.read();
  }
  return cResult;
}

void getVersion() {
  Serial.print("Firmware version ");
  Serial.print(FW_MAJOR);
  Serial.print(".");
  Serial.print(FW_MINOR);
  Serial.print("\r\n");
}

int read_Hall_Pos() {
  reading = readBytes(AK7375_ADDR, AK7375_POS, 2);
  unsigned int nReading;
  nReading = reading[0] & 0xFF;
  nReading = nReading << 8;
  nReading |= reading[1] & 0xF0;
  nReading = nReading >> 4;
  return nReading;
}

void setFreq(int nFreq) {
  nPWM_FREQ = nFreq;
  Serial.print(nFreq);
  Serial.print("kHz");
  Serial.print("\r\n");
}

void setPWM(int nLength) {
  int nTotal_cnt = TOTAL_CNT(nPWM_FREQ);
  ta = TAC(nTotal_cnt, nDuty);
  tb = nTotal_cnt - ta - 30;
  cnt1 = (nLength & 0x00FF);
  cnt2 = (nLength & 0xFF00) >> 8;
}

int convert_ms_to_pulse(int ms, int nFreq) {
  return (ms * nFreq);
}

void rise_fall() {
  writeByte(BU6456_ADDR, OPT + TA, ta);
  writeByte(BU6456_ADDR, OPT + BRAKE1, 0x1);
  writeByte(BU6456_ADDR, OPT + TB, tb);
  writeByte(BU6456_ADDR, OPT + BRAKE2, 0x18);
  writeByte(BU6456_ADDR, OPT + CNT1, cnt1);
  writeByte(BU6456_ADDR, OPT + CNT2, cnt2);
  writeByte(BU6456_ADDR, OPT + OSC, 0x0);
  writeByte(BU6456_ADDR, OPT + V1, 0xFF);
  writeByte(BU6456_ADDR, OPT + V2, 0xFF);
  writeByte(BU6456_ADDR, OPT + STEP, 0x0);

}

void backward() {
  writeByte(BU6456_ADDR, OPT + SET, 0x80);
  rise_fall();
  writeByte(BU6456_ADDR, OPT + SET, 0x84);
  delay(50);
}

void forward() {
  writeByte(BU6456_ADDR, OPT + SET, 0x81);
  rise_fall();
  writeByte(BU6456_ADDR, OPT + SET, 0x85);
  delay(50);
}

void stop_Motor(){
  writeByte(BU6456_ADDR, OPT + SET, 0x80);
}

void calibration() {
  setPWM(convert_ms_to_pulse(200, nPWM_FREQ));                        
  backward();
  delay(200);
  for (int i = 0; i < 3; i++) {
    setPWM(convert_ms_to_pulse(70, nPWM_FREQ));                        
    backward();
    delay(200);
    Serial.print("Min Hall value : ");
    Serial.print(read_Hall_Pos());
    Serial.print("\r\n");
    delay(200);
    forward();
    delay(200);
    Serial.print("Max Hall value : ");
    Serial.print(read_Hall_Pos());
    Serial.print("\r\n");
    delay(200);
  }
  setPWM(3000);
  backward();
  delay(500);
  Serial.print("Finished \r\n");
}

void writeNVM(int addr, int data){
  writeByte(AK7375_ADDR, 0xAE, 0x3B);
  delay(20);
  writeByte(AK7375_ADDR, 0xF0, 0x05);
  delay(20);
  //writeByte(AK7375_ADDR, 0xAE, 0x01);
  Serial.print(NVM_BASE + addr);
  Serial.print("\r\n");
  delay(20);
}

void readNVM(int addr){
  //writeByte(AK7375_ADDR, 0xAE, 0x3B);
  reading = readBytes(AK7375_ADDR, NVM_BASE + addr, 1);
  delay(20);
  //writeByte(AK7375_ADDR, 0xAE, 0x01);
  //unsigned int nReading;
  //nReading = reading[0];
  Serial.print(reading[0], HEX);
  Serial.print("\r\n");
}

void position_pid(int nTarget) {
  char nStop = 0;
  int nTime = 0;
  int nError_Prev = 0;
  int nError = 0;
  int nIntegral = 0;
  int nDerv = 0;
  TCNT1 = 0;
  while (nStop != 1) {
    nCurrent_Pos = read_Hall_Pos();
    nError_Prev = nError;
    nError = nTarget - nCurrent_Pos;
    nIntegral = nIntegral + nError;
    nDerv = nError - nError_Prev;
    if (abs(nError) <= 15) {
      Serial.print(TCNT1);
      Serial.print("\r\n");
      nStop = 1;
    } else {
      nTime = abs(kp * nError + ki * nIntegral + kd * nDerv);
      setPWM(convert_ms_to_pulse(nTime, nPWM_FREQ));
      if (nError >= 0) {     
        forward();
      } else {
        backward();
      }
    }
  }
}

void serialRead() {
  int i = 0;
  nRecv = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    nRecv = 1;
    if (c != '\n')
    {
      command[i] = c;
      i = i + 1;
    }
  }
}

void loop() {
  memset(reading, 0, 2);
  delay(1000);                  // wait a bit since people have to read the output :)
  serialRead();
  if (nRecv == 1) {
    nRecv = 0;
    DecipherCommand(command);
  }
  memset(command, 0, sizeof(command));
}


