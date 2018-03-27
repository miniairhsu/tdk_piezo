#ifndef I2C1_H		//Do only once the first time this file is used
#define	I2C1_H
#define FW_MAJOR 1
#define FW_MINOR 0
#define AK7375_ADDR 0xF
#define CONT1      0x02
#define AK7375_POS 0x84
#define NVM_BASE 0xF0
#define NVM_SIZE 16


#define BU6456_ADDR 0xC
#define SET 0x0
#define TA 0x1
#define BRAKE1 0x2
#define TB 0x03
#define BRAKE2 0x04
#define CNT1 0x05
#define CNT2 0x06
#define OSC 0x07
#define V1 0x08
#define V2 0x09
#define STEP 0xA
#define OPT 0x80

//pwm parameter
#define FREQ_DEFAULT 15000
#define PWM_DEFAULT 120
#define TOTAL_CNT(f) (FREQ_DEFAULT/f) // f in kHz 
#define TAC(cnt, d) (cnt * d / 100) //TA counts => cnt => total count, d => duty in percentage
#define BR1 1
#define BR2 24

//PID controller parameter
#define kp 0.01
#define ki 0.0002
#define kd 0.005

int nPWM_FREQ = 120;
int nDuty = 20;
int convert_ms_to_pulse(int ms, int nFreq);
void calibration();
char * readBytes(int addr, int reg, int nBytes);
int read_Hall_Pos();
void writeByte(int addr, int reg, char data);
void setPWM(int nLength);
void rise_fall();
void getVersion();
void forward();
void backward();
void setFreq(int nFreq);
void position_pid(int nTarget)__attribute__((__optimize__("O2")));
void stop_Motor();
void writeNVM(int addr, int data)__attribute__((__optimize__("O2")));
void readNVM(int addr)__attribute__((__optimize__("O1")));
#endif


