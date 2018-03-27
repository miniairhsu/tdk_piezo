#include "io.h"
#include "I2C1.h"
extern unsigned int nReading;
extern char* nvm;
void ReplaceChar(char *szCommand, char cOld, char cNew){
  int i, nLen;
  nLen = strlen(szCommand);
  for (i = 0; i < nLen; i++)
  {
    if (szCommand[i] == cOld) szCommand[i] = cNew;
  }
}




/*
 * Function:  upCase 
 * --------------------
 * convert string to uppercase:
 *
 * p: string to be converted 
 * 
 *  returns: None
 */
void upCase(char *p){
  char i;
  i = 0;
  while(p[i]){
    p[i] = toupper(p[i]);
    i++;
  }
}

/*
 * Function:  DecipherCommand 
 * --------------------
 * decode command then execute:
 * 
 *  p: command string
 * 
 *  returns: int 
 */
int DecipherCommand(char *p){
  int nV1, nV2, nV3, nV4;
  int nPN ; 
  int i;
  unsigned short sVal;
  char p1[16], p2[32];
  char cV1;
  upCase(p); 

//calibrate motor
#define CMD_CALIBRATION ">CALIBRATION" 
#define LEN_CALIBRATION (sizeof(CMD_CALIBRATION)-1)
  if (strncmp(p, CMD_CALIBRATION, LEN_CALIBRATION) == 0) {
    calibration();
    goto DecipherOK;
  }
//enable debug mode
#define CMD_DEBUG ">DEBUG" 
#define LEN_DEBUG (sizeof(CMD_DEBUG)-1)
  if (strncmp(p, CMD_DEBUG, LEN_DEBUG) == 0) {
    //ReplaceChar(p, ':', ' ');
    nPN = sscanf(&p[LEN_DEBUG], "%d", &nV1); //nV1 debug mode number 
    nGDB = nV1;                //nV1 = 0, debug mode disable
    //=================================
    //nGDB = 1 => display lcc result
    //nGDB = 2 => display eeprom contents
    //nGDB = 3 => display motor position
    //=================================
    forward();
    goto DecipherOK;
  }

//forward motor calibration
#define CMD_CALFORWARD ">CALFORWARD" 
#define LEN_CALFORWARD (sizeof(CMD_CALFORWARD)-1)
  if (strncmp(p, CMD_CALFORWARD, LEN_CALFORWARD) == 0) {
    sscanf(&p[LEN_CALFORWARD], "%d", &nV1); //nV1 => pulse duration in ms
    setPWM(convert_ms_to_pulse(nV1, nPWM_FREQ));                        
    forward();  
    delay(500);
    Serial.print("Hall value : ");
    Serial.print(read_Hall_Pos());
    Serial.print("\r\n");
    delay(500);                                   
    goto DecipherOK;
  }

//backward motor calibration
#define CMD_CALBACKWARD ">CALBACKWARD" 
#define LEN_CALBACKWARD (sizeof(CMD_CALBACKWARD)-1)
  if (strncmp(p, CMD_CALBACKWARD, LEN_CALBACKWARD) == 0) {
    sscanf(&p[LEN_CALBACKWARD], "%d", &nV1); //nV1 => pulse duration in ms
    setPWM(convert_ms_to_pulse(nV1, nPWM_FREQ));                        
    backward();  
    delay(500);
    Serial.print("Hall value : ");
    Serial.print(read_Hall_Pos());
    Serial.print("\r\n");
    delay(500);                                   
    goto DecipherOK;
  }

//forward motor
#define CMD_FORWARD ">FORWARD" 
#define LEN_FORWARD (sizeof(CMD_FORWARD)-1)
  if (strncmp(p, CMD_FORWARD, LEN_FORWARD) == 0) {
    //ReplaceChar(p, ':', ' ');
    sscanf(&p[LEN_FORWARD], "%d", &nV1); //nV2 pulse no
    setPWM(nV1);                        
    forward();                                     
    goto DecipherOK;
  }
  
//backward motor
#define CMD_BACKWARD ">BACKWARD" 
#define LEN_BACKWARD (sizeof(CMD_BACKWARD)-1)
  if (strncmp(p, CMD_BACKWARD, LEN_BACKWARD) == 0) {
    //ReplaceChar(p, ':', ' ');
    sscanf(&p[LEN_BACKWARD], "%d", &nV1); //nV1 pulse no
    setPWM(nV1);                                                                        
    backward();
    goto DecipherOK;
  }

//move motor PID controller
#define CMD_PID ">MOVE " 
#define LEN_PID (sizeof(CMD_PID)-1)
  if (strncmp(p, CMD_PID, LEN_PID) == 0) {
    //ReplaceChar(p, ':', ' ');
    
    sscanf(&p[LEN_PID], "%d", &nV1); //nV1 target
    position_pid(nV1);
    goto DecipherOK;
  }

//get FW version
#define CMD_GETVERSION ">VERSION" 
#define LEN_GETVERSION (sizeof(CMD_GETVERSION)-1)
  if (strncmp(p, CMD_GETVERSION, LEN_GETVERSION) == 0) {
    getVersion();
    goto DecipherOK;
  }

//get FW version
#define CMD_INFO ">INFO" 
#define LEN_INFO (sizeof(CMD_INFO)-1)
  if (strncmp(p, CMD_INFO, LEN_INFO) == 0) {
    getVersion();
    goto DecipherOK;
  }

  
//read hall sensor ADC
#define CMD_READHALL ">READHALL" 
#define LEN_READHALL (sizeof(CMD_READHALL)-1)
  if (strncmp(p, CMD_READHALL, LEN_READHALL) == 0) {
    //ReplaceChar(p, ':', ' ');
    //nPN = sscanf(&p[LEN_DEBUG], "%d", &nV1); //nV1 debug mode number 
    Serial.print("Hall value : ");
    Serial.print(read_Hall_Pos());
    Serial.print("\r\n");
    goto DecipherOK;
  }

//set PWM frequency 
#define CMD_SETFREQ ">SETFREQ" 
#define LEN_SETFREQ (sizeof(CMD_SETFREQ)-1)
  if (strncmp(p, CMD_SETFREQ, LEN_SETFREQ) == 0) {
    //ReplaceChar(p, ':', ' ');
    sscanf(&p[LEN_SETFREQ], "%d", &nV1); //nV1 PWM frequency 
    setFreq(nV1);
    goto DecipherOK;
  }

//set PWM duty 
#define CMD_SETDUTY ">SETDUTY" 
#define LEN_SETDUTY (sizeof(CMD_SETDUTY)-1)
  if (strncmp(p, CMD_SETDUTY, LEN_SETDUTY) == 0) {
    //ReplaceChar(p, ':', ' ');
    sscanf(&p[LEN_SETDUTY], "%d", &nV1); //nV1 PWM duty 
    nDuty = nV1;
    Serial.print(nDuty);
    Serial.print("%");
    Serial.print("\r\n");
    goto DecipherOK;
  }

//Write NVM to EEPROM
#define CMD_WRNVM ">WRNVM" 
#define LEN_WRNVM (sizeof(CMD_WRNVM)-1)
  if (strncmp(p, CMD_WRNVM, LEN_WRNVM) == 0) {
    sscanf(&p[LEN_WRNVM], "%d %d", &nV1, &nV2); //nV1 PWM duty 
    writeNVM(nV1, nV2);
    Serial.print("NVM write done\r\n");
    goto DecipherOK;
  }

//Read NVM from EEPROM
#define CMD_RDNVM ">RDNVM" 
#define LEN_RDNVM (sizeof(CMD_RDNVM)-1)
  if (strncmp(p, CMD_RDNVM, LEN_RDNVM) == 0) {
    sscanf(&p[LEN_RDNVM], "%d", &nV1); //nV1 PWM duty 
    readNVM(nV1);
    goto DecipherOK;
  }
  
  DecipherFail:
    //Serial.printf("Invalid CMD\r\n");
    return -1;
  DecipherOK:
    return 0;
}

