#include <ArduinoJson.h>
#include <XBee.h>
#include <DS1307RTC.h>
#include <Time.h>
#include <Wire.h>
#include <TimeAlarms.h>

//Pins
#define SWTCH1 7 //przelacznik 1
#define SWTCH2  8 //przelacznik 2
#define IN1 12 //Manualne włączenie swtch1
#define IN2 13 //Manualne włączenie swtch2

//XBEE related 
#define XBEE_DATA_LENGTH 65
#define XBEE_STATUS_RESPONSE_TIMEOUT 500
#define XBEE_BOUD_RATE 9600

//Type of outgoing commands

#define REPLAY "rep"
#define ERR "err"
#define INFO "info"
#define SUN_SET 3
#define SUN_RISE 4


//Type of incming commands

#define READ_ANALOG "RA"
#define READ_DIGITAL "RD"
#define WRITE_ANALOG "WA"
#define WRITE_DIGITAL "WD"
#define HARDWARE_CLOCK_DISPLAY "HCD"
#define DIGITAL_CLOCK_DISPLAY "DCD"
#define GET_LIGHTS_ON "GLON"
#define GET_LIGHTS_OFF "GLOF"


//Error types
#define UNKNOWN_CMD 1 //Unknown incoming command
#define JSON_DECODING_FAILED 2
#define TIME_NOT_SET 11 //Time not set

//Others
#define SERIAL_BOUD_RATE 9600
#define ANALOG_TIMER 60
#define ANALOG_DOWN 0
#define ANALOG_UP 1023
#define ANALOG_TOLERANCE 10
#define Long 20.7971600
#define Lat 52.2103900
#define Req -0.833
#define dsep "-"
#define tsep ":"

char XBee_Data[XBEE_DATA_LENGTH];
StaticJsonBuffer<200> jsonBuffer;
int dl;
int winter=1;
String loffStr;
String lonStr;

// one byte payload
uint8_t payload[XBEE_DATA_LENGTH];
// TODO replace with address of your coordinator (Connected to the Java app)
uint32_t COORD_MSB_ADDRESS = 0x0013a200;
uint32_t COORD_LSB_ADDRESS = 0x40b189b7;

XBee xbee = XBee();

// Coordinator
XBeeAddress64 addr64 = XBeeAddress64(COORD_MSB_ADDRESS, COORD_LSB_ADDRESS);

ZBTxRequest tx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();

int prevPinA2 = 0;
int prevPinA3 = 0;
int prevPinA4 = 0;
int prevPinA5 = 0;

int prevIn1=LOW;
int prevIn2=LOW;

JsonObject& root = jsonBuffer.createObject();

void setup(){
  pinMode(SWTCH1, OUTPUT);  
  pinMode(SWTCH2, OUTPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  Serial.begin(SERIAL_BOUD_RATE);
  Serial1.begin(XBEE_BOUD_RATE);
  xbee.begin(Serial1);
   if (timeStatus() == timeNotSet){
    setTimeRTC();
  }  
  setAlarm();
}

void loop(){
 xbee.readPacket();
  if (xbee.getResponse().isAvailable()){
    handleXBeeResponse();
  }
  else if (xbee.getResponse().isError()) {
    Serial1.println(xbee.getResponse().getErrorCode(), DEC);
  }
  checkInputPins();
  Alarm.delay(1);
}

void checkInputPins(){
  int state;
  state=digitalRead(IN1);
  if (state!= prevIn1){
    manualSwitch(SWTCH1,digitalRead(IN1));
    prevIn1=state;
  }
  state=digitalRead(IN2);
  if (state != prevIn2){
    manualSwitch(SWTCH2,digitalRead(IN2));
    prevIn2=state;
  }
}

void setTimeRTC(){
  time_t t=0;
   
  while (t<=0){
    //replay (ERR,NULL,TIME_NOT_SET,NULL,"");
    delay(1000);
    t=RTC.get();
  }
  setTime(t);
}

void setAlarm(){
  
  float *SunSR;
  int *sunRise,*sunSet;
  //int HR,HS,MR,MS,SR,SS;
  
  SunSR=calcSunInt(year(),month(),day());
  
  sunRise=sunInt2TimeDigits(*SunSR);

  Alarm.alarmOnce(*sunRise+1,*(sunRise+1),*(sunRise+2), lightsOff);
  Alarm.alarmOnce(*sunSet+1,*(sunSet+1),*(sunSet+2), lightsOn);
  Alarm.alarmRepeat(0,0,10,setAlarm);
  Alarm.timerRepeat(ANALOG_TIMER,checkAnalogPins);
  loffStr=print2digits(*sunRise+1)+":"+print2digits(*(sunRise+1))+":"+print2digits(*(sunRise+2));
  lonStr=print2digits(*sunSet+1)+":"+print2digits(*(sunSet+1))+":"+print2digits(*(sunSet+2));
  //showSun();
}

void showSun(){
  //debug("showSun");
  replay(INFO,NULL,NULL,NULL,"loffStr",getLights(1).c_str());
  replay(INFO,NULL,NULL,NULL,"loffStr",getLights(0).c_str());
}

float * calcSunInt(int Rok, int M, int D){
  //Calculates sun rise ret[0] and sun set ret[1] as float
  static float ret[2];

  float J=367L*Rok-int(7*(Rok+int((M+9)/12))/4)+int(275*M/9)+D-730531.5;
  float Cent=J/36525;
  float L=fmod((4.8949504201433+628.331969753199*Cent),6.28318530718);
  float G = fmod((6.2400408+628.3019501*Cent),6.28318530718);
  float O = 0.409093-0.0002269*Cent;
  float F = 0.033423*sin(G)+0.00034907*sin(2*G);
  float E = 0.0430398*sin(2*(L+F)) - 0.00092502*sin(4*(L+F)) - F;
  float A = asin(sin(O)*sin(L+F));
  float C = (sin(0.017453293*Req)-sin(0.017453293*Lat)*sin(A))/(cos(0.017453293*Lat)*cos(A));
  
  ret[0] = (PI - (E+0.017453293*Long+1*acos(C)))*57.29577951/15; // Sun Rise
  ret[1] = (PI - (E+0.017453293*Long+(-1)*acos(C)))*57.29577951/15; //Sun Set
  
  return ret;
}

int * sunInt2TimeDigits(float sunR){
  //Converts time in int to array of ret[]={hh,mm,ss}
  static int ret[3];
  int Sec;
  
  ret[0]=(int)sunR;
  Sec=(sunR-ret[0])*3600;
  ret[1]=Sec/60;
  ret[2]=(Sec-(ret[1]*60));
  
  return ret;
  
}

void lightsOff(){
  //debug("lightsOff");
  digitalWrite(SWTCH1, LOW);
}

void lightsOn(){
  //debug("lightsOn");
  digitalWrite(SWTCH1, HIGH);
}


String print2digits(int number) {
  String str="";
  if (number >= 0 && number < 10) {
    str="0"+String(number);
  }
  else{
    str=String(number);
  }
  return str;
}

void checkAnalogPins(){
  int state = analogRead(A2);
  if ((state>=ANALOG_DOWN) && (state<=ANALOG_UP) && (abs(state-prevPinA2)>ANALOG_TOLERANCE)){
    onAnalogPinChange(A2);
    prevPinA2=state;
  }
  /*
  state = analogRead(A3);
   if ((state>=ANALOG_DOWN) && (state<=ANALOG_UP) && (abs(state-prevPinA3)>ANALOG_TOLERANCE)){
    onAnalogPinChange(A3);
    prevPinA3=state;
  }
  state = analogRead(A4);
   if ((state>=ANALOG_DOWN) && (state<=ANALOG_UP) && (abs(state-prevPinA4)>ANALOG_TOLERANCE)){
    onAnalogPinChange(A4);
    prevPinA4=state;
  }
  state = analogRead(A5);
   if ((state>=ANALOG_DOWN) && (state<=ANALOG_UP) && (abs(state-prevPinA5)>ANALOG_TOLERANCE)){
    onAnalogPinChange(A5);
    prevPinA3=state;
  }
  */
}

void onAnalogPinChange(int pin){
  //debug ("In onAnalogPinChange);
  replay(INFO,NULL,pin,analogRead(pin),"PSTAT",NULL);
}

void manualSwitch(int pin, int state){
  //debug("In manualSwitch)
  digitalWrite(pin, state);
  replay(INFO,NULL,pin,digitalRead(pin),"PSTAT",NULL);
}


void handleXBeeResponse(){
  //debug("In handleXBeeResponse");
  memset(&XBee_Data[0], 0, sizeof(XBee_Data));
  if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE){  
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx); 
      if (rx.getDataLength()>=XBEE_DATA_LENGTH){
        dl=XBEE_DATA_LENGTH;
      }
      else {
        dl=rx.getDataLength();
      }
      for (int count=0;count<=dl-1;count++){
        XBee_Data[count]=rx.getData(count);
      }
      JsonObject& root = jsonBuffer.parseObject(XBee_Data);
      if (!root.success()){
        replay (ERR,NULL,JSON_DECODING_FAILED,NULL,NULL,NULL);
      }else{
      executeAction(root["cmd"],root["time"],root["p1"],root["p2"]);
      }
  }
}

String digitalClockDisplay(){
  //debug("digitalClockDisplay");
  return String(year())+dsep+print2digits(month())+dsep+print2digits(day())+" "+hour()+tsep+print2digits(minute())+tsep+print2digits(second());
}

String hardwareClockDisplay(){
  //debug("hardwareClockDisplay");
  tmElements_t tm;
  RTC.read(tm);
  return String(tmYearToCalendar(tm.Year))+dsep+print2digits(tm.Month)+dsep+print2digits(tm.Day)+" "+tm.Hour+tsep+print2digits(tm.Minute)+tsep+print2digits(tm.Second);
}

void executeAction(const char* cmd,long time,int p1,int p2){
      int o1,o2;
      String o3,o4;
      String strCmd=cmd;
      const char* rtype=REPLAY;
      if (strCmd==READ_ANALOG){
          o1=p1;
          o2 = analogRead(p1);      
      }else if(strCmd.equals(READ_DIGITAL)){
           o1=p1;
           o2 = digitalRead(p1);      
      }else if (strCmd==WRITE_ANALOG){
          analogWrite(p1,p2);
          o1=p1;
          o2 = analogRead(p1);        
      }else if (strCmd==WRITE_DIGITAL){
           digitalWrite(p1,p2);
           o1=p1;
           o2 = digitalRead(p1);      
      }else if(strCmd==HARDWARE_CLOCK_DISPLAY){
        o3=hardwareClockDisplay();
      }else if (strCmd==DIGITAL_CLOCK_DISPLAY){
        o3=digitalClockDisplay();
      }else if(strCmd==GET_LIGHTS_ON){
        o3="lonStr";
        o4=getLights(1);
      }else if (strCmd==GET_LIGHTS_OFF){
        o3="loffStr";
        o4=getLights(0);
      }else{
           rtype=ERR;
           o1=UNKNOWN_CMD;
           o3="cmd";
           o4=strCmd;
      }

      replay (rtype,time,o1,o2,o3.c_str(),o4.c_str());
}

String getLights(int flg){
  if(flg){
    return lonStr;
  }else{
    return loffStr;
  }
}

void replay(const char* rtype,long time, int p1, int v1, const char* p2, const char* v2){

      //JsonObject& root = jsonBuffer.createObject();
      root["tp"] = rtype;
      root["tm"] = time;
      root["p1"] = p1;
      root["v1"] = v1;
      root["p2"] = p2;
      root["v2"] = v2;
      char buffer[65];
      memset(&buffer[0], 0, sizeof(buffer));
      root.printTo(buffer, sizeof(buffer));
      memcpy(payload,buffer, 65);
      xbee.send(tx);
      memset(&payload[0], 0, sizeof(payload));/*8
      if (xbee.readPacket(XBEE_STATUS_RESPONSE_TIMEOUT)) {
        Serial.println("Timeout");
        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
          xbee.getResponse().getZBTxStatusResponse(txStatus);
          Serial.println("Got response");
          if (txStatus.isSuccess()) {
            Serial.println("OK");
          }else{
            Serial.println("NACK:");  //no ACK
            // TODO resend with same frame id
          }
        }      
      } else if (xbee.getResponse().isError()) {
        Serial.println("TXERR");  //TX error
        Serial.println(xbee.getResponse().getErrorCode());
      } else {
        Serial.println("TXTOUT");  //TX Timeout
    // local XBee did not provide a timely TX Status Response -- should not happen if radio is configured and wired correctly
    // did you switch the TX/RX jumpers back to XBee?
    // is your baud rate correct?
    // in API mode?
  }
  */
}
