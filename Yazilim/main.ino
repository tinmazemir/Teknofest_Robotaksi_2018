#include <Wire.h>
#include <QTRSensors.h>
#include <TinyGPSPlus.h>
#include <Adafruit_TCS34725.h>
#include <math.h>
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2000  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2
#define INPUT_PIN 7
#define INPUT_PIN2 5
#define INPUT_PIN3 9
#define RGB1RED    53
#define RGB1GREEN  41
#define RGB1BLUE   39
#define RGB2RED    42
#define RGB2BLUE   43
#define RGB3RED    52
#define RGB3BLUE   48
#define RGB3GREEN  40
#define IR_SENSOR  10
#define GPS_BUTTON 44

#define TEST_MODE	1     //YARISMADA BUNU 0 YAP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define DEBUG_COLOR 0
#define DEBUG_CALIBRATE 0
#define DEBUG_LINE_SENSOR 0
#define DEBUG_MIDDLE_SENSORS 0
#define DEBUG_GPS	0

/**************** HIZ KONTROLU ******************/
#define APPROACH_HORIZONTAL_LINE_SPEED_RATE 1.0 //Kirmizi, mavi, yesil yatay cizgilere kamera gordukten sonra aracin hizinin dusurulme yuzdesi
#define ON_HORIZONTAL_LINE_SPEED 80	         //Kirmizi, mavi, yesil uzerindeki hiz
#define AUTONOMOUS_SPEED 50 //Otonom surus hizi
#define TURN_SPEED  255                          //Donus hizi
#define MOTOR_BIAS 0.7                          //Dusurdukce saga daha cok donuyor
/************************************************/

/****************** OZELLIKLER ******************/
#define CHECK_OBSTACLE	0		             //Engel sensoru aktif mi
#define GPS_ENABLED	0                            //GPS aktif mi
#define CALIBRATION_ENABLED 1                        //Kalibrasyon aktif mi
#define WHITE_LINE  1			             //Beyaz cizgi icin 1, siyah cizgi icin 0
#define STOP_ON_CAMERA 0
/************************************************/

/****************** SURELER ******************/
#define BRAKE_DURATION  0		             //Ne kadar sure ile sert fren yapacak
#define JUNCTION_INTERVAL 2000	                     //Ne kadar sure ile kavsaklara karsi kor oluyor
#define CAMERA_JUNCTION_INTERVAL 5000
#define OBSTACLE_DELAY 2000		             //Ne kadar sure ile engellere karsi kor oluyor
#define COLOR_COUNT_PERIOD  1	                     //Kac loopta bir renk sensorunden deger okuyoruz
#define DEBOUNCE_DELAY  500		             //GPS koordinatlarini kaydetme butonunun debounce
#define FRONT_LINE_CROSS_DELAY  500                  //Kavsak tanimak icin ondeki sensorun cizgiden gecisinden ne kadar zaman icinde yandaki sensor de cizgiden gecmeli
#define CALIBRATE_DURATION  8000         -             //Kalibrasyon suresi
#define COLORED_LINE_JUNCTION_IGNORE_DELAY  2000
#define OBSTACLE_SKIP_DELAY  100
/************************************************/

/******************** GPS ***********************/
#define GPS_SENSITIVITY  25.0	//GPS hassasligi (dairenin yaricapi)
#define MAX_GPS_LOC_COUNT  1	//Kac tane GPS lokasyonu kaydedecegiz
/************************************************/

/******************** PID ***********************/
#define KP  150.0//0.3
#define KD  300.0//0.6 
/************************************************/

/***************** SENSORLER ********************/
#define WHITE_LINE_THRESHOLD  200  //Cizgi sensorunun beyaz okumasi icin deger kactan kucuk olmali (0 en beyaz 1000 en siyah)
/************************************************/


#if TEST_MODE
#define RED_WAIT_TIME 5000		//Kirmizi isikta ne kadar bekleyecek
#define GREEN_WAIT_TIME 5000	//Durakta ne kadar bekleyecek
#define REMOTE_CONTROL 1
#else
#define REMOTE_CONTROL 0
#define RED_WAIT_TIME 30000		
#define GREEN_WAIT_TIME 10000	
#endif

#define MAXROUTE 64				//Rota uzerindeki maksimum donus sayisi


enum {
   RED = 1,
   GREEN = 2,
   BLUE = 3,
   BLACK = 4,
   UNKNOWN_COLOR = 5 
};
typedef enum{
  TURNLEFT = 0,
  TURNRIGHT = 1
}action;

typedef struct {
   double lat;
   double lng; 
   int captured;
}gpsLocation;

typedef struct{
  int jcount;
  action a;
}route;

int val;
int val2;
int m1pwm;
int m2pwm;
int m1 = 8;
int m2 = 6;
int r = 0;
int in1 = 45;
int in2 = 47;
int in3 = 49;
int in4 = 51;
int finished = 0;
int amode = 0;
int ignoreJunction = 0;
int junctionCount = 0;
int closeToGPSPoint = 0;
int gpsCaptureStart = 0;
int gpsCaptureDone = 0;
int junctionCounted = 0;
int colorCounter = 0;
int redSeenCount = 0;
int greenSeenCount = 0;
int junctionSeen = 0;
int direction = 1;
int startbyte = 0;
int disable = 0;
int brake = 0;
int redStopped = 0;
int greenStopped = 0;
int brakeHard = 0;
int brakeHardComplete = 0;
int camposition;
int sensorOutOfStop = 1;
int lastStopped = 0;
int locCount = 0;
int turnState = 0;
int routeCounter = 0;
int rightSensor;
int leftSensor;
int parkstate = 7;
int laneState = 0;
int onRightLane = 0;
int ptr = 0;
int firstTurn = 1;
int calibrated = 0;
int frontLeftLineCrossed = 0;
int frontRightLineCrossed = 0;
char incomingByte[20];
route path[MAXROUTE] = {{2,TURNLEFT}, {3,TURNRIGHT},{4,TURNLEFT},{5,TURNRIGHT},{6,TURNRIGHT},{8,TURNRIGHT},{9,TURNLEFT}}; //CHANGE HERE
//route path[MAXROUTE] = {{1,TURNRIGHT}};
double lat;
double lng;
float lastError = 0;
float Kp = KP;
float Kd = KD;
unsigned int sensorValues[NUM_SENSORS];
unsigned int sensorValuesPrev[NUM_SENSORS];
unsigned long pwm;
unsigned long start = 0;
unsigned long pwm2;
unsigned long start2 = 0;
unsigned long pwm3;
unsigned long start3 = 0;
unsigned long pwm4;
unsigned long start4 = 0;
unsigned long pwm5;
unsigned long start5 = 0;
unsigned long redStopTime = 0;
unsigned long greenStopTime = 0;
unsigned long brakeHardStartTime = 0;
unsigned long debounceStartTime = 0;
unsigned long junctionCountTime = 0;
unsigned long lastStopTime = 0;
unsigned long ignoreJunctionTimer = 0;
unsigned long lastObstacleTime = 0;
unsigned long lastTurnAroundTime = 0;
unsigned long frontLeftLineCrossedTime = 0;
unsigned long frontRightLineCrossedTime = 0;
unsigned long calibrateStartTime = 0;
unsigned long sensorOutOfStopTime = 0;
unsigned long escapeObstacleStartTime = 0;
unsigned long junctionSeenTime = 0;
int parking = 0;
long oldSignal = 1500;
gpsLocation gpsloc[MAX_GPS_LOC_COUNT];

QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

QTRSensorsRC qtra((unsigned char[]) {A9, A10},
  2, TIMEOUT, QTR_NO_EMITTER_PIN);
unsigned int sensorValuesSide[2];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS,  TCS34725_GAIN_16X);

TinyGPSPlus gps;

void calibrate ()
{
  int i;
#if CALIBRATION_ENABLED 
  qtrrc.calibrate();    
  qtra.calibrate();  
  
#if DEBUG_CALIBRATE
  for(i = 0; i < 8; i++) {
     Serial.print(qtrrc.calibratedMinimumOn[i]);
     Serial.print(",");
  }
  Serial.println("");   
  for(i = 0; i < 8; i++) {
     Serial.print(qtrrc.calibratedMinimumOff[i]);
     Serial.print(",");
  }
  Serial.println("");   
  for(i = 0; i < 8; i++) {
     Serial.print(qtrrc.calibratedMaximumOn[i]);
     Serial.print(",");
  }
  Serial.println("");   
  for(i = 0; i < 8; i++) {
     Serial.print(qtrrc.calibratedMaximumOff[i]);
     Serial.print(",");
  }   
  Serial.println("");
#endif  
#else

  qtrrc.calibratedMinimumOn[0] = 776;
  qtrrc.calibratedMinimumOn[1] = 648;
  qtrrc.calibratedMinimumOn[2] = 592;
  qtrrc.calibratedMinimumOn[3] = 596;
  qtrrc.calibratedMinimumOn[4] = 592;
  qtrrc.calibratedMinimumOn[5] = 584;
 
  qtrrc.calibratedMinimumOn[6] = 768;
  qtrrc.calibratedMinimumOn[7] = 1072;

  //qtrrc.calibratedMinimumOff[0] = 192;

  qtrrc.calibratedMinimumOff[1] = 256;
  qtrrc.calibratedMinimumOff[2] = 1288;
  qtrrc.calibratedMinimumOff[3] = 256;

  qtrrc.calibratedMinimumOff[4] = 30448;
  qtrrc.calibratedMinimumOff[5] = 4;
  qtrrc.calibratedMinimumOff[6] = 1;
  qtrrc.calibratedMinimumOff[7] = 20001;
  
  qtrrc.calibratedMaximumOn[0] = 2500;
  qtrrc.calibratedMaximumOn[1] = 2268;
  qtrrc.calibratedMaximumOn[2] = 2500;
  qtrrc.calibratedMaximumOn[3] = 1952;
  qtrrc.calibratedMaximumOn[4] = 1940;
  qtrrc.calibratedMaximumOn[5] = 1704;
  qtrrc.calibratedMaximumOn[6] = 2152;
  qtrrc.calibratedMaximumOn[7] = 2500;
  qtrrc.calibratedMaximumOff[0] = 192;
  qtrrc.calibratedMaximumOff[1] = 256;
  qtrrc.calibratedMaximumOff[2] = 1288;
  qtrrc.calibratedMaximumOff[3] = 256;
  qtrrc.calibratedMaximumOff[4] = 30448;
  qtrrc.calibratedMaximumOff[5] = 4;
  qtrrc.calibratedMaximumOff[6] = 1;
  qtrrc.calibratedMaximumOff[7] = 20001;
  
#endif
}

void setup() {  
  // put your setup code here, to run once:
  Serial.begin (115200);
  Serial2.begin (9600);
  pinMode(18, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(INPUT_PIN, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(RGB1RED, OUTPUT);
  pinMode(RGB1GREEN, OUTPUT);
  pinMode(RGB1BLUE, OUTPUT);
  pinMode(RGB2RED, OUTPUT);
  pinMode(RGB2BLUE, OUTPUT);
  pinMode(RGB3RED, OUTPUT);
  pinMode(RGB3GREEN, OUTPUT);
  pinMode(RGB3BLUE, OUTPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(GPS_BUTTON, INPUT);
  
  digitalWrite(RGB1RED, HIGH);
  digitalWrite(RGB1GREEN, HIGH);
  digitalWrite(RGB1BLUE, HIGH);
  digitalWrite(RGB2RED, HIGH);
  digitalWrite(RGB2BLUE, HIGH);
  digitalWrite(RGB3RED, HIGH);
  digitalWrite(RGB3GREEN, HIGH);
  digitalWrite(RGB3BLUE, HIGH);

#if REMOTE_CONTROL
  attachInterrupt(5, IRQcounter, CHANGE);
  attachInterrupt(1, IRQcounter2, CHANGE);
  attachInterrupt(4, IRQcounter4, CHANGE);
#endif

  memset(gpsloc, 0, sizeof(gpsloc));
  
  TCCR4B &= ~7;  // this is 111 in binary and is used as an eraser
  TCCR4B |= 1;   // this operation (AND plus NOT),  set the three bits in TCCR4B to 4
  
  tcs.begin();
  delay(2000);
  digitalWrite(RGB2BLUE, LOW);
  calibrateStartTime = millis();
}

int check_obstacle()
{
  if (digitalRead(IR_SENSOR) == HIGH)
    return 0;
  return 1;
}

void slowdown() {
  if (lastStopped == 0 || sensorOutOfStop)
  {
    brake = 1;
    if (brakeHard == 0)
    {
      brakeHard = 1;
      brakeHardComplete = 0;
      drive(-255, -255);
      brakeHardStartTime = millis();
    }
    digitalWrite(RGB1RED, LOW);
  }
}

void stop(int color)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  if (color == RED)
  {
    digitalWrite(RGB3RED, LOW); 
  }
  else if (color == GREEN)
  {
    digitalWrite(RGB3GREEN, LOW);
  }
}

void finish()
{
  stop(RED);
  //Serial.println("FINISHED");
  finished = 1; 
}

void IRQcounter() { //sag joystick
  if (start > 0) {
    pwm = micros() - start;
    if (pwm > 10000)
       start = micros();
    else
       start = 0;
  }
  else
    start = micros();
}

void IRQcounter2() { //sol joystick
  if (start2 > 0) {
    pwm2 = micros() - start2;
    if (pwm2 > 10000)
       start2 = micros();
    else
       start2 = 0;
  }
  else
    start2 = micros();
}

void IRQcounter4() { //otonom modu
  if (start5 > 0) {
    pwm5 = micros() - start5;
    if (pwm5 > 10000)
       start5 = micros();
    else
       start5 = 0;
  }
  else
    start5 = micros();
}

unsigned long getSignal() { //Kp, Kd
  
  float KpTemp, KdTemp;
  pwm3 = pulseIn(INPUT_PIN2, HIGH, 20000); //Kp
  pwm4 = pulseIn(INPUT_PIN, HIGH, 20000);  //Kd
  long ileri = pulseIn(INPUT_PIN3, HIGH, 20000); //vites 
  KpTemp = (8.0 * ((float)pwm3 - 892)) / 908.0;
  if (KpTemp < 0)
    KpTemp = 0;
  KdTemp = (8.0 * ((float)pwm4 - 892)) / 908.0;
  if (KdTemp < 0)
    KdTemp = 0;
  if (KpTemp > Kp + 0.5 || KpTemp < Kp - 0.5)
    Kp = KpTemp;
  if (KdTemp > Kd + 0.5 || KdTemp < Kd - 0.5)
    Kd = KdTemp;
  
  if (ileri > 1500)
    direction = -1;
  else
    direction = 1;
  
}

int getAutoMode()
{
  if (pwm5 < 1200)
  {
    return 1;   //full autonomous
  }
  if (pwm5 < 1700) 
  {
    return 2;  //semi autonomous
  }
  
  return 3; //manual
}

void readGPS() {
  int i;
  double distance;
  while (Serial2.available()) {
    if (gps.encode(Serial2.read())) {
      lat = gps.location.lat() * 100000;
      lng = gps.location.lng() * 100000;
#if DEBUG_GPS	  
      Serial.print(lat, 6);
      Serial.print(",");
      Serial.println(lng, 6);
#endif	  
      for (i = 0; i < MAX_GPS_LOC_COUNT; i++)
      {
         if (gpsloc[i].captured == 0)
           continue;
         distance = (gpsloc[i].lat - lat)*(gpsloc[i].lat - lat) + (gpsloc[i].lng - lng)*(gpsloc[i].lng - lng);
         if (distance <= GPS_SENSITIVITY)
         {
            closeToGPSPoint = 1;
            slowdown();
            digitalWrite(RGB3BLUE, LOW);
            break;
         }
      }
      if (closeToGPSPoint && i == MAX_GPS_LOC_COUNT)
      {
        closeToGPSPoint = 0;
        if (redStopped == 0 && greenStopped == 0)
        {
          brake = 0;
          digitalWrite(RGB1RED, HIGH);
          digitalWrite(RGB3BLUE, HIGH);
        }
      }
    }
  }
}

void captureGPS() {
  if (digitalRead(GPS_BUTTON) == LOW)
  {
    if (gpsCaptureStart == 0)
      debounceStartTime = millis(); 
    
    gpsCaptureStart = 1;
    if (gpsCaptureDone == 0 && millis() - debounceStartTime > DEBOUNCE_DELAY)
    {
      gpsloc[locCount].lat = lat;
      gpsloc[locCount].lng = lng;
      gpsloc[locCount].captured = 1;
      digitalWrite(RGB2BLUE, LOW);
      locCount++;
      if (locCount == MAX_GPS_LOC_COUNT)
        locCount = 0;
      gpsCaptureDone = 1;  
    }
  }
  else
  {
    gpsCaptureDone = 0;
    gpsCaptureStart = 0;
    digitalWrite(RGB2BLUE, HIGH);
  }
}

void readCommand() {
  char c;
  if (junctionSeen == 1)
  {
    if (millis() - junctionSeenTime > CAMERA_JUNCTION_INTERVAL)
    {
      junctionSeen = 0;
    }  
  }
  while (Serial.available()) {
    c = Serial.read();
    if (c == 's') {
      startbyte = 1;
      ptr = 0;
    }
    else if (c == 'b') {
      if (amode == 1 || amode == 2) 
        slowdown(); 
    }
    else if (c == 'l') {
       junctionSeen = 1;
       junctionSeenTime = millis(); 
    }
    else if (startbyte == 1) {
      if (c == '\r') {
        incomingByte[ptr] = 0;
        startbyte = 0;
        ptr = 0;
        camposition = atoi(incomingByte);
        camposition = 3500 + (((camposition - 33) * 3500) / 25);
      }
      else {
        incomingByte[ptr] = c;
        if (ptr < sizeof(incomingByte) - 1)
          ptr++;
      }
    }
  } 
}

void drive(int solpwm, int sagpwm) {
  if (solpwm < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    solpwm = -1 * solpwm;
  }
  else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  if (sagpwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    sagpwm = -1 * sagpwm;
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(m1, solpwm);
  analogWrite(m2, sagpwm);
}

void junctionCounter()
{
  /*
  Serial.print("junctionSeen:");
  Serial.println(junctionSeen);
  */
  if (junctionSeen == 0)
    return;
    
  junctionCount++;
  /*
  Serial.print("junctionCount:");
  Serial.println(junctionCount);
  */
  if (((junctionCount - 1) / 3) % 2 == 0)
  {
    digitalWrite(RGB1RED, HIGH);
    digitalWrite(RGB1BLUE, LOW);
  }
  else
  {
    digitalWrite(RGB1RED, LOW);
    digitalWrite(RGB1BLUE, HIGH);
  }
  if ((junctionCount-1) % 3 == 0)
  {  
    if (((junctionCount - 1) / 3) % 2 == 0) 
      digitalWrite(RGB2RED, HIGH);
    else
      digitalWrite(RGB2BLUE, HIGH);
  }
  else
  {
    if (((junctionCount - 1) / 3) % 2 == 0)
      digitalWrite(RGB2BLUE, LOW);
    else
      digitalWrite(RGB2RED, LOW);
  }
  if ((junctionCount % 3) == 0) 
  {
    if (((junctionCount - 1) / 3) % 2 == 0)    
      digitalWrite(RGB3BLUE, LOW);
    else
      digitalWrite(RGB3RED, LOW);
  }
  else
  {
    if (((junctionCount - 1) / 3) % 2 == 0)
      digitalWrite(RGB3RED, HIGH);
    else
      digitalWrite(RGB3BLUE, HIGH);
  } 
}

void turnRight(){
  switch(turnState){
    case 0:
        //if(sensorValues[0] > 500){
          drive(-TURN_SPEED, -TURN_SPEED);
          delay(100);
          drive(TURN_SPEED,(-2 * TURN_SPEED) / 4);
          turnState = 15;
          //Serial.println("TURN STATE 15");
        //}
        break;
    case 15:
      if (sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 &&
          sensorValues[4] > 500 && sensorValues[5] > 500 && sensorValues[6] > 500 && sensorValues[7] > 500) {
        turnState = 1;
        //Serial.println("TURN STATE 1");  
      }     
      break;   
    case 1:
      drive(TURN_SPEED,(-2 * TURN_SPEED)/4);
      if(sensorValues[0] < WHITE_LINE_THRESHOLD){
        turnState = 2;
        //Serial.print("TURN STATE 2:");
        //Serial.println(sensorValues[0]);
      }
      break;
  }  
} 

void turnLeft(){
  switch(turnState){
    case 0:
        //if(sensorValues[7] > 500){
          drive(-TURN_SPEED, -TURN_SPEED);
          delay(100);
          drive((-3 * TURN_SPEED)/4,TURN_SPEED);
          turnState = 15;
          //Serial.println("TURN STATE 15");
        //}
        break;
    case 15:
      if (sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 &&
          sensorValues[4] > 500 && sensorValues[5] > 500 && sensorValues[6] > 500 && sensorValues[7] > 500) {
        turnState = 1;
        //Serial.println("TURN STATE 1");
      } 
      break;      
    case 1:
      drive((-3 * TURN_SPEED) / 4, TURN_SPEED);
      if(sensorValues[7] < WHITE_LINE_THRESHOLD){
        turnState = 2;
        //Serial.print("TURN_STATE 2:");
        //Serial.println(sensorValues[7]);
      }
      break;
  }  
}

void turnAroundLeft() {
  switch(turnState){
    case 0:
      drive(-TURN_SPEED,TURN_SPEED);
      if(sensorValues[7] < WHITE_LINE_THRESHOLD){
        //Serial.print("sensorValue:");
        //Serial.println(sensorValues[7]);
        turnState = 2;
      }
      break;
  }   
}

void turnAroundRight() {
  switch(turnState){
    case 0:
      drive(TURN_SPEED,-TURN_SPEED);
      if(sensorValues[0] < WHITE_LINE_THRESHOLD){
        //Serial.print("sensorValue:");
        //Serial.println(sensorValues[0]);
        turnState = 2;
      }
      break;
  }    
}

int turningLeft = 0;
int turningRight = 0;

/*
void followRoute(){
  int j = path[routeCounter].jcount;
  action a = path[routeCounter].a;
  if(junctionCount == j){
    if(a == TURNRIGHT){
      if (rightSensor == 1 || turningRight == 1) {
        turnRight();
        turningRight = 1;
        if(turnState == 2){
          turnState = 0;
          turningRight = 0;
          if(routeCounter < MAXROUTE){
            routeCounter += 1;
            junctionSeen = 0;
            Serial.print("routeCounter:");
            Serial.println(routeCounter);
          }
        }
      }
      else {
        turnState = 0;
        turningRight = 0;   
        if(routeCounter < MAXROUTE){
          routeCounter += 1;
          junctionSeen = 0;
          Serial.print("routeCounter:");
          Serial.println(routeCounter);
        }
      }
    }
    else if (a == TURNLEFT) {
      if (leftSensor == 1 || turningLeft) {
        turnLeft();
        turningLeft = 1;
        if(turnState == 2){
          turnState = 0;
          turningLeft = 0;
          if(routeCounter < MAXROUTE){
            routeCounter += 1;
            junctionSeen = 0;
            Serial.print("routeCounter:");
            Serial.println(routeCounter);
          }
        }
      }
      else {
        turnState = 0;
        turningLeft = 0;    
        if(routeCounter < MAXROUTE){
          routeCounter += 1;
          junctionSeen = 0;
          Serial.print("routeCounter:");
          Serial.println(routeCounter);
        }
      }        
    }  
  }
}
*/

void followRoute(){
  int j = path[routeCounter].jcount;
  action a = path[routeCounter].a;
  if(junctionCount == j)
  {
    if(a == TURNRIGHT){
        turnRight();
        if(turnState == 2){
          turnState = 0;
          if(routeCounter < MAXROUTE){
            routeCounter += 1;
            junctionSeen = 0;
            //Serial.print("routeCounter:");
            //Serial.println(routeCounter);
          }
        }
    }
    else if (a == TURNLEFT) {
      turnLeft();
      if(turnState == 2){
        turnState = 0;
        if(routeCounter < MAXROUTE){
          routeCounter += 1;
          junctionSeen = 0;
          //Serial.print("routeCounter:");
          //Serial.println(routeCounter);
        }
      }
    }
  }  
}


int leftsensorPrinted = 0;
int rightsensorPrinted = 0;

void middleSensors()
{
  qtra.readCalibrated(sensorValuesSide);
#if WHITE_LINE
  unsigned int leftSensorVal = sensorValuesSide[0];
  unsigned int rightSensorVal = sensorValuesSide[1];
#else
  unsigned int leftSensorVal = 1000 - sensorValuesSide[0];
  unsigned int rightSensorVal = 1000 - sensorValuesSide[1];
#endif

#if DEBUG_MIDDLE_SENSORS
  Serial.print("middleSensors:");
  Serial.print(leftSensorVal);
  Serial.print(",");
  Serial.println(rightSensorVal);
#endif
  if(rightSensorVal < WHITE_LINE_THRESHOLD)
  {
#if DEBUG_MIDDLE_SENSORS    
    if (rightsensorPrinted == 0) {
      Serial.print("rightSensor:");
      Serial.println(rightSensorVal);
      rightsensorPrinted = 1;
    }
#endif
    rightSensor = 1;
  }
  else
  {   
    rightsensorPrinted = 0;
    rightSensor = 0;
  }
  
  if(leftSensorVal < WHITE_LINE_THRESHOLD)
  {
#if DEBUG_MIDDLE_SENSORS     
    if (leftsensorPrinted == 0) {
      Serial.print("leftSensor:");
      Serial.println(leftSensorVal);
      leftsensorPrinted = 1; 
    }    
#endif
    leftSensor = 1;
  }
  else
  {
    leftSensor = 0;
    leftsensorPrinted = 0;
  }
}

void changeLane()
{ 
  switch(laneState)
  {
    case 0:
      if (onRightLane)
        drive(-255, 255);
      else
        drive(255, -255);
      laneState = 1;
      break;
    case 1:
      if (check_obstacle() == 0 && sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 &&
          sensorValues[4] > 500 && sensorValues[5] > 500 && sensorValues[6] > 500 && sensorValues[7] > 500)
      {
        laneState = 25;
        escapeObstacleStartTime = millis();
      }
      break;
    case 25:
      if (millis() - escapeObstacleStartTime > OBSTACLE_SKIP_DELAY)
      {
        laneState = 2;
        drive(80, 80);
      }
      break;
    case 2:
      if (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD)
        laneState = 3;
      break;
    case 3:
      if (sensorValues[0] > 500 && sensorValues[1] > 500 && sensorValues[2] > 500 && sensorValues[3] > 500 &&
          sensorValues[4] > 500 && sensorValues[5] > 500 && sensorValues[6] > 500 && sensorValues[7] > 500)
        laneState = 4;
      break;
    case 4:
      if (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD)
        laneState = 0;
      lastObstacleTime = millis();
      onRightLane = !onRightLane;
      break;
  } 
}

static int oldparkstate = -1;

void park()
{
  int color;
  if (parkstate != oldparkstate)
  {
    oldparkstate = parkstate;
    Serial.println(parkstate);
  }
  switch (parkstate)
  {  
    case 0:
      turnRight();
      if (turnState == 2)
      {
        turnState = 0;
        parkstate = 1; 
      }
      break;
    case 1:
      color = getColor();
      if (color == RED)
      {
        parkstate = 0;
        finish();  
      }
      else if (color == BLUE || check_obstacle() == 1)
      {
        parkstate = 22;  
      }
      break;
    case 22:
      drive(-50,-50);
      if (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[1] < WHITE_LINE_THRESHOLD || sensorValues[2] < WHITE_LINE_THRESHOLD ||
          sensorValues[3] < WHITE_LINE_THRESHOLD || sensorValues[4] < WHITE_LINE_THRESHOLD || sensorValues[5] < WHITE_LINE_THRESHOLD ||
          sensorValues[6] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD) {
        parkstate = 2;      
      }
      break;
    case 2:
      turnAroundLeft();
      if (turnState == 2)
      {
        turnState = 0;
        parkstate = 3;
        lastTurnAroundTime = millis();
      }
      break;
    case 3:
      color = getColor();
      if (color == RED)
      {
        parkstate = 0;
        finish();  
      }
      else if (color == BLUE || check_obstacle() == 1)
      {
        parkstate = 35;   
      }    
      break;
    case 35:
      drive(-50,-50);
      if (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[1] < WHITE_LINE_THRESHOLD || sensorValues[2] < WHITE_LINE_THRESHOLD ||
          sensorValues[3] < WHITE_LINE_THRESHOLD || sensorValues[4] < WHITE_LINE_THRESHOLD || sensorValues[5] < WHITE_LINE_THRESHOLD ||
          sensorValues[6] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD) {
        parkstate = 4;      
      }    
      break;
    case 4:
      turnAroundRight();
      if (turnState == 2)
      {
        turnState = 0;
        parkstate = 5;
        lastTurnAroundTime = millis();  
      }
      break;
    case 5:
      if (leftSensor == 1 && 
         (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[1] < WHITE_LINE_THRESHOLD || sensorValues[2] < WHITE_LINE_THRESHOLD ||
          sensorValues[3] < WHITE_LINE_THRESHOLD || sensorValues[4] < WHITE_LINE_THRESHOLD || sensorValues[5] < WHITE_LINE_THRESHOLD ||
          sensorValues[6] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD)) {
        turnLeft();
        parkstate = 6;
      } 
      break;
    case 6:
      turnLeft();
      if (turnState == 2)
      {
        turnState = 0;
        parkstate = 7;  
      }
      break;
    case 7:
      if (leftSensor == 1 &&
          (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[1] < WHITE_LINE_THRESHOLD || sensorValues[2] < WHITE_LINE_THRESHOLD ||
          sensorValues[3] < WHITE_LINE_THRESHOLD || sensorValues[4] < WHITE_LINE_THRESHOLD || sensorValues[5] < WHITE_LINE_THRESHOLD ||
          sensorValues[6] < WHITE_LINE_THRESHOLD || sensorValues[7] < WHITE_LINE_THRESHOLD)) {        
        parkstate = 0;
      }     
      break; 
  } 
}

int positionPrev = 0;

void pid(long mspeed, long maxSpeed)
{
  unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, WHITE_LINE);
  int positiondiff;
  long totaldiff = 0;
  float error;
  int i;
  long motorSpeed;
  long rightMotorSpeed;
  long leftMotorSpeed;
  int obstacle = 0;
#if CHECK_OBSTACLE 
  obstacle = check_obstacle();
#endif
  
#if !WHITE_LINE
  for (i = 0; i < NUM_SENSORS; i++)
    sensorValues[i] = 1000 - sensorValues[i];
#endif

  if (finished)
    return;
  
  middleSensors();
  
  if (routeCounter >= 19) 
  {
      park();
      parking = 1;
  }
    
  if (parkstate != 1 && parkstate != 3 && parkstate != 5 && parkstate !=7)
  {
      lastError = 0;
      return;
  }

  //if (laneState != 0 || 
  //    (obstacle && 
  //    (sensorValues[3] < WHITE_LINE_THRESHOLD || sensorValues[4] < WHITE_LINE_THRESHOLD) && 
  //     sensorValues[0] > 500 && sensorValues[7] > 500)) {
    
  if (parking == 0 && (laneState != 0 || (obstacle && (millis() - lastObstacleTime) > OBSTACLE_DELAY))) {
    changeLane();
  }
  
  if (laneState != 0)
  { 
    lastError = 0;
    return;
  }
  
#if 0

/*
  totaldiff = (int)sensorValues[0] - (int)sensorValuesPrev[0] + (int)sensorValues[1] - (int)sensorValuesPrev[1] + (int)sensorValues[2] - (int)sensorValuesPrev[2] +
              (int)sensorValues[3] - (int)sensorValuesPrev[3] + (int)sensorValues[4] - (int)sensorValuesPrev[4] + (int)sensorValues[5] - (int)sensorValuesPrev[5] +
              (int)sensorValues[6] - (int)sensorValuesPrev[6] + (int)sensorValues[7] - (int)sensorValuesPrev[7];
  
  memcpy(sensorValuesPrev, sensorValues, sizeof(sensorValues));
  if (totaldiff < -900 && junctionCounted == 0) {
    Serial.println(totaldiff);
    junctionCounted = 1;
    junctionCounter();
    junctionCountTime = millis();   
  }
*/
  
#else

  if (sensorValues[0] < WHITE_LINE_THRESHOLD || sensorValues[1] < WHITE_LINE_THRESHOLD)
  {
    //if (frontRightLineCrossed == 0)
    //  Serial.println("Right front crossed");
    frontRightLineCrossed = 1;
    frontRightLineCrossedTime = millis();
  }
  if (sensorValues[7] < WHITE_LINE_THRESHOLD || sensorValues[6] < WHITE_LINE_THRESHOLD)
  {
    //if (frontLeftLineCrossed == 0)
    //  Serial.println("Left front crossed");
    frontLeftLineCrossed = 1;
    frontLeftLineCrossedTime = millis();
  }

  if (frontRightLineCrossed == 1 && millis() - frontRightLineCrossedTime > FRONT_LINE_CROSS_DELAY)
  {
    //Serial.println("Right cross timer expired");
    frontRightLineCrossed = 0;
  }
  if (frontLeftLineCrossed == 1 && millis() - frontLeftLineCrossedTime > FRONT_LINE_CROSS_DELAY)
  {
    //Serial.println("Left cross timer expired");
    frontLeftLineCrossed = 0;
  }


  if (sensorOutOfStop == 1 && millis() - sensorOutOfStopTime > COLORED_LINE_JUNCTION_IGNORE_DELAY &&
      junctionCounted == 0 && rightSensor == 1 && frontRightLineCrossed == 1)
  {
    //Serial.println("Right middle sensor crossed");
    junctionCounted = 1;
    if (greenSeenCount >= 0 && redSeenCount >= 1) { //CHANGE HERE
      junctionCounter();
    }
    junctionCountTime = millis();
  }
  else if (sensorOutOfStop == 1 && millis() - sensorOutOfStopTime > COLORED_LINE_JUNCTION_IGNORE_DELAY &&
           junctionCounted == 0 && leftSensor == 1 && frontLeftLineCrossed == 1)
  {
    //Serial.println("Left middle sensor crossed");
    junctionCounted = 1;
    if (greenSeenCount >= 0 && redSeenCount >= 1) { //CHANGE HERE
      junctionCounter();
    } 
    junctionCountTime = millis();
  }
#endif

  if (junctionCounted == 1 && millis() - junctionCountTime > JUNCTION_INTERVAL) {
    junctionCounted = 0;
    //Serial.println("Junction timer expired");
    //Serial.println("");
    //Serial.println("");
  }

  if (parking == 0 && greenSeenCount >= 0 && redSeenCount >= 1) { //CHANGE HERE
    followRoute();
    if(turnState != 2 && turnState != 0){
      lastError = 0;
      return;
    }
  }
  
  
  error = atan2((((int)position - 3500) * 8) / 3500, 14);
  /*
  Serial.print(position);
  Serial.print(",");
  Serial.println(error);
  delay(10);
  */  
  if (sensorOutOfStop)
  {
    if (position == 0) //left of line
    {
      if (firstTurn) {
        leftMotorSpeed = TURN_SPEED;
        rightMotorSpeed = TURN_SPEED/2;
        drive(leftMotorSpeed, rightMotorSpeed);
        delay(10);
        firstTurn = 0;
      }
      
      leftMotorSpeed = TURN_SPEED/2;
      rightMotorSpeed = -TURN_SPEED;
    }
    else if (position == 7000) //right of line
    {
       if (firstTurn) {
         rightMotorSpeed = TURN_SPEED;
         leftMotorSpeed = TURN_SPEED/2;
         drive(leftMotorSpeed, rightMotorSpeed);
         delay(10);
         firstTurn = 0;
       }
       rightMotorSpeed = TURN_SPEED/2;
       leftMotorSpeed = -TURN_SPEED; 
    }
    else {
      firstTurn = 1;
      motorSpeed = (long)(Kp * error + Kd * (error - lastError));
      lastError = error;
      if (motorSpeed > 0) {
        rightMotorSpeed = mspeed + motorSpeed;
        leftMotorSpeed = mspeed - motorSpeed;
      }
      else {
        rightMotorSpeed = mspeed + motorSpeed;
        leftMotorSpeed = mspeed - motorSpeed;
      }
      
      if (leftMotorSpeed > 0 && rightMotorSpeed > 0)
        rightMotorSpeed = (int)((float)rightMotorSpeed * MOTOR_BIAS);
    }
  }
  else {
    leftMotorSpeed = ON_HORIZONTAL_LINE_SPEED;
    rightMotorSpeed = ON_HORIZONTAL_LINE_SPEED * MOTOR_BIAS;
  }
  
  if (rightMotorSpeed > maxSpeed )
    rightMotorSpeed = maxSpeed;
  if (leftMotorSpeed > maxSpeed )
    leftMotorSpeed = maxSpeed;
  if (rightMotorSpeed < -255)
    rightMotorSpeed = -255;
  if (leftMotorSpeed < -255)
    leftMotorSpeed = -255;

#if DEBUG_LINE_SENSOR
  Serial.print(sensorValues[0]);
  Serial.print(",");  
  Serial.print(sensorValues[1]);
  Serial.print(",");  
  Serial.print(sensorValues[2]);
  Serial.print(",");  
  Serial.print(sensorValues[3]);
  Serial.print(",");  
  Serial.print(sensorValues[4]);
  Serial.print(",");  
  Serial.print(sensorValues[5]);
  Serial.print(",");  
  Serial.print(sensorValues[6]);
  Serial.print(",");  
  Serial.print(sensorValues[7]);
  Serial.print(",");    
  Serial.println(position);
#endif

  if (brake != 1 && redStopped == 0 && greenStopped == 0) {
    drive(leftMotorSpeed, rightMotorSpeed);
  }
  else if ((brakeHard == 0 || brakeHardComplete == 1) && redStopped == 0 && greenStopped == 0) {
    drive((long)((float)leftMotorSpeed * APPROACH_HORIZONTAL_LINE_SPEED_RATE), 
	      (long)((float)rightMotorSpeed * APPROACH_HORIZONTAL_LINE_SPEED_RATE));
  }
}

void manual ()
{
  val2 = map(pwm2, 1000, 2000, 0, 255);
  val = map(pwm, 1000, 2000, -255, 255);

  if (val2 < 30) {
    val2 = 0;
  }
  if (val < 30 && val > -30) {
    val = 0;
  }
  m1pwm = val2;
  m2pwm = val2;

  m2pwm = m2pwm - val;
  m1pwm = m1pwm + val;
  if (val2 > 30) {
    if (m2pwm < 0) {
      m2pwm = 0;
    }
    if (m1pwm < 0) {
      m1pwm = 0;
    }
  }
  
  if (m1pwm > 0 && m2pwm > 0)
    m2pwm = (int)((float)m2pwm * MOTOR_BIAS);

  if (m1pwm > 255) {
    m1pwm = 255;
  }
  if (m2pwm > 255) {
    m2pwm = 255;
  }
  if (m1pwm < -255) {
    m1pwm = -255;
  }
  if (m2pwm < -255) {
    m2pwm = -255;
  }
  m1pwm = direction * m1pwm;
  m2pwm = direction * m2pwm;

  /*
  if (brake != 1 && redStopped == 0 && greenStopped == 0) {
    drive(m1pwm, m2pwm);
  }
  else if ((brakeHard == 0 || brakeHardComplete == 1) && redStopped == 0 && greenStopped == 0) {
    drive((long)((float)m1pwm * 0.55), (long)((float)m2pwm * 0.55));
  }
  */
  drive(m1pwm, m2pwm); 
}

void semiautonomous()
{
  val2 = map(pwm2, 1000, 2000, 0, 255);
  pid(val2, 255);
}

void autonomous()
{
  long maxSpeed = min(AUTONOMOUS_SPEED * 2, 255);
  pid(AUTONOMOUS_SPEED, maxSpeed);
}

int getColor()
{
  return UNKNOWN_COLOR;
  uint16_t clear, red, green, blue;
  colorCounter++;
  if (colorCounter % COLOR_COUNT_PERIOD == 0)
  {
    colorCounter = 0;
    tcs.getRawData(&red, &green, &blue, &clear);
  }
  else
  {
    return UNKNOWN_COLOR;
  }
  
  uint32_t sum = clear;
  float Totalr, Totalg, Totalb;
  Totalr = red; Totalr /= sum;
  Totalg = green; Totalg /= sum;
  Totalb = blue; Totalb /= sum;
  Totalr *= 256; Totalg *= 256; Totalb *= 256;
  
#if DEBUG_COLOR 
  Serial.print(Totalr ); Serial.print(" "); Serial.print(Totalg); Serial.print(" "); Serial.print(Totalb); Serial.print(" :");  
#endif  

  if(Totalr > 128 && Totalr < 255 && Totalg > 0 && Totalg < 90 && Totalb > 0 && Totalb < 90 )
  {
#if DEBUG_COLOR    
    Serial.println("RED");
#endif  
    return RED;
  }
  /*
  else if (Totalr > 80 && Totalr < 105 && Totalg > 75 && Totalg < 95 && Totalb > 60 && Totalb < 95){
#if DEBUG_COLOR      
    Serial.println("BLUE");
#endif
    return BLUE;
  }
  */
  else if (Totalr > 90 && Totalr < 115 && Totalg > 75 && Totalg < 90 && Totalb > 50 && Totalb < 60){
#if DEBUG_COLOR      
    Serial.println("BLACK");
#endif
    return BLACK;
  }                  
  else if(Totalr > 60 && Totalr < 100 && Totalg > 100 && Totalg < 255 && Totalb > 40 && Totalb < 80 ){
#if DEBUG_COLOR      
    Serial.println("GREEN.....................................................");
#endif 
    return GREEN;
  }
  else{
#if DEBUG_COLOR      
    Serial.println("COLOR NOT DETECTED");
#endif
    return UNKNOWN_COLOR;
  }  
}

void loop() {
  int color;
#if GPS_ENABLED
  captureGPS();
#endif

#if !STOP_ON_CAMERA
  color = getColor();
#endif

  if (calibrated == 0)
  {
    calibrate(); 
    if (millis() - calibrateStartTime > CALIBRATE_DURATION)
    {
      calibrated = 1;
      digitalWrite(RGB2BLUE, HIGH);
    }   
  }
  
  readCommand();
  
  if (finished == 0)
  {
#if GPS_ENABLED	  
    readGPS();
#endif
    if (brake == 1 || lastStopped == 1)
    {
#if STOP_ON_CAMERA      
      color = getColor();
#endif      
      if (lastStopped == 1 && color != RED && color != GREEN)
      {
        sensorOutOfStop = 1;
        sensorOutOfStopTime = millis();
        //Serial.println("SENSOR OUT OF STOP!!!");
        lastStopped = 0;
      }
    }
      
    if (brake)
    {
      if (brakeHard == 1 && (millis() - brakeHardStartTime) > BRAKE_DURATION)
      {
        brakeHardComplete = 1;
      }
#if STOP_ON_CAMERA        
      if (color == RED && redStopped == 0)
      {
        redStopTime = millis();  
        stop(color);
        redStopped = 1;
      }
      else if (color == GREEN && greenStopped == 0)
      {
        greenStopTime = millis();  
        stop(color);
        greenStopped = 1;
      }
#endif
    }

#if !STOP_ON_CAMERA    
    if (color == RED && redStopped == 0 && lastStopped == 0 && millis() - lastStopTime > 3000)
    {
      redStopTime = millis();  
      stop(color);
      redStopped = 1;
      brake = 1;
      brakeHardComplete = 1;
    }
    else if (color == GREEN && greenStopped == 0 && lastStopped == 0 && millis() - lastStopTime > 3000)
    {
      greenStopTime = millis();  
      stop(color);
      greenStopped = 1;
      brake = 1;
      brakeHardComplete = 1;
    }
#endif    
    
    if (redStopped)
    {
       if (millis() - redStopTime > RED_WAIT_TIME)
       {
         digitalWrite(RGB1RED, HIGH);
         digitalWrite(RGB3RED, HIGH);
         lastStopTime = millis();
         lastStopped = 1;
         sensorOutOfStop = 0;
         //Serial.println("SENSOR ON STOP!!!");
         brake = 0;
         brakeHard = 0;
         redStopped = 0;
         redSeenCount++;
         //Serial.print("RED SEEN:");
         //Serial.println(redSeenCount);
       }
    }
    if (greenStopped)
    {
       if (millis() - greenStopTime > GREEN_WAIT_TIME)
       { 
         digitalWrite(RGB1RED, HIGH);
         digitalWrite(RGB3GREEN, HIGH);
         lastStopTime = millis();
         lastStopped = 1;
         sensorOutOfStop = 0;
         brake = 0;
         brakeHard = 0;
         greenStopped = 0;
         greenSeenCount++;
         //Serial.print("GREEN SEEN:");
         //Serial.println(greenSeenCount);
       }
    }
  }

#if REMOTE_CONTROL == 0
  amode = 1;
  disable = 0;
#else
  
  amode = getAutoMode();
  
  if (pwm2 < 900) //remote control switched off
  {
      drive(0,0);
      disable = 1;
  }
  else 
  {
      disable = 0;
  }  
#endif

  if (disable == 0)
  {
    if (amode == 3)
    {
      manual();
    }
    else if (amode == 1)
    {
      autonomous();
    }
    else if (amode == 2)
    {
      semiautonomous();
    }
  }
#if DEBUG_COLOR || DEBUG_LINE_SENSOR || DEBUG_MIDDLE_SENSORS || DEBUG_GPS
  delay(20);
#endif
}

