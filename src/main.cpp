#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> 

//Debug and Safe Mode
const bool SAFE_MODE = true; 
const bool DEBUG = true; 

//Declare Adafruit PWM Driver with Adress 0 
Adafruit_PWMServoDriver lightDriver = Adafruit_PWMServoDriver(0x40);  

//Default Values
#define DEFAULT_ANALOG_VALUE 512 
#define DEFAULT_DIGITAL_VALUE 0
#define STANDARD_STICK_TOLERANCE 50 

//Pin Declarations
#define PIN_CE 7
#define PIN_CSN 8

#define PIN_FORWARD 2
#define PIN_BACKWARD 4
#define PIN_PWM 3 

#define PIN_HORN 1000

#define PIN_LIGHT_FL_W 0
#define PIN_LIGHT_FL_R 1
#define PIN_LIGHT_FR_W 2
#define PIN_LIGHT_FR_R 3
#define PIN_LIGHT_FT 4

#define PIN_LIGHT_RL_W 5
#define PIN_LIGHT_RL_R 6
#define PIN_LIGHT_RR_W 7
#define PIN_LIGHT_RR_R 8
#define PIN_LIGHT_RT 9

#define PIN_LIGHT_INTERIOR 10

//Global functions 
void serialPrint(int number, int places);
void writeMotor(uint8_t ly, bool direction); 
void writeHorn(bool active); 
void writeExteriorLights(bool direction, bool lightsActive, bool rearLightsActive, bool lz1Active);
void writeInteriorLights(bool active);   

//Global Variables 
bool goingForward = true;
bool lxInputIsIgnored = false; 
bool isStopped = true; 

bool rxlIsLocked = false; 

//Radio setup 
RF24 radio(PIN_CE, PIN_CSN); // CE, CSN
const byte address[6] = "00100";

//Data Package 
struct Data_Package {
  int lx = DEFAULT_ANALOG_VALUE;
  int ly = DEFAULT_ANALOG_VALUE; 
  bool lz = DEFAULT_DIGITAL_VALUE; 

  int rx = DEFAULT_ANALOG_VALUE; 
  int ry = DEFAULT_ANALOG_VALUE; 
  bool rz = DEFAULT_DIGITAL_VALUE; 
};
Data_Package data; 



void setup() {
  Serial.begin(9600);
  Serial.println("Program: RC Receiver");
  Serial.println("Setup - Start");  

  //Initialize Radio Communication
  bool radioInitStatus = radio.begin();
  Serial.print("Radio Initialization: ");
  Serial.println(radioInitStatus ? "Success" : "Failed");

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  //Set Pin Modes 
  /*pinMode(PIN_PWM, OUTPUT); 
  pinMode(PIN_FORWARD, OUTPUT); 
  pinMode(PIN_BACKWARD, OUTPUT); 
  pinMode(PIN_HORN, OUTPUT); 
*/

  pinMode(SDA, OUTPUT); 
  pinMode(SCL, OUTPUT); 

  //Initialize Adafruit PWM Driver 
  bool adafruitPwmDriverInitStatus = lightDriver.begin();
  Serial.print("Adafruit PWM Driver Initilitation: "); 
  Serial.println(adafruitPwmDriverInitStatus ? "Success" : "Failed");
  lightDriver.setPWMFreq(60); 
  
  Serial.println("Setup - End");
}

void loop() {
   
   
  //LX: Richtung V> R< 
  //LY: Fahrregler 
  //LZ: Horn 

  //RX: Kabinenbeleuchtung  < / FZ1 (1 weiße Leuchte auf Pufferhöhe) > 
  //RY: Umschalten Hecklichter / Licht an/aus 
  //RZ: Hauptschalter?  / Rangierschalter (langsamere V-max)? 

  //Check radio availability, read if available, else stop
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
  } else {
    data.lx = DEFAULT_ANALOG_VALUE; 
    data.ly = DEFAULT_ANALOG_VALUE; 
    data.lz = DEFAULT_DIGITAL_VALUE; 
    data.rx = DEFAULT_ANALOG_VALUE; 
    data.ry = DEFAULT_ANALOG_VALUE; 
    data.rz = DEFAULT_DIGITAL_VALUE; 
  }

  //Stick and Switch Handling 
  //LY
  uint8_t pwm = map(data.ly, 0, 1024, 0, 255);
    

  //LX
  isStopped = pwm == 0;
  if (isStopped) {
    if (data.lx < 250) {
      goingForward = true; 
      lxInputIsIgnored = true; 
    } else if (data.lx > 1024 - 250) {
      goingForward = false; 
      lxInputIsIgnored = true;
    } else {
      lxInputIsIgnored = false; 
    } 
  }


  //LZ
  bool hornActive = data.lz;  

  //RX 

  //RY

  //RZ


  //Pin Writing - only if Safe Mode is disabled  
  if (!SAFE_MODE) {
    writeMotor(pwm, goingForward); 
    writeHorn(hornActive); 
  }
  
  //DEBUG Prints: 
  if (DEBUG) {
    //DEBUG LX: 
    Serial.print("lx: "); 
    serialPrint(data.lx, 4); 
    Serial.print("   |   isStopped: "); 
    Serial.print(isStopped);
    Serial.print("   |   goingForward: "); 
    Serial.print(goingForward);
    Serial.print("   |   lxInputIgnored: "); 
    Serial.print(lxInputIsIgnored);
    Serial.println();
    
    //DEBU LY: 
    Serial.print("ly: "); 
    serialPrint(data.ly, 4); 
    Serial.print("   |   pwm: "); 
    Serial.print(pwm);
    Serial.print("   |   isStopped: "); 
    Serial.print(isStopped);
    Serial.println();
  }
}

void writeMotor(uint8_t pwm, bool direction) {
  //Safe to never write both pins high at the same time
  if (direction) {
    digitalWrite(PIN_BACKWARD, LOW); 
    digitalWrite(PIN_FORWARD, HIGH);
  } else {
    digitalWrite(PIN_FORWARD, LOW); 
    digitalWrite(PIN_BACKWARD, HIGH);
  }
  analogWrite(PIN_PWM, pwm);
} 

void writeHorn(bool active) {
  digitalWrite(PIN_HORN, active); 
}

void writeExteriorLights(bool direction, bool lightsActive, bool rearLightsActive, bool lz1Active) {
  if (lightsActive) {
    if (lz1Active) {
    } else {
      if (direction) {
        if (rearLightsActive) {
        } else {
        }
      } else {
        if (rearLightsActive) {
        } else {
        }
      }
    }
  } else {
  }
}

void writeInteriorLights(bool active) {

} 

//temporary function to print numbers nicely
void serialPrint(int number, int places) {
  char buffer[16];
  // Format-String dynamisch bauen, z.B. "%04d", "%06d", ...
  char format[8];
  snprintf(format, sizeof(format), "%%0%dd", places);
  snprintf(buffer, sizeof(buffer), format, number);
  Serial.print(buffer);
}

