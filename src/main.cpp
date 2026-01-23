#include <nRF24L01.h>
#include <RF24.h>

//Debug and Safe Mode
const bool SAFE_MODE = true; 
const bool DEBUG_MODE = true; 

//Default Values
#define DEFAULT_ANALOG_VALUE 512 
#define DEFAULT_DIGITAL_VALUE 0
#define DEFAULT_STICK_TOLERANCE 256

//Pin Declarations
#define PIN_CE 9
#define PIN_CSN 10

#define PIN_PWM 6
#define PIN_FORWARD 7
#define PIN_BACKWARD 8

#define PIN_HORN 1000

#define PIN_LIGHT_FL_A 5
#define PIN_LIGHT_FL_B 4
#define PIN_LIGHT_FR_A 3
#define PIN_LIGHT_FR_B 2 
#define PIN_LIGHT_FT 1

#define PIN_LIGHT_RL_A  
#define PIN_LIGHT_RL_B  
#define PIN_LIGHT_RR_A 
#define PIN_LIGHT_RR_B  
#define PIN_LIGHT_RT 

#define PIN_LIGHT_INTERIOR 0

//Global functions 
void serialPrint(int number, int places);

//Global Variables 
bool goingForward = true;
bool lxInputIsIgnored = false; 
bool isStopped = true; 

bool hornActive = false; 

bool rxlIsLocked = false; 
bool interiorLightsActive = false;  

bool rxrIsLocked = false; 
bool lz1Active = false; 

bool ryuIsLocked = false; 
bool lightsActive = false; 

bool rylIsLocked = false; 
bool rearLightsActive = false; 

bool lowGearEnabled = false; 

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

/*-----------------------------------------------------------------------------------------------------------
SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP - SETUP
-----------------------------------------------------------------------------------------------------------*/

void setup() {
  //Serial.begin(9600);
  Serial.println("Program: RC Receiver ESP32");
  Serial.println("Setup - Start");  

  //Initialize Radio Communication
  bool radioInitStatus = radio.begin();
  Serial.print("Radio Initialization: ");
  Serial.println(radioInitStatus ? "Success" : "Failed");

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  //Set Pin Modes 

  Serial.print("SAFE MODE ");
  Serial.println(SAFE_MODE ? "enabled, all Pin-Interactions disabled" : "disabled, Pin-Interactions enabled"); 
  
  Serial.println("Setup - End");
}

/*----------------------------------------------------------------------------------------------------
LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP - LOOP 
----------------------------------------------------------------------------------------------------*/

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
  hornActive = data.lz;  

  //RX - Left Side (>512)

  //RX - Right Side (<512) 

  //RY - Lower (<512)
  
  //RY - Upper (>512)

  //RZ
  lowGearEnabled = data.rz; 
  
  //Pin Writing - only if Safe Mode is disabled  
  if (!SAFE_MODE) {
   // writeMotor(pwm, goingForward, lowGearEnabled); 
   // writeHorn(hornActive); 
   // writeInteriorLights(interiorLightsActive);
   // writeExteriorLights(goingForward, lightsActive, rearLightsActive, lz1Active); 
  }
  
  //DEBUG Prints: 
  if (DEBUG_MODE) {
    //DEBUG LX: 
    /*Serial.print("lx: "); 
    serialPrint(data.lx, 4); 
    Serial.print("   |   isStopped: "); 
    Serial.print(isStopped);
    Serial.print("   |   goingForward: "); 
    Serial.print(goingForward);
    Serial.print("   |   lxInputIgnored: "); 
    Serial.print(lxInputIsIgnored);
    Serial.println();
    
    //DEBUG LY: 
    Serial.print("ly: "); 
    serialPrint(data.ly, 4);
    Serial.print("   |   low gear: "); 
    Serial.print(lowGearEnabled); 
    Serial.print("   |   pwm: "); 
    Serial.print(pwm);
    Serial.print("   |   isStopped: "); 
    Serial.print(isStopped);
    Serial.println();

    //DEBUG RX: 
    Serial.print("rx: "); 
    serialPrint(data.rx, 4); 
    Serial.print("   |   rxl is locked: "); 
    Serial.print(rxlIsLocked);
    Serial.print("   |   interior light active: "); 
    Serial.print(interiorLightsActive);
    Serial.println();*/

    //DEBU RY: 
    Serial.print("rx: "); 
    serialPrint(data.rx, 4); 
    Serial.print("   |   ryu is locked: "); 
    Serial.print(ryuIsLocked);
    Serial.print("   |   light active: "); 
    Serial.print(lightsActive);
    Serial.println();
  }
}

/*-----------------------------------------------------------------------------------------------------------
PIN WRITING - PIN WRITING - PIN WRITING - PIN WRITING - PIN WRITING - PIN WRITING - PIN WRITING - PIN WRITING
-----------------------------------------------------------------------------------------------------------*/

void writeLz1(); 
void writeFrontLightsWhiteAndRearRed();
void writeRearLightsRed(); 
void writeRearLightsOff();
void writeRearLightsWhite();
void writeFrontLightsRed(); 
void writeFrontLightsOff(); 


void writeMotor(uint8_t pwm, bool direction, bool lowGearEnabled) {
  //Safe to never write both pins high at the same time
  if (direction) {
    digitalWrite(PIN_BACKWARD, LOW); 
    digitalWrite(PIN_FORWARD, HIGH);
  } else {
    digitalWrite(PIN_FORWARD, LOW); 
    digitalWrite(PIN_BACKWARD, HIGH);
  }
  if (lowGearEnabled) pwm = pwm / 2; 
  analogWrite(PIN_PWM, pwm);
} 

void writeHorn(bool active) {
  digitalWrite(PIN_HORN, active ? HIGH : LOW); 
}

void writeExteriorLights(bool direction, bool lightsActive, bool rearLightsActive, bool lz1Active) {
  if (lightsActive) {
    if (lz1Active) {
      //write LZ1
      writeLz1();
    } else {
      if (direction) {
        //write front lights white
        writeFrontLightsWhiteAndRearRed(); 
        if (rearLightsActive) {
          //write rear lights red
          writeRearLightsRed();
        } else {
          //write rear lights off
          writeRearLightsOff();
        }
      } else {
        //write rear lights white
        writeRearLightsWhite();
        if (rearLightsActive) {
          //Write front lights red
          writeFrontLightsRed();
        } else {
          //write front lights off
          writeFrontLightsOff();
        }
      }
    }
  } else {
    //write all lights off
    writeFrontLightsOff();
    writeRearLightsOff(); 
  }
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


/*-------------------------------------------------------------------------------------------------------------
HELPER FUNCTIONS - HELPER FUNCTIONS - HELPER FUNCTIONS - HELPER FUNCTIONS - HELPER FUNCTIONS - HELPER FUNCTIONS
-------------------------------------------------------------------------------------------------------------*/

void handleLowerStickInput_0_512(int &data, bool &lockVar, bool &outPut) {
  if (data < DEFAULT_STICK_TOLERANCE) {
    if (!lockVar) {
      outPut = !outPut;
      lockVar = true; 
    }
  } else {
    lockVar = false; 
  }
}

void handleUpperStickInput_512_1024(int &data, bool &lockVar, bool &outPut) {
  if (data > 1024 - DEFAULT_STICK_TOLERANCE) {
    if (!lockVar) {
      outPut = !outPut;
      lockVar = true; 
    }
  } else {
    lockVar = false; 
  }
}