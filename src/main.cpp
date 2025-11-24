#include <nRF24L01.h>
#include <RF24.h>

bool debug = false; 

class Stick {
  public: 
    Stick(int lowerLimit, int upperLimit, int assignedPin, int boundryTolerance) {
      this->boundryTolerance = boundryTolerance;  
      this->lowerLimit = lowerLimit; 
      this->upperLimit = upperLimit;
      this->assignedPin = assignedPin;  
      this->currentPosition = (lowerLimit + upperLimit) / 2; 
    }

    void update(int &value) {
      if (value >= lowerLimit && value <= upperLimit) {
        this->currentPosition = value; 
      }
    }

    int assignedPin; 
    int boundryTolerance; 
    int currentPosition;  
    int lowerLimit; 
    int upperLimit; 
};


class PWMStick: public Stick {
  public: 
    PWMStick(int lowerLimit, int upperLimit, int assignedPWMPin, int forwardPin, int backWardPin, int boundryTolerance = 50): Stick(lowerLimit, upperLimit, assignedPWMPin, boundryTolerance) { 
      this->forwardPin = forwardPin; 
      this->backwardPin = backWardPin; 
    }

    void setForward() {
      digitalWrite(backwardPin, LOW); 
      digitalWrite(forwardPin, HIGH); 
    }

    void setBackward() {
      digitalWrite(forwardPin, LOW); 
      digitalWrite(backwardPin, HIGH);
    }

    int forwardPin; 
    int backwardPin; 
};

class BinaryStick: public Stick {
  public: 
    BinaryStick(int lowerLimit, int upperLimit, int assignedPin, int boundryTolerance = 50): Stick(lowerLimit, upperLimit, assignedPin, boundryTolerance) {
      this->isActive = false;  
    }

    void update(int &value) {
      Stick::update(value);
      if (currentPosition < (lowerLimit + boundryTolerance)) {
        isActive = false; 
      } else if (currentPosition > (upperLimit - boundryTolerance)) {
        isActive = true; 
      }
    }

    bool isActive; 
};

class ToggleStick: public BinaryStick {
  public: 
    ToggleStick(int lowerLimit, int upperLimit, int assignedPin, int boundryTolerance = 50): BinaryStick(lowerLimit, upperLimit, assignedPin, boundryTolerance) {
      this->isActive = false;  
      this -> isLocked = false; 
    }

    void update(int &value) {
      Stick::update(value);
      if (currentPosition < (lowerLimit + boundryTolerance)) {
        toggleActive();
        isLocked = true;  
      } else if (isLocked && currentPosition > (upperLimit - boundryTolerance)) {
        toggleActive(); 
        isLocked = false; 
      }
    }

    bool isLocked;
    void writeIfAllowedAndLock() {
      if (!isLocked) {
        digitalWrite(assignedPin, isActive);
        isLocked = true; 
      }
    }

  private: 
    void toggleActive() {
      isActive = !isActive; 
    }
};



//Pin Declarations
#define PIN_CE 7
#define PIN_CSN 8

#define PIN_FORWARD 2
#define PIN_BACKWARD 4
#define PIN_PWM 3 

#define PIN_HORN 1000

#define PIN_LIGHTS 0000
#define PIN_LZ1  0000
#define PIN_PLACEHOLDER1 0000
#define PIN_PLACEHOLDER2 0000




int pwm = 0; 
bool direction = true; //true: forward, false: backward
bool lockDirection = true; 

bool lightsOn = false;
bool fz1Active = false; 
bool rearLightsOn = false;

bool ignoreRXInput = false; 

RF24 radio(PIN_CE, PIN_CSN); // CE, CSN
const byte address[6] = "00100";

//Datenpaket 
struct Data_Package {
  int lx = 0;
  int ly = 0; 
  bool lz = 0; 

  int rx = 0; 
  int ry = 0; 
  bool rz = 0; 
};
Data_Package data;

const int standardStickTolerance = 50; 

BinaryStick lx(0, 1024, 1000, standardStickTolerance); 
PWMStick ly(0, 1024, PIN_PWM, PIN_FORWARD, PIN_BACKWARD, standardStickTolerance); 

ToggleStick ryu(512, 1024, PIN_LIGHTS, standardStickTolerance);
ToggleStick ryl(0, 1024, PIN_LZ1, standardStickTolerance); 
ToggleStick rxl(512, 1024, PIN_PLACEHOLDER1, standardStickTolerance); 
ToggleStick rxr(512, 1024, PIN_PLACEHOLDER2, standardStickTolerance);

void setup() {
  Serial.begin(9600);
  Serial.println("Setup - Start");

  //Initialize Radio Communication
  bool init_status = radio.begin();
  Serial.print("Radio Initialization: ");
  Serial.println(init_status ? "Success" : "Failed");

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  pinMode(PIN_FORWARD, OUTPUT);
  pinMode(PIN_BACKWARD, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_HORN, OUTPUT);
  
  Serial.println("Setup - End");
}

void loop() {
  /* 
  LX: Richtung V> R< 
  LY: Fahrregler 
  LZ: Horn 

  RX: Umschalten Hecklichter < / FZ1 (1 weiße Leuchte auf Pufferhöhe) > 
  RY: Kabinenbeleuchtung / Licht an/aus 
  RZ: Hauptschalter?  / Rangierschalter (langsamere V-max)?
  */

  //Check radio availability
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
  }

  lx.update(data.lx); 
  ly.update(data.ly); 

  rxl.update(data.rx); 
  rxr.update(data.rx); 
  ryl.update(data.ry); 
  ryu.update(data.ry); 

  //LX - Direction 

  if (!lockDirection) {
    if (data.ly <= standardStickTolerance) {
      direction = true; 
      switchOff(PIN_BACKWARD);
      switchOn(PIN_FORWARD);
    } else if (data.ly >= (1024 - standardStickTolerance)) {
      direction = false; 
      switchOff(PIN_FORWARD); 
      switchOn(PIN_BACKWARD); 
    }
  }

  //LY - Throttle
  if (data.ly == 0) {
    lockDirection = false;
  } else if(!lockDirection) {
    lockDirection = true; 
  }
  pwm = map(data.ly, 0, 1024, 0, 255);
  analogWrite(PIN_PWM, pwm);
  
  //LZ - Horn 
  if (data.ly == 1) {
    switchOn(PIN_HORN); 
  } else {
    switchOff(PIN_HORN); 
  }
  
  //RX Left Half - Rear Lights 
  if (data.rx > (1024 - standardStickTolerance)) {
    toggleBool(rearLightsOn); 
    ignoreRXInput = true; 
  } else if (ignoreRXInput && data.rx <= (512 + standardStickTolerance)) {
    ignoreRXInput = false; 
  }

  //RX Right Half - Fz1 
  if (data.rx < standardStickTolerance) {
    toggleBool(rearLightsOn); 
    ignoreRXInput = true; 
  } else if (ignoreRXInput && data.rx <= (512 + standardStickTolerance)) {
    ignoreRXInput = false; 
  }

  //RY
  //RZ



  
/*   //LY
  pwm = map(data.ly, 40, 980, 0, 255); 
  if (pwm == 0) {
    lockDirection = false;
  } else {
    lockDirection = true; 
  }
  analogWrite(PIN_PWM, pwm);

  //RY
  if (!lockDirection) {
    if (data.ry > 700) {
      direction = true; 
      digitalWrite(PIN_BACKWARD, !direction);
      digitalWrite(PIN_FORWARD, direction);
    } else if (data.ry < 100) {
      direction = false; 
      digitalWrite(PIN_FORWARD, direction);
      digitalWrite(PIN_BACKWARD, !direction);
    }
  }

  Serial.print(data.ly);
  Serial.print("    ");
  Serial.print(pwm);
  Serial.print("    ");
  Serial.print(lockDirection);
  Serial.print("  |  ");
  Serial.print(data.ry);
  Serial.print("    ");
  Serial.println(direction);

  if (debug == true) {
    Serial.print(" lx:");
    Serial.print(data.lx);
    Serial.print(" ly:");
    Serial.print(data.ly);
    Serial.print(" lz:");
    Serial.print(data.lz);
    Serial.print(" rx:");
    Serial.print(data.rx);
    Serial.print(" ry:");
    Serial.print(data.ry);
    Serial.print(" rz:");
    Serial.print(data.rz);
    Serial.println();
  } */

  // digitalWrite(PIN_BACKWARD, 1);
  // delay(500);
  // digitalWrite(PIN_BACKWARD, 0);
  // delay(500);
  // digitalWrite(PIN_FORWARD, 1);
  // delay(500);
  // digitalWrite(PIN_FORWARD, 0);
  // delay(500);

  // for(int i = 0; i <= 255; i++) {
  //   
  //   Serial.println(i);
  //   delay(10);
  // }

  //   for(int i = 255; i >= 0; i--) {
  //   analogWrite(PIN_PWM, i);
  //   Serial.println(i);
  //   delay(10);
  // }
}

void switchOn(int pin) {
  digitalWrite(pin, HIGH); 
}

void switchOff(int pin) {
  digitalWrite(pin, LOW); 
}

void toggleBool(bool &ref) {
  ref = !ref;
}