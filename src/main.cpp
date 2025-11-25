#include <nRF24L01.h>
#include <RF24.h>

//Pin Declarations
#define PIN_CE 7
#define PIN_CSN 8

/* #define PIN_FORWARD 2
#define PIN_BACKWARD 4
#define PIN_PWM 3 

#define PIN_HORN 1000

#define PIN_HORN 13
#define PIN_LIGHTS 12
#define PIN_LZ1  11
#define PIN_PLACEHOLDER1 10
#define PIN_PLACEHOLDER2 9
#define PIN_PLACEHOLDER3 15 */

/* class Stick {
  public: 
    Stick(int lowerLimit, int upperLimit, int assignedPin, int boundryTolerance):
      lowerLimit(lowerLimit),
      upperLimit(upperLimit), 
      assignedPin(assignedPin),
      boundryTolerance(boundryTolerance),
      currentPosition((lowerLimit + upperLimit) / 2)
    {
      /* this->boundryTolerance = boundryTolerance;  
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

    virtual void write() = 0; 

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
      this->direction = true;  
    }

    void setDirectionSafe(bool dir) {
      if (currentPosition != 0) return; 
      this->direction = dir; 
    }

    void write() override {
      if (currentPosition == 0) {
        if(direction) { 
          writeForward(); 
        } else {
          writeBackward(); 
        }
      } else {
        writeMappedPWM(); 
      }
      Serial.print("   dir: "); 
      Serial.print(direction); 
      Serial.print("   current_val: ");
      Serial.print(currentPosition);  
    }
    
    int forwardPin; 
    int backwardPin;

  private:
    
    void writeMappedPWM() {
      int pwmSignal = map(currentPosition, 0, 1024, 0, 255); 
      //analogWrite(assignedPin, pwmSignal);
    } 

    void writeForward() { 
      //digitalWrite(backwardPin, LOW); 
      //digitalWrite(forwardPin, HIGH); 
    }

    void writeBackward() { 
      //digitalWrite(forwardPin, LOW); 
      //digitalWrite(backwardPin, HIGH);
    }

    bool direction; 
};

class BinaryStick: public Stick {
  public: 
    BinaryStick(int lowerLimit, int upperLimit, int assignedPin, int boundryTolerance = 50): Stick(lowerLimit, upperLimit, assignedPin, boundryTolerance) {
      this->isActive = false;  
    }

    void write() override {
      //digitalWrite(assignedPin, isActive);
      Serial.print("   isActive: ");
      Serial.print(isActive);
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
      Serial.print("   toggleStick value: ");
      Serial.print(value); 
      Serial.print("   isActive: ");
      Serial.print(isActive);  
    }

    void write() override {
      if (!isLocked) {
        //digitalWrite(assignedPin, isActive);
        isLocked = true; 
      }
    }

    bool isLocked;

  private: 
    void toggleActive() {
      isActive = !isActive; 
    }
};

class Switch {
  public:
    Switch(int assignedPin) {
      this->assignedPin = assignedPin; 
      this->active = false; 
    }

    void update(bool value) {
      this->active = value; 
    }

    void write() {
      //digitalWrite(assignedPin, active); 
    }

  private: 
    int assignedPin; 
    bool active; 
};
 */



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

/* BinaryStick lx(0, 1024, 1000, standardStickTolerance); 
PWMStick ly(0, 1024, PIN_PWM, PIN_FORWARD, PIN_BACKWARD, standardStickTolerance); 
Switch lz(PIN_HORN); 
ToggleStick ryu(512, 1024, PIN_LIGHTS, standardStickTolerance);
ToggleStick ryl(0, 1024, PIN_LZ1, standardStickTolerance); 
ToggleStick rxl(512, 1024, PIN_PLACEHOLDER1, standardStickTolerance); 
ToggleStick rxr(512, 1024, PIN_PLACEHOLDER2, standardStickTolerance);
Switch rz(PIN_PLACEHOLDER3); */

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
  
  Serial.println("Setup - End");
}

void loop() {
  Serial.println("hello");
  /* 
  LX: Richtung V> R< 
  LY: Fahrregler 
  LZ: Horn 

  RX: Umschalten Hecklichter < / FZ1 (1 weiße Leuchte auf Pufferhöhe) > 
  RY: Kabinenbeleuchtung / Licht an/aus 
  RZ: Hauptschalter?  / Rangierschalter (langsamere V-max)?
  */
  /*Serial.println("Start");

  //Check radio availability
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
  }

  lx.update(data.lx); 
  ly.update(data.ly); 
  lz.update(data.lz);
  rxl.update(data.rx); 
  rxr.update(data.rx); 
  ryl.update(data.ry); 
  ryu.update(data.ry);
  rz.update(data.rz);  

  lx.write(); 
  Serial.print(" | ");
  ly.write(); 
  Serial.print(" | ");
  lz.write(); 
  Serial.print(" | ");
  
  rxl.write();
  Serial.print(" | ");
  rxr.write();
  Serial.print(" | ");
  ryu.write();
  Serial.print(" | ");
  ryl.write();
  Serial.print(" | ");
  rz.write(); 
  Serial.print(" | "); */
}