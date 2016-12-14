#include <Servo.h>
#include <SoftwareSerial.h>

//Communications
#define Rx 10
#define Tx 11
SoftwareSerial Xbee (Rx, Tx);

//Line Following
int rValue, lValue, mValue;
Servo right;
Servo left;
const int rSens = 8; //QTI Sensor on the right side of the Bot
const int mSens = 7;
const int lSens = 6;
/* refers to left wheel, actuall
  y on the right side because
  if left wheel moves forward, the Bot turns right. */
const int rServo = 13; //Pin connected to the servo
const int lServo = 12;
//  Sensor Value pins:  as found through testing
int rCutoff = 120;
int lCutoff = 120;
int mCutoff = 120;

// Serial LCD
const int TxPin = 14; //Set pin for LCD
SoftwareSerial myLCD = SoftwareSerial(255, TxPin);

// Turning into the circle
boolean hash;
int hashTurnCount = 0;

// Challenge specific tasks
int result = 0; //Hold result from sensing
boolean sendMessage;
boolean brain, water, cyber, nukes, light;
boolean ourVal;
int total = 0;

// Timer
unsigned long timeout = 65000;
unsigned long currentTime = 0;

void hashTurn();
void detachMotor();
int sense();
void dispRes(int k);
void sendReceive();
void celebrate();
long rcTime(int k);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {

  //Line following setup
  Serial.begin(9600);
  right.attach(rServo);
  left.attach(lServo);
  pinMode(rServo, OUTPUT);
  pinMode(lServo, OUTPUT);

  //Sensing setup
  pinMode(3, INPUT);
  pinMode(9, OUTPUT);
  pinMode(15, INPUT);
  pinMode(16, OUTPUT);

  //Xbee setup
  Xbee.begin(9600);
  delay(500);
  pinMode(13, INPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  //LCD setup
  myLCD.begin(9600);
  pinMode(TxPin, OUTPUT);
  digitalWrite(TxPin, HIGH);

  // Tunring setup
  hash = false;
  sendMessage = false;

  // Challenges
  brain = false;
  water = false;
  cyber = false;
  nukes = false;
  light = false;
  ourVal = false;

}


void loop() {
  lineFollow();
  Serial.println("out of while loop");

  if (hash && hashTurnCount == 0) {
    Serial.println("hash detected");
    hashTurn();
    hashTurnCount++;
    detachMotor();
    delay(800);
    result = sense();  //Get result from sensing
    dispRes(result);
    sendMessage = true;
  }

  sendReceive();

  if (currentTime > timeout) {
    celebrate();
  }
  delay(10);
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Helper Methods

void lineFollow() {
  while (!hash) {
    rValue = rcTime(rSens);
    lValue = rcTime(lSens);
    mValue = rcTime(mSens);
    Serial.print(lValue);
    Serial.print(" ");
    Serial.print(mValue);
    Serial.print(" ");
    Serial.println(rValue);
    delay(10);

    //GOING STRAIGHT --------------------------------------------------------
    if (mValue >= mCutoff) {
      right.write(180);
      left.write(0);
    }
    //TURNING SHARP RIGHT ---------------------------------------------------
    if (lValue < lCutoff && mValue < mCutoff && rValue >= rCutoff) {
      left.write(100);
    }
    //TURNING SHARP LEFT ----------------------------------------------------
    else if (lValue >= lCutoff && mValue < mCutoff && rValue < rCutoff) {
      right.write(90);
    }
    //TURNING GRADUAL LEFT --------------------------------------------------
    else if (lValue >= lCutoff && mValue >= mCutoff && rValue < rCutoff) {
      left.write(60);
      right.write(90);
    }
    //TURNING GRADUAL RIGHT -------------------------------------------------
    else if (lValue < lCutoff && mValue >= mCutoff && rValue >= rCutoff) {
      right.write(120);
      left.write(90);
    }
    //STOPPING --------------------------------------------------------------
    if (lValue > lCutoff && mValue > mCutoff && rValue > rCutoff) {
      right.write(93);
      left.write(92);
      delay(900);
      hash = true;
    }
  }
}

void sendReceive(){

    while (sendMessage && currentTime <= timeout) {
      myLCD.write(12);                    // Clear
      myLCD.write(17);                    // Turn backlight on
      delay(5);
      if (result == 0) {
        sendMes('t');
        if (!ourVal) {
          total++;
          ourVal = true;
        }
      } else {
        sendMes('s');
      }

      total = receiveAndAdd(total);
      Serial.println(total);

      delay(100);
      myLCD.write(12);                    // Clear
      myLCD.write(17);                    // Turn backlight on
      delay(5);
      myLCD.print(total);
      myLCD.write(13);                    // Form feed
      printMes(total);
      delay(500);
      myLCD.write(18);                    // Turn backlight off

      currentTime = millis();
      Serial.println(currentTime);
    }
  }

  void hashTurn() {
    left.write(97);
    right.write(98);
    delay(1000);
    right.write(180);
    left.write(0);
    delay(700);
    right.write(93);
    left.write(92);
    delay(1000);
  }

  void detachMotor() {
    right.detach();
    left.detach();
  }

  void dispRes(int k) {
    if (k == 0) {
      myLCD.print("Skyscraper");
      delay(2500);
    } else {
      myLCD.print("Cottage");
      delay(2500);
    }
  }

  void printMes(int total) {
    if (total == 0) {
      myLCD.print("Fusion energy");
    } else if (total == 1) {
      myLCD.print("Nitrogen cycle");
    } else if (total == 2) {
      myLCD.print("Advance health");
    } else if (total == 3) {
      myLCD.print("Engineer med");
    } else if (total == 4) {
      myLCD.print("Virtual reality");
    } else if (total == 5) {
      myLCD.print("Personallearn");
    } else if (total == 6) {
      myLCD.print("Discovery tools");
    }
  }

  void sendMes(char k) {
    Xbee.print(k);
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
    delay(50);
  }

  int receiveAndAdd(int num) {
    int count = 0;
    while (Xbee.available() && count < 6) {
      char in = Xbee.read();
      count++;
      Serial.println(in);

      if (!brain) {
        if (in == 'm' || in == 'n') {
          brain = true;
        }
        if (in == 'm') {
          num++;
        }
      }
      if (!water) {
        if (in == 'c' || in == 'd') {
          water = true;
        }
        if (in == 'c') {
          num++;
        }
      }
      if (!cyber) {
        if (in == '@' || in == 'x') {
          cyber = true;
        }
        if (in == '@') {
          num++;
        }
      }
      if (!nukes) {
        if (in == 'a' || in == 'b') {
          nukes = true;
        }
        if (in == 'a') {
          num++;
        }
      }
      if (!light) {
        if (in == 'W' || in == 'L') {
          light = true;
        }
        if (in == 'W') {
          num++;
        }
      }
    }
    digitalWrite(4, HIGH);
    delay(500);
    digitalWrite(4, LOW);
    return num;
  }

  int sense() {
    int irLeft = irDetect(9, 3, 38000);       // Check for object
    int irRight = irDetect(16, 15, 38000);       // Check for object
    int value = 1;
    if (irLeft == 0 || irRight == 0) {
      digitalWrite(2, HIGH);
      tone(2, 3000, 1000);
      delay(1500);
      digitalWrite(2, LOW);
      value = 0;
    }
    Serial.println(irLeft);                    // Display 1/0 no detect/detect
    Serial.println(irRight);
    delay(500);
    return value;
  }

  int irDetect(int irLedPin, int irReceiverPin, long frequency)
  {
    tone(irLedPin, frequency, 8);
    delay(1);                                  // Wait 1 ms
    int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
    delay(1);                                  // Down time before recheck
    return ir;                                 // Return 1 no detect, 0 detect
  }


  long rcTime(int sPin) {
    long time = 0;
    pinMode(sPin, OUTPUT);
    digitalWrite(sPin, HIGH);
    delay(1);
    pinMode(sPin, INPUT);
    digitalWrite(sPin, LOW);
    while (digitalRead(sPin)) {
      time++;
    }
    return time;
  }

  void celebrate() {

    left.attach(12);
    right.attach(13);

    left.writeMicroseconds(1700);//move forward
    right.writeMicroseconds(1300);
    delay(500);
    left.writeMicroseconds(1700);//rotate right
    right.writeMicroseconds(1700);
    delay(250);
    left.writeMicroseconds(1300); //rotate left
    right.writeMicroseconds(1300);
    delay(250);
    left.writeMicroseconds(1300);//move backward
    right.writeMicroseconds(1700);
    delay(500);
    left.writeMicroseconds(1700);//rotate right
    right.writeMicroseconds(1700);
    delay(750);
    left.writeMicroseconds(1500);
    right.writeMicroseconds(1500);

  }
