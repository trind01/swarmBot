/*
 * Team Greige and Team Zanzibar
 * Phase 5
 * Thomas Rind, Anthony Nguyen, Danielle Skufca, Kayla Nies, John Patrick
*/
 
const int forward_R = 10;           // Right Wheel Pin
const int backward_R = 11;
const int forward_L = 12;           // Left Wheel Pin
const int backward_L = 9;

const int botSpeed = 120;           //Forward and Reverse Speed
const int highTurnSpeed = 200;      //The speed to turn in the opposite direction
const int lowTurnSpeed = 50;        //The speed to turn in that direction

const int blueLEDPin = 52;          //Blue LED Pin
const int yellowLEDPin = 50;        //Yellow LED Pin
const int redLEDPin = 48;           //Red LED Pin
struct segment7 {                   //Seven Segment Pins
  const int a = 42;
  const int b = 44;
  const int c = 46;
  const int d = 34;
  const int e = 36;
  const int f = 40;
  const int g = 38;
};
segment7 seg7;

const int colorSensePin = A8;       //Pin to read in voltage from photodiode
int colorVal;                       //Value read in from photodiode
const int detectionThreshold = 10;  //Number of detections used color to be detected
int redCount = 0;                   //Number of times red has been detected
int yellowCount = 0;                //Number of times yellow has been detected
int blueCount = 0;                  //Number of times blue has been detected
int lostPathCount = 0;              //Number of times the path has been lost
int redFind = 0;

struct colorThresholds {            //threshold limits for color detection
  int bLo;
  int bHi;
  int rLo;
  int rHi;
  int yLo;
  int yHi;
  int blHi; 
};
colorThresholds thresh;

//Hall Sensor Detection
const int hsReadIn = 18;
bool hsState = false;
int mineCount = 0;

//Collision Detection System
const int backBump = 32;
const int leftBump = 43;
const int rightBump = 47;
const int frontBump = 45;

//Comms System
const int receiver = 2;
const int transmitSig = 5;
const int transmitEn = 7;
enum MESSAGE {M00,M200,M300,M400,M500};
MESSAGE message = M00;

//Bot State Info
const int powSwitch = 51; //Blue Switch
const int botSwitch = 53; //Either Bot 1 or 2 red switch
bool botNum;              //True=Bot1 False=Bot2
enum BOTSTATE {WAIT,STATE1,FIND,TRACE,MAGNET,COMMS,COLLIDE,RETURN,GOHOME,FINSIH};
BOTSTATE botState = WAIT;


///////////////////////////////////////////////////////////////////////
//                          START                                    //
///////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  
  pinMode(forward_R,OUTPUT);    //Drive Pins
  pinMode(forward_L,OUTPUT);
  pinMode(backward_R,OUTPUT);
  pinMode(backward_L,OUTPUT);
  pinMode(colorSensePin,INPUT); //ColorSense Pin
  pinMode(blueLEDPin,OUTPUT);   //LED Pins
  pinMode(yellowLEDPin,OUTPUT);
  pinMode(redLEDPin,OUTPUT);
  pinMode(seg7.a, OUTPUT);      //7-Segment Pins
  pinMode(seg7.b, OUTPUT);
  pinMode(seg7.c, OUTPUT);
  pinMode(seg7.d, OUTPUT);
  pinMode(seg7.e, OUTPUT);
  pinMode(seg7.f, OUTPUT);
  pinMode(seg7.g, OUTPUT);
  pinMode(hsReadIn,INPUT);      //Hall Sensor Pin
  pinMode(backBump,INPUT);      //Collision Pins
  pinMode(frontBump,INPUT);
  pinMode(rightBump,INPUT);
  pinMode(leftBump,INPUT);
  pinMode(receiver,INPUT);      //Comms Pin
  pinMode(transmitSig,OUTPUT);
  pinMode(transmitEn,OUTPUT);
  pinMode(powSwitch,INPUT);    //Bot State Pins
  pinMode(botSwitch,INPUT);

//Needed For Transmitter
  // sets COM Output Mode to FastPWM with toggle of OC3A on compare match with OCR3A
  // also sets WGM to mode 15: FastPWM with top set by OCR3A
  TCCR3A = _BV(COM3A0) | _BV(COM3B0) | _BV(WGM30) | _BV(WGM31);
  // sets WGM as stated above; sets clock scaling to "divide by 8"
  TCCR3B = _BV(WGM32) | _BV(WGM33) |  _BV(CS31);
  // above sets the counter value at which register resets to 0x0000;
  // generate 18.523 kHz when OCR3A=53 on Mega pin 5
  OCR3A = 53;


//Hall Sensor Interrupt.
  attachInterrupt(digitalPinToInterrupt(hsReadIn), hSDetection, RISING);

//Set Color Sensor Thresholds
  thresh.bLo = 140;  //blueThresh - 30; // F 140
  thresh.bHi = 185; //blueThresh + 15; // 175
  thresh.rLo = 60; //redThresh - 25; //F 80 
  thresh.rHi = 112;//redThresh + 20; //F 110
  thresh.yLo = 190; //yellowThresh - 5; //F 190 L180
  thresh.yHi = 210; //yellowThresh + 10; //F 210 L210
  thresh.blHi = 30;//yellowThresh - 130;

//Display MineCount
  //set7Seg(mineCount); 

//Determine what Bot it is
  botNum = digitalRead(botSwitch);

}

//////////////////////////////////////////////////////////////////////
//                            LOOP                                  //
//////////////////////////////////////////////////////////////////////

void loop() {
  switch(botState){
    case WAIT:
      idle();
      break;
    case STATE1:
      state1();
      break;
    case FIND:
      findpath();
      break;
    case TRACE:
      trace();
      break;
    case MAGNET:
      magnet();
      break;
    case COMMS:
      commsMine();
      break;
    case COLLIDE:
      collide();
      break;
    case RETURN:
      returnToBase();
      break;
    case GOHOME:
      goHome();
      break;
    case FINSIH:
      break;
  }
  //colorVal = analogRead(colorSensePin);
  //Serial.println(colorVal);
  
}
///////////////////////////////////////////////////////
//               State Functions                     //
///////////////////////////////////////////////////////

/*
 * void idle()
 * void state1()
 * void findpath()
 * void trace()
 * void magnet()
 * void commsMine()
 * void collide()
 * void returnToBase()
 * void firstYellow()
 */

/*
 * Makes Bot wait for the power switch to be flipped and
 * if bot 2 makes the bot wait until it hears a 300ms
 * Signal
*/
void idle(){
  //Wait for power switch to be flipped before starting
  while(!digitalRead(powSwitch)){}
  botState = STATE1;
  digitalWrite(yellowLEDPin,HIGH);
  delay(500);
  //if bot 2 wait until you hear a 300ms Signal
  if(!botNum){
    while(message != M300){Comms();}
  }
  forward();
}

//Hits wall, changes angle and reverses to find the path.
void state1(){
  //Wait until hits wall
  while(!digitalRead(frontBump) && !digitalRead(leftBump) && !digitalRead(rightBump)){
    colorVal = analogRead(colorSensePin);
    if (colorVal >= thresh.yHi || colorVal <= thresh.yLo){
      if(yellowCount > 5){
        digitalWrite(yellowLEDPin,LOW);
      }
      yellowCount++;
    }
  };
  brake();
  //Move back a little so there is room to rotate
  reverse();
  delay(50);
  brake();
  //Each bot needs to rotate in a different angle
  if(!botNum){
    pivotRight(30);
  }
  else{
    pivotLeft(30);
  }
  //start reversing and advance to next state to start looking
  //for the path
  reverse();
  botState = FIND;
}

//Keep going backwards until you find the path
//Each Bot needs to find a different path
//Once the path is found pivot so bot is more on path
void findpath(){
  colorVal = analogRead(colorSensePin);
  if(!botNum){
    if(colorVal <= thresh.bHi && colorVal >= thresh.bLo){
      if(blueCount > 10){
        brake();
        forward();
        delay(50);
        brake();
        resetColorDetection();
        botState = TRACE;
        pivotLeft(90);
      }
      blueCount++;
    }
  }
  else{
    if(colorVal <= thresh.rHi && colorVal >= thresh.rLo){
      if(redCount > 20){
        Serial.println("REDDDDDDDDDDDDD");
        redFind++;
        if(redFind < 3){
          reverse();
          delay(100);
        }
        else{
          brake();
          forward();
          delay(50);
          brake();
          resetColorDetection();
          botState = TRACE;
          pivotRight(60); 
        } 
      }
      redCount++;
    }
  }
}

//Follow path until a mine is found or it hits the front wall
void trace(){
  colorVal = analogRead(colorSensePin);
  if(digitalRead(frontBump) || digitalRead(leftBump) || digitalRead(rightBump)){
      brake();
      botState = COLLIDE;
      return;
  }
  if(!botNum){
    if(colorVal <= thresh.bHi && colorVal >= thresh.bLo){
      if(blueCount > 5){
        digitalWrite(blueLEDPin,HIGH);
        forward();
        resetColorDetection();
      }
      blueCount++;
    }
    else{
      bool pathDetected = false;
      int i = 1;
      lostPathCount++;
      if(lostPathCount > 10){
        while(!pathDetected){
          if(digitalRead(frontBump) || digitalRead(leftBump) || digitalRead(rightBump)){
            brake();
            botState = COLLIDE;
            return;
          }
          pathDetected = pathCorrectRight(i); 
          if(pathDetected){
            resetColorDetection();
            break;
          }
          i++;
          pathDetected = pathCorrectLeft(i);
          i++;
          if(pathDetected){
            resetColorDetection();
            break;
          }
        } 
      }
    }
  }
  else{
    if(colorVal <= thresh.rHi && colorVal >= thresh.rLo){
      if(redCount > 5){
        digitalWrite(redLEDPin,HIGH);
        forward();
        resetColorDetection();
      }
      redCount++;
    }
    else{
      bool pathDetected = false;
      int i = 1;
      lostPathCount++;
      if(lostPathCount > 10){
        while(!pathDetected){
          if(digitalRead(frontBump) || digitalRead(leftBump) || digitalRead(rightBump)){
            brake();
            botState = COLLIDE;
            return;
          }
          pathDetected = pathCorrectLeft(i); //pathCorrectRight
          if(pathDetected){
            resetColorDetection();
            break;
          }
          i++;
          pathDetected = pathCorrectRight(i); //pathCorrectLeft
          i++;
          if(pathDetected){
            resetColorDetection();
            break;
          }
        } 
      }
    }
  }
}

/*
 * Once the magnet is found Flash the blue LED
 * In the event that this is called when the 
 * HS stops detecting the magnet the State of
 * Bot won't change
*/
void magnet(){
  brake();
  for(int i=0;i<2;i++){
    digitalWrite(blueLEDPin,HIGH);
    delay(50);
    digitalWrite(blueLEDPin,LOW);
    delay(50);
  }
  botState = COMMS;
}

/*
 * When a bot detects the land mine send 
 * the appropriate message to the CCP.
 * When the bot gets a message back from 
 * the CCP advance to the next state.
 */
void commsMine(){
  //Send the 200ms signal if bot 1
  if(botNum){
    transmit(220);
    pivotRight(50);
    while (message != M200){Comms();}; 
    pivotLeft(50);
  }
  else{
    delay(50);
    transmit(440);
    delay(200);
    transmit(440);
    pivotRight(90);
    while (message != M400){Comms();}
    pivotLeft(90);
  }
  botState = TRACE;
  message = M00;
}

/*
 * When the bot hits the wall after tracing 
 * it will  pivot 180 degrees the line it 
 * illuminates the yellow LED.
 * Bot 1 will send out a 300ms Signal and wait for a 500ms Signal
 * Bot 2 will send out a 500ms.
 * both will then start returning in the next state
 */
void collide(){
  for(int i=0;i<2;i++){
    digitalWrite(redLEDPin,HIGH);
    delay(200);
    digitalWrite(redLEDPin,LOW);
    delay(200);
  }
  //Back up a little so bot can turn around.
  reverse();
  delay(150);
  brake();
  digitalWrite(yellowLEDPin,HIGH);
  if(botNum){
    pivotRight(170); //non fresh 210 fresh 170
    delay(2000);
    unsigned long check = millis();
    transmit(330);
    delay(2000);
    transmit(330);
    pivotRight(20);
    while(message!= M500){Comms();}
    pivotLeft(80);
    message = M00;
  }
  else{
    pivotLeft(150);
    delay(2000);
    transmit(600);
    delay(1000);
    transmit(600);
  }
  botState = RETURN;
}

/*
 * Bot traces respective line until it reaches the 
 * yellow square in the center of the other end.
 */
void returnToBase(){
  colorVal = analogRead(colorSensePin);
  if(!botNum){
    if(colorVal <= thresh.bHi && colorVal >= thresh.bLo){
      if(blueCount > 5){
        forward();
        resetColorDetection();
      }
      blueCount++;
    }
    else if (colorVal <= thresh.yHi && colorVal >= thresh.yLo){
      if(yellowCount > 10){
        brake();
        resetColorDetection();
        botState = GOHOME;
      }
      yellowCount++;
    }
    else{
      bool pathDetected = false;
      int i = 1;
      lostPathCount++;
      if(lostPathCount > 10){
        while(!pathDetected){
          if(digitalRead(frontBump)){
            brake();
            botState = GOHOME;
            return;
          }
          pathDetected = pathCorrectRight(i);
          if(pathDetected){
            resetColorDetection();
            break;
          }
          i++;
          pathDetected = pathCorrectLeft(i);
          i++;
          if(pathDetected){
            resetColorDetection();
            break;
          }
        } 
      }
    }
  }
  else{
    if(colorVal <= thresh.rHi && colorVal >= thresh.rLo){
      if(redCount > 5){
        forward();
        resetColorDetection();
      }
      redCount++;
    }
    else if (colorVal <= thresh.yHi && colorVal >= thresh.yLo){
      if(yellowCount > 10){
        brake();
        resetColorDetection();
        botState = GOHOME;
      }
      yellowCount++;
    }
    else{
      bool pathDetected = false;
      int i = 1;
      lostPathCount++;
      if(lostPathCount > 5){
        while(!pathDetected){
          if(digitalRead(frontBump)){
            brake();
            botState = GOHOME;
            return;
          }
          pathDetected = pathCorrectLeft(i);
          if(pathDetected){
            resetColorDetection();
            break;
          }
          i++;
          pathDetected = pathCorrectRight(i);
          i++;
          if(pathDetected){
            resetColorDetection();
            break;
          }
        } 
      }
    }
  }
}

/*
 * From the center yellow square each bot will turn
 * and move to their respective yellow squares.
 * After that bot 1 makes a 500ms while bot 2
 * waits to hear it. Then bot 2 makes a 500ms
 * while bot 1 waits to hear it.
 * Afterwards each bot flashes their Blue and Red LEDs
 * 10 times.
 */
void goHome(){
  if(botNum){
    pivotLeft(100);
  }
  else{
    pivotRight(100);
  }
  forward();
  bool homeFound = false;
  while(!homeFound){
    if(digitalRead(frontBump) || digitalRead(leftBump) || digitalRead(rightBump)){
      brake();
      homeFound = true;
    }
  }
  if(botNum){
    reverse();
    delay(200);
    pivotLeft(90);
    transmit(600);
    pivotLeft(50);
    unsigned long wait = millis();
    while(millis() - wait < 3000){if(message != M500){Comms();}};
    transmit(600);
    wait = millis();
    while(millis() - wait < 3000){if(message != M500){Comms();}};
    transmit(600);
    wait = millis();
    while(millis() - wait < 3000){if(message != M500){Comms();}};
    transmit(600);
    pivotLeft(100);
    while(message != M500){Comms();}
  }
  else{
    reverse();
    delay(100);
    pivotRight(180);
    while(message != M500){Comms();}
    pivotLeft(60);
    transmit(600);
    delay(3000);
    transmit(600);
    delay(3000);
    transmit(600);
    delay(3000);
    transmit(600);
  }
  for(int i = 0; i < 10; i++){
    digitalWrite(redLEDPin,HIGH);
    digitalWrite(blueLEDPin,HIGH);
    delay(500);
    digitalWrite(redLEDPin,LOW);
    digitalWrite(blueLEDPin,LOW);
    delay(500);
  }
  digitalWrite(redLEDPin,HIGH);
  botState = FINSIH;
}



///////////////////////////////////////////////////////
//              Helper Functions                     //
///////////////////////////////////////////////////////
/*
 *void forward()
 *void reverse()
 *void brake()
 *bool pathCorrectRight(int iteration)
 *bool pathCorrectLeft(int iteration)
 *void pivotRight(int degree)
 *void pivotLeft(int degree)
 *void resetColorDetection()
 *void hsDetection()
 *void commsInterrupt()
 *void Comms()
 *void transmit(int length)
 *void set7Seg(int num)
 */

//Moves bot forward
void forward(){
  digitalWrite(backward_R,LOW);
  digitalWrite(backward_L,LOW);
  analogWrite(forward_R,botSpeed);
  analogWrite(forward_L,botSpeed);
  return;
}

//Moves bot backwards
void reverse(){
  digitalWrite(forward_R,LOW);
  digitalWrite(forward_L,LOW);
  analogWrite(backward_R,botSpeed);
  analogWrite(backward_L,botSpeed);
  return;
}

//Stops the bot
void brake(){
  digitalWrite(forward_R,LOW);
  digitalWrite(forward_L,LOW);
  digitalWrite(backward_R,LOW);
  digitalWrite(backward_L,LOW);
  return;
}

//Searches for the path by pivoting to the right
bool pathCorrectRight(int iteration){
  digitalWrite(backward_R,LOW);
  digitalWrite(forward_L,LOW);
  analogWrite(forward_R,botSpeed);
  analogWrite(backward_L,botSpeed);
  unsigned long timeCheck = millis();
  int b = 0;
  int y = 0;
  int r = 0;
  while( (millis()-timeCheck) < 200 * iteration){     
    colorVal = analogRead(colorSensePin);
    if (colorVal <= thresh.bHi && colorVal >= thresh.bLo && !botNum){
      b++;
    }
    else if (botState == RETURN && colorVal <= thresh.yHi && colorVal >= thresh.yLo){
      y++;
    }
    else if (colorVal <= thresh.rHi && colorVal >= thresh.rLo && botNum){
      r++;
    }
    if(b > detectionThreshold || r > detectionThreshold || y > detectionThreshold){
      brake();
      return true;
    }
  }
  brake();
  return false;
}

//Will search for the path turning to the left
bool pathCorrectLeft(int iteration){
  digitalWrite(backward_L,LOW);
  digitalWrite(forward_R,LOW);
  analogWrite(forward_L,botSpeed);
  analogWrite(backward_R,botSpeed);
  unsigned long timeCheck = millis();
  int b = 0;
  int y = 0;
  int r = 0;
  while( (millis()-timeCheck) < 200 * iteration){     
    colorVal = analogRead(colorSensePin);
    if (botNum && colorVal <= thresh.bHi && colorVal >= thresh.bLo && !botNum){
      b++;
    }
    else if (botState == RETURN && colorVal <= thresh.yHi && colorVal >= thresh.yLo){
      y++;
    }
    else if (!botNum && colorVal <= thresh.rHi && colorVal >= thresh.rLo && botNum){
      r++;
    }
    if(b > detectionThreshold || r > detectionThreshold || y > detectionThreshold){
      return true;
    }
  }
  brake();
  return false;
}

//pivots the bot right by the number of degrees
void pivotRight(int degree){
  digitalWrite(backward_R,LOW);
  digitalWrite(forward_L,LOW);
  analogWrite(backward_L,botSpeed);
  analogWrite(forward_R,botSpeed);
  unsigned long timeCheck = millis();
  while( (millis()-timeCheck) < (2300.0*(degree/360.0))){
    //700ms to complete pivot
  }
  brake();
  return;
}

//pivots the bot Left by the number of degrees
void pivotLeft(int degree){
  digitalWrite(backward_L,LOW);
  digitalWrite(forward_R,LOW);
  analogWrite(backward_R,botSpeed);
  analogWrite(forward_L,botSpeed);
  unsigned long timeCheck = millis();
  while( (millis()-timeCheck) < (2300.0*(degree/360.0))){
    //700ms to complete pivot
  }
  brake();
  return;
}

//Resets the detections for all colors
void resetColorDetection(){
  redCount = 0;
  yellowCount = 0;
  lostPathCount = 0;
  blueCount = 0;
}

//Interrupt for when there is change in hsDetection
void hSDetection(){
  hsState = true;
  if(mineCount == 0){
    botState = MAGNET;
    mineCount++;
  }
  set7Seg(mineCount);
}


void Comms(){
  int signalLength = 0;
  int signalCount = 0;
  int signalLowCount = 0; 
  long timeStart = millis();
  while(signalCount < 60 && signalLowCount < 30){
    while(millis() - timeStart  < 10){}
    if(digitalRead(receiver)){
      signalLength++;
    }
    else{
      signalLowCount++;
    }
    signalCount++;
    timeStart = millis();
  }
  Serial.print("SIGNAL");
  Serial.println(signalLength);
  if(35 < signalLength and signalLength <= 45){
    message = M400;
    //set7Seg(4);
  }
  else if(25 < signalLength and signalLength <= 35){
    message = M300;
    //set7Seg(3);
  }
  else if(15 < signalLength and signalLength <= 25){
    message = M200;
    //set7Seg(2);
  }
  else if(45 < signalLength and signalLength <= 60){
    message = M500;
    //set7Seg(5);
  }
}

void transmit(int length){
  digitalWrite(transmitEn,HIGH);
  delay(length);
  digitalWrite(transmitEn,LOW);
}

//Set the 7 Segment with a number 0-9
void set7Seg(int num){
  if(num < 0 || num > 9){
    return;
  }
  // TURNING OFF 7SEG JUST FOR NOW!!!
  //num = 10;

  
  switch(num){
    case 0:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,HIGH);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,LOW);
      break;
    case 1:
      digitalWrite(seg7.a,LOW);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,LOW);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,LOW);
      digitalWrite(seg7.g,LOW);
      break;
    case 2:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,LOW);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,HIGH);digitalWrite(seg7.f,LOW);
      digitalWrite(seg7.g,HIGH);
      break;
    case 3:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,LOW);
      digitalWrite(seg7.g,HIGH);
      break;
    case 4:
      digitalWrite(seg7.a,LOW);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,LOW);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,HIGH);
      break;
    case 5:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,LOW);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,HIGH);
      break;
    case 6:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,LOW);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,HIGH);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,HIGH);
      break;
    case 7:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,LOW);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,LOW);
      digitalWrite(seg7.g,LOW);
      break;
    case 8:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,HIGH);digitalWrite(seg7.e,HIGH);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,HIGH);
      break;
    case 9:
      digitalWrite(seg7.a,HIGH);digitalWrite(seg7.b,HIGH);digitalWrite(seg7.c,HIGH);
      digitalWrite(seg7.d,LOW);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,HIGH);
      digitalWrite(seg7.g,HIGH);
      break;
    case 10:
      digitalWrite(seg7.a,LOW);digitalWrite(seg7.b,LOW);digitalWrite(seg7.c,LOW);
      digitalWrite(seg7.d,LOW);digitalWrite(seg7.e,LOW);digitalWrite(seg7.f,LOW);
      digitalWrite(seg7.g,LOW);
      break;
  }
}
