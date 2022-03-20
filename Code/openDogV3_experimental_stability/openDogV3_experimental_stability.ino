//ODrive
#include <ODriveArduino.h>

// IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//I2C LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// ramp lib
#include <Ramp.h>

RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}


//ODrive Objects
ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial2);
ODriveArduino odrive3(Serial3);
ODriveArduino odrive4(Serial4);
ODriveArduino odrive5(Serial5);
ODriveArduino odrive6(Serial6);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t menuDown;
  int16_t Select;
  int16_t menuUp;
  int16_t toggleBottom;
  int16_t toggleTop;
  int16_t toggle1;
  int16_t toggle2;
  int16_t mode;
  int16_t RLR;
  int16_t RFB;
  int16_t RT;
  int16_t LLR;
  int16_t LFB;
  int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int toggleTopOld;
int remoteState;
int remoteStateOld;

float RLR = 0;
float RFB = 0;
float RT = 340;
float LLR = 0;
float LFB = 0;
float LT = 0;

float RLRFiltered = 0;
float RFBFiltered = 0;
float RTFiltered = 340;
float LLRFiltered = 0;
float LFBFiltered = 0;
float LTFiltered = 0;
int filterFlag1 = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

long previousIMUMillis = 0;    // set up timers

long previousInterpMillis = 0;    // set up timers
int interpFlag = 0;

unsigned long remoteMillis;

int loopTime;
int prevLoopTime;

int stepFlag = 0;
long previousStepMillis = 0;
int stepStartFlag = 0;

int requested_state;
int mode;
int modeOld;
int modeFlag;
int menuFlag;
int modeConfirm;
int modeConfirmFlag = 0;
int runMode = 0;

float longLeg1;
float shortLeg1;
float legLength1;
float longLeg2;
float shortLeg2;
float legLength2;
float footOffset;

float fr_RFB;
float fl_RFB;
float bl_RFB;
float br_RFB;
float fr_RLR;
float fl_RLR;
float bl_RLR;
float br_RLR;;
float fr_LT;
float fl_LT;
float bl_LT;
float br_LT;

float fr_LLR;
float fl_LLR;
float br_LLR;
float bl_LLR;

int timer1;   // FB gait timer
int timer2;   // LR gait timer
int timer3;   // Yaw gait timer
float timerScale;   // resulting timer after calcs
float timerScale2;   // multiplier

// ODrive offsets from power up
// ratio is 10:1 so 1 'turn' is 36'.


float offSet20 = -0.1;      //ODrive 2, axis 0      // knee - right front
float offSet30 = -0.45;      //ODrive 3, axis 0     // knee - right rear
float offSet50 = -0.05;      //ODrive 5, axis 0     // knee - left front
float offSet60 = -0.4;      //ODrive 6, axis 0       // knee - left rear

float offSet21 = -0.1;      //ODrive 2, axis 1     // shoulder - right front
float offSet31 = 0.45;      //ODrive 3, axis 1     // shoulder - right rear
float offSet51 = 0.66;      //ODrive 5, axis 0     // shoulder - left front
float offSet61 =  -0.08;      //ODrive 6, axis 1     // shoulder - left rear

float offSet10 = 0.27;      //ODrive 1, axis 0     // hips - right front
float offSet11 = 0.1;      //ODrive 1, axis 1     // hips - right back
float offSet40 = 0.07;      //ODrive 4, axis 0     // hips - left front
float offSet41 = 0.35;      //ODrive 4, axis 1     // hips - left back

// IMU variables

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gyr_Gain 0.00763358 

float AccelX;
float AccelY;
float AccelZ;

float GyroX;
float GyroY;
float GyroZ;

float mixX;
float mixY;

float pitchAccel, rollAccel;

float IMUpitch;
float IMUroll;

// Dynamics stability variables

float legTransX;
float legTransY;
float legTransXFiltered;
float legTransYFiltered;

float legRoll;
float legPitch;
float legRollFiltered;
float legPitchFiltered;



//***********************************
//***********************************
class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      int output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpFRX;        // interpolation objects
Interpolation interpFRY;
Interpolation interpFRZ;
Interpolation interpFRT;

Interpolation interpFLX;        // interpolation objects
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLT;

Interpolation interpBRX;        // interpolation objects
Interpolation interpBRY;
Interpolation interpBRZ;
Interpolation interpBRT;

Interpolation interpBLX;        // interpolation objects
Interpolation interpBLY;
Interpolation interpBLZ;
Interpolation interpBLT;

//***********************************
//***********************************

// ****************** SETUP ******************************

void setup() {


  // initialize serial communication
  Serial.begin(115200);

  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);


  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();

  //LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("openDog V3");
  lcd.setCursor(0, 1);
  lcd.print("S:0  C:0");

}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

    if (mydata_remote.toggleBottom == 1) {
        // read IMU
        Wire.begin();   
        accelgyro.initialize(); 
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        AccelX = ax;
        AccelY = ay;
        AccelZ = az;
        GyroX = Gyr_Gain * (gx);
        GyroY = Gyr_Gain * (gy)*-1;
        GyroZ = Gyr_Gain * (gz);
      
        AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
        AccelX = (atan2(AccelX, AccelZ) * 180 / PI);
    
        float dt = 0.01;
        float K = 0.9;
        float A = K / (K + dt);
      
        mixX = A *(mixX+GyroX*dt) + (1-A)*AccelY;    
        mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX; 
    
        IMUpitch = mixX + 2.7;      // trim IMU to zero
        IMUroll = mixY - 5;      
    }
    else {
        // ignore IMU data
        Wire.end();
        IMUpitch = 0;      // IMU data is zeero, do not read IMU 
        IMUroll = 0;
    }

    //check loop is actually running the speed we want
    //loopTime = currentMillis - prevLoopTime;
    //prevLoopTime = currentMillis;
    //Serial.println(loopTime);

    


    // check for radio data
    if (radio.available()) {
      radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
      remoteMillis = currentMillis;
    }

    // is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 500) {
      remoteState = 0;
    }
    else {
      remoteState = 1;
    }    

    // threshold remote data
    // some are reversed based on stick wiring in remote
    RFB = (thresholdStick(mydata_remote.RFB)) * -1;
    RLR = (thresholdStick(mydata_remote.RLR));
    RT = thresholdStick(mydata_remote.RT);
    LFB = (thresholdStick(mydata_remote.LFB)) * -1;
    LLR = thresholdStick(mydata_remote.LLR);
    LT = thresholdStick(mydata_remote.LT);

    
    // stop the dog if the remote becomes disconnected
    if (remoteState == 0) {     
      RFB = 0;
      RLR = 0;
      RT = 0;
      LFB = 0;
      LLR = 0;
      LT = 0;
    }


    // mode select

    if (mydata_remote.menuUp == 1 && menuFlag == 0) {
      menuFlag = 1;
      mode = mode + 1;
      mode = constrain(mode, 0, 10);
    }
    else if (mydata_remote.menuDown == 1 && menuFlag == 0) {
      menuFlag = 1;
      mode = mode - 1;
      mode = constrain(mode, 0, 10);
    }
    else if (mydata_remote.menuDown == 0 && mydata_remote.menuUp == 0) {
      menuFlag = 0;
    }

    if (mode != modeOld) {            // only update the LCD if the data has changed
      lcd.setCursor(0, 1);
      lcd.print("    ");
      lcd.setCursor(0, 1);
      lcd.print("S:");
      lcd.print(mode);
    }
    modeOld = mode;       // bookmark previous data so we can see if it's changed

    if (mydata_remote.Select == 1) {
      modeConfirm = mode;             // make the actual mode be the potential mode when the select button is pressed
      lcd.setCursor(5, 1);            // display current mode on LCD
      lcd.print("    ");
      lcd.setCursor(5, 1);
      lcd.print("C:");
      lcd.print(modeConfirm);
    }

    // display remote connection state
    if (remoteState != remoteStateOld) {
      if (remoteState == 0) {
        lcd.setCursor(13, 0);
        lcd.print("N");
      }
      else if (remoteState == 1) {
        lcd.setCursor(13, 0);
        lcd.print("C");
      }
    }
    remoteStateOld = remoteState;

    // display motor enable state

    if (mydata_remote.toggleTop != toggleTopOld) {
      if (mydata_remote.toggleTop == 1) {
        lcd.setCursor(15, 0);
        lcd.print("E");
      }
      else if (mydata_remote.toggleTop == 0) {
        lcd.setCursor(15, 0);
        lcd.print("D");
      }
    }
    toggleTopOld = mydata_remote.toggleTop;

    // ****** MAIN MENU ******* 

    if (mydata_remote.Select == 0) {                            // make sure the button is released before it can be pressed again
              modeConfirmFlag = 0;
    }

    if (modeConfirm == 1 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {             // init ODrives with low gains
      Serial.println("Init Odrives mode 1");
      OdriveInit1();
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 2 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // default to 45' shoulder and knees
      Serial.println("Knees mode 2");
      applyOffsets1();
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 3 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // default to hip position
      Serial.println("Shoulders mode 3");
      applyOffsets2();
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 4 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // turn up gains
      Serial.println("Modify Gains mode 4");
      modifyGains();
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 5 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // turn up gains
      Serial.println("Kinematics mode 5");
      interpFlag = 0;
      previousInterpMillis = currentMillis;
      runMode = 1;
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 6 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // turn up gains
      Serial.println("Walking Mode 6");
      interpFlag = 0;
      previousInterpMillis = currentMillis;
      runMode = 2;
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 10 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // turn up gains
      Serial.println("Going Home Mode 10");
      interpFlag = 0;
      previousInterpMillis = currentMillis;
      runMode = 10;
      modeConfirmFlag = 1;
    }

    else if (modeConfirm == 9 && modeConfirmFlag == 0 && mydata_remote.Select == 1) {       // turn up gains
      Serial.println("Interp Test");
      interpFlag = 0;
      previousInterpMillis = currentMillis;
      runMode = 9;
      modeConfirmFlag = 1;
    }
    
    // **** END OF MAIN MENU **** 
 

    if (runMode == 10) {         // put the legs back on the stand

        int offset1 = 70;

        kinematics (1, -offset1, 0, 270, 0, 0, 0, 0, 0);   // front right
        kinematics (2, -offset1, 0, 270, 0, 0, 0, 0, 0);   // front left
        kinematics (3, offset1, 0, 270, 0, 0, 0, 0, 0);   // back left
        kinematics (4, offset1, 0, 270, 0, 0, 0, 0, 0);   // back right
    }

    else if (runMode == 1) {
      
    // ** inverse kinematics demo ** 

        // scale sticks to mm
        RFB = map(RFB,-462,462,-100,100);
        RLR = map(RLR,-462,462,-100,100);        
        RT = map(RT,-462,462,240,440);
        RT = constrain(RT,240,380);      
        LFB = map(LFB,-462,462,-15,15);        
        LLR = map(LLR,-462,462,-15,15);        
        LT = map(LT,-462,462,-20,20);      

        // filter sticks
        RFBFiltered = filter(RFB, RFBFiltered, 40);
        RLRFiltered = filter(RLR, RLRFiltered, 40);
        RTFiltered = filter(RT, RTFiltered, 40);
        LFBFiltered = filter(LFB, LFBFiltered, 40);
        LLRFiltered = filter(LLR, LLRFiltered, 40);
        LTFiltered = filter(LT, LTFiltered, 40);
                  
        kinematics (1, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);   // front right
        kinematics (2, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);   // front left
        kinematics (3, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);   // back left
        kinematics (4, RFBFiltered, RLRFiltered, RTFiltered, LLRFiltered, LFBFiltered, LTFiltered, 0, 0);   // back right
      
 }

   else if (runMode == 2) {
      // simple walking
      
      RFB = map(RFB,-462,462,-50,50);  // mm
      RLR = map(RLR,-462,462,-25,25);  // mm
      LT = map(LT,-462,462,-25,25);      // degrees 

      RFBFiltered = filter(RFB, RFBFiltered, 15);
      RLRFiltered = filter(RLR, RLRFiltered, 15);
      LTFiltered = filter(LT, LTFiltered, 15);

      longLeg1 = 340;
      shortLeg1 = 200;
      longLeg2 = 340;
      shortLeg2 = 200;

      footOffset = 0;
      timer1 = 80;   // FB gait timer  80
      //timer2 = 75;   // LR gait timer
      //timer3 = 75;   // LR gait timer

      if (RFBFiltered > -0.1 && RFBFiltered < 0.1 && RLRFiltered > -0.1 && RLRFiltered < 0.1  && LTFiltered > -0.1 && LTFiltered < 0.1 ) {    // controls are centred or near enough
      
      // position legs a default standing positionS
          legLength1 = longLeg1;
          legLength2 = longLeg2;
          fr_RFB = 0;
          fl_RFB = 0;
          bl_RFB = 0;
          br_RFB = 0;
          fr_RLR = footOffset;
          fl_RLR = -footOffset;
          bl_RLR = -footOffset;
          br_RLR = footOffset;
          fr_LT = 0;
          fl_LT = 0;
          bl_LT = 0;
          br_LT = 0;        
      }
      
      //walking
      else {
       
          if (stepFlag == 0 && currentMillis - previousStepMillis > timerScale) {
              legLength1  = shortLeg1;
              legLength2 = longLeg2; 
              fr_RFB = 0-RFBFiltered;
              fl_RFB = RFBFiltered;
              bl_RFB = 0-RFBFiltered;
              br_RFB = RFBFiltered;
              fr_RLR = (footOffset -RLRFiltered) + LT;
              fl_RLR = (-footOffset +RLRFiltered) - LT;
              bl_RLR = (-footOffset - RLRFiltered) - LT;
              br_RLR = (footOffset + RLRFiltered) + LT;
              //fr_RLR = LT;
              //fl_RLR = 0-LT;
              //bl_RLR = 0-LT;
              //br_RLR = LT;
              stepFlag = 1;              
              previousStepMillis = currentMillis;
          }

          else if (stepFlag == 1 && currentMillis - previousStepMillis > timerScale) {
              legLength1 = longLeg1;
              legLength2 = longLeg2;
              fr_RFB = 0-RFBFiltered;
              fl_RFB = RFBFiltered;
              bl_RFB = 0-RFBFiltered;
              br_RFB = RFBFiltered;
              fr_RLR = (footOffset -RLRFiltered) + LT;
              fl_RLR = (-footOffset +RLRFiltered) - LT;
              bl_RLR = (-footOffset -RLRFiltered) - LT;
              br_RLR = (footOffset +RLRFiltered) + LT;
              //fr_RLR = LT;
              //fl_RLR = 0-LT;
              //bl_RLR = 0-LT;
              //br_RLR = LT;                        

              stepFlag = 2;              
              previousStepMillis = currentMillis;
          }

          else if (stepFlag == 2 && currentMillis - previousStepMillis > timerScale) {
              legLength1 = longLeg1;
              legLength2 = shortLeg2;
              fr_RFB = RFBFiltered;
              fl_RFB = 0-RFBFiltered;
              bl_RFB = RFBFiltered;
              br_RFB = 0-RFBFiltered;
              fr_RLR = (footOffset +RLRFiltered) - LT;
              fl_RLR = (-footOffset -RLRFiltered) + LT;
              bl_RLR = (-footOffset +RLRFiltered) + LT;
              br_RLR = (footOffset -RLRFiltered) - LT;
              //fr_RLR = 0-LT;
              //fl_RLR = LT;
              //bl_RLR = LT;
              // br_RLR = 0-LT; 
              stepFlag = 3;              
              previousStepMillis = currentMillis;
          }

          else if (stepFlag == 3 && currentMillis - previousStepMillis > timerScale) {
              legLength1 = longLeg1;
              legLength2 = longLeg2;
              fr_RFB = RFBFiltered;
              fl_RFB = 0-RFBFiltered;
              bl_RFB = RFBFiltered;
              br_RFB = 0-RFBFiltered;
              fr_RLR = (footOffset +RLRFiltered) - LT;
              fl_RLR = (-footOffset -RLRFiltered) + LT;
              bl_RLR = (-footOffset +RLRFiltered) + LT;
              br_RLR = (footOffset -RLRFiltered) - LT;
              //fr_RLR = 0-LT;
              //fl_RLR = LT;
              //bl_RLR = LT;
              //br_RLR = 0-LT; 
              stepFlag = 0;              
              previousStepMillis = currentMillis;
          }

          float stepLength;
          float stepWidth;
          float stepAngle;
          float stepHyp;

          // timer calcs

          stepLength = abs(fr_RFB);
          stepWidth = abs(fr_RLR);

          if (stepLength == 0.0) {
            stepLength = 0.01;   // avoid divide by zero
          }

          stepAngle = atan(stepLength/stepWidth);  // radians       // work out actual distance of step
          stepHyp = abs(stepLength/sin(stepAngle));    // mm

          timerScale =  timer1 + (stepHyp/3.5);              
      }

      legTransX = IMUpitch * -2;
      legTransY = IMUroll * -2;

      legTransXFiltered = filter(legTransX,legTransXFiltered,50);
      legTransYFiltered = filter(legTransY,legTransYFiltered,50);

      legRoll = IMUroll * -0.5;
      legPitch = IMUpitch * 0.5;

      legRollFiltered = filter(legRoll,legRollFiltered,60);
      legPitchFiltered = filter(legPitch,legPitchFiltered,60);      

   
      kinematics (1, fr_RFB - legTransXFiltered, fr_RLR - legTransYFiltered, legLength1, legRollFiltered, legPitchFiltered, 0, 1, (timerScale*0.8));   // front right
      kinematics (2, fl_RFB - legTransXFiltered, fl_RLR - legTransYFiltered, legLength2, legRollFiltered, legPitchFiltered, 0, 1, (timerScale*0.8));   // front left
      kinematics (3, bl_RFB - legTransXFiltered, bl_RLR - legTransYFiltered, legLength1, legRollFiltered, legPitchFiltered, 0, 1, (timerScale*0.8));   // back left
      kinematics (4, br_RFB - legTransXFiltered, br_RLR - legTransYFiltered, legLength2, legRollFiltered, legPitchFiltered, 0, 1, (timerScale*0.8));   // back right             
   }



  }     // end of timed loop

}       // end  of main loop
