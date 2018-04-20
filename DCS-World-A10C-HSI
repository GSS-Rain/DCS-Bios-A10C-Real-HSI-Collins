/*
 Joe Sim Youtube Channel
        HSI Demo
https://youtu.be/Qio-dkG8YvM
        Real HSI
        Collins Model 331A-9G 
        P/N 792-6091-016     
    Motor Driver L298N Module
    Nema 08 Motor at 6VDC, but it gets warm at this voltage.
    Works at 4.5VDC but the motor moves a little slower.
 Problem arose when trying to use CRS Pointer variable.
 When the CRS Pointer is stationary, example set on 015,
 the DCS-Bios variable "hsiCrsBuffer" changes when the HSI compass dial "hsiHdgBuffer" starts rotating.
 This causes the real HSI CRS pointer to move away from what ever it was set to because it
 is tracking the CRS Pointer variable "hsiCrsBuffer".
 So as a solution, I now use Coarse Counter A and B which shows the numbers 000 to 359.
 When the HSI compass dial rotates, the Counter varibles "hsiCcABuffer" and "hsiCcBBuffer" does not change.
 The end result is the CRS Pointer spins with the compass dial and does not move away from
 the bearing that it was set to, for example 015.
 The challenge was Counte "A" range is 0 to 65535 which equals 0 to 35 (First two digits in 000).
 Counter "B" range is 0 to 65535 which is 0 to 9 (Last digit in 000).
 One minor thing to still work out is the two variables are not exactly aligned.
 For example "B" will cycle from 9.99 to 0 one tick before "A" will go from 00 to 01
 So every ten degrees, the needle may role back as "B" resets right before "A" increment and set it back on track.
*/
#define DCSBIOS_IRQ_SERIAL
#include <AccelStepper.h>
#include "DcsBios.h"
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C  lcd(0x3F,2,1,0,4,5,6,7);

// Motor pin definitions
#define mtrPin1  2     // IN1 on the L298N Motor driver #1
#define mtrPin2  3
#define mtrPin3  4
#define mtrPin4  5
#define mtrPin5  6     // IN1 on the L298N Motor driver #2
#define mtrPin6  7
#define mtrPin7  8
#define mtrPin8  9
#define mtrPin9  10    // IN1 on the L298N Motor driver #3
#define mtrPin10  11
#define mtrPin11  12
#define mtrPin12  13

// We need to monitor varible CRS Counter A to see the numbers 0 to 35.
DcsBios::IntegerBuffer hsiCcABuffer(0x1056, 0xffff, 0, NULL);

// number of steps for 1 revolution of real HSI Coarse Needle
#define CRS_STEPS_PER_REVOLUTION 600 // For CRS Counter A
#define C2_STEPS_PER_REVOLUTION 17  // Number of steps (16.6667) for Coarse Needle 10 degrees movement on real HSI (For CRS Counter B

// number of steps for 1 revolution of real HSI compass card
#define AZM_STEPS_PER_REVOLUTION 600 

// number of steps for 1 revolution of real HSI Bearing Pointer
#define BRG_STEPS_PER_REVOLUTION 600 

// Offset is obtained by trial and error.
// Has little to do with actual positioning of the home switch.
#define CRS_ZERO_OFFSET -20
#define AZM_ZERO_OFFSET 255
#define BRG_ZERO_OFFSET -17

// 4 = Full4wire setup.  PWM pin output is defined in code above
AccelStepper stepper_CRS(4, mtrPin1, mtrPin3, mtrPin2, mtrPin4);
AccelStepper stepper_AZM(4, mtrPin5, mtrPin7, mtrPin6, mtrPin8);
AccelStepper stepper_BRG(4, mtrPin9, mtrPin11, mtrPin10, mtrPin12);

int CRS_currentStepperPosition = 0; // current stepper pos (in steps, 0 to CRS_STEPS_PER_REVOLUTION -1)
int AZM_currentStepperPosition = 0;
int BRG_currentStepperPosition = 0;

signed long CRS_lastAccelStepperPosition;
signed long AZM_lastAccelStepperPosition;
signed long BRG_lastAccelStepperPosition;

 // Used to Home Stepper at startup
long CRS_initial_homing = -1;
long AZM_initial_homing = -1;
long BRG_initial_homing = -1;

int TENS_STEPS = 0;  // Used to add CRS A & B Counters for total steps to move motor

int Azumith_Flag = 24; // Used as the Power Off Flag
int GS_Flag = 26; // Collin HSI has Glide Slope Deviation
int INS_Flag  = 28;  // Used to indicate HARS as the navigational source 
int NAV_Flag = 30;

// Define pin used for proximity switch
#define CRS_home_switch A15
#define AZM_home_switch A8
#define BRG_home_switch A12

// Course Select Knob
DcsBios::RotaryEncoder hsiCrsKnob("HSI_CRS_KNOB", "-1000", "+1000", A1, A0);

// Nav Mode Select Panel HARS Button
// Show INS Flag if HARS is selected as Nav Source instead of EGI
void onNmspHarsBtnChange(unsigned int INS_Flag_newValue) {
  unsigned int INS_Flag_Value = (INS_Flag_newValue & 0xffff) >> 0;
  if (INS_Flag_Value == 0)
  {
      digitalWrite(GS_Flag, HIGH);   
  }
  else
  {
      digitalWrite(GS_Flag, LOW);       
  }
}
DcsBios::IntegerBuffer nmspHarsBtnBuffer(0x1110, 0x0100, 8, onNmspHarsBtnChange);

// The real HSI has the Glide Slope Deviation, not the ADI
void onAdiGswarnFlagChange(unsigned int GS_Flag_newValue) {
  unsigned int GS_Flag_Value = (GS_Flag_newValue & 0xffff) >> 0;
  
  if (GS_Flag_Value < 25000)
  {
      digitalWrite(GS_Flag, LOW);   
  }
  else
  {
      digitalWrite(GS_Flag, HIGH);       
  }
}
DcsBios::IntegerBuffer adiGswarnFlagBuffer(0x103e, 0xffff, 0, onAdiGswarnFlagChange);

// HSI Bearing Flag
void onHsiBearingFlagChange(unsigned int NAV_Flag_newValue) {
  unsigned int NAV_Flag_Value = (NAV_Flag_newValue & 0xffff) >> 0;
  if (NAV_Flag_Value < 25000)
  {
      digitalWrite(NAV_Flag, LOW);   
  }
  else
  {
      digitalWrite(NAV_Flag, HIGH);       
  }
}
DcsBios::IntegerBuffer hsiBearingFlagBuffer(0x104a, 0xffff, 0, onHsiBearingFlagChange);

// Use Azumith Flag as Power Off Flag
void onHsiPwroffFlagChange(unsigned int Azumith_Flag_newValue) {
  unsigned int Azumith_Flag_Value = (Azumith_Flag_newValue & 0xffff) >> 0;
  if (Azumith_Flag_Value < 25000)
  {
      digitalWrite(Azumith_Flag, LOW);   
  }
  else
  {
      digitalWrite(Azumith_Flag, HIGH);       
  }
}
DcsBios::IntegerBuffer hsiPwroffFlagBuffer(0x1046, 0xffff, 0, onHsiPwroffFlagChange);

void setup() {
  DcsBios::setup();
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH); // Turn backlight off by setting it to LOW instead of HIGH
  lcd.begin(16, 2); // Config LCD as a 16x2 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("H:");  // Azumith raw data, for trouble shooting
  lcd.setCursor(9, 0);
  lcd.print("R:");  // Coarse Pointer raw data
  lcd.setCursor(0, 1);
  lcd.print("C:"); // CRS Counter A raw data
  lcd.setCursor(9, 1);
  lcd.print("C:"); // CRS Counter B raw data
  pinMode (GS_Flag, OUTPUT);
  pinMode (INS_Flag, OUTPUT);
  pinMode (NAV_Flag, OUTPUT);    
  pinMode (Azumith_Flag, OUTPUT);    
  
  // Homing Code was based on Brainy Bits
  // https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/
    
  // COASRE HOMING
  pinMode(CRS_home_switch, INPUT_PULLUP);  // LOW when in zero position, HIGH otherwise

  // Set Max Speed and Acceleration for startup during homing
  stepper_CRS.setMaxSpeed(200);
  stepper_CRS.setSpeed(100);
  stepper_CRS.setAcceleration(25);

  // Make the Stepper move CCW until the switch is activated
  while (digitalRead(CRS_home_switch)) {
    stepper_CRS.moveTo(CRS_initial_homing);  // Set the position to move to
    CRS_initial_homing--;  // Decrease by 1 for next move if needed
    stepper_CRS.run();  // Start moving the stepper
    delay(5);
  }
  stepper_CRS.setCurrentPosition(0);  // Set the current position as zero for now
  stepper_CRS.setMaxSpeed(200);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper_CRS.setSpeed(100);
  stepper_CRS.setAcceleration(25);  // Set Acceleration of Stepper
  CRS_initial_homing = 1;

  while (!digitalRead(CRS_home_switch)) { // Make the Stepper move CW until the switch is deactivated
    stepper_CRS.moveTo(CRS_initial_homing);
    stepper_CRS.run();
    CRS_initial_homing++;
    delay(5);
  }
  
  // Zero offset code came from DCS Forums
  // Add zero offset
  stepper_CRS.runToNewPosition(stepper_CRS.currentPosition() + CRS_ZERO_OFFSET);

  // tell the AccelStepper library that we are at the real North Postion that matches the simulator
  stepper_CRS.setCurrentPosition(0);  // Set the current position as zero for now
  CRS_lastAccelStepperPosition = 0;
  stepper_CRS.setMaxSpeed(600);    // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper_CRS.setSpeed(400);
  stepper_CRS.setAcceleration(100);  // Set Acceleration of Stepper

// Homing Code was based on Brainy Bits
// https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/

// AZIMUTH HOMING
  pinMode(AZM_home_switch, INPUT_PULLUP);  // LOW when in zero position, HIGH otherwise

  stepper_AZM.setMaxSpeed(200);
  stepper_AZM.setSpeed(100);
  stepper_AZM.setAcceleration(25);

  while (digitalRead(AZM_home_switch)) {
    stepper_AZM.moveTo(AZM_initial_homing);
    AZM_initial_homing--;
    stepper_AZM.run();
    delay(5);
  }

  stepper_AZM.setCurrentPosition(0);
  stepper_AZM.setMaxSpeed(200);
  stepper_AZM.setSpeed(100);
  stepper_AZM.setAcceleration(25);
  AZM_initial_homing = 1;

  while (!digitalRead(AZM_home_switch)) {
    stepper_AZM.moveTo(AZM_initial_homing);
    stepper_AZM.run();
    AZM_initial_homing++;
    delay(5);
  }
  // Zero offset code came from DCS Forums
  // Add zero offset
  stepper_AZM.runToNewPosition(stepper_AZM.currentPosition() + AZM_ZERO_OFFSET);

  stepper_AZM.setCurrentPosition(0);
  AZM_lastAccelStepperPosition = 0;
  stepper_AZM.setMaxSpeed(600);
  stepper_AZM.setSpeed(400);
  stepper_AZM.setAcceleration(100);


// Homing Code was based on Brainy Bits
// https://www.brainy-bits.com/setting-stepper-motors-home-position-using-accelstepper/

// Bearing Pointer HOMING
  pinMode(BRG_home_switch, INPUT_PULLUP);  // LOW when in zero position, HIGH otherwise

  stepper_BRG.setMaxSpeed(400);
  stepper_BRG.setSpeed(200);
  stepper_BRG.setAcceleration(65);

  while (digitalRead(BRG_home_switch)) {
    stepper_BRG.moveTo(BRG_initial_homing);
    BRG_initial_homing--;
    stepper_BRG.run();
    delay(5);
  }
  stepper_BRG.setCurrentPosition(0);
  stepper_BRG.setMaxSpeed(400);
  stepper_BRG.setSpeed(200);
  stepper_BRG.setAcceleration(65);
  BRG_initial_homing = 1;

  while (!digitalRead(BRG_home_switch)) {
    stepper_BRG.moveTo(BRG_initial_homing);
    stepper_BRG.run();
    BRG_initial_homing++;
    delay(5);
  }
  // Zero offset code came from DCS Forums
  // Add zero offset
  stepper_BRG.runToNewPosition(stepper_BRG.currentPosition() + BRG_ZERO_OFFSET);

  stepper_BRG.setCurrentPosition(0);
  BRG_lastAccelStepperPosition = 0;
  stepper_BRG.setMaxSpeed(600);
  stepper_BRG.setSpeed(400);
  stepper_BRG.setAcceleration(100);
 }

void loop() {
  DcsBios::loop();

// move stepper motors
   stepper_CRS.run();
   stepper_AZM.run();
   stepper_BRG.run();   
}

// HSI Course Counter B (0 to 9)
void onHsiCcBChange(unsigned int C2_newValue) {

// Coarse Counter B varible always imdediately updates when CRS knob is rotated
// Coarse Counter A only updates at about every 10 degree knob rotation.
// So locate Counter A logic inside call routine for counter B

  unsigned int BUFFER_DATA = (hsiCcABuffer.getData() & 0xffff) >> 0;
  
  lcd.setCursor(2, 1); // this is for trouble shooting
  lcd.print("      "); // Clears screen of stale data
  lcd.setCursor(2, 1);
  lcd.print(BUFFER_DATA); // Displays Course Counter A raw data

// This mapping detemine how many stepper counts to add to Coarse B Counter steps.
  TENS_STEPS = map(BUFFER_DATA, 0, 65535, CRS_STEPS_PER_REVOLUTION - 1, 0);

// Add CRS Counter A steps + CRS counter B steps = (Tens slot 00 plus ones slot 0)   
  unsigned int C2_Value = (C2_newValue & 0xffff) >> 0;
  int CRS_targetPosition = (map(C2_Value, 0, 65535, C2_STEPS_PER_REVOLUTION - 1, 0) + TENS_STEPS);
  
  lcd.setCursor(11, 1); // this is for trouble shooting
  lcd.print("      "); // Clears screen of stale data
  lcd.setCursor(11, 1);
  lcd.print(C2_Value); // Displays Course Counter B raw data

  // adjust CRS_currentStepperPosition to include the distance our stepper motor
  // was moved since we last updated it
  // Copied this section of code from DCS forums
  
  int CRS_movementSinceLastUpdate = stepper_CRS.currentPosition() - CRS_lastAccelStepperPosition;
  CRS_currentStepperPosition += CRS_movementSinceLastUpdate;
  CRS_lastAccelStepperPosition = stepper_CRS.currentPosition();

  if (CRS_currentStepperPosition < 0) CRS_currentStepperPosition += CRS_STEPS_PER_REVOLUTION;
  if (CRS_currentStepperPosition > CRS_STEPS_PER_REVOLUTION) CRS_currentStepperPosition -= CRS_STEPS_PER_REVOLUTION;

  int CRS_delta = CRS_targetPosition - CRS_currentStepperPosition;

  // if we would move more than 180 degree counterclockwise, move clockwise instead
  if (CRS_delta < -(CRS_STEPS_PER_REVOLUTION / 2)) CRS_delta += CRS_STEPS_PER_REVOLUTION;

  // if we would move more than 180 degree clockwise, move counterclockwise instead
  if (CRS_delta > (CRS_STEPS_PER_REVOLUTION / 2)) CRS_delta -= CRS_STEPS_PER_REVOLUTION;

  // tell AccelStepper to move relative to the current position
     stepper_CRS.move(CRS_delta);  
}
DcsBios::IntegerBuffer hsiCcBBuffer(0x1058, 0xffff, 0, onHsiCcBChange);

// HSI Coarse Needle
void onHsiCrsChange(unsigned int new_CRS_Value) {
  unsigned int Coarse_Value = (new_CRS_Value & 0xffff) >> 0;

  lcd.setCursor(11, 0);  // this is for trouble shooting
  lcd.print("      "); // Clears screen of stale data
  lcd.setCursor(11, 0);
  lcd.print(Coarse_Value); // Displays CRS Pointer raw data  

}
DcsBios::IntegerBuffer hsiCrsBuffer(0x1054, 0xffff, 0, onHsiCrsChange);

// HSI Heading
void onHsiHdgChange(unsigned int new_AZM_Value) {
  unsigned int Heading_Value = (new_AZM_Value & 0xffff) >> 0;
  int AZM_targetPosition = map(Heading_Value, 0, 65535, AZM_STEPS_PER_REVOLUTION - 1, 0);


  lcd.setCursor(2, 0);  // this is for trouble shooting
  lcd.print("      "); // Clears screen of stale data
  lcd.setCursor(2, 0);
  lcd.print(Heading_Value); // Displays Heading raw data

  // adjust AZM_currentStepperPosition to include the distance our stepper motor
  // was moved since we last updated it
  // Copied this section of code from DCS forums
  
  int AZM_movementSinceLastUpdate = stepper_AZM.currentPosition() - AZM_lastAccelStepperPosition;
  AZM_currentStepperPosition += AZM_movementSinceLastUpdate;
  AZM_lastAccelStepperPosition = stepper_AZM.currentPosition();

  if (AZM_currentStepperPosition < 0) AZM_currentStepperPosition += AZM_STEPS_PER_REVOLUTION;
  if (AZM_currentStepperPosition > AZM_STEPS_PER_REVOLUTION) AZM_currentStepperPosition -= AZM_STEPS_PER_REVOLUTION;

  int AZM_delta = AZM_targetPosition - AZM_currentStepperPosition;

  // if we would move more than 180 degree counterclockwise, move clockwise instead
  if (AZM_delta < -(AZM_STEPS_PER_REVOLUTION / 2)) AZM_delta += AZM_STEPS_PER_REVOLUTION;

  // if we would move more than 180 degree clockwise, move counterclockwise instead
  if (AZM_delta > (AZM_STEPS_PER_REVOLUTION / 2)) AZM_delta -= AZM_STEPS_PER_REVOLUTION;

  // tell AccelStepper to move relative to the current position
  stepper_AZM.move(AZM_delta);
}
DcsBios::IntegerBuffer hsiHdgBuffer(0x104c, 0xffff, 0, onHsiHdgChange);

// HSI Bearing Pointer 1
void onHsiBearing1Change(unsigned int new_BRG_Value) {
  unsigned int Bearing_Value = (new_BRG_Value & 0xffff) >> 0;
  int BRG_targetPosition = map(Bearing_Value, 0, 65535, BRG_STEPS_PER_REVOLUTION - 1, 0);

  // Copied this section of code from DCS forums
  
  int BRG_movementSinceLastUpdate = stepper_BRG.currentPosition() - BRG_lastAccelStepperPosition;
  BRG_currentStepperPosition += BRG_movementSinceLastUpdate;
  BRG_lastAccelStepperPosition = stepper_BRG.currentPosition();

  if (BRG_currentStepperPosition < 0) BRG_currentStepperPosition += BRG_STEPS_PER_REVOLUTION;
  if (BRG_currentStepperPosition > BRG_STEPS_PER_REVOLUTION) BRG_currentStepperPosition -= BRG_STEPS_PER_REVOLUTION;

  int BRG_delta = BRG_targetPosition - BRG_currentStepperPosition;

  // if we would move more than 180 degree counterclockwise, move clockwise instead
  if (BRG_delta < -(BRG_STEPS_PER_REVOLUTION / 2)) BRG_delta += BRG_STEPS_PER_REVOLUTION;

  // if we would move more than 180 degree clockwise, move counterclockwise instead
  if (BRG_delta > (BRG_STEPS_PER_REVOLUTION / 2)) BRG_delta -= BRG_STEPS_PER_REVOLUTION;

  // tell AccelStepper to move relative to the current position
  stepper_BRG.move(BRG_delta);
}
DcsBios::IntegerBuffer hsiBearing1Buffer(0x104e, 0xffff, 0, onHsiBearing1Change);
