/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\13764                                            */
/*    Created:      Sat Mar 06 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LF                   motor         1               
// LR                   motor         3               
// RF                   motor         2               
// RR                   motor         4               
// Lcl                  motor         7               
// Rcl                  motor         6               
// trans                motor         20              
// shot                 motor         5               
// Controller1          controller                    
// Inertial             inertial      13              
// OpticalU             optical       12              
// OpticalD             optical       11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
motor_group L(LF, LR);
motor_group R(RF, RR);
motor_group cl(Lcl, Rcl);
motor_group shoot(trans, shot);
//smartdrive Drivetrain = smartdrive(L, R, Inertial, 319.19, 320, 40, mm, 1);//with gyro
//drivetrain Drivetrain = drivetrain(L, R, 319.19, 320, 40, mm, 1);//without ~
competition Competition;
double absHeading = 0;
bool isDriver = false;
#define NOTHING 0
#define SORT1 1
#define SPINCL 2
#define STOPTRANS 3
#define BACKOUT 4
#define REVERSESHOT 5
#define SPINSHORT 6
vex::color red[] = {color::red, color::orange, color::yellow};
vex::color blue[] = {color::blue, color::green, color::cyan};
int multitasker_flag = NOTHING;

double reduceAngle( double angleDeg ) { //Takes an input angle in degrees and converts it to a positive angle value below 360.
  while(!(angleDeg >= 0 && angleDeg < 360)) {
    if( angleDeg < 0 ) { angleDeg += 360; }
    if(angleDeg >= 360) { angleDeg -= 360; }
  }
  return(angleDeg);
}
void updatePosition() { //Approximates the motion of the robot as an arc, and updates its position accordingly
  absHeading = reduceAngle((Inertial.heading(degrees)*357/360));
}
int positionTrack() { //Background thread used to position track full time.
  int i = 0;
  while(1){
    //updateEncoders();
    updatePosition();
    if((i % 20 == 0) && (!isDriver)) {
      //Controller1.Screen.newLine();
      //Controller1.Screen.print(absHeading);
    }
    i++;
  }
}
void intake(int timeOut = 6000){
  Brain.Timer.reset();
  cl.spin(forward);
  trans.spin(forward);
  shot.spin(forward);
  waitUntil(OpticalU.isNearObject() || Brain.Timer.time() > timeOut);
  Controller1.Screen.print("aaa");

  cl.stop();
  trans.stop();
  shot.stop();
}
bool checkColor(bool red, bool down = true){
  int colorID = 0;
  if (down){
    if (OpticalD.isNearObject()){
      if(OpticalD.color() == vex::color::blue || OpticalD.color() == vex::color::cyan || OpticalD.color() == vex::color::green || OpticalD.color() == vex::color::purple){
        colorID = 2;
      }else if(OpticalD.color() == color::red){
        colorID = 1;
      }
    }
  }else{
    if (OpticalU.isNearObject()){
      if(OpticalU.color() == vex::color::blue || OpticalU.color() == vex::color::cyan || OpticalU.color() == vex::color::green || OpticalU.color() == vex::color::purple){
        colorID = 2;
      }else if(OpticalU.color() == color::red || OpticalU.color() == color::orange || OpticalU.color() == color::yellow){
        colorID = 1;
      }
    }
  }
  if (red){
    if (colorID == 1){
      return true;
    }
  }else{
    if (colorID == 2){
      return true;
    }
  }
  return false;
}
void drive(double dist, bool wait = true, int vel = 50, int timeOut = 90000000){
  LF.setRotation(0, degrees);
  LR.setRotation(0, degrees);
  RF.setRotation(0, degrees);
  RR.setRotation(0, degrees);
  double deg = dist * 1.128; //2pir/dist=360/deg
  LF.startSpinFor(forward, deg, degrees, vel, velocityUnits::pct);
  LR.startSpinFor(forward, deg, degrees, vel, velocityUnits::pct);
  RF.startSpinFor(forward, deg, degrees, vel, velocityUnits::pct);
  RR.spinFor(forward, deg, degrees, vel, velocityUnits::pct, wait);/*
  debug();*/
}
int iTurnPID(double ang, int timeOut = 1000, bool left = true, bool right = true, double kp = 0.8, double kd = 1.0, double b = 0.0, double maxPower = 50.0){//period=0.533, ku=4
  //left right -3
  int i = 0;  
  //Inertial.setRotation(initAng, degrees);
  
  //double error = ang - Inertial.rotation(degrees);
  double error;
  if(ang - absHeading <= 180 && (ang - absHeading >= -180)){
    error = ang - absHeading;
  }else if(ang - absHeading > 0){
    error = -(360 - (ang - absHeading));
  }else{
    error = (ang - absHeading + 360);
  }

  double prevError = 0;
  double derivative;
  double mmp;
  Brain.Timer.reset();
  while(fabs(error) > 1 && Brain.Timer.time() < timeOut){
    //error = ang - Inertial.rotation(degrees);
    //error = ang - absHeading;
    if(ang - absHeading <= 180 && (ang - absHeading >= -180)){
      error = ang - absHeading;
    }else if(ang - absHeading > 0){
      error = -(360 - (ang - absHeading));
    }else{
      error = (ang- absHeading + 360);
    }
    derivative = error - prevError;
    mmp = kp * error + kd * derivative;
    /*if(mmp > 0){mmp += 5;}
    else if(mmp < 0){mmp -= 5;}
    if(fabs(mmp) > fabs(maxPower)){
      if(mmp >= 0){mmp = maxPower + 1;}
      else{mmp = -maxPower - 1;}
    }*/
    i++;
    if(i % 10 == 0){
      //Controller1.Screen.setCursor(1, 1);
      //Controller1.Screen.print(Inertial.rotation(degrees));
      //Controller1.Screen.setCursor(2, 1);
      //Controller1.Screen.print(error);
    }
    if (left){
    LF.spin(forward, mmp, percent);
    LR.spin(forward, mmp, percent);
    }else{
      LF.setStopping(hold);
      LR.setStopping(hold);
      LF.stop();
      LR.stop();
    }
    if (right){
    RF.spin(reverse, mmp, percent);
    RR.spin(reverse, mmp, percent);    
    }else{
      RF.setStopping(hold);
      RR.setStopping(hold);
      RF.stop();
      RR.stop();
    }
    prevError = error;
    vex::task::sleep(20);
  }
  /*Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(Inertial.rotation(degrees));
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print(error);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("%f", mmp);*/
  LF.stop();
  LR.stop();
  RF.stop();
  RR.stop();
  return 1;
}
int drivePID(int dist, double kp = 0.2362, double kd = 0){
  int i = 0;
  int leftError = dist / (3.14 * 101.6) * 360; //*1.128
  int leftPrevError = leftError;
  int leftDerivative = 0;

  LF.setPosition(0, degrees);
  LR.setPosition(0, degrees);
  RF.setPosition(0, degrees);
  RR.setPosition(0, degrees);
  //fabs(LR.velocity(percent)) + abs(leftError) > 15
  while(true){
    //int averagePosition = (LF.position(degrees) + LR.position(degrees) + RF.position(degrees) + RR.position(degrees))/4;
    //int averagePosition = (LE.position(degrees) + RE.position(degrees)) / 2;
    //leftError = dist / (3.14 * 101.6) * 360 - averagePosition;
    leftError = dist / (3.14 * 101.6) * 360 - LR.rotation(degrees);
    leftDerivative = leftError - leftPrevError;
    double lmmp = kp * leftError + kd * leftDerivative;
    /*if(fabs(lmmp) > fabs(maxPower)){
      if(lmmp >= 0){lmmp = maxPower;}
      else{lmmp = -maxPower;}
    }*/
      i++;
    /*if(i % 10 == 0){
      debug();
    }*/
    LF.spin(forward, lmmp, percent);
    LR.spin(forward, lmmp, percent);
    RF.spin(forward, lmmp, percent);
    RR.spin(forward, lmmp, percent);
    leftPrevError = leftError;
    vex::task::sleep(20);
  }
  LF.stop();
  LR.stop();
  RF.stop();
  RR.stop();
  return 1;
}
void descoreN(int n, int timeOut){
  Brain.Timer.reset();
  cl.spin(forward);
  trans.spin(forward);
  shot.spin(reverse);
  for(int i = 0; i < n; i ++){
    waitUntil(checkColor(false, true) || Brain.Timer.value() > timeOut);
  }
  cl.stop();
  trans.spinFor(3, turns, false);
  shot.spinFor(3, turns);
}
void grabN(int n, int timeOut){
  Brain.Timer.reset();
  cl.spin(forward);
  trans.spin(forward);
  shot.spin(reverse);
  for(int i = 0; i < n; i ++){
    waitUntil(checkColor(false, true) || Brain.Timer.value() > timeOut);
  }
}
void spinN(int n, int timeOut){
  Brain.Timer.reset();
  cl.spin(reverse);
  trans.spin(reverse);
  shot.spin(reverse);
  for(int i = 0; i < n; i ++){
    waitUntil((!OpticalD.isNearObject()) || Brain.Timer.value() > timeOut);
        waitUntil((!OpticalD.isNearObject()) || Brain.Timer.value() > timeOut);

  }
}
void descoreBack(){
  Brain.Timer.reset();
  waitUntil(checkColor(false, true) || Brain.Timer.time() > 3000);
  shot.spinFor(reverse, 10, turns);
}
void reverseShot(){
  waitUntil(shot.isDone());
  shot.spinFor(reverse, 5, turns);
}
void sort1(bool red, int timeOut = 5000){
  //trans.spinFor(forward, 0.5, turns);
  trans.spin(forward);
  shot.spin(forward);
  cl.spin(forward);
  //waitUntil(OpticalD.isNearObject());
  waitUntil(checkColor(red, true) || Brain.Timer.time() > timeOut);
  Controller1.Screen.print("aa");
  cl.stop();
  //trans.stop();
  shot.stop();
  trans.spinFor(forward, 7, turns);//6
  //shot.spinFor(reverse, 3, turns);
}
void sort2(bool red, int timeOut = 5000){
  //trans.spinFor(forward, 0.5, turns);
  trans.spin(forward);
  shot.spin(forward);
  cl.spin(forward);
  //waitUntil(OpticalD.isNearObject());
  waitUntil(checkColor(red, true) || Brain.Timer.time() > timeOut);
  Controller1.Screen.print("aa");
  cl.stop();
  //trans.stop();
  shot.stop();
  trans.spinFor(forward, 2, turns);
  shot.spinFor(reverse, 4, turns, false);

  trans.spinFor(forward, 6, turns);
}
void goalAlign( float timeMsec, float v) { //Drives the robot into a goal for a specified time at a specified voltage.
  Brain.resetTimer();
  while (Brain.timer(timeUnits::msec) < timeMsec) {
    LF.spin(directionType::fwd, v, voltageUnits::volt);
    LR.spin(directionType::fwd, v, voltageUnits::volt);
    RF.spin(directionType::fwd, v, voltageUnits::volt);
    RR.spin(directionType::fwd, v, voltageUnits::volt);
  }
  LF.spin(directionType::fwd, 0, voltageUnits::volt);
  LR.spin(directionType::fwd, 0, voltageUnits::volt);
  RF.spin(directionType::fwd, 0, voltageUnits::volt);
  RR.spin(directionType::fwd, 0, voltageUnits::volt);
}
void spinCl(){
  cl.spinFor(forward, 5, turns, false);
  trans.spinFor(forward, 5, turns);
}
int multitasker_callback() {
  /*
   * This is the callback for the Multitasker thread.
   * It waits until the variable multitasker_flag is changed
   * to a value, and then calls the function associated with that
   * value and sets multitasker_flag back to NOTHING.
   */
  while (true) {
    if (multitasker_flag==SORT1) {
      sort1(false);
      multitasker_flag = NOTHING;
    }else if (multitasker_flag == SPINCL){
      intake(3000);
      multitasker_flag = NOTHING;
    }else if (multitasker_flag == STOPTRANS){
      waitUntil(checkColor(true, false));
      cl.stop();
      trans.stop();
      multitasker_flag = NOTHING;
    }
    else if (multitasker_flag == BACKOUT){
      descoreBack();
      multitasker_flag = NOTHING;
    }else if (multitasker_flag == REVERSESHOT){
      reverseShot();
      multitasker_flag = NOTHING;
    }
    else if (multitasker_flag == SPINSHORT){
      intake(2000);
      multitasker_flag = NOTHING;
    }
    task::sleep(25);
  }
  return 0;
}
void simultaneously(int flag) {
  /*
   * Assigns the function given to the Multitasker thread, or if the thread is busy,
   * to the Polytasker thread, or if the thread is busy, the Supertasker thread.
   */
  if (multitasker_flag==NOTHING) {multitasker_flag = flag;}
}
void test(){
  task positionTracking = vex::task(positionTrack);
  task multitasker = task(multitasker_callback);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
    //iTurnPID(0, 2000);
    simultaneously(SPINSHORT);

    drive(1350, true, 80);

    iTurnPID(90, 1200, false, true, 1.6);

    drive(200, true, 40);
    //simultaneously(BACKOUT);
    waitUntil(!(trans.isSpinning() || cl.isSpinning() || shot.isSpinning()));
    wait(2, seconds);
            Controller1.Screen.print("B");

    cl.spinFor(forward, 6, turns, false);
    shot.spinFor(reverse, 8, turns, false);
    trans.spinFor(forward, 7, turns);
    Controller1.Screen.print("C");

  // temp center
    drive(-400);
}
void newRow(){
  bool red = true;
  LF.setTimeout(2, seconds);
  LR.setTimeout(2, seconds);
  RF.setTimeout(2, seconds);
  RR.setTimeout(2, seconds);

  task positionTracking = vex::task(positionTrack);
  task multitasker = task(multitasker_callback);
  OpticalU.setLightPower(100, percent);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
  //Inertial.setHeading(180 / 357 * 360, degrees);
  //corner 1
    trans.spinFor(forward, 0.5, turns, false);
  //side 1
    drive(900, false, 70);
    //intake(3000);
    iTurnPID(90);
    drive(450, true, 60);
    iTurnPID(180, 1200);
    drive(300, true, 40);
    //shot.spinFor(forward, 0.5, turns, false);
    //simultaneously(BACKOUT);
    sort2(!red);

  //corner 2
    //cl.spinFor(forward, 2, turns, false);
    //trans.spinFor(forward, 5, turns, false);

    drive(-150, true, 70);
    iTurnPID(95, 1200, true, false, 1.6);
    
    drive(1300, true, 70);
    iTurnPID(135, 1200);

    drive(300, false, 40);

    intake(3000);

    shot.spinFor(forward, 3, turns, false);
    cl.spinFor(forward, 6, turns, false);
    trans.spinFor(forward, 6, turns);
}
void homeRow(){
  //auton code here
  //
  bool red = false;//red: false, blue: true
  //00
  cl.setVelocity(100, percent);
  trans.setVelocity(100, percent);
  shot.setVelocity(100, percent);
  task multitasker = task(multitasker_callback);
  OpticalU.setLightPower(100, percent);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
  //Inertial.setHeading(301.1/357*360, degrees);
  trans.spinFor(forward, 0.5, turns, false);

  drive(820, true, 70);//850
  iTurnPID(225, 1200);
  drive(600, false, 100);//600
  //goalAlign(500, 8);
  cl.spin(forward);
  trans.spin(forward);
  shot.spin(forward);
  waitUntil(checkColor(red, true));
  cl.stop();
  waitUntil(checkColor(red, false));
  cl.stop();
  shot.spinFor(forward, 2, turns, false);
  cl.spinFor(reverse, 5, turns, false);

  trans.spinFor(forward, 4, turns);
  //sort1(red);
  wait(0.5, seconds);//
  drive(-700, 80);//700
  iTurnPID(270, 1200);
  drive(-1870, true, 100);//1900
  iTurnPID(135, 2000, true, true, 1.2);

  drive(720, false, 100);
  //goalAlign(500, 8);
  cl.spin(forward);
  trans.spin(forward);
  shot.spin(forward);
  waitUntil(checkColor(red, true));
  cl.spinFor(reverse, 5, turns, false);
  waitUntil(checkColor(red, false));
  shot.spinFor(forward, 2, turns, false);
  trans.spinFor(forward, 4.5, turns);
  //sort1(red);
  drive(-700, true, 100);
}
void singleUniformL(void){
  //
  bool red = false;//red: true, blue: false
  //00
  task positionTracking = vex::task(positionTrack);
  task multitasker = task(multitasker_callback);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
  //Inertial.setHeading(180 / 357 * 360, degrees);
  trans.spinFor(forward, 0.5, turns);
  drive(-410, true, 70);//420
  iTurnPID(45);
  drive(630, false, 80);//650
  sort1(!red);
  cl.spinFor(reverse, 7, turns, false);
  trans.spinFor(reverse, 7, turns,false);

  drive(-630, true, 80);//650

  iTurnPID(180);/*
  drive(-1000, true, 70);//1050
  iTurnPID(0, 1700, true, true, 1.2);
  drive(350, false, 60);  
  sort2(!red, 5000);
  //shot.spinFor(reverse, 3, turns);

  trans.spinFor(forward, 5, turns, false);  
  cl.spinFor(reverse, 5, turns, false);
  drive(-500, true, 100);
  */
}
void singleUniformR(void){
bool red = false;//red: true, blue: false
  //00
  task positionTracking = vex::task(positionTrack);
  task multitasker = task(multitasker_callback);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
  //Inertial.setHeading(180 / 357 * 360, degrees);
  trans.spinFor(forward, 0.5, turns);
  drive(-500, true, 70);//420
  iTurnPID(315);
  drive(640, false, 80);//650
  sort1(!red);
  cl.spinFor(reverse, 7, turns, false);
  trans.spinFor(reverse, 7, turns,false);

  drive(-630, true, 80);//650

  iTurnPID(270);
  drive(-850);//1050
  iTurnPID(0, 1700, true, true, 1.2);
  drive(450, false, 100);  
  sort2(!red);
  //shot.spinFor(reverse, 3, turns);

  trans.spinFor(forward, 5, turns, false);  
  cl.spinFor(reverse, 5, turns, false);
  drive(-500, true, 100);
}
//void doubleL();
//void doubleR();
void skills(){
  bool red = true;
  LF.setTimeout(2, seconds);
  LR.setTimeout(2, seconds);
  RF.setTimeout(2, seconds);
  RR.setTimeout(2, seconds);

  task positionTracking = vex::task(positionTrack);
  task multitasker = task(multitasker_callback);
  OpticalU.setLightPower(100, percent);
  OpticalD.setLightPower(100, percent);
  waitUntil(!Inertial.isCalibrating());
  //Inertial.setHeading(180 / 357 * 360, degrees);
  //corner 1
    trans.spinFor(forward, 0.5, turns, false);
  //side 1
    drive(900, false, 70);
    intake(3000);
    iTurnPID(90);
    drive(450, true, 60);
    iTurnPID(180, 1200);
    drive(300, true, 40);
    //shot.spinFor(forward, 0.5, turns, false);
    //simultaneously(BACKOUT);
    cl.spinFor(forward, 6, turns, false);
    shot.spinFor(reverse, 8, turns, false);

    trans.spinFor(forward, 7, turns);

  //corner 2
    //cl.spinFor(forward, 2, turns, false);
    //trans.spinFor(forward, 5, turns, false);

    drive(-150, true, 70);
    iTurnPID(95, 1200, true, false, 1.6);
    
    drive(1370, false, 70);
    intake(3000);
    iTurnPID(135, 1200);
    drive(300, true, 40);

    shot.spinFor(forward, 3, turns, false);
    cl.spinFor(forward, 6, turns, false);
    trans.spinFor(forward, 6, turns);

  //side 2
    cl.spinFor(reverse, 7, turns, false);
    trans.spinFor(reverse, 7, turns, false);

    drive(-550, true, 70);
    wait(1, seconds);
    iTurnPID(0, 3000, true, true, 0.6);

    drive(1150, false, 80);
    intake(3000);
    waitUntil(RR.isDone());

    wait(0.5, seconds);
    iTurnPID(90, 1200, false, true, 1.6);

    drive(300, true, 40);
    //simultaneously(BACKOUT);
    //waitUntil(!(trans.isSpinning() || cl.isSpinning() || shot.isSpinning()));
    //wait(2, seconds);
    cl.spinFor(forward, 6, turns, false);
    shot.spinFor(reverse, 8, turns, false);
    trans.spinFor(forward, 7, turns);

  //corner 3
    //cl.spinFor(reverse, 7, turns, false);
     cl.spinFor(reverse, 7, turns, false);
    trans.spinFor(reverse, 7, turns, false);
    drive(-600);
    iTurnPID(0, 2000);
    simultaneously(SPINCL);
    drive(1300, true, 70);
    wait(0.5, seconds);
    drive(-550, true, 70);
    iTurnPID(45, 1200);
    drive(800, true, 40);
    shot.spinFor(forward, 2, turns, false);
    cl.spinFor(forward, 5, turns, false);
    trans.spinFor(forward, 6, turns);

//temp center
    drive(-950);
    iTurnPID(270, 3500, true, false, 1.6);
    drive(800, false);
    intake();
    waitUntil(RR.isDone());
    wait(1, seconds);
    drive(-400);
    iTurnPID(180);
    LF.setTimeout(3, seconds);
    LR.setTimeout(3, seconds);
    RF.setTimeout(3, seconds);
    RR.setTimeout(3, seconds);
    drive(400);
    wait(0.5, seconds);
    drive(-100);
    drive(300);
    wait(0.5, seconds);
    drive(-100);
    trans.spin(forward);
    shot.spin(forward);
    wait(5, seconds);
    drive(-200);



  //temp
  drive(-500, true, 70);
    iTurnPID(270, 1200, true, false, 1.6);
    
    drive(1400, false, 70);
    intake(3000);
    iTurnPID(315, 1200);
    drive(300, true, 40);

    shot.spinFor(forward, 3, turns, false);
    cl.spinFor(forward, 6, turns, false);
    trans.spinFor(forward, 6, turns);

  //side 2
    cl.spinFor(reverse, 7, turns, false);
    trans.spinFor(reverse, 7, turns, false);

    drive(-550, true, 70);
    wait(1, seconds);
    iTurnPID(180, 3000, true, true, 0.6);

    drive(1150, false, 80);
    intake(3000);
    waitUntil(RR.isDone());

    wait(0.5, seconds);
    iTurnPID(270, 1200, false, true, 1.6);

    drive(300, true, 40);
    //simultaneously(BACKOUT);
    //waitUntil(!(trans.isSpinning() || cl.isSpinning() || shot.isSpinning()));
    //wait(2, seconds);
    cl.spinFor(forward, 6, turns, false);
    shot.spinFor(reverse, 8, turns, false);
    trans.spinFor(forward, 7, turns);
  //side 3
   /* cl.spinFor(reverse, 7, turns, false);
    drive(-800, true, 70);
    iTurnPID(270, 1200);
    drive(1120, false, 70);
    spinCl();
    waitUntil(RR.isDone());
    iTurnPID(180, 1200);
    drive(600, false, 90);
    shot.spinFor(forward, 2, turns, false);
    cl.spinFor(forward, 5, turns, false);
    trans.spinFor(forward, 4, turns);
*/
  //corner 4
  /*
    cl.spinFor(reverse, 7, turns, false);
    drive(-200, true, 70);
    iTurnPID(270, 1200);
    drive(1090, false, 70);
    spinCl();
    waitUntil(RR.isDone());
    iTurnPID(315, 1200);
    drive(200, false, 70);
    shot.spinFor(forward, 2, turns, false);
    cl.spinFor(forward, 5, turns, false);
    trans.spinFor(forward, 4, turns);

  //side 4
    cl.spinFor(reverse, 7, turns, false);
    drive(-250, true, 70);
    iTurnPID(180, 1200);
    drive(1400, false, 80);
    spinCl();
    waitUntil(RR.isDone());
    iTurnPID(270, 1200);
    drive(200, false, 90);
  
    shot.spinFor(forward, 2, turns, false);
    cl.spinFor(forward, 5, turns, false);
    trans.spinFor(forward, 4, turns);

  //center
  cl.spinFor(reverse, 2, turns, false);
  iTurnPID(90, 1200);
  drive(800, false, 80);
  spinCl();
  L.spinFor(forward, 2, turns, false);
  trans.spin(forward);
  shot.spin(forward);*/
}
void control(void){
  //driver control here
  int i = 0;
  isDriver = true;
  LF.setStopping(brake);
  LR.setStopping(brake);
  RF.setStopping(brake);
  RR.setStopping(brake);

  while (true) {
    i++;
    // drivetrain
    // tank drive
    if (fabs(static_cast<float>(Controller1.Axis3.position())) > 5.0) {
      L.setVelocity((Controller1.Axis3.position())*0.95, percent);
    }else {
      L.setVelocity(0.0, percent);
    }
    if (fabs(static_cast<float>(Controller1.Axis2.position())) > 5.0) {
      R.setVelocity((Controller1.Axis2.position())*0.95, percent);
    }
    else {
      R.setVelocity(0.0, percent);
    }
    L.spin(forward);
    R.spin(forward);/*
    //Left archade drive
    if(fabs(static_cast<float>(Controller1.Axis3.position()))+  fabs(static_cast<float>(Controller1.Axis4.position())) > 5.0){
      L.setVelocity(Controller1.Axis3.position() + Controller1.Axis4.position(), percent);
      R.setVelocity(Controller1.Axis3.position() - Controller1.Axis4.position(), percent);
    }else{
      L.setVelocity(0, percent);
      R.setVelocity(0, percent);
    }
    L.spin(forward);
    R.spin(forward);*/
    //intake
  
    if(Controller1.ButtonL2.pressing()){
      cl.spin(forward);
    }else if(Controller1.ButtonL1.pressing()){
      cl.spin(reverse);
    }else{
      cl.stop();
    }
    if(Controller1.ButtonR2.pressing()){
      trans.spin(forward);
      shot.spin(forward);
    }
    else if(Controller1.ButtonR1.pressing()){
      trans.spin(forward);
      shot.spin(reverse);
    }
    else if(Controller1.ButtonB.pressing()){
      shot.spin(reverse);
      trans.spin(reverse);
    }else{
      
      trans.stop();
      shot.stop();
    }
   
  wait(20, msec);
  }
}
void preAuton(void){
  //init code here
  vexcodeInit();
  Controller1.Screen.print("HR");
  Inertial.calibrate();
  Brain.Timer.reset();
  cl.setVelocity(100, percent);
  trans.setVelocity(100, percent);
  shot.setVelocity(100, percent);
  task positionTracking = vex::task(positionTrack);
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(singleUniformL);
  Competition.drivercontrol(control);
  // Run the pre-autonomous function.
  preAuton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  return 0;
}
