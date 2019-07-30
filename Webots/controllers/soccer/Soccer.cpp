#include "Soccer.hpp"
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
//#include <webots/Speaker.hpp>
#include <webots/Keyboard.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2VisionManager.hpp>

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include <blackboard.h>

using namespace webots;
using namespace managers;
using namespace std;

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

Soccer::Soccer():
    Robot()
{
  mTimeStep = getBasicTimeStep();

  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  mCamera = getCamera("Camera");
  mCamera->enable(2*mTimeStep);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);
  //mSpeaker = getSpeaker("Speaker");

  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  mVisionManager = new RobotisOp2VisionManager(mCamera->getWidth(), mCamera->getHeight(), 28, 20, 50, 45, 0, 30);
}

Soccer::~Soccer() {
}

void Soccer::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Soccer::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Ball detection based on the ball color using the Vision Manager
// - return: indicates if the algorithm found the ball
// - args: return the position of the ball [-1.0, 1.0]


void Soccer::writeImagetoMemory() 
{
  //cout << "-------teste-------" << endl;
  static int image_width  = mCamera->getWidth();
  static int image_height = mCamera->getHeight();
  int index=2;
  *(mem) = image_width;
  *(mem+1) = image_height;
  const unsigned char *image = mCamera->getImage();
  for (int x = 0; x < image_height; x++)
    for (int y = 0; y < image_width; y++) {
      //int r = mCamera->imageGetRed(image, image_width, x, y);
      //int g = mCamera->imageGetGreen(image, image_width, x, y);
      //int b = mCamera->imageGetBlue(image, image_width, x, y);
      //printf("red=%d, green=%d, blue=%d", r, g, b);
      *(mem+index) = mCamera->imageGetRed(image, image_width, y, x);
      *(mem+index+1) = mCamera->imageGetGreen(image, image_width, y, x);
      *(mem+index+2) = mCamera->imageGetBlue(image, image_width, y, x);
      index+=3;
   }
}


// function containing the main feedback loop
void Soccer::run() {

  cout << "---------------Demo of ROBOTIS OP2---------------" << endl;
  cout << "This demo illustrates all the possibilities available for the ROBOTIS OP2." << endl;
  cout << "This includes motion playback, walking algorithm and image processing." << endl;

  using_shared_memory();

  // Use the speaker to present
  //mSpeaker->speak("Hi, my name is ROBOTIS OP2. I can walk, use my camera to find the ball, and perform complex motion like kicking the ball for example.", 1.0);

  // First step to update sensors values
  myStep();

  // set eye led to green
  mEyeLED->set(0x00FF00);

  // play the hello motion
  //mMotionManager->playPage(1); // init position
  //mMotionManager->playPage(24); // hello
  //mMotionManager->playPage(9); // walkready position
  //wait(200);

  // play the motion preparing the robot to walk
  mGaitManager->start();
  mGaitManager->step(mTimeStep);

  mMotors[1]->setPosition(minMotorPositions[1]+4.3);//450 //620
  mMotors[0]->setPosition(-1.3);//450 //620

  mMotors[19]->setPosition(.12);//450 //620
  
  bool actions = false;
  int key = 0;
  bool isWalking = false;
  //int fup = 0;
  //int fdown = 0;
  //const double acc_tolerance = 80.0;
  //const double acc_step = 20;
  *(mem+1000007) = 0; //variavel que informa que uma acao foi executada
  *(mem+1000012) = 0;

  // main loop
  while (true) {

    bool kb_control = false;
    writeImagetoMemory();
    //mem = (int*)im;
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    // get keyboard key
    //cout<<"im"<< int(*(im)) <<" "<<int(*(im+1)) << endl;
    //cout<< int(*(mem)) << int(*(mem+1)) << endl;
    //cout << "Decision = " << *(mem+1000000) <<endl;

    // get keyboard key
    while((key = mKeyboard->getKey()) >= 0) {
      switch(key) {
        case ' ' : // Space bar
          if(isWalking) {
            mGaitManager->stop();
            isWalking = false;
            wait(200);
          }
          else {
            mGaitManager->start();
            isWalking = true;
            wait(200);
          }
          break;
        case Keyboard::UP :
          mGaitManager->setXAmplitude(1.0);
          break;
        case Keyboard::DOWN :
          mGaitManager->setXAmplitude(-1.0);
          break;
        case Keyboard::RIGHT :
          mGaitManager->setAAmplitude(-0.5);
          break;
        case Keyboard::LEFT :
          mGaitManager->setAAmplitude(0.5);
          break;
        case 'C' :
          mGaitManager->stop();
          wait(500);
          mMotionManager->playPage(13); // left kick          
          mMotionManager->playPage(9); // init position
          mGaitManager->start();  
          break;
        case 'V' :
          mGaitManager->stop();
          wait(500);
          mMotionManager->playPage(12); // right kick          
          mMotionManager->playPage(9); // init position
          mGaitManager->start();  
          break;
      }
      kb_control = true;
    }
    
    if (*(mem+1000008) == 1)
    {
      cout << "Decision = " << *(mem+1000000) <<endl;
      if(!kb_control)
      {
        if(*(mem+1000000) == 0) // stop with gait
            mGaitManager->setXAmplitude(0.0);
        if(*(mem+1000000) == 1) // center -  walk foward
            mGaitManager->setXAmplitude(1.0);
        if(*(mem+1000000) == 2) // walk backward
            mGaitManager->setXAmplitude(-1.0);
        if(*(mem+1000000) == 3) // turn
        {
            mGaitManager->setYAmplitude(1.0);
            mGaitManager->setAAmplitude(-0.18);
        }
        if(*(mem+1000000) == 4) // turn
        {
            mGaitManager->setYAmplitude(-1.0);
            mGaitManager->setAAmplitude(0.18);
        }
        if(*(mem+1000000) == 5) // center -  walk foward
            mGaitManager->setAAmplitude(-0.28);
        if(*(mem+1000000) == 6) // walk backward
            mGaitManager->setAAmplitude(0.28);
        if(*(mem+1000000) == 7) // left kick
        { 
              mGaitManager->stop();
              wait(200);          
              mMotionManager->playPage(13); // left kick
              mMotionManager->playPage(9); // init position
              mGaitManager->start();
              actions = true;  
        }
        if(*(mem+1000000) == 8) // right kick 
        { 
              mGaitManager->stop();
              wait(200);
              mMotionManager->playPage(12); // right kick          
              mMotionManager->playPage(9); // init position
              mGaitManager->start();
              actions = true;
        }      
      }
  
      if (*(mem+1000012) == 1)
      {
        while (true)
        {
          writeImagetoMemory();
          mGaitManager->stop();
          isWalking = false;
          wait(50);
        }
      }
  
      if(actions == false)
      {
        for(int con=0; con<30; con++) //10 -> 200ms
        {
        
          //===================================================================
          // count how many steps the accelerometer
          // says that the robot is down
/*          const double *acc = mAccelerometer->getValues();
          if (acc[1] < 512.0 - acc_tolerance)
            fup++;
          else
            fup = 0;
      
          if (acc[1] > 512.0 + acc_tolerance)
            fdown++;
          else
            fdown = 0;
      
          // the robot face is down
          if (fup > acc_step) {
            *(mem+1000005) = 1;
            fup = 0;
          }
          // the back face is down
          else if (fdown > acc_step) {
            *(mem+1000005) = 1;
            fdown = 0;
          }*/
          //===================================================================
        
          mGaitManager->step(mTimeStep);
          mMotors[0]->setPosition(-1.0);//450 //620
          mMotors[1]->setPosition(1.0);//450 //620
          myStep();
        }
      }
      *(mem+1000007) = 1; //Informa que a acao foi executada
      //*(mem+1000008) = 0; //Trava para esperar o DQN executar a acao
      actions = false;
    }
    
    mGaitManager->step(mTimeStep);
    mMotors[0]->setPosition(-1.0);//450 //620
    mMotors[1]->setPosition(1.0);//450 //620
    // step
    myStep(); 
  }
}
