// Description:   Simple soccer player showing how to use the middleware between webots and
//                the robotis-op2 framework

#ifndef SOCCER_HPP
#define SOCCER_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class RobotisOp2MotionManager;
  class RobotisOp2GaitManager;
  class RobotisOp2VisionManager;
}

namespace webots {
  class Motor;
  class LED;
  class Camera;
  class Accelerometer;
  class PositionSensor;
  class Gyro;
  //class Speaker;
  class Keyboard;
};

class Soccer : public webots::Robot {
  public:
                                     Soccer();
    virtual                         ~Soccer();
    void                             run();

  private:
    int                              mTimeStep;

    void                             myStep();
    void                             wait(int ms);
    bool                             getBallCenter(double &x, double &y);
    void                             writeImagetoMemory();

    webots::Motor                    *mMotors[NMOTORS];
    webots::PositionSensor           *mPositionSensors[NMOTORS];
    webots::LED                      *mEyeLED;
    webots::LED                      *mHeadLED;
    webots::LED                      *mBackLedRed;
    webots::LED                      *mBackLedGreen;
    webots::LED                      *mBackLedBlue;
    webots::Camera                   *mCamera;
    webots::Accelerometer            *mAccelerometer;
    webots::Gyro                     *mGyro;
    //webots::Speaker                  *mSpeaker;
    webots::Keyboard                 *mKeyboard;

    managers::RobotisOp2MotionManager  *mMotionManager;
    managers::RobotisOp2GaitManager    *mGaitManager;
    managers::RobotisOp2VisionManager  *mVisionManager;
};

#endif
