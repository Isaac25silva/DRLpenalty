/*******************************************************************************************************/
/* Description:  Wrapper of the Robot Webots API for the ROBOTIS OP2 real robot                          */
/*******************************************************************************************************/

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <map>
#include <sys/time.h>

#include <minIni.h>

namespace Robot {
  class CM730;
  class LinuxCM730;
}

namespace webots {
  class Device;
  class Accelerometer;
  class Camera;
  class Gyro;
  class LED;
  class Motor;
  class PositionSensor;
  class Speaker;
  class Keyboard;

  class Robot {
    public:

                           Robot();
      virtual             ~Robot();

      virtual int          step(int duration);
      std::string          getName() const;
      double               getTime() const;
      int                  getMode() const;
      double               getBasicTimeStep() const;
      Accelerometer       *getAccelerometer(const std::string &name) const;
      Camera              *getCamera(const std::string &name) const;
      Gyro                *getGyro(const std::string &name) const;
      LED                 *getLED(const std::string &name) const;
      Motor               *getMotor(const std::string &name) const;
      PositionSensor      *getPositionSensor(const std::string &name) const;
      Speaker             *getSpeaker(const std::string &name) const;
      Keyboard            *getKeyboard() const { return mKeyboard; }

      // not member(s) of the Webots API function: please don't use
      ::Robot::CM730      *getCM730() const { return mCM730; }
      static Robot        *getInstance() { return cInstance; }

    private:
      void                 initDevices();
      void                 initRobotisOp2();
      void                 LoadINISettings(minIni *ini, const std::string &section);
      Device              *getDevice(const std::string &name) const;

      static Robot        *cInstance;

      std::map<const std::string, Device *> mDevices;

      int                  mTimeStep;
      Keyboard            *mKeyboard;
      ::Robot::LinuxCM730 *mLinuxCM730;
      ::Robot::CM730      *mCM730;
      struct timeval       mStart;
      double               mPreviousStepTime;
  };
}

#endif //ROBOT_HPP
