/*******************************************************************************************************/
/* Description:  Wrapper of the Gyro Webots API for the ROBOTIS OP2 real robot                           */
/*******************************************************************************************************/

#ifndef GYRO_HPP
#define GYRO_HPP

#include <webots/Robot.hpp>
#include <webots/Device.hpp>

namespace webots {
  class Gyro: public Device  {
    public:
                    Gyro(const std::string &name); //Use Robot::getGyro() instead
      virtual      ~Gyro();

      virtual void  enable(int samplingPeriod);
      virtual void  disable();
      const double *getValues() const;
      int           getSamplingPeriod() const;

    private:
      void          setValues(const int *integerValues);

      double        mValues[3];

      friend int Robot::step(int duration);
  };
}

#endif // GYRO_HPP
