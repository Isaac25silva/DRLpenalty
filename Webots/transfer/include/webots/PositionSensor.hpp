/*******************************************************************************************************/
/* Description:  Wrapper of the PositionSensor Webots API for the ROBOTIS OP2 real robot                 */
/*******************************************************************************************************/

#ifndef POSITION_SENSOR_HPP
#define POSITION_SENSOR_HPP

#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <map>

namespace webots {
  class PositionSensor: public Device  {
    public:
      enum {
        ROTATIONAL = 0
      };

                    PositionSensor(const std::string &name); //Use Robot::getMotor() instead
      virtual      ~PositionSensor();

      virtual void  enable(int samplingPeriod);
      virtual void  disable();
      int           getSamplingPeriod() const;
      double        getValue() const;

      int           getType() const;
    private:
      static void   initStaticMap();
      static std::map<const std::string, int> mNamesToIDs;
      static std::map<const std::string, int> mNamesToInitPos;

      void          setPresentPosition(int position);

      // For Bulk Read //
      int           mPresentPosition;

      int           mFeedback;

      friend int Robot::step(int duration);
      friend     Robot::Robot();
  };
}

#endif //POSITION_SENSOR_HPP
