/*******************************************************************************************************/
/* Description:  Wrapper of the LED Webots API for the ROBOTIS OP2 real robot                            */
/*******************************************************************************************************/

#ifndef LED_HPP
#define LED_HPP

#include <webots/Robot.hpp>
#include <webots/Device.hpp>

#include <map>

namespace webots {
  class LED: public Device  {
    public:
                                              LED(const std::string &name); //Use Robot::getLED() instead
      virtual                                ~LED();
      virtual void                            set(int value);
      int                                     get() const;

    private:
      static void                             initStaticMap();
      static void                             setBackPanel(int state);
      static int                              mBackPanel;

      static std::map<const std::string, int> mNamesToIDs;

      void                                    setColor(int color);

      int                                     mColor;

      friend int Robot::step(int samplingPeriod);
  };
}

#endif // LED_HPP
