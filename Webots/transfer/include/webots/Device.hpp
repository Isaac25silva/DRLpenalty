/*******************************************************************************************************/
/* Description:  Wrapper of the Device Webots API for the ROBOTIS OP2 real robot                         */
/*******************************************************************************************************/

#ifndef DEVICE_HPP
#define DEVICE_HPP

#define WB_USING_CPP_API
#include <string>

namespace webots {
  class Device {
    public:
      virtual           ~Device() {}
      const std::string &getName() const { return mName; }

    protected:
                         Device(const std::string &n) : mName(n) {}

    private:
      std::string        mName;
  };
}

#endif //DEVICE_HPP
