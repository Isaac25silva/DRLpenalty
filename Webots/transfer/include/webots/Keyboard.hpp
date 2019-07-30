/********************************************************************************************/
/* Description:  Wrapper of the Keyboard Webots API for the ROBOTIS OP2 real robot            */
/********************************************************************************************/

#ifndef KEYBOARD_HPP
#define KEYBOARD_HPP

#include <webots/Robot.hpp>
#include <webots/Device.hpp>

#include <pthread.h>

class KeyboardInterface;

namespace webots {
  class Keyboard {
    public:
      enum {
        END=312,
        HOME,
        LEFT,
        UP,
        RIGHT,
        DOWN,
        PAGEUP=366,
        PAGEDOWN,
        NUMPAD_HOME=375,
        NUMPAD_LEFT,
        NUMPAD_UP,
        NUMPAD_RIGHT,
        NUMPAD_DOWN,
        NUMPAD_END=382,
        KEY=0x0000ffff,
        SHIFT=0x00010000,
        CONTROL=0x00020000,
        ALT=0x00040000
      };

      Keyboard(); //Use Robot::getKeyboard() instead
      virtual ~Keyboard();
      virtual void enable(int samplingPeriod);
      virtual void disable();
      int getSamplingPeriod() const;
      int getKey() const;

    protected:
      static void         *KeyboardTimerProc(void *param);// thread function

    private:
      KeyboardInterface   *mKeyboardInterface;
      int                  mKeyboardRate;
      pthread_t            mKeyboardThread; // thread structure

      void resetKeyboard();

      friend int Robot::step(int duration);
  };
}

#endif // KEYBOARD_HPP
