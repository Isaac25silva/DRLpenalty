/*******************************************************************************************************/
/* Description:  Wrapper of the Speaker Webots API for the ROBOTIS OP2 real robot                        */
/*******************************************************************************************************/

#ifndef SPEAKER_HPP
#define SPEAKER_HPP

#include <webots/Device.hpp>

#include <unistd.h>
#include <sys/wait.h>

namespace webots {
  class Speaker: public Device  {
    public:
                    Speaker(const std::string &name); //Use Robot::getSpeaker() instead
      virtual      ~Speaker();
      static void playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance, bool loop);
      void stop(const std::string &sound);
      void setLanguage(const std::string &language);
      std::string getLanguage() { return mLanguage; }
      void speak(const std::string &text, double volume);

      // kept only for backward compatibility should not be used (since Webots 8.5)
      virtual void  playFile(const char *filename) __attribute__ ((deprecated));
      virtual void  playFileWait(const char *filename) __attribute__ ((deprecated));
      virtual void  speak(const char *text, const char *voice = "en", int speed = 175) __attribute__ ((deprecated));
      virtual void  speakFile(const char *filename, const char *voice = "en", int speed = 175) __attribute__ ((deprecated));
    private:
      pid_t mSpeakPID;
      std::string mLanguage;
  };
}

#endif // SPEAKER_HPP
