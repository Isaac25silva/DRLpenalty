#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>

#include "keyboardInterface.hpp"

#include <iostream>
#include <unistd.h>
#include <stdlib.h>

using namespace webots;
using namespace std;

void *Keyboard::KeyboardTimerProc(void *param) {
  ((Keyboard*)param)->mKeyboardInterface->startListenKeyboard();
  return NULL;
}

Keyboard::Keyboard() {
    mKeyboardRate = 0;
    mKeyboardInterface = new KeyboardInterface();
}

Keyboard::~Keyboard() {
}

void Keyboard::enable(int samplingPeriod) {
  if (mKeyboardRate <= 0) {
    // Starting keyboard listenning in a thread
    int error = 0;

    mKeyboardInterface->createWindow();

    // create and start the thread
    if ((error = pthread_create(&this->mKeyboardThread, NULL, this->KeyboardTimerProc, this)) !=  0) {
      cerr << "Keyboard thread error = " << error << endl;
      exit(-1);
    }
  }

  mKeyboardRate = samplingPeriod;
}

void Keyboard::disable() {
  if (mKeyboardRate > 0) {
    int error = 0;
    // End the thread
    if ((error = pthread_cancel(this->mKeyboardThread)) != 0) {
      cerr << "Keyboard thread error = " << error << endl;
      exit(-1);
    }
    mKeyboardInterface->closeWindow();
    mKeyboardInterface->resetKeyPressed();
  }

  mKeyboardRate = 0;
}

int Keyboard::getKey() const {
  if (mKeyboardRate >= 0)
    return mKeyboardInterface->getKeyPressed();
  else
    return -1;
}

int Keyboard::getSamplingPeriod() const {
  return mKeyboardRate;
}

void Keyboard::resetKeyboard() {
  if (mKeyboardRate >= 0)
    mKeyboardInterface->resetKeyPressed();
}
