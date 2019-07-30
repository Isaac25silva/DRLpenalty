###############################################################
#
# Purpose: Makefile for building a wrapper between the Webots
#          API and the robotis-op2 real robot framework
#
###############################################################

# -------------------------------------------------------------
# TO MODIFY:

# paths
ROBOTISOP2_ROOT ?= /robotis

# -------------------------------------------------------------

TARGET = wrapper.a
CXX_SOURCES = \
  ../src/Robot.cpp \
  ../src/Motion.cpp \
  ../src/Motor.cpp \
  ../src/PositionSensor.cpp \
  ../src/LED.cpp \
  ../src/Accelerometer.cpp \
  ../src/Gyro.cpp \
  ../src/Camera.cpp \
  ../src/Keyboard.cpp \
  ../src/Speaker.cpp
OBJECTS = $(CXX_SOURCES:.cpp=.o)
INCLUDE_DIRS = -I$(ROBOTISOP2_ROOT)/Linux/include -I$(ROBOTISOP2_ROOT)/Framework/include -I../include -I../keyboard

AR = ar
ARFLAGS = cr
CXX = g++
CXXFLAGS += -c -O2 -DLINUX -Wall $(INCLUDE_DIRS)
LIBS += ../keyboard/keyboardInterface.a
LINK_DEPENDENCIES = ../keyboard/keyboardInterface.a
ROBOTISOP2_STATIC_LIBRARY = $(ROBOTISOP2_ROOT)/Linux/lib/darwin.a

all: $(TARGET)

clean:
	rm -f $(TARGET) $(OBJECTS)

$(ROBOTISOP2_STATIC_LIBRARY):
	make -C $(ROBOTISOP2_ROOT)/Linux/build

$(TARGET): $(ROBOTISOP2_STATIC_LIBRARY) $(OBJECTS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJECTS) $(ROBOTISOP2_STATIC_LIBRARY) $(LIBS) $(LINK_DEPENDENCIES)
	chmod 755 $(TARGET)
