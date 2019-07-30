// Description:   Manage the entree point function

#include "Soccer.hpp"

#include <cstdlib>

using namespace webots;

int main(int argc, char **argv)
{
  Soccer *controller = new Soccer();
  controller->run();
  delete controller;
  return EXIT_FAILURE;
}
