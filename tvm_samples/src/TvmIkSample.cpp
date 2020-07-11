/* Author: Masaki Murooka */

#include "PostureGenerator.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "TvmIkSample");

  tvm_samples::PostureGenerator gen;

  gen.run();
}
