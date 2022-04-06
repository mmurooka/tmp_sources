#include <iostream>

#include <ProjectA/ProjectA.h>
#include <ProjectB/ProjectB.h>

using namespace ProjectB;

void funcB()
{
  ProjectA::funcA();

  std::cout << "Defined in ProjectB.cpp" << std::endl;
}
