#include <iostream>

#include <ProjectA/ProjectA.h>
#include <ProjectB/ProjectB.h>

using namespace ProjectB;

void funcB()
{
  ProjectA::ClassA classA;

  ProjectA::funcA();

  std::cout << "Defined in ProjectB.cpp" << std::endl;
}
