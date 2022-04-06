#include <memory>

namespace Eigen
{
class QLDDirect;
}

namespace ProjectA
{
class ClassA
{
 public:
  ClassA()
  {
  }

  std::shared_ptr<Eigen::QLDDirect> qld_;
};

void funcA();
}
