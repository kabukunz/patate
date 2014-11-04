#include "femUtils.h"


namespace Vitelotte
{


template <class Derived0, class Derived1>
inline typename Eigen::MatrixBase<Derived0>::Scalar det2(const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1)
{
    return _v0.x() * _v1.y() - _v0.y() * _v1.x();
}


}
