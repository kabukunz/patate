#ifndef _FEM_UTILS_H_
#define _FEM_UTILS_H_

#include <Eigen/Core>


namespace Vitelotte
{

template <class Derived0, class Derived1>
inline typename Eigen::MatrixBase<Derived0>::Scalar det2(
        const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1);


#include "femUtils.hpp"

} // namespace Vitelotte

#endif
