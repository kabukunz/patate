/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.

 \authors Moos Hueting, Nicolas Mellado

*/


#ifndef _SHELVES_BBOX_
#define _SHELVES_BBOX_

#include <limits>

namespace Shelves{

/*!
 \brief Axis-Aligned Bounding Box in arbitrary dimension
 */
template <typename _Scalar, int _Dim>
class AABBox
{
public:
    typedef _Scalar Scalar;
    enum { Dim = _Dim };

    typedef Eigen::Matrix<Scalar, Dim, 1> VectorType;

    AABB() { _min.setConstant( std::numeric_limits<Scalar>::max() / 2);
             _max.setConstant(-std::numeric_limits<Scalar>::max() / 2); }
    AABB(const VectorType& min, const VectorType& max) : _min(min), _max(max) {}
    AABB(const AABB& bb) : _min(bb._min), _max(bb._max) {}
    template <class InputIt>
    AABB(InputIt first, InputIt last) { extendTo(first, last); }

    inline AABB<Scalar, Dim>& operator=(const AABB<Scalar, Dim>& bb)
    { _min = bb._min; _max = bb._max; return (*this); }

    inline bool operator==(const AABB<Scalar, Dim>& bb) const
    { return _min == bb._min && _max == bb._max; }

    template <class VectorTypeDerived>
    inline void extendTo(const VectorTypeDerived& q)
    { _min = (q.array() < _min.array()).select(q, _min);
      _max = (q.array() > _max.array()).select(q, _max); }

    template <class InputIt>
    inline void extendTo(InputIt first, InputIt last)
    { std::for_each(first, last,
                    [this](const typename InputIt::value_type& q)
                    { return extendTo(q); });
    }

    inline bool contains(const VectorType& q) const
    { return ((q.array() > _min.array()) && (q.array() < _max.array())).all(); }

    inline Scalar diagonalLength() const
    { return (_max - _min).norm(); }

    inline VectorType center() const
    { return _min + ((_max - _min) / Scalar(2.)); }

    inline const VectorType& min() const
    { return _min; }

    inline const VectorType& max() const
    { return _max; }

    template <int d>
    inline Scalar length() const
    { return (_max - _min)(d); }

    inline Scalar length(int d) const
    { return (_max - _min)(d); }

protected:
    VectorType _min, _max;

private:

}; // class AABB
} // namespace Shelves

#endif // BBOX_H




