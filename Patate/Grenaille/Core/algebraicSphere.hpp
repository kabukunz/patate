/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


/*!
Use gradient descent
*/
template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::VectorType
AlgebraicSphere<DataPoint, _WFunctor, T>::project( const VectorType& _q ) const
{
    MULTIARCH_STD_MATH(min)

    // turn to centered basis
    const VectorType lq = _q-m_p;

    //if(_isPlane)
    //{
    VectorType grad;
    VectorType dir  = m_ul+Scalar(2.)*m_uq*lq;
    Scalar ilg      = Scalar(1.)/dir.norm();
    dir             = dir*ilg;
    Scalar ad       = m_uc + m_ul.dot(lq) + m_uq * lq.squaredNorm();
    Scalar delta    = -ad*min(ilg,Scalar(1.));
    VectorType proj = lq + dir*delta;

    for (int i=0; i<16; ++i)
    {
        grad  = m_ul+Scalar(2.)*m_uq*proj;
        ilg   = Scalar(1.)/grad.norm();
        delta = -(m_uc + proj.dot(m_ul) + m_uq * proj.squaredNorm())*min(ilg,Scalar(1.));
        proj += dir*delta;
    }
    return proj + m_p;
    //}
    //return other - _ul * dot(other,_ul) + _uc;
    //return normalize(other-_center) * _r + _center;
}

template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::Scalar
AlgebraicSphere<DataPoint, _WFunctor, T>::potential( const VectorType &_q ) const
{  
    // turn to centered basis
    const VectorType lq = _q-m_p;

    return m_uc + lq.dot(m_ul) + m_uq * lq.squaredNorm();
}


template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::VectorType
AlgebraicSphere<DataPoint, _WFunctor, T>::primitiveGradient( const VectorType &_q ) const
{
        // turn to centered basis
        const VectorType lq = _q-m_p;  
        return (m_ul + Scalar(2.f) * m_uq * lq);
}

/**************************************************************************/
/* Experimentation (CM)                                                   */
/**************************************************************************/
#ifdef PATATE_EXPERIMENTAL
template < class DataPoint, class _WFunctor, typename T>
void
AlgebraicSphere<DataPoint, _WFunctor, T>::setParameters(const Scalar& uc, const VectorType& ul, const Scalar& uq, const VectorType& p)
{
    m_uc = uc;
    m_ul = ul;
    m_uq = uq;
    m_p = p;
    m_isNormalized = false;
    Base::m_eCurrentState = STABLE;
}

template < class DataPoint, class _WFunctor, typename T>
void
AlgebraicSphere<DataPoint, _WFunctor, T>::changeBasis(const VectorType& new_base)
{
    Scalar ulnb = m_ul.transpose() * new_base;
    Scalar nbnb = new_base.transpose() * new_base;
    setParameters(m_uc - ulnb + m_uq * nbnb,
                  m_ul - 2.0 * m_uq * new_base,
                  m_uq,
                  new_base);
    applyPrattNorm();
}

template < class DataPoint, class _WFunctor, typename T>
AlgebraicSphere<DataPoint, _WFunctor, T>
AlgebraicSphere<DataPoint, _WFunctor, T>::combine(const AlgebraicSphere& q1, const AlgebraicSphere &q2, Scalar alpha)
{
    // same base
    VectorType new_base = q2.m_p + alpha * (q1.m_p - q2.m_p);
    AlgebraicSphere q1_new_base = q1;
    AlgebraicSphere q2_new_base = q2;
    q1_new_base.changeBasis(new_base);
    q2_new_base.changeBasis(new_base);

    // compute Salpha = S2 + alpha(S1 - S2)
    Scalar new_uc       = q2_new_base.m_uc + alpha * (q1_new_base.m_uc - q2_new_base.m_uc);
    VectorType new_ul   = q2_new_base.m_ul + alpha * (q1_new_base.m_ul - q2_new_base.m_ul);
    Scalar new_uq       = q2_new_base.m_uq + alpha * (q1_new_base.m_uq - q2_new_base.m_uq);
    VectorType new_p    = new_base;

    setParameters(new_uc, new_ul, new_uq, new_p);
    applyPrattNorm();
}

template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::Scalar
AlgebraicSphere<DataPoint, _WFunctor, T>::distanceSegSphere(const VectorType& v0, const VectorType& v1)
{
    VectorType v0_centered = v0 - m_p;
    VectorType v1_centered = v1 - m_p;

    AlgebraicSphere prim;
    prim.setParameters(0.0, 0.5 * m_ul + m_uq * v0_centered, (1.0/3.0) * m_uq, m_p);
    applyPrattNorm();

    // S(v0) + S'(v1 - v0)
    Scalar norm = (v1 - v0).norm();

    Scalar dist = (potential(v0_centered) + prim.potential(v1_centered - v0_centered)) * norm;
    return dist;
}
#endif //PATATE_EXPERIMENTAL
