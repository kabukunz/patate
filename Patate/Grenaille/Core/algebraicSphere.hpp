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
AlgebraicSphere<DataPoint, _WFunctor, T>::combine(const AlgebraicSphere& q1, const AlgebraicSphere &q2, Scalar alpha)
{
    VectorType new_base = q1.m_p + alpha * (q2.m_p - q1.m_p);
    AlgebraicSphere q1_new_base = q1;
    AlgebraicSphere q2_new_base = q2;

    // compute Salpha = S1 + alpha(S2 - S1)
    Scalar new_uc       = q1_new_base.m_uc + alpha * (q2_new_base.m_uc - q1_new_base.m_uc);
    VectorType new_ul   = q1_new_base.m_ul + alpha * (q2_new_base.m_ul - q1_new_base.m_ul);
    Scalar new_uq       = q1_new_base.m_uq + alpha * (q2_new_base.m_uq - q1_new_base.m_uq);
    VectorType new_p    = new_base;
    //new_ul.normalize(); // no need

    setParameters(new_uc, new_ul, new_uq, new_p);

    applyPrattNorm();
}

template < class DataPoint, class _WFunctor, typename T>
void
AlgebraicSphere<DataPoint, _WFunctor, T>::combine(const AlgebraicSphere& q0, const AlgebraicSphere& q1, const AlgebraicSphere &q2, Scalar gamma1, Scalar gamma2)
{
    VectorType new_base = gamma1 * q0.m_p + gamma2 * q1.m_p + (1.0 - gamma1 - gamma2) * q2.m_p;
    AlgebraicSphere q0_new_base = q0;
    AlgebraicSphere q1_new_base = q1;
    AlgebraicSphere q2_new_base = q2;

    // compute Salpha = gamma1 * S0 + gamma2 * S1 + (1.0 - gamma1 - gamma2) * S2
    Scalar new_uc       = gamma1 * q0_new_base.m_uc + gamma2 * q1_new_base.m_uc + (1.0 - gamma1 - gamma2) * q2_new_base.m_uc;
    VectorType new_ul   = gamma1 * q0_new_base.m_ul + gamma2 * q1_new_base.m_ul + (1.0 - gamma1 - gamma2) * q2_new_base.m_ul;
    Scalar new_uq       = gamma1 * q0_new_base.m_uq + gamma2 * q1_new_base.m_uq + (1.0 - gamma1 - gamma2) * q2_new_base.m_uq;
    VectorType new_p    = new_base;
    new_ul.normalize();

    setParameters(new_uc, new_ul, new_uq, new_p);

    applyPrattNorm();
}

template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::Scalar
AlgebraicSphere<DataPoint, _WFunctor, T>::distanceSegSphere(const VectorType& v0, const VectorType& v1)
{
    VectorType v0_centered = v0 - m_p;
    VectorType v1_centered = v1 - m_p;
    VectorType seg = v1_centered - v0_centered;

    AlgebraicSphere prim;
    prim.setParameters(0.0, 0.5 * m_ul + m_uq * v0_centered, (1.0/3.0) * m_uq, VectorType::Zero()); //v0_centered

    Scalar norm = (v1 - v0).norm(); // v1 and v0 centered

    // (S(v0) + S'(v1 - v0)) * norm(v1-v0)
    Scalar dist = (potential(v0) + prim.potential(seg)) * norm; // to have the same results than the short paper
                                                                // eurographics 2017 (2)


    //Scalar dist = (potential(v0_centered) + prim.potential(seg)) * norm;  // to have the same results than the short paper
                                                                            // eurographics 2017 (1)
    return dist;
}

template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::Scalar
// https://en.wikipedia.org/wiki/Barycentric_coordinate_system
AlgebraicSphere<DataPoint, _WFunctor, T>::distanceFaceSphere(const VectorType& v0, const VectorType& v1, const VectorType& v2, const VectorType& n, Scalar gradient_weight, Scalar min_radius, Scalar max_radius)
{
    VectorType v0_centered = v0 - m_p;
    VectorType v1_centered = v1 - m_p;
    VectorType v2_centered = v2 - m_p;

    AlgebraicSphere prim;
    // \TODO{ Understand what has to be centered or not }
    // \TODO{ How to factorize the formula }
    prim.setParameters(0.0, (1.0/6.0) * m_ul + (1.0/3.0) * m_uq * v2_centered, (1.0/12.0) * m_uq, VectorType::Zero());
    Scalar residual = (1.0/12.0) * m_uq * (v1_centered - v2_centered).dot(v0_centered - v2_centered);
    Scalar area = ( ( ( v1 - v0 ).cross( v2 - v0 ) ).norm() * 0.5 );

    Scalar dist = 0.0;
    dist = 2.0 * area * (0.5 * potential(v2) + prim.potential(v0 - v2) + prim.potential(v1 - v2) + residual) ;

    // test
    //gradient_weight = 1.0 - ((radius() - min_radius) / (max_radius - min_radius));
    //gradient_weight = ((radius() - min_radius) / (max_radius - min_radius));
    //gradient_weight = (radius() / (max_radius - min_radius));

    // adding gradient
    //dist += gradient_weight * (1.0 - ( m_ul.dot(n) + 2.0 * m_uq * v2.dot(n) - 2.0 * m_uq * basisCenter().dot(n) +
    //                m_uq * (v0 - v2).dot(n) + m_uq * (v1 - v0).dot(n) +
    //                (1.0/3.0) * m_uq * (v0 + v2 - 2.0 * v1).dot(n)));

    return dist;
}

template < class DataPoint, class _WFunctor, typename T>
typename DataPoint::Scalar
// https://en.wikipedia.org/wiki/Barycentric_coordinate_system
AlgebraicSphere<DataPoint, _WFunctor, T>::gradientFaceSphere(const VectorType& v0, const VectorType& v1, const VectorType& v2, const VectorType& n, Scalar gradient_weight, Scalar min_radius, Scalar max_radius)
{
    Scalar dist_gradient = 0.0;
    Scalar dt = 0.01;
    VectorType grad;
    for (Scalar gamma2 = 0.0; gamma2 <= 1.0; gamma2 += dt) // we go through the face
    {
        for (Scalar gamma1 = 0.0; gamma1 <= 1.0 - gamma2; gamma1 += dt)
        {
            grad = m_ul + 2.0 * m_uq * (gamma1 * v0 + gamma2 * v1 + (1.0 - gamma1 - gamma2) * v2);
            grad.normalize();
            //dist_gradient += gradient_weight * (1.0 - grad.dot(n));
            dist_gradient += (1.0 - grad.dot(n));
        }
    }
    return dist_gradient;
    //test
    //gradient_weight = 1.0 - ((radius() - min_radius) / (max_radius - min_radius));
    //gradient_weight = (radius() - min_radius) / (max_radius - min_radius);
    //gradient_weight = (radius() / (max_radius - min_radius));

    // adding gradient
    //Scalar dist = gradient_weight * (1.0 - ( m_ul.dot(n) + 2.0 * m_uq * v2.dot(n) - 2.0 * m_uq * basisCenter().dot(n) +
    //                m_uq * (v0 - v2).dot(n) + m_uq * (v1 - v0).dot(n) +
    //                (1.0/3.0) * m_uq * (v0 + v2 - 2.0 * v1).dot(n)));

    //return dist;
}

template < class DataPoint, class _WFunctor, typename T>
bool
AlgebraicSphere<DataPoint, _WFunctor, T>::hasSameParameter(const AlgebraicSphere& q1) const
{
    Scalar epsilon = Eigen::NumTraits<Scalar>::dummy_precision();
    return ((q1.m_uc - m_uc) < epsilon && (q1.m_ul - m_ul).norm() < epsilon && (q1.m_uq - m_uq) < epsilon);
}

#endif //PATATE_EXPERIMENTAL
