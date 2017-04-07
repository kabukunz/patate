/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/



template <class DataPoint, class WeightKernel>
typename DistWeightFunc<DataPoint, WeightKernel>::Scalar
DistWeightFunc<DataPoint, WeightKernel>::w( const VectorType& _q, 
					                        const DataPoint&) const
{
    Scalar d  = _q.norm();  
    return (d <= m_t) ? m_wk.f(d/m_t) : Scalar(0.);
}

template <class DataPoint, class WeightKernel>
typename DistWeightFunc<DataPoint, WeightKernel>::VectorType
DistWeightFunc<DataPoint, WeightKernel>::spacedw(   const VectorType& _q, 
						                            const DataPoint&) const
{
    VectorType result = VectorType::Zero();
    Scalar d = _q.norm();
    if (d <= m_t && d != Scalar(0.)) result = (_q / (d * m_t)) * m_wk.df(d/m_t);
    return result;
}

template <class DataPoint, class WeightKernel>
typename DistWeightFunc<DataPoint, WeightKernel>::Scalar
DistWeightFunc<DataPoint, WeightKernel>::scaledw(   const VectorType& _q, 
						                            const DataPoint&) const
{
    Scalar d  = _q.norm();  
    return (d <= m_t) ? Scalar( - d*m_wk.df(d/m_t)/(m_t*m_t) ) : Scalar(0.);
}


//-----------------------------------------------

/*
Scalar phi(Scalar x, Scalar hi)
{
    return std::pow(1.0 - (x / (hi * hi)), 4.0);
}

Scalar dphi(Scalar x, Scalar hi)
{
    return (-4.0 / (hi * hi)) * std::pow(1.0 - (x / (hi * hi)), 3.0);
}

template <class DataPoint, class WeightKernel>
typename RIMLSWeightFunc<DataPoint, WeightKernel>::Scalar
RIMLSWeightFunc<DataPoint, WeightKernel>::w( const VectorType& _q, const DataPoint& _d)
{
    VectorType px = m_x - _q;
    VectorType n = _d.normal();
    Scalar m_fx = px.dot(n);
    Scalar alpha = 0.0;
    if (m_i > 0)
    {
        Scalar omega    = std::exp(-((fx - m_f)/m_sigma_r)*((fx - m_f)/m_sigma_r));
        Scalar omega_n  = std::exp(-((n - m_grad_f).norm()/m_sigma_n)*((n - m_grad_f).norm()/m_sigma_n));
        alpha           = omega * omega_n;
    }
    else
    {
        alpha = 1.0;
    }

    Scalar m_w = alpha * phi(px.norm() * px.norm(), m_hi);
    Scalar m_grad_w = alpha * 2.0 * px * dphi(px.norm() * px.norm(), m_hi);

    return w;
}
*/
