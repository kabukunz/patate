

template < class DataPoint, class _WFunctor, typename T>
void
MlsSphereFitDer<DataPoint, _WFunctor, T>::init(const VectorType& _evalPos)
{
    Base::init(_evalPos);

    m_d2Uc = Matrix::Zero(),
    m_d2Uq = Matrix::Zero();
    m_d2Ul = MatrixArray::Zero();

    m_d2SumDotPN = Matrix::Zero();
    m_d2SumDotPP = Matrix::Zero();
    m_d2SumW     = Matrix::Zero();

    m_d2SumP = MatrixArray::Zero();
    m_d2SumN = MatrixArray::Zero();
}

template < class DataPoint, class _WFunctor, typename T>
bool
MlsSphereFitDer<DataPoint, _WFunctor, T>::addNeighbor(const DataPoint& _nei)
{
    bool bResult = Base::addNeighbor(_nei);

    if(bResult)
    {
        // centered basis
        VectorType q = _nei.pos() - Base::basisCenter();

        Matrix d2w;
        d2w;//TODO(thib) computes d2w wrt Scale/Space der type

        m_d2SumDotPN += d2w * _nei.normal().dot(q);
        m_d2SumDotPP += d2w * q.squaredNorm();
        m_d2SumW     += d2w;

        for(int i=0; i<Dim; ++i)
        {
            m_d2SumP.template block<DerDim,DerDim>(0,i*Dim) += d2w * q[i];
            m_d2SumN.template block<DerDim,DerDim>(0,i*Dim) += d2w * _nei.normal()[i];
        }
    }
    return bResult;
}

template < class DataPoint, class _WFunctor, typename T>
FIT_RESULT
MlsSphereFitDer<DataPoint, _WFunctor, T>::finalize()
{
    Base::finalize();

    if (this->isReady())
    {
        Matrix sumdSumPdSumN  = Matrix::Zero();
        Matrix sumd2SumPdSumN = Matrix::Zero();
        Matrix sumd2SumNdSumP = Matrix::Zero();
        Matrix sumdSumPdSumP  = Matrix::Zero();
        Matrix sumd2SumPdSumP = Matrix::Zero();

        for(int i=0; i<Dim; ++i)
        {
            sumdSumPdSumN  += Base::m_dSumN.row(i).transpose()*Base::m_dSumP.row(i);
            sumd2SumPdSumN += m_d2SumP.template block<DerDim,DerDim>(0,i*Dim)*Base::m_sumN(i);
            sumd2SumNdSumP += m_d2SumN.template block<DerDim,DerDim>(0,i*Dim)*Base::m_sumP(i);
            sumdSumPdSumP  += Base::m_dSumP.row(i).transpose()*Base::m_dSumP.row(i);
            sumd2SumPdSumP += m_d2SumP.template block<DerDim,DerDim>(0,i*Dim)*Base::m_sumP(i);
        }

        Scalar invSumW = Scalar(1.)/Base::m_sumW;

        Matrix d2Nume = m_d2SumDotPN
            - invSumW*invSumW*invSumW*invSumW*(
                    Base::m_sumW*Base::m_sumW*(  Base::m_sumW*(sumdSumPdSumN+sumdSumPdSumN.transpose()+sumd2SumPdSumN+sumd2SumNdSumP)
                                               + Base::m_dSumW.transpose()*(Base::m_sumN.transpose()*Base::m_dSumP + Base::m_sumP.transpose()*Base::m_dSumN)
                                               - (Base::m_sumP.transpose()*Base::m_sumN)*m_d2SumW.transpose()
                                               - (Base::m_dSumN.transpose()*Base::m_sumP + Base::m_dSumP.transpose()*Base::m_sumN)*Base::m_dSumW)
                    - Scalar(2.)*Base::m_sumW*Base::m_dSumW.transpose()*(Base::m_sumW*(Base::m_sumN.transpose()*Base::m_dSumP + Base::m_sumP.transpose()*Base::m_dSumN)
                                                                         - (Base::m_sumP.transpose()*Base::m_sumN)*Base::m_dSumW));

        Matrix d2Deno = m_d2SumDotPP
            - invSumW*invSumW*invSumW*invSumW*(
                Base::m_sumW*Base::m_sumW*(  Scalar(2.)*Base::m_sumW*(sumdSumPdSumP+sumd2SumPdSumP)
                                           + Scalar(2.)*Base::m_dSumW.transpose()*(Base::m_sumP.transpose()*Base::m_dSumP)
                                           - (Base::m_sumP.transpose()*Base::m_sumP)*m_d2SumW.transpose()
                                           - Scalar(2.)*(Base::m_dSumP.transpose()*Base::m_sumP)*Base::m_dSumW)
                - Scalar(2.)*Base::m_sumW*Base::m_dSumW.transpose()*(Scalar(2.)*Base::m_sumW*Base::m_sumP.transpose()*Base::m_dSumP
                                                                     - (Base::m_sumP.transpose()*Base::m_sumP)*Base::m_dSumW));

        Scalar deno2 = Base::m_deno*Base::m_deno;

        m_d2Uq = Scalar(.5)/(deno2*deno2)*(deno2*(  Base::m_dDeno.transpose()*Base::m_dNume
                                                  + Base::m_deno*d2Nume
                                                  - Base::m_dNume.transpose()*Base::m_dDeno
                                                  - Base::m_nume*d2Deno)
                                           - Scalar(2.)*Base::m_deno*Base::m_dDeno.transpose()*(Base::m_deno*Base::m_dNume - Base::m_nume*Base::m_dDeno));

        for(int i=0; i<Dim; ++i)
        {
            m_d2Ul.template block<DerDim,DerDim>(0,i*Dim) = invSumW*(
                  m_d2SumN.template block<DerDim,DerDim>(0,i*Dim)
                - Scalar(2.)*(  m_d2Uq*Base::m_sumP[i]
                              + Base::m_dSumP.row(i).transpose()*Base::m_dUq
                              + Base::m_uq*m_d2SumP.template block<DerDim,DerDim>(0,i*Dim)
                              + Base::m_dUq.transpose()*Base::m_dSumP.row(i))
                - Base::m_ul[i]*m_d2SumW
                - Base::m_dUl.row(i).transpose()*Base::m_dSumW
                - Base::m_dSumW.transpose()*Base::m_dUl.row(i));
        }
        
        Matrix sumdUldSumP = Matrix::Zero();
        Matrix sumUld2SumP = Matrix::Zero();
        Matrix sumd2UlsumP = Matrix::Zero();
        Matrix sumdSumPdUl = Matrix::Zero();

        for(int i=0; i<Dim; ++i)
        {
            sumdUldSumP += Base::m_dUl.row(i).transpose()*Base::m_dSumP.row(i);
            sumUld2SumP += Base::m_ul[i]*m_d2SumP.template block<DerDim,DerDim>(0,i*Dim);
            sumd2UlsumP += m_d2Ul.template block<DerDim,DerDim>(0,i*Dim)*Base::m_sumP[i];
            sumdSumPdUl += Base::m_dSumP.row(i).transpose()*Base::m_dUl.row(i);
        }

        m_d2Uc = -invSumW*(
              sumdUldSumP
            + sumUld2SumP
            + sumd2UlsumP
            + sumdSumPdUl
            + Base::m_dUq.transpose()*Base::m_dSumDotPP
            + Base::m_uq*m_d2SumDotPP
            + Base::m_dSumDotPP.transpose()*Base::m_dUq
            + m_d2Uq*Base::m_sumDotPP
            + Base::m_uc*m_d2SumW
            + Base::m_dUc.transpose()*Base::m_dSumW
            - Base::m_dSumW.transpose()*Base::m_dUc);
    }

    return Base::m_eCurrentState;
}

template < class DataPoint, class _WFunctor, typename T>
typename MlsSphereFitDer<DataPoint, _WFunctor, T>::ScalarArray
MlsSphereFitDer<DataPoint, _WFunctor, T>::dPotential() const
{
    //TODO(thib) handle spaceDer/scaleDer
    return Base::m_dUc + Base::m_ul;
}

template < class DataPoint, class _WFunctor, typename T>
typename MlsSphereFitDer<DataPoint, _WFunctor, T>::VectorArray
MlsSphereFitDer<DataPoint, _WFunctor, T>::dNormal() const
{
    //TODO(thib) handle spaceDer/scaleDer
    //TODO(thib) this is just the hessian for now, need to normalize by the norm and then differenciate
    return m_d2Uc + Base::m_dUl + Base::m_dUl.transpose() + Scalar(2.)*Base::m_uq*VectorArray::Ones();
}