
template < class _Mesh, typename _Scalar >
FVElementBuilder<_Mesh, _Scalar>::FVElementBuilder(Scalar sigma)
  : m_sigma(sigma)
{
}

template < class _Mesh, typename _Scalar >
unsigned
FVElementBuilder<_Mesh, _Scalar>::
    nCoefficients(const Mesh& mesh, Face element) const
{
    return 81;
}


template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    addCoefficients(InIt& it, const Mesh& mesh, Face element) const
{
    assert(mesh.valence(element) == 3);

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

    // TODO: remove some code duplication by moving stuff here
    bool flat = false;
    for(int i = 0; i < 3; ++i)
    {
        if(mesh.hasFlatGradient(mesh.toVertex(*hit)))
        {
            flat = true;
            break;
        }
        ++hit;
    }

    if(flat)
        processFV1ElementFlat(it, mesh, element);
    else
        processFV1Element(it, mesh, element);
}

template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    processFV1Element(InIt& it, const Mesh& mesh, Face element) const
{
    assert(mesh.valence(element) == 3);

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

    bool orient[3];
    Vector v[3];
    unsigned nodes[9];
    --hit;
    for(int i = 0; i < 3; ++i)
    {
        v[i] = (mesh.position(mesh.toVertex(*hit)) -
                mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
        orient[i] = mesh.fromVertex(*hit).idx() > mesh.toVertex(*hit).idx();
        nodes[3+i] = mesh.midNode(*hit);
        nodes[6+i] = mesh.gradientNode(*hit);
        ++hit;
        nodes[i] = mesh.toNode(*hit);
    }

    Vector p[] = {
        FemVector::Zero(),
        v[2],
        -v[1]
    };

    Scalar area = det2(v[0], v[1]) / 2.;

    assert(area > 0);

    Vector3 a, b, c, d, l;
    for(size_t i0 = 0; i0 < 3; ++i0)
    {
        size_t i1 = (i0+1)%3;
        size_t i2 = (i0+2)%3;
        a(i0) = det2(p[i1], p[i2]);
        b(i0) = -v[i0](1);
        c(i0) = v[i0](0);
        d(i0) = .5 - v[i0].dot(-v[i2]) / v[i0].squaredNorm();
        l(i0) = v[i0].norm();
    }

// TODO : check that debug info
/*#ifndef _DEBUG
        for(size_t i = 0; i < 3; ++i)
    {
                for(size_t j = 0; j < 3; ++j)
        {
                        FemScalar v = a(i) + b(i) * p[j](0) + c(i) * p[j](1);
                        FemScalar r = (i==j)? 2.*area: 0.;
                        assert(std::abs(v - r) < 1.e-8);
                }

                assert(std::abs(l(i) - std::sqrt(b(i)*b(i) + c(i)*c(i))) < 1.e-8);
        }
#endif*/

    Vector3 dx2[9];
    Vector3 dy2[9];
    Vector3 dxy[9];

    for(size_t i = 0; i < 3; ++i)
    {
    // 25%:
        dx2[i+0] = funcVertexBasis(i, Deriv_XX, a, b, c, d, l, area);
        dx2[i+3] = funcMidpointBasis(i, Deriv_XX, a, b, c, d, l, area);
        dx2[i+6] = funcMidpointDerivBasis(i, Deriv_XX, a, b, c, d, l, area);
        dy2[i+0] = funcVertexBasis(i, Deriv_YY, a, b, c, d, l, area);
        dy2[i+3] = funcMidpointBasis(i, Deriv_YY, a, b, c, d, l, area);
        dy2[i+6] = funcMidpointDerivBasis(i, Deriv_YY, a, b, c, d, l, area);
        dxy[i+0] = funcVertexBasis(i, Deriv_XY, a, b, c, d, l, area);
        dxy[i+3] = funcMidpointBasis(i, Deriv_XY, a, b, c, d, l, area);
        dxy[i+6] = funcMidpointDerivBasis(i, Deriv_XY, a, b, c, d, l, area);
    };

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = i; j < 9; ++j)
        {
            EIGEN_ASM_COMMENT("MYBEGIN");

            Vector6 basis = multBasis(dx2[i]+dy2[i], dx2[j]+dy2[j])
                                    + (1.-m_sigma) * (2. * multBasis(dxy[i], dxy[j])
                                                    - multBasis(dx2[i], dy2[j])
                    - multBasis(dy2[i], dx2[j]));

            FemScalar value = integrateQuadTriangle(v, basis, area);

            EIGEN_ASM_COMMENT("MYEND");

            if((i < 6 || orient[i%3]) != (j < 6 || orient[j%3]))
            {
                value *= -1;
            }

            *(it++) = Triplet(nodes[i], nodes[j], value);
            if(i != j)
                *(it++) = Triplet(nodes[j], nodes[i], value);
//            _elemStiffness(i, j) = value;
//            _elemStiffness(j, i) = value;
        }
    }
}

template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    processFV1ElementFlat(InIt& it, const Mesh& mesh, Face element) const
{
    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

    // FIXME: This implementation assume that the element is equilateral.

    bool orient[3];
    Vector v[3];
    unsigned nodes[9];
    --hit;
    unsigned flatVertex;
    for(int i = 0; i < 3; ++i)
    {
        v[i] = (mesh.position(mesh.toVertex(*hit)) -
                mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
        orient[i] = mesh.fromVertex(*hit).idx() > mesh.toVertex(*hit).idx();
        nodes[3+i] = mesh.midNode(*hit);
        nodes[6+i] = mesh.gradientNode(*hit);
        ++hit;
        nodes[i] = mesh.toNode(*hit);
        if(mesh.hasFlatGradient(mesh.toVertex(*hit)))
            flatVertex = i;
    }

    FemScalar area = det2(v[0], v[1]) / 2.;
    assert(area > 0);

    FemScalar sqrtArea = std::sqrt(area);
    FemScalar ffd = (14.-12.*m_sigma)/(3.*area);
    FemScalar ffo = (5.-6.*m_sigma)/(3.*area);
    FemScalar fgd = (12.*m_sigma+4.)/(3.*area);
    FemScalar fgo = (6.*m_sigma-14.)/(3.*area);
    FemScalar fhd = -1.754765350603323/sqrtArea;
    FemScalar fho = (0.43869133765083*(6.*m_sigma-4.))/sqrtArea;
    FemScalar ggd = (24.*m_sigma+104.)/(3.*area);
    FemScalar ggo = -(24.*m_sigma+40.)/(3.*area);
    FemScalar ghd = -(5.264296051809969*m_sigma+8.773826753016614)/sqrtArea;
    FemScalar gho = 7.019061402413293/sqrtArea;
    FemScalar hhd = 6.928203230275509;
    FemScalar hho = 0.;

    typedef Eigen::Matrix<Scalar, 9, 9> StiffnessMatrix;
    StiffnessMatrix elemStiffness;
    elemStiffness <<
        ffd, ffo, ffo,	fgd, fgo, fgo,	fhd, fho, fho,
        ffo, ffd, ffo,	fgo, fgd, fgo,	fho, fhd, fho,
        ffo, ffo, ffd,	fgo, fgo, fgd,	fho, fho, fhd,

        fgd, fgo, fgo,	ggd, ggo, ggo,	ghd, gho, gho,
        fgo, fgd, fgo,	ggo, ggd, ggo,	gho, ghd, gho,
        fgo, fgo, fgd,	ggo, ggo, ggd,	gho, gho, ghd,

        fhd, fho, fho,	ghd, gho, gho,	hhd, hho, hho,
        fho, fhd, fho,	gho, ghd, gho,	hho, hhd, hho,
        fho, fho, fhd, 	gho, gho, ghd,	hho, hho, hhd;

    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

    static const Vector9 u8_1((Vector9()
            <<  -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,                        0.,
                 -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
                                         1.,                       -1.,                        0.).finished());
    static const Vector9 u9_1((Vector9()
            << -2.659424899780574/sqrtArea,                        0., -0.3799178428258/sqrtArea,
                -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
                                        1.,                        0.,                       -1.).finished());
    static const Vector9 u7_2((Vector9()
            << -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,                        0.,
               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
                                     -1.,                          1.,                        0.).finished());
    static const Vector9 u9_2((Vector9()
            <<                        0., -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,
               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
                                      0.,                          1.,                       -1.).finished());
    static const Vector9 u7_3((Vector9()
            << -0.3799178428258/sqrtArea,                        0., -2.659424899780574/sqrtArea,
               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
                                     -1.,                         0,                          1.).finished());
    static const Vector9 u8_3((Vector9()
            <<                        0., -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,
               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
                                      0.,                       -1.,                          1.).finished());

    if(flatVertex == 0)
    {
        elemStiffness.row(7) = u8_1;
        elemStiffness.col(7) = u8_1;
        elemStiffness.row(8) = u9_1;
        elemStiffness.col(8) = u9_1;
    }
    else if(flatVertex == 1)
    {
        elemStiffness.row(6) = u7_2;
        elemStiffness.col(6) = u7_2;
        elemStiffness.row(8) = u9_2;
        elemStiffness.col(8) = u9_2;
    }
    else
    {
        elemStiffness.row(6) = u7_3;
        elemStiffness.col(6) = u7_3;
        elemStiffness.row(7) = u8_3;
        elemStiffness.col(7) = u8_3;
    }

    // Flip gradient sign where needed.
    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = i; j < 9; ++j)
        {
            Scalar sign = 1;
            if((i < 6 || orient[i % 3]) != (j < 6 || orient[j % 3]))
            {
                sign = -1;
            }

            *(it++) = Triplet(nodes[i], nodes[j], elemStiffness(i, j) * sign);
            if(i != j)
                *(it++) = Triplet(nodes[j], nodes[i], elemStiffness(j, i) * sign);
        }
    }
}
