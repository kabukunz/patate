
template < class _Mesh, class _ElementBuilder >
FemSolver<_Mesh, _ElementBuilder>::FemSolver(Mesh* _mesh, const ElementBuilder& elementBuilder)
  : m_mesh(_mesh),
    m_solved(false),
    m_elementBuilder(elementBuilder)
{
//    initMatrices();
}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::initMatrices()
//{
//    m_linM1 <<
//           1, -0.5, -0.5,
//        -0.5,    0,  0.5,
//        -0.5,  0.5,    0;

//    m_linM2 <<
//           0, -0.5,  0.5,
//        -0.5,    1, -0.5,
//         0.5, -0.5,    0;

//    m_linM3 <<
//           0,  0.5, -0.5,
//         0.5,    0, -0.5,
//        -0.5, -0.5,    1;

//    // 'Magic' initialization of base matrices for quadratic elements.
//    // See Paul Tsipouras, Compact representation of triangular finite
//    //	elements for Poisson's equation, International Journal for
//    //	Numerical Methods in Engineering, Volume 11, Issue 3, pages 419-430,
//    //	1977

//    const FemScalar _1_6 = 1. / 6.;
//    const FemScalar _2_3 = 2. / 3.;
//    const FemScalar _4_3 = 4. / 3.;

//    m_quadM1 <<
//            1,  _1_6,  _1_6,     0, -_2_3, -_2_3,
//         _1_6,     0, -_1_6,  _2_3,     0, -_2_3,
//         _1_6, -_1_6,     0,  _2_3, -_2_3,     0,
//            0,  _2_3,  _2_3,  _4_3, -_4_3, -_4_3,
//        -_2_3,     0, -_2_3, -_4_3,  _4_3,  _4_3,
//        -_2_3, -_2_3,     0, -_4_3,  _4_3,  _4_3;

//    m_quadM2 <<
//            0,  _1_6, -_1_6,     0,  _2_3, -_2_3,
//         _1_6,     1,  _1_6, -_2_3,     0, -_2_3,
//        -_1_6,  _1_6,     0, -_2_3,  _2_3,     0,
//            0, -_2_3, -_2_3,  _4_3, -_4_3,  _4_3,
//         _2_3,     0,  _2_3, -_4_3,  _4_3, -_4_3,
//        -_2_3, -_2_3,     0,  _4_3, -_4_3,  _4_3;

//    m_quadM3 <<
//            0, -_1_6,  _1_6,     0, -_2_3,  _2_3,
//        -_1_6,     0,  _1_6, -_2_3,     0,  _2_3,
//         _1_6,  _1_6,     1, -_2_3, -_2_3,     0,
//            0, -_2_3, -_2_3,  _4_3,  _4_3, -_4_3,
//        -_2_3,     0, -_2_3,  _4_3,  _4_3, -_4_3,
//         _2_3,  _2_3,     0, -_4_3, -_4_3,  _4_3;

//    FemMatrix1010 perm = FemMatrix1010::Zero();
//    perm(0, 0) = 1;
//    perm(1, 3) = 1;
//    perm(2, 6) = 1;
//    perm(3, 1) = 1;
//    perm(4, 2) = 1;
//    perm(5, 4) = 1;
//    perm(6, 5) = 1;
//    perm(7, 7) = 1;
//    perm(8, 8) = 1;
//    perm(9, 9) = 1;

//    FemMatrix1010 m10;
//    m10 <<
//       308448, -231336,  136080,  -31752,  -27216,  -27216,  -31752,  136080, -231336,       0,
//            0,  612360, -489888,  108864,  122472,  122472,       0, -122472,  612360, -734832,
//            0,       0,  612360, -258552, -612360,  122472,       0, -122472, -122472,  734832,
//            0,       0,       0,       0,  258552, -108864,   31752,       0,       0,       0,
//            0,       0,       0,       0,  612360,  244944, -108864,  122472,  122472, -734832,
//            0,       0,       0,       0,       0,  612360,  258552, -612360,  122472, -734832,
//            0,       0,       0,       0,       0,       0,       0, -258552,  108864,       0,
//            0,       0,       0,       0,       0,       0,       0,  612360, -489888,  734832,
//            0,       0,       0,       0,       0,       0,       0,       0,  612360, -734832,
//            0,       0,       0,       0,       0,       0,       0,       0,       0, 1469664;
//    m_cubM1 = m10.selfadjointView<Eigen::Upper>();

//    m10 <<
//        0, -258552,  108864,  -31752,       0,       0,   31752, -108864,  258552,       0,
//        0,  612360, -489888,  136080, -122472, -122472,       0,  122472, -612360,  734832,
//        0,       0,  612360, -231336,  612360, -122472,       0,  122472,  122472, -734832,
//        0,       0,       0,  308448, -231336,  136080,  -31752,  -27216,  -27216,       0,
//        0,       0,       0,       0,  612360, -489888,  108864,  122472,  122472, -734832,
//        0,       0,       0,       0,       0,  612360, -258552, -612360,  122472,  734832,
//        0,       0,       0,       0,       0,       0,       0,  258552, -108864,       0,
//        0,       0,       0,       0,       0,       0,       0,  612360,  244944, -734832,
//        0,       0,       0,       0,       0,       0,       0,       0,  612360, -734832,
//        0,       0,       0,       0,       0,       0,       0,       0,       0, 1469664;
//    m_cubM2 = m10.selfadjointView<Eigen::Upper>();

//    m10 <<
//        0,  258552, -108864,   31752,       0,       0,  -31752,  108864, -258552,       0,
//        0,  612360,  244944, -108864,  122472,  122472,  -27216,  122472, -612360, -734832,
//        0,       0,  612360,  258552, -612360,  122472,  -27216,  122472,  122472, -734832,
//        0,       0,       0,       0, -258552,  108864,  -31752,       0,       0,       0,
//        0,       0,       0,       0,  612360, -489888,  136080, -122472, -122472,  734832,
//        0,       0,       0,       0,       0,  612360, -231336,  612360, -122472, -734832,
//        0,       0,       0,       0,       0,       0,  308448, -231336,  136080,       0,
//        0,       0,       0,       0,       0,       0,       0,  612360, -489888, -734832,
//        0,       0,       0,       0,       0,       0,       0,       0,  612360,  734832,
//        0,       0,       0,       0,       0,       0,       0,       0,       0, 1469664;
//    m_cubM3 = m10.selfadjointView<Eigen::Upper>();

//    m_cubM1 = perm * m_cubM1 * perm.transpose();
//    m_cubM2 = perm * m_cubM2 * perm.transpose();
//    m_cubM3 = perm * m_cubM3 * perm.transpose();
//    m_cubM1 /= FemScalar(362880); // 4536*80 = 9!
//    m_cubM2 /= FemScalar(362880);
//    m_cubM3 /= FemScalar(362880);
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::processElement(const Face& _elem, FemVector* _v, bool* _orient)
//{
//    static FemMatrixX elemStiffness;
//    elemStiffness.resize(m_inMesh->sizeElement(), m_inMesh->sizeElement());

//    switch(m_inMesh->type())
//    {
//    case FemInMesh::Quadratic:
//        assert(!m_biharmonic);
//        processQuadraticElement(_elem, _v, elemStiffness);
//        break;

//    case FemInMesh::Fraeijs:
//        assert(m_biharmonic);
//        if(!m_inMesh->isFlat(_elem))
//            processFV1Element(_elem, _v, _orient, elemStiffness);
//        else
//            processFV1ElementFlat(_elem, _v, _orient, elemStiffness);
//        break;

//    default:
//        assert(false);
//    }

//    for(size_t layer = 0; layer <= size_t(m_inMesh->isSingular(_elem)); ++layer)
//    {
//        assert(!layer || m_inMesh->isSingular(_elem));

//        Node n[FemInMesh::m_nodePerElement - 1];

//        for(size_t i = 0; i < m_inMesh->sizeElement(); ++i)
//        {
//            n[i] = (!layer || i != m_inMesh->singularVertex(_elem)) ? m_inMesh->node(_elem, i) : m_inMesh->singularNode(_elem);
//            assert(n[i].isValid());
//        }

//        for(size_t i = 0; i < m_inMesh->sizeElement(); ++i)
//        {
//            for(size_t j = 0; j < m_inMesh->sizeElement(); ++j)
//            {
//                addToStiffness(n[i], n[j], elemStiffness(i, j));
//            }
//        }
//    }
//}

//template < class _Mesh, class _ElementBuilder >
//size_t
//FemSolver<_Mesh, _ElementBuilder>::rowFromNode(Node _node)
//{
//    return _node.isUnknown() ? _node.index() : m_stiffnessMatrixTopLeft.rows() + _node.index();
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::addToStiffness(Node _i, Node _j, FemScalar _value)
//{
//    int row = rowFromNode(_i);

//    if(row < m_stiffnessMatrixTopRight.rows())
//    {
//        if(_j.isUnknown())
//            m_stiffnessMatrixTopLeft.coeffRef(row, _j.index()) += _value;
//        else
//            m_stiffnessMatrixTopRight.coeffRef(row, _j.index()) += _value;
//    }
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::processQuadraticElement(const Face& _elem, FemVector* _v, FemMatrixX& _elemStiffness)
//{
//    assert(m_inMesh->type() == FemInMesh::Quadratic);

//    FemScalar inv4A = 1. / (2. * det2(_v[0], _v[1]));

//    assert(inv4A > 0);

//    _elemStiffness = (m_quadM1 * _v[0].squaredNorm() + m_quadM2 * _v[1].squaredNorm() + m_quadM3 * _v[2].squaredNorm()) * inv4A;
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::processFV1Element(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness)
//{
//        assert(m_inMesh->type() == FemInMesh::Fraeijs);

//        FemMatrix1010 m;
//        FemVector p[] =
//    {
//        FemVector::Zero(),
//        _v[2],
//        -_v[1]
//        };

//        FemScalar area = det2(_v[0], _v[1]) / 2.;

//        assert(area > 0);

//        FemVector3 a, b, c, d, l;
//        for(size_t i0 = 0; i0 < 3; ++i0)
//    {
//                size_t i1 = (i0+1)%3;
//                size_t i2 = (i0+2)%3;
//                a(i0) = det2(p[i1], p[i2]);
//                b(i0) = -_v[i0](1);
//                c(i0) = _v[i0](0);
//                d(i0) = .5 - _v[i0].dot(-_v[i2]) / _v[i0].squaredNorm();
//                l(i0) = _v[i0].norm();
//        }

//// TODO : check that debug info
///*#ifndef _DEBUG
//        for(size_t i = 0; i < 3; ++i)
//    {
//                for(size_t j = 0; j < 3; ++j)
//        {
//                        FemScalar v = a(i) + b(i) * p[j](0) + c(i) * p[j](1);
//                        FemScalar r = (i==j)? 2.*area: 0.;
//                        assert(std::abs(v - r) < 1.e-8);
//                }

//                assert(std::abs(l(i) - std::sqrt(b(i)*b(i) + c(i)*c(i))) < 1.e-8);
//        }
//#endif*/

//        FemVector3 dx2[9];
//        FemVector3 dy2[9];
//        FemVector3 dxy[9];

//        for(size_t i = 0; i < 3; ++i)
//    {
//    // 25%:
//                dx2[i+0] = funcVertexBasis(i, Deriv_XX, a, b, c, d, l, area);
//                dx2[i+3] = funcMidpointBasis(i, Deriv_XX, a, b, c, d, l, area);
//                dx2[i+6] = funcMidpointDerivBasis(i, Deriv_XX, a, b, c, d, l, area);
//                dy2[i+0] = funcVertexBasis(i, Deriv_YY, a, b, c, d, l, area);
//                dy2[i+3] = funcMidpointBasis(i, Deriv_YY, a, b, c, d, l, area);
//                dy2[i+6] = funcMidpointDerivBasis(i, Deriv_YY, a, b, c, d, l, area);
//                dxy[i+0] = funcVertexBasis(i, Deriv_XY, a, b, c, d, l, area);
//                dxy[i+3] = funcMidpointBasis(i, Deriv_XY, a, b, c, d, l, area);
//                dxy[i+6] = funcMidpointDerivBasis(i, Deriv_XY, a, b, c, d, l, area);
//        };

//        for(size_t i = 0; i < 9; ++i)
//    {
//                for(size_t j = i; j < 9; ++j)
//        {
//                        EIGEN_ASM_COMMENT("MYBEGIN");

//                        FemVector6 basis = multBasis(dx2[i]+dy2[i], dx2[j]+dy2[j])
//                                                + (1.-m_sigma) * (2. * multBasis(dxy[i], dxy[j])
//                                                                - multBasis(dx2[i], dy2[j])
//                                - multBasis(dy2[i], dx2[j]));

//                        FemScalar value = integrateQuadTriangle(_v, basis, area);

//                        EIGEN_ASM_COMMENT("MYEND");

//                        if((i < 6 || _orient[i%3]) != (j < 6 || _orient[j%3]))
//            {
//                                value *= -1;
//            }

//                        _elemStiffness(i, j) = value;
//                        _elemStiffness(j, i) = value;
//                }
//        }
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::processFV1ElementFlat(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness)
//{
//    assert(m_inMesh->type() == FemInMesh::Fraeijs);

//    FemScalar area = det2(_v[0], _v[1]) / 2.;
//    assert(area > 0);

//    FemScalar sqrtArea = std::sqrt(area);
//    FemScalar ffd = (14.-12.*m_sigma)/(3.*area);
//    FemScalar ffo = (5.-6.*m_sigma)/(3.*area);
//    FemScalar fgd = (12.*m_sigma+4.)/(3.*area);
//    FemScalar fgo = (6.*m_sigma-14.)/(3.*area);
//    FemScalar fhd = -1.754765350603323/sqrtArea;
//    FemScalar fho = (0.43869133765083*(6.*m_sigma-4.))/sqrtArea;
//    FemScalar ggd = (24.*m_sigma+104.)/(3.*area);
//    FemScalar ggo = -(24.*m_sigma+40.)/(3.*area);
//    FemScalar ghd = -(5.264296051809969*m_sigma+8.773826753016614)/sqrtArea;
//    FemScalar gho = 7.019061402413293/sqrtArea;
//    FemScalar hhd = 6.928203230275509;
//    FemScalar hho = 0.;

//    _elemStiffness <<
//        ffd, ffo, ffo,	fgd, fgo, fgo,	fhd, fho, fho,
//        ffo, ffd, ffo,	fgo, fgd, fgo,	fho, fhd, fho,
//        ffo, ffo, ffd,	fgo, fgo, fgd,	fho, fho, fhd,

//        fgd, fgo, fgo,	ggd, ggo, ggo,	ghd, gho, gho,
//        fgo, fgd, fgo,	ggo, ggd, ggo,	gho, ghd, gho,
//        fgo, fgo, fgd,	ggo, ggo, ggd,	gho, gho, ghd,

//        fhd, fho, fho,	ghd, gho, gho,	hhd, hho, hho,
//        fho, fhd, fho,	gho, ghd, gho,	hho, hhd, hho,
//        fho, fho, fhd, 	gho, gho, ghd,	hho, hho, hhd;

//    typedef Eigen::Matrix<FemScalar, 9, 1> FemVector9;

//    static const FemVector9 u8_1((FemVector9()
//            <<  -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,                        0.,
//                 -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                         1.,                       -1.,                        0.).finished());
//    static const FemVector9 u9_1((FemVector9()
//            << -2.659424899780574/sqrtArea,                        0., -0.3799178428258/sqrtArea,
//                -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                        1.,                        0.,                       -1.).finished());
//    static const FemVector9 u7_2((FemVector9()
//            << -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,                        0.,
//               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                     -1.,                          1.,                        0.).finished());
//    static const FemVector9 u9_2((FemVector9()
//            <<                        0., -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,
//               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                      0.,                          1.,                       -1.).finished());
//    static const FemVector9 u7_3((FemVector9()
//            << -0.3799178428258/sqrtArea,                        0., -2.659424899780574/sqrtArea,
//               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
//                                     -1.,                         0,                          1.).finished());
//    static const FemVector9 u8_3((FemVector9()
//            <<                        0., -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,
//               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
//                                      0.,                       -1.,                          1.).finished());

//    if(m_inMesh->flatVertex(_elem) == 0)
//    {
//        _elemStiffness.row(7) = u8_1;
//        _elemStiffness.col(7) = u8_1;
//        _elemStiffness.row(8) = u9_1;
//        _elemStiffness.col(8) = u9_1;
//    }
//    else if(m_inMesh->flatVertex(_elem) == 1)
//    {
//        _elemStiffness.row(6) = u7_2;
//        _elemStiffness.col(6) = u7_2;
//        _elemStiffness.row(8) = u9_2;
//        _elemStiffness.col(8) = u9_2;
//    }
//    else
//    {
//        _elemStiffness.row(6) = u7_3;
//        _elemStiffness.col(6) = u7_3;
//        _elemStiffness.row(7) = u8_3;
//        _elemStiffness.col(7) = u8_3;
//    }

//    // Flip gradient sign where needed.
//    for(size_t i = 0; i < 9; ++i)
//    {
//        for(size_t j = i; j < 9; ++j)
//        {
//            if((i < 6 || _orient[i % 3]) != (j < 6 || _orient[j % 3]))
//            {
//                _elemStiffness(i, j) *= -1;
//                _elemStiffness(j, i) *= -1;
//            }
//        }
//    }
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::updateResult()
//{
//    assert(m_inMesh->nodeList().unknownSize() == (size_t)m_x.rows());
//    for(size_t i = 0; i < m_inMesh->nodeList().unknownSize(); ++i)
//    {
//        m_inMesh->nodeList().setValue(Node(Node::UnknownNode, i), m_x.row(i));
//    }
//}

//template < class _Mesh, class _ElementBuilder >
//void
//FemSolver<_Mesh, _ElementBuilder>::solve()
//{
////TODO : check that debug info
///*#ifndef _DEBUG
//    for (int k = 0; k < m_stiffnessMatrixTopLeft.outerSize(); ++k)
//    {
//        for (FemSMatrixX::InnerIterator it(m_stiffnessMatrixTopLeft, k); it; ++it)
//        {
//            assert(!(it.value() != it.value()));
//        }
//    }
//    for (int k = 0; k < m_stiffnessMatrixTopRight.outerSize(); ++k)
//    {
//        for (FemSMatrixX::InnerIterator it(m_stiffnessMatrixTopRight, k); it; ++it)
//        {
//            assert(!(it.value() != it.value()));
//        }
//    }
//#endif*/

//    multiSolve(m_stiffnessMatrixTopLeft, m_rhs, m_x, m_nbCells);

//    updateResult();

//    m_solved = true;
//}

//template < class _Mesh, class _ElementBuilder >
//template<typename SpMatrix>
//void
//FemSolver<_Mesh, _ElementBuilder>::depthFirstOrdering(const SpMatrix& _mat, Eigen::VectorXi& _p, std::vector<int>& _ranges)
//{
//    int n = _mat.cols();
//    _p.resize(n);
//    _ranges.push_back(0);


//    Eigen::Matrix<bool,Eigen::Dynamic,1> mask(n);
//    mask.fill(true);
//    Eigen::VectorXi stack(n);

//    int r = 1;
//    int count = 1;
//    int k = 0;

//    stack[0] = 0;
//    mask[0] = false;

//    while(count > 0)
//    {
//        int j = stack[--count];
//        _p[k++] = j;

//        for(typename SpMatrix::InnerIterator it(_mat, j); it ;++it)
//        {
//            if(mask[it.index()])
//            {
//                stack[count++] = it.index();
//                mask[it.index()] = false;
//            }
//        }

//        if(count == 0)
//        {
//            while(r < n && !mask[r]) ++r;
//            if(r < n)
//            {
//                _ranges.push_back(k);
//                stack[count++] = r;
//                mask[r] = false;
//            }
//        }
//    }

//    _ranges.push_back(n);
//}

//template < class _Mesh, class _ElementBuilder >
//template<typename SpMatrix, typename Rhs, typename Res>
//void
//FemSolver<_Mesh, _ElementBuilder>::multiSolve(const SpMatrix& _A, const Rhs& _b, Res& _x, unsigned& _nbRanges)
//{
//    typedef typename SpMatrix::Scalar Scalar;
//    typedef Eigen::SparseMatrix<Scalar,Eigen::ColMajor> SparseMatrixType;

//    Eigen::VectorXi pi;
//    std::vector<int> ranges;

//    depthFirstOrdering(_A, pi, ranges);

//    Eigen::PermutationMatrix<Eigen::Dynamic> perm(pi), permInv(perm.inverse());
//    SparseMatrixType mat;
//    mat.template selfadjointView<Eigen::Lower>() = _A.template selfadjointView<Eigen::Lower>().twistedBy(permInv);

//    _x = permInv * _b;

//    _nbRanges = ranges.size()-1;

//    std::cerr << "Split in " << _nbRanges << " ranges:";
//    for(unsigned k=0; k<_nbRanges; ++k)
//    {
//        std::cerr << " " << ranges[k+1]-ranges[k];
//    }
//    std::cerr << "\n";

//#ifdef _OPENMP
//    bool msg = false;
//#pragma omp parallel for schedule(static,1)
//#endif
//    for(unsigned k = 0; k < _nbRanges; ++k)
//    {
//#ifdef _OPENMP
//        if(!msg && omp_get_thread_num() == 0)
//        {
//            std::cerr << "Using OpenMP parallelization: " << omp_get_num_threads() << " threads\n";
//            msg = true;
//        }
//#endif
//        int start = ranges[k];
//        int size  = ranges[k+1]-start;
//        SparseMatrixType L(size,size);
//        L.reserve(size*20);

//        for(int j = 0; j < size; ++j)
//        {
//            L.startVec(j);
//            for(typename SpMatrix::InnerIterator it(mat, start + j); it; ++it)
//            {
//                L.insertBackByOuterInnerUnordered(j, it.index() - start) = it.value();
//            }
//        }

//        L.finalize();
//        Eigen::SimplicialLDLT<SparseMatrixType> ldlt(L);
//        _x.middleRows(start,size) = ldlt.solve(_x.middleRows(start, size));
//    }

//    _x = perm * _x;
//}

template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::build()
{
    unsigned nCoefficients = 0;
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        nCoefficients += m_elementBuilder.nCoefficients(*m_mesh, *elem);
    }

    TripletVector coefficients(nCoefficients);
    TripletVectorIterator it = coefficients.begin();
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        m_elementBuilder.addCoefficients(it, *m_mesh, *elem);
    }
    assert(it == coefficients.end());

    m_stiffnessMatrix.resize(m_mesh->nNodes(), m_mesh->nNodes());
    m_stiffnessMatrix.setFromTriplets(
                coefficients.begin(), coefficients.end());

    std::cout << "Stiffness matrix:\n" << Eigen::MatrixXd(m_stiffnessMatrix) << "\n";

    sort();

    for(int i = 0; i < m_ranges.size()-1; ++i)
        std::cout << "range " << m_ranges[i] << " -> " << m_ranges[i+1] << "\n";
    for(int i = 0; i < m_mesh->nNodes(); ++i)
        std::cout << "perm " << i << " -> " << m_perm[i] << "\n";

    Eigen::PermutationMatrix<Eigen::Dynamic> perm(m_perm);
    StiffnessMatrix sorted(m_mesh->nNodes(), m_mesh->nNodes());
    sorted.template selfadjointView<Eigen::Lower>() =
            m_stiffnessMatrix.template selfadjointView<Eigen::Lower>().twistedBy(perm.inverse());
    std::cout << "Stiffness matrix (sorted):\n"
              << Eigen::MatrixXd(sorted) << "\n";

    ///////////////////////////////////////////////////////////////////////////
    // Old stuff
    /*
    m_biharmonic = _biharmonic;

    int nbUnknownNodes = m_inMesh->nodeList().unknownSize();
    int nbConstraintNodes = m_inMesh->nodeList().constraintSize();
    int nbRows = nbUnknownNodes;

    m_stiffnessMatrixTopLeft.resize(nbRows, nbUnknownNodes);
    m_stiffnessMatrixTopRight.resize(nbRows, nbConstraintNodes);

    Eigen::VectorXi tlSizes(Eigen::VectorXi::Zero(nbUnknownNodes));
    Eigen::VectorXi trSizes(Eigen::VectorXi::Zero(nbConstraintNodes));

    for(Face_iterator elem = m_inMesh->faces_begin(); elem != m_inMesh->faces_end(); ++elem)
    {
        int size = m_inMesh->sizeElement() + m_inMesh->isSingular(*elem);
        for(size_t i = 0; i < m_inMesh->sizeElement(); ++i)
        {
            if(m_inMesh->node(*elem, i).isConstraint())
                trSizes(m_inMesh->node(*elem, i).index()) += size;
            else
                tlSizes(m_inMesh->node(*elem, i).index()) += size;
        }

        if(m_inMesh->isSingular(*elem))
        {
            if(m_inMesh->singularNode(*elem).isConstraint())
                trSizes(m_inMesh->singularNode(*elem).index()) += size;
            else
                tlSizes(m_inMesh->singularNode(*elem).index()) += size;
        }
    }

    m_stiffnessMatrixTopLeft.reserve(tlSizes);
    m_stiffnessMatrixTopRight.reserve(trSizes);

    m_b.resize(nbConstraintNodes, nbChannels);
    m_x.resize(nbUnknownNodes, nbChannels);
    m_rhs.resize(nbRows, nbChannels);

    for(Face_iterator elem = m_inMesh->faces_begin(); elem != m_inMesh->faces_end(); ++elem)
    {
        FemVector v[3];
        bool orient[3];

        size_t i = 0;

        Vertex_around_face_circulator fvit= m_inMesh->vertices(*elem), fvend = fvit;

        do
        {
            Vertex ccwVertex = *(++fvit);
            --fvit;
            Vertex cwVertex = *(--fvit);
            ++fvit;

            //TODO : make it 3d
            surface_mesh::Point p = m_inMesh->position(cwVertex) - m_inMesh->position(ccwVertex);
            v[i] = FemVector(p.x, p.y);
            orient[i] = m_inMesh->compVx(cwVertex, ccwVertex);

            ++fvit;
            ++i;
        }
        while (fvit != fvend);

        processElement(*elem, v, orient);
    }

    for(NodeList::ValueIterator it = m_inMesh->nodeList().constraintsBegin(); it != m_inMesh->nodeList().constraintsEnd(); ++it)
    {
        m_b.row(it - m_inMesh->nodeList().constraintsBegin()) = *it;
    }

    m_rhs = -(m_stiffnessMatrixTopRight * m_b);
    */
}

template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::sort()
{
    int n = m_stiffnessMatrix.cols();
    m_perm.resize(n);

    m_ranges.clear();


    Eigen::Matrix<bool, Eigen::Dynamic, 1> mask(n);
    mask.fill(true);
    Eigen::VectorXi stack(n);

    int sPos = 0;   // search position
    int count = 0;  // nb items in stack
    int nUnk = 0;   // nb found unknown nodes

    while(sPos < n && m_mesh->isConstraint(sPos)) ++sPos;
    if(sPos < n)
    {
        m_ranges.push_back(nUnk);
        stack[count++] = sPos;
        mask[sPos] = false;
    }

    while(count > 0)
    {
        int col = stack[--count];
        m_perm[nUnk++] = col;

        for(typename StiffnessMatrix::InnerIterator it(m_stiffnessMatrix, col); it ;++it)
        {
            if(mask[it.index()] && !m_mesh->isConstraint(it.index()))
            {
                stack[count++] = it.index();
                mask[it.index()] = false;
            }
        }

        if(count == 0)
        {
            while(sPos < n && (!mask[sPos] || m_mesh->isConstraint(sPos))) ++sPos;
            if(sPos < n)
            {
                m_ranges.push_back(nUnk);
                stack[count++] = sPos;
                mask[sPos] = false;
            }
        }
    }

    m_ranges.push_back(nUnk);

    for(int col = 0; col < n; ++col)
    {
        assert(m_mesh->isConstraint(col) == mask[col]);
        if(mask[col])
            m_perm[nUnk++] = col;
    }
}

template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::solve()
{
    // Apply permutation
    Eigen::PermutationMatrix<Eigen::Dynamic> perm(m_perm), permInv(perm.inverse());
    StiffnessMatrix mat;
    mat/*.template selfadjointView<Eigen::Lower>()*/ =
            m_stiffnessMatrix.template selfadjointView<Eigen::Lower>().twistedBy(permInv);

    // compute RHS
    unsigned nUnknowns = m_ranges.back();
    unsigned nConstraints = m_stiffnessMatrix.cols() - nUnknowns;
    Matrix constraints(nConstraints, (unsigned)Mesh::Chan);

    // As sort() keep ordering, constraints will have the right indices.
    unsigned count = 0;
    for(unsigned i = 0; i < m_mesh->nNodes(); ++i)
    {
        if(m_mesh->isConstraint(i))
        {
            constraints.row(count++) =
                    m_mesh->nodeValue(i).template cast<Scalar>();
        }
    }
    assert(count == nConstraints);

    m_x.resize(nUnknowns, Mesh::Chan);
    m_x = mat.topRightCorner(nUnknowns, nConstraints) * constraints;

    std::cerr << "constraints:\n" << constraints << "\n";
    std::cerr << "rhs:\n" << m_x << "\n";

    unsigned nbRanges = m_ranges.size()-1;

    std::cerr << "Split in " << nbRanges << " ranges:";
    for(unsigned k=0; k<nbRanges; ++k)
    {
        std::cerr << " " << m_ranges[k+1] - m_ranges[k];
    }
    std::cerr << "\n";

#ifdef _OPENMP
    bool msg = false;
#pragma omp parallel for schedule(static,1)
#endif
    for(unsigned k = 0; k < nbRanges; ++k)
    {
#ifdef _OPENMP
        if(!msg && omp_get_thread_num() == 0)
        {
            std::cerr << "Using OpenMP parallelization: " << omp_get_num_threads() << " threads\n";
            msg = true;
        }
#endif
        int start = m_ranges[k];
        int size  = m_ranges[k+1]-start;
        StiffnessMatrix L(size,size);
        L.reserve(size*20);

        for(int j = 0; j < size; ++j)
        {
            L.startVec(j);
            for(typename StiffnessMatrix::InnerIterator it(mat, start + j); it; ++it)
            {
                if(it.index() < j || it.index() >= nUnknowns) continue;
                L.insertBackByOuterInnerUnordered(j, it.index() - start) = it.value();
            }
        }

        L.finalize();
        Eigen::SimplicialLDLT<StiffnessMatrix> ldlt(L);
        m_x.middleRows(start,size) = ldlt.solve(m_x.middleRows(start, size));
    }

    std::cout << "Result:\n" << m_x << "\n";

    for(unsigned i = 0; i < nUnknowns; ++i)
    {
        assert(!m_mesh->isConstraint(m_perm[i]));
        m_mesh->nodeValue(m_perm[i]) = -m_x.row(i).
                    template cast<typename Mesh::Scalar>();
    }

    m_solved = true;
//    m_x = Eigen::PermutationMatrix<Eigen::Dynamic>(m_perm.head(nUnknowns)) * m_x;
//    std::cout << "Result (permuted):\n" << m_x << "\n";
}
