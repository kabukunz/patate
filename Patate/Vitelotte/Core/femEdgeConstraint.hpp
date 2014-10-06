
EdgeConstraint::EdgeConstraint() :
    m_tear(false), m_tearGradient(false)
{
        init();
}

EdgeConstraint::EdgeConstraint(const HalfedgeConstraint& _left, const HalfedgeConstraint& _right) :
    m_tear(_left.tear()), m_tearGradient(_left.tearGradient())
{
    assert(_left.tear() == _right.tear() && _left.tearGradient() == _right.tearGradient());
    const HalfedgeConstraint* hc[2] = { &_left, &_right };

    for(size_t i = 0; i < CONSTRAINT_ARRAY_SIZE_2; ++i)
    {
        m_diffuse			    [i]	= hc[i]->diffuse();
        m_contour			    [i]	= hc[i]->contour();
        m_constraintGradient	[i]	= hc[i]->constraintGradient();
    }

    for(size_t i = 0; i < CONSTRAINT_ARRAY_SIZE_4; ++i)
    {
        m_color			[i] = hc[i & Side]->color(	    (i & Vert) >> 1);
        m_gradient		[i] = hc[i & Side]->gradient(	(i & Vert) >> 1);
    }
}

inline bool EdgeConstraint::diffuse(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    return m_diffuse[_i];
}

inline void EdgeConstraint::setDiffuse(size_t _i, bool _diffuse)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    m_diffuse[_i] = _diffuse;
}

inline bool EdgeConstraint::contour(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    return m_contour[_i];
}

inline void EdgeConstraint::setContour(size_t _i, bool _contour)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    m_contour[_i] = _contour;
}

inline const FemColor& EdgeConstraint::color(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_4);
    return m_color[_i];
}

inline void EdgeConstraint::setColor(size_t _i, const FemColor& _color)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_4);
    m_color[_i] = _color;
}

inline bool EdgeConstraint::constraintGradient(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    return m_constraintGradient[_i];
}

inline void EdgeConstraint::setConstraintGradient(size_t _i, bool _constraintGradient)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    m_constraintGradient[_i] = _constraintGradient;
}

inline const FemColor& EdgeConstraint::gradient(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_4);
    return m_gradient[_i];
}

inline void EdgeConstraint::setGradient(size_t _i, const FemColor& _gradient)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_4);
    m_gradient[_i] = _gradient;
}

//   ls       rl        rt       rs
// s ----------- t -> t ----------- s
//   rs       rt        lr       ls
void EdgeConstraint::flip()
{
    std::swap(m_diffuse				[Left],				m_diffuse			    [Right]);
    std::swap(m_color				[Left | Source],	m_color				    [Right | Target]);
    std::swap(m_color				[Left | Target],	m_color				    [Right | Source]);
    std::swap(m_contour				[Left],				m_contour			    [Right]);
    std::swap(m_gradient			[Left | Source],	m_gradient			    [Right | Target]);
    std::swap(m_gradient			[Left | Target],	m_gradient			    [Right | Source]);
    std::swap(m_constraintGradient	[Left],				m_constraintGradient	[Right]);
}

void EdgeConstraint::init()
{
    for(size_t i = 0; i < CONSTRAINT_ARRAY_SIZE_2; ++i)
    {
        m_diffuse[i] = false;
        m_constraintGradient[i] = false;
        m_contour[i] = 0;
    }
}
