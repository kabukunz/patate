
HalfedgeConstraint::HalfedgeConstraint() :
    m_tear(false), m_tearGradient(false),
    m_diffuse(false), m_contour(false), m_constraintGradient(false)
{}

inline const FemColor& HalfedgeConstraint::color(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    return m_color[_i];
}

inline void HalfedgeConstraint::setColor(size_t _i, const FemColor& _color)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    m_color[_i] = _color;
}

inline const FemColor& HalfedgeConstraint::gradient(size_t _i) const
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    return m_gradient[_i];
}

inline void HalfedgeConstraint::setGradient(size_t _i, const FemColor& _gradient)
{
    assert(_i < CONSTRAINT_ARRAY_SIZE_2);
    m_gradient[_i] = _gradient;
}
