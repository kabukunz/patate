#ifndef _FEM_VERTEX_CONSTRAINT_H_
#define _FEM_VERTEX_CONSTRAINT_H_

#include "femUtils.h"

class VertexConstraint
{
public:
    inline VertexConstraint() : m_diffuse(false)/*, m_color(0., 0., 0., 1.)*/, m_constraintGradient(false)
    {}

    inline bool diffuse() const { return m_diffuse; }
    inline void setDiffuse(bool _diffuse) { m_diffuse = _diffuse; }

    inline const FemColor& color() const { return m_color; }
    inline void setColor(const FemColor& _color) { m_color = _color; }

    inline bool constraintGradient() const { return m_constraintGradient; }
    inline void setConstraintGradient(bool _constraintGradient) { m_constraintGradient = _constraintGradient; }

    inline const FemColor& gradient() const { return m_gradient; }
    inline void setGradient(const FemColor& _gradient) { m_gradient = _gradient; }

private:
    bool m_diffuse;
    FemColor m_color;

    bool m_constraintGradient;
    FemColor m_gradient;
};

#endif