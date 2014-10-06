#ifndef _FEM_HALFEDGE_CONSTRAINT_H_
#define _FEM_HALFEDGE_CONSTRAINT_H_

//#include "femEdgeConstraint.h"
#include <cassert>
#include "femUtils.h"
#include "defines.h"


namespace Vitelotte
{


class HalfedgeConstraint
{
public:
    enum
    {
        Source,
        Target
    };

public:
    HalfedgeConstraint();

    /*HalfedgeConstraint(const EdgeConstraint& _ec, size_t _side) :
        m_tear(_ec.tear()), m_tearGradient(_ec.tearGradient()),
        m_diffuse(_ec.diffuse(_side)), m_contour(_ec.contour(_side)), m_constraintGradient(_ec.constraintGradient(_side))
    {
        for(size_t i = 0; i < 2; ++i)
        {
            m_color[i]		= _ec.color		(_side | ((i==0) ? Source : Target));
            m_gradient[i]   = _ec.gradient	(_side | ((i==0) ? Source : Target));
        }
    }*/

    inline bool tear() const { return m_tear; }
    inline void setTear(bool _tear) { m_tear = _tear; }

    inline bool tearGradient() const { return m_tearGradient; }
    inline void setTearGradient(bool _tearGradient) { m_tearGradient = _tearGradient; }

    inline bool diffuse() const { return m_diffuse; }
    inline void setDiffuse(bool _diffuse) { m_diffuse = _diffuse; }

    inline bool contour() const { return m_contour; }
    inline void setContour(bool _contour) { m_contour = _contour; }
    inline const FemColor& color(size_t _i) const;

    inline void setColor(size_t _i, const FemColor& _color);

    inline bool constraintGradient() const { return m_constraintGradient; }
    inline void setConstraintGradient(bool _constraintGradient) { m_constraintGradient = _constraintGradient; }

    inline const FemColor& gradient(size_t _i) const;
    inline void setGradient(size_t _i, const FemColor& _gradient);

private:

    bool m_tear;				            // Do we have different value nodes on both sides ?
    bool m_tearGradient;		            // Do we have different gradient nodes on both sides ?

    bool m_diffuse;			                // Does the left/right sides diffuse color ?
    FemColor m_color[CONSTRAINT_ARRAY_SIZE_2];		// Colors of extremities. Left and right colors should be equal if !tear.
    bool m_contour;			                // != 0 if the left/right side is a "contour". (only with !tear and !diffuse)

    bool m_constraintGradient;              // Is the gradient constrained ?
    FemColor m_gradient[CONSTRAINT_ARRAY_SIZE_2];	// Gradient constraints.
};


#include "femHalfedgeConstraint.hpp"

} // namespace Vitelotte

#endif
