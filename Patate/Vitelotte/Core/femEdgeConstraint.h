#ifndef _FEM_EDGE_CONSTRAINT_H_
#define _FEM_EDGE_CONSTRAINT_H_


#include <cassert>
#include "femUtils.h"
#include "femHalfedgeConstraint.h"
#include "defines.h"


namespace Vitelotte
{


class EdgeConstraint
{
public:
    enum
    {
        Left		= 0,
        Right		= 1,
        Source		= 0,
        Target		= 2,
        Side		= 1,	// side mask
        Vert		= 2,	// vertex mask
        SourceLeft	= Source | Left,
        SourceRight	= Source | Right,
        TargetLeft	= Target | Left,
        TargetRight	= Target | Right
    };

public:
    EdgeConstraint();
    EdgeConstraint(const HalfedgeConstraint& _left, const HalfedgeConstraint& _right);

    //HalfedgeConstraint toHalfedge(size_t _side)

    inline bool tear() const { return m_tear; }
    inline void setTear(bool _tear) { m_tear = _tear; }

    inline bool tearGradient() const { return m_tearGradient; }
    inline void setTearGradient(bool _tearGradient) { m_tearGradient = _tearGradient; }

    inline bool diffuse(size_t _i) const;
    inline void setDiffuse(size_t _i, bool _diffuse);

    inline bool contour(size_t _i) const;
    inline void setContour(size_t _i, bool _contour);

    inline const FemColor& color(size_t _i) const;
    inline void setColor(size_t _i, const FemColor& _color);

    inline bool constraintGradient(size_t _i) const;
    inline void setConstraintGradient(size_t _i, bool _constraintGradient);

    inline const FemColor& gradient(size_t _i) const;
    inline void setGradient(size_t _i, const FemColor& _gradient);

//  inline const BezierSegment3& curve() const { return m_curve; }
//  inline void setCurve(const BezierSegment3& _curve) { m_curve = _curve; }

//  void split(FemScalar pos, EdgeConstraint& ec0, EdgeConstraint& ec1) const;
//  void split(FemScalar p0, FemScalar p1, EdgeConstraint& ec) const;

    //   ls       rl        rt       rs
    // s ----------- t -> t ----------- s
    //   rs       rt        lr       ls
    void flip();

private:
    void init();

private:

    bool m_tear;				    // Do we have different value nodes on both sides ?
    bool m_tearGradient;		    // Do we have different gradient nodes on both sides ?

    bool m_diffuse[CONSTRAINT_ARRAY_SIZE_2];			    // Does the left/right sides diffuse color ?
    FemColor m_color[CONSTRAINT_ARRAY_SIZE_4];		    // Colors of extremities. Left and right colors should be equal if !tear.
    bool m_contour[CONSTRAINT_ARRAY_SIZE_2];	    		// != 0 if the left/right side is a "contour". (only with !tear and !diffuse)

    bool m_constraintGradient[CONSTRAINT_ARRAY_SIZE_2];   // Is the gradient constrained ?
    FemColor m_gradient[CONSTRAINT_ARRAY_SIZE_4];		    // Gradient constraints.

    //BezierSegment3 _curve;
};


#include "femEdgeConstraint.hpp"

} // namespace Vitelotte

#endif
