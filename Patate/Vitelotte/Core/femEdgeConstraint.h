#ifndef _FEM_EDGE_CONSTRAINT_H_
#define _FEM_EDGE_CONSTRAINT_H_

#include <cassert>
#include "femUtils.h"
#include "femHalfedgeConstraint.h"
#include "defines.h"

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
    EdgeConstraint() :
        m_tear(false), m_tearGradient(false) 
    {
            init();
    }

    EdgeConstraint(const HalfedgeConstraint& _left, const HalfedgeConstraint& _right) :
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

    //HalfedgeConstraint toHalfedge(size_t _side)

    inline bool tear() const { return m_tear; }
    inline void setTear(bool _tear) { m_tear = _tear; }

    inline bool tearGradient() const { return m_tearGradient; }
    inline void setTearGradient(bool _tearGradient) { m_tearGradient = _tearGradient; }

    inline bool diffuse(size_t _i) const
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        return m_diffuse[_i];
    }

    inline void setDiffuse(size_t _i, bool _diffuse)
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        m_diffuse[_i] = _diffuse;
    }

    inline bool contour(size_t _i) const
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        return m_contour[_i];
    }

    inline void setContour(size_t _i, bool _contour)
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        m_contour[_i] = _contour;
    }

    inline const FemColor& color(size_t _i) const
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_4);
        return m_color[_i];
    }

    inline void setColor(size_t _i, const FemColor& _color)
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_4);
        m_color[_i] = _color;
    }

    inline bool constraintGradient(size_t _i) const
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        return m_constraintGradient[_i];
    }

    inline void setConstraintGradient(size_t _i, bool _constraintGradient)
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_2);
        m_constraintGradient[_i] = _constraintGradient;
    }

    inline const FemColor& gradient(size_t _i) const
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_4);
        return m_gradient[_i];
    }

    inline void setGradient(size_t _i, const FemColor& _gradient)
    {
        assert(_i < CONSTRAINT_ARRAY_SIZE_4);
        m_gradient[_i] = _gradient;
    }

//  inline const BezierSegment3& curve() const { return m_curve; }
//  inline void setCurve(const BezierSegment3& _curve) { m_curve = _curve; }

//  void split(FemScalar pos, EdgeConstraint& ec0, EdgeConstraint& ec1) const;
//  void split(FemScalar p0, FemScalar p1, EdgeConstraint& ec) const;

    //   ls       rl        rt       rs
    // s ----------- t -> t ----------- s
    //   rs       rt        lr       ls
    void flip()
    {
        std::swap(m_diffuse				[Left],				m_diffuse			    [Right]);
        std::swap(m_color				[Left | Source],	m_color				    [Right | Target]);
        std::swap(m_color				[Left | Target],	m_color				    [Right | Source]);
        std::swap(m_contour				[Left],				m_contour			    [Right]);
        std::swap(m_gradient			[Left | Source],	m_gradient			    [Right | Target]);
        std::swap(m_gradient			[Left | Target],	m_gradient			    [Right | Source]);
        std::swap(m_constraintGradient	[Left],				m_constraintGradient	[Right]);
    }

private:
    void init()
    {
        for(size_t i = 0; i < CONSTRAINT_ARRAY_SIZE_2; ++i)
        {
            m_diffuse[i] = false;
            m_constraintGradient[i] = false;
            m_contour[i] = 0;
        }
    }

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

#endif