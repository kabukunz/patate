#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_
#define _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_


#include <vector>

#include <Patate/vitelotte.h>


class VGMeshWithCurves : public Vitelotte::VGMesh<float>
{
public:
    typedef Vitelotte::VGMesh<float> Base;

    typedef Base::Vector Vector;
    typedef Base::NodeValue NodeValue;

    typedef std::map<float, NodeValue> ValueGradient;

    struct Curve : public BaseHandle
    {
        explicit Curve(int _idx = -1) : BaseHandle(_idx) {}
        std::ostream& operator<<(std::ostream& os) const { return os << 'c' << idx(); }
    };

    struct HalfedgeCurveConnectivity
    {
        Curve curve;
        Halfedge next;
        float pos;
    };

    struct CurveInfo
    {
        Halfedge firstHalfedge;
        Halfedge lastHalfedge;
        unsigned flags;
        ValueGradient gradient[4];
    };

    enum
    {
        VALUE_TEAR      = 0x01,
        GRADIENT_TEAR   = 0x02
    };

    enum
    {
        LEFT        = 0x00,
        RIGHT       = 0x01,
        VALUE       = 0x00,
        GRADIENT    = 0x02,

        VALUE_LEFT      = VALUE     | LEFT,
        VALUE_RIGHT     = VALUE     | RIGHT,
        GRADIENT_LEFT   = GRADIENT  | LEFT,
        GRADIENT_RIGHT  = GRADIENT  | RIGHT
    };

public:
    VGMeshWithCurves();

    inline Curve  curve(Halfedge h) const { return m_halfedgeCurveConn[h].curve; }
    inline Curve& curve(Halfedge h)       { return m_halfedgeCurveConn[h].curve; }

    inline float  curvePos(Halfedge h) const { return m_halfedgeCurveConn[h].pos; }
    inline float& curvePos(Halfedge h)       { return m_halfedgeCurveConn[h].pos; }

    inline Halfedge  nextCurveHalfedge(Halfedge h) const { return m_halfedgeCurveConn[h].next; }
    inline Halfedge& nextCurveHalfedge(Halfedge h)       { return m_halfedgeCurveConn[h].next; }

    inline unsigned nCurves() const { return m_curves.size(); }
    Curve addCurve(unsigned flags);

    using Base::isValid;
    inline bool isValid(Curve c) const { return c.isValid() && c.idx() < nCurves(); }

    void addHalfedgeToCurve(Curve c, Halfedge h, float from, float to);

    inline Halfedge  firstHalfedge(Curve c) const { return m_curves[c.idx()].firstHalfedge; }
    inline Halfedge& firstHalfedge(Curve c)       { return m_curves[c.idx()].firstHalfedge; }
    inline Halfedge   lastHalfedge(Curve c) const { return m_curves[c.idx()]. lastHalfedge; }
    inline Halfedge&  lastHalfedge(Curve c)       { return m_curves[c.idx()]. lastHalfedge; }

    inline bool valueTear(Curve c)    const { return m_curves[c.idx()].flags & VALUE_TEAR;    }
    inline bool gradientTear(Curve c) const { return m_curves[c.idx()].flags & GRADIENT_TEAR; }

    inline unsigned flags(Curve c) const { return m_curves[c.idx()].flags; }
    void setFlags(Curve c, unsigned flags);

    const ValueGradient& valueGradient(Curve c, unsigned which) const;
          ValueGradient& valueGradient(Curve c, unsigned which);

private:
    Base::HalfedgeProperty<HalfedgeCurveConnectivity> m_halfedgeCurveConn;

    std::vector<CurveInfo> m_curves;

};


#endif
