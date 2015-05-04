#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_
#define _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_


#include <vector>

#include <Patate/vitelotte.h>

#include "bezierCurve.h"


template <typename _Value>
class PicewiseLinearFunction
{
public:
    typedef _Value Value;

private:
    typedef std::map<float, Value> Samples;

public:
    typedef typename Samples::iterator Iterator;
    typedef typename Samples::const_iterator ConstIterator;

public:
    PicewiseLinearFunction() {}

    bool     empty() const { return m_samples.empty(); }
    unsigned size()  const { return m_samples.size(); }

    bool         has   (float x) const { return m_samples.count(x); }
          Value& sample(float x)       { return m_samples.at(x); }
    const Value& sample(float x) const { return m_samples.at(x); }

    void add(float x, const Value& v) { m_samples.insert(std::make_pair(x, v)); }
    void remove(float x) { m_samples.erase(x); }
    void clear() { m_samples.clear(); }

    Iterator      begin()       { return m_samples.begin(); }
    ConstIterator begin() const { return m_samples.begin(); }
    Iterator      end()         { return m_samples.end(); }
    ConstIterator end()   const { return m_samples.end(); }

    Value operator()(float x) const;

private:
    Samples m_samples;
};


class VGMeshWithCurves : public Vitelotte::VGMesh<float, Vitelotte::Dynamic, Vitelotte::Dynamic>
{
public:
    typedef Vitelotte::VGMesh<float, Vitelotte::Dynamic, Vitelotte::Dynamic> Base;

    typedef Base::Scalar Scalar;
    typedef Base::Vector Vector;
    typedef Base::Value Value;
    typedef Base::Gradient Gradient;

    typedef PicewiseLinearFunction<Value> ValueFunction;

    typedef ::BezierCurve<Vector> BezierCurve;

    typedef Gradient::ConstantReturnType UnconstrainedGradientType;

    struct Curve : public BaseHandle
    {
        explicit Curve(int _idx = -1) : BaseHandle(_idx) {}
        std::ostream& operator<<(std::ostream& os) const { return os << 'c' << idx(); }
    };

    struct PointConstraint : public BaseHandle
    {
        explicit PointConstraint(int _idx = -1) : BaseHandle(_idx) {}
        std::ostream& operator<<(std::ostream& os) const { return os << "pc" << idx(); }
    };

    struct HalfedgeCurveConnectivity
    {
        Curve     curve;
        Halfedge  next;
        float     pos;
    };

    struct CurveInfo
    {
        Halfedge       firstHalfedge;
        Halfedge       lastHalfedge;
        unsigned       flags;
        ValueFunction  gradient[4];
        BezierCurve    bezierCurve;
    };

    struct PointConstraintInfo
    {
        Vertex    vertex;
        Value     value;
        Gradient  gradient;
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
    VGMeshWithCurves(unsigned nDims, unsigned nCoeffs);
    VGMeshWithCurves(const VGMeshWithCurves& other);

    VGMeshWithCurves& operator=(const VGMeshWithCurves& other);
    VGMeshWithCurves& assign(const VGMeshWithCurves& other);

    inline Curve  curve(Halfedge h) const { return m_halfedgeCurveConn[h].curve; }
    inline Curve& curve(Halfedge h)       { return m_halfedgeCurveConn[h].curve; }

    inline float  fromCurvePos(Halfedge h) const
        { return m_halfedgeCurveConn[oppositeHalfedge(h)].pos; }
    inline float& fromCurvePos(Halfedge h)
        { return m_halfedgeCurveConn[oppositeHalfedge(h)].pos; }
    inline float  toCurvePos(Halfedge h) const { return m_halfedgeCurveConn[h].pos; }
    inline float& toCurvePos(Halfedge h)       { return m_halfedgeCurveConn[h].pos; }

    inline Halfedge  nextCurveHalfedge(Halfedge h) const { return m_halfedgeCurveConn[h].next; }
    inline Halfedge& nextCurveHalfedge(Halfedge h)       { return m_halfedgeCurveConn[h].next; }
    inline Halfedge  prevCurveHalfedge(Halfedge h) const
        { return m_halfedgeCurveConn[oppositeHalfedge(h)].next; }
    inline Halfedge& prevCurveHalfedge(Halfedge h)
        { return m_halfedgeCurveConn[oppositeHalfedge(h)].next; }

    inline unsigned nCurves() const { return m_curves.size(); }
    Curve addCurve(unsigned flags);

    using Base::isValid;
    inline bool isValid(PointConstraint pc) const
        { return pc.isValid() && unsigned(pc.idx()) < nPointConstraints(); }
    inline bool isValid(Curve c) const { return c.isValid() && unsigned(c.idx()) < nCurves(); }

    void addHalfedgeToCurve(Curve c, Halfedge h, float from, float to);

    inline Halfedge  firstHalfedge(Curve c) const { return m_curves.at(c.idx()).firstHalfedge; }
    inline Halfedge& firstHalfedge(Curve c)       { return m_curves.at(c.idx()).firstHalfedge; }
    inline Halfedge   lastHalfedge(Curve c) const { return m_curves.at(c.idx()). lastHalfedge; }
    inline Halfedge&  lastHalfedge(Curve c)       { return m_curves.at(c.idx()). lastHalfedge; }

    inline bool valueTear(Curve c)    const { return m_curves.at(c.idx()).flags & VALUE_TEAR;    }
    inline bool gradientTear(Curve c) const { return m_curves.at(c.idx()).flags & GRADIENT_TEAR; }

    inline unsigned flags(Curve c) const { return m_curves.at(c.idx()).flags; }
    void setFlags(Curve c, unsigned flags);
    void setFlagsRaw(Curve c, unsigned flags);

    const ValueFunction& valueFunction(Curve c, unsigned which) const;
          ValueFunction& valueFunction(Curve c, unsigned which);
    const ValueFunction& valueFunctionRaw(Curve c, unsigned which) const;
          ValueFunction& valueFunctionRaw(Curve c, unsigned which);

    const BezierCurve& bezierCurve(Curve c) const { return m_curves.at(c.idx()).bezierCurve; }
          BezierCurve& bezierCurve(Curve c)       { return m_curves.at(c.idx()).bezierCurve; }

//    inline PointConstraint  pointConstraint(Vertex v) const
//      { return m_pointConstraintConn[v]; }
//    inline PointConstraint& pointConstraint(Vertex v)
//      { return m_pointConstraintConn[v]; }

    using Base::vertex;
    inline Vertex vertex(PointConstraint pc) const { return m_pointConstraints[pc.idx()].vertex; }
    inline void setVertex(PointConstraint pc, Vertex vx);

    inline PointConstraint pointConstraint(Vertex vx) { return m_pointConstraintConn[vx]; }

    inline bool isValueConstraint(PointConstraint pc) const
        { return !isnan(m_pointConstraints[pc.idx()].value(0)); }
    using Base::isGradientConstraint;
    inline bool isGradientConstraint(PointConstraint pc) const
        { return !isnan(m_pointConstraints[pc.idx()].gradient(0, 0)); }

    using Base::value;
    inline const Value& value(PointConstraint pc) const
        { return m_pointConstraints[pc.idx()].value; }
    inline       Value& value(PointConstraint pc)
        { return m_pointConstraints[pc.idx()].value; }

    inline const Gradient& gradient(PointConstraint pc) const
        { return m_pointConstraints[pc.idx()].gradient; }
    inline       Gradient& gradient(PointConstraint pc)
        { return m_pointConstraints[pc.idx()].gradient; }

    inline unsigned nPointConstraints() const { return m_pointConstraints.size(); }
    PointConstraint addPointConstraint(Vertex vx);


    void clear();
    void setNodesFromCurves();

    inline UnconstrainedGradientType unconstrainedGradientValue() const
    { return Gradient::Constant(nCoeffs(), nDims(), std::numeric_limits<Scalar>::quiet_NaN()); }

    Value evalValueFunction(Curve c, unsigned which, float pos) const;


protected:
    void copyVGMeshWithCurvesMembers(const VGMeshWithCurves& other);

    typedef Node NodePair[2];
    void addGradientNodes(Node nodes[2], Curve c, unsigned gType, float pos);

protected:
    Base::VertexProperty<PointConstraint> m_pointConstraintConn;
    Base::HalfedgeProperty<HalfedgeCurveConnectivity> m_halfedgeCurveConn;

    std::vector<PointConstraintInfo> m_pointConstraints;
    std::vector<CurveInfo> m_curves;

};


#endif