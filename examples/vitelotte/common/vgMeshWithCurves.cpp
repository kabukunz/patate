#include "vgMeshWithCurves.h"



template <typename _Value>
typename PicewiseLinearFunction<_Value>::Value
    PicewiseLinearFunction<_Value>::operator()(float x) const
{
    assert(!empty());

    if(x <= m_samples.begin()->first)
        return m_samples.begin()->second;
    if(x >= m_samples.rbegin()->first)
        return m_samples.rbegin()->second;

    ConstIterator next = m_samples.upper_bound(x);
    ConstIterator prev = next;
    --next;

    float alpha = (x - prev->first) / (next->first - prev->first);
    return (1 - alpha) * prev->second + alpha * next->second;
}


VGMeshWithCurves::VGMeshWithCurves()
{
    m_halfedgeCurveConn = addHalfedgeProperty<HalfedgeCurveConnectivity>("h:curveConnectivity");
}


VGMeshWithCurves::VGMeshWithCurves(unsigned nDims, unsigned nCoeffs)
    : Base(nDims, nCoeffs, 0)
{
    m_halfedgeCurveConn = addHalfedgeProperty<HalfedgeCurveConnectivity>("h:curveConnectivity");
}


VGMeshWithCurves::VGMeshWithCurves(const VGMeshWithCurves& other)
    : Base(other)
{
    copyVGMeshWithCurvesMembers(other);
}


VGMeshWithCurves& VGMeshWithCurves::operator=(const VGMeshWithCurves& other)
{
    if(&other == this) return *this;

    Base::operator=(other);
    copyVGMeshWithCurvesMembers(other);

    return *this;
}


VGMeshWithCurves& VGMeshWithCurves::assign(const VGMeshWithCurves& other)
{
    if(&other == this) return *this;

    Base::assign(other);
    copyVGMeshWithCurvesMembers(other);

    return *this;
}


VGMeshWithCurves::Curve VGMeshWithCurves::addCurve(unsigned flags)
{
    Curve c(nCurves());
    CurveInfo ci;
    ci.flags = flags;
    m_curves.push_back(ci);
    return c;
}


void VGMeshWithCurves::addHalfedgeToCurve(Curve c, Halfedge h, float from, float to)
{
    assert(isValid(c));
    assert(isValid(h));

    CurveInfo& ci = m_curves[c.idx()];
    HalfedgeCurveConnectivity& hcc = m_halfedgeCurveConn[h];
    HalfedgeCurveConnectivity& ohcc = m_halfedgeCurveConn[oppositeHalfedge(h)];

    assert(!hcc.curve.isValid());
    assert(!ohcc.curve.isValid());

    hcc.curve = c;
    hcc.pos = to;
    ohcc.curve = c;
    ohcc.pos = from;
    if(ci.firstHalfedge.isValid())
    {
        assert(ci.lastHalfedge.isValid());
        HalfedgeCurveConnectivity& phcc = m_halfedgeCurveConn[ci.lastHalfedge];

        phcc.next = h;
        ohcc.next = ci.lastHalfedge;
        ci.lastHalfedge = h;
    }
    else
    {
        ci.firstHalfedge = h;
        ci.lastHalfedge = h;
    }
}


void VGMeshWithCurves::setFlags(Curve c, unsigned flags)
{
    assert(isValid(c));

    CurveInfo& ci = m_curves[c.idx()];
    if(flags & VALUE_TEAR && !(ci.flags & VALUE_TEAR))
        ci.gradient[VALUE_RIGHT] = ci.gradient[VALUE_LEFT];
    if(flags & GRADIENT_TEAR && !(ci.flags & GRADIENT_TEAR))
        ci.gradient[GRADIENT_RIGHT] = ci.gradient[GRADIENT_LEFT];
    ci.flags = flags;
}


void VGMeshWithCurves::setFlagsRaw(Curve c, unsigned flags)
{
    assert(isValid(c));

    CurveInfo& ci = m_curves[c.idx()];
    ci.flags = flags;
}


const VGMeshWithCurves::ValueFunction&
VGMeshWithCurves::valueFunction(Curve c, unsigned which) const
{
    return const_cast<VGMeshWithCurves*>(this)->valueFunction(c, which);
}


VGMeshWithCurves::ValueFunction&
VGMeshWithCurves::valueFunction(Curve c, unsigned which)
{
    assert(isValid(c));
    assert(which < 4);

    switch(which)
    {
    case VALUE_RIGHT:
        if(!valueTear(c))
            which = VALUE_LEFT;
        break;
    case GRADIENT_RIGHT:
        if(!gradientTear(c))
            which = GRADIENT_LEFT;
        break;
    }

    return m_curves[c.idx()].gradient[which];
}


const VGMeshWithCurves::ValueFunction&
VGMeshWithCurves::valueFunctionRaw(Curve c, unsigned which) const
{
    return const_cast<VGMeshWithCurves*>(this)->valueFunctionRaw(c, which);
}


VGMeshWithCurves::ValueFunction&
VGMeshWithCurves::valueFunctionRaw(Curve c, unsigned which)
{
    assert(isValid(c));
    assert(which < 4);

    return m_curves[c.idx()].gradient[which];
}


VGMeshWithCurves::PointConstraint VGMeshWithCurves::addPointConstraint()
{
    PointConstraint pc(nPointConstraints());
    PointConstraintInfo pci;
    pci.value = unconstrainedValue();
    pci.gradient = unconstrainedGradientValue();
    m_pointConstraints.push_back(pci);
    return pc;
}


void VGMeshWithCurves::clear()
{
    Base::clear();
    m_pointConstraints.clear();
    m_curves.clear();
}


void VGMeshWithCurves::setNodesFromCurves()
{
    m_nprops.resize(0);
    for(HalfedgeIterator hit = halfedgesBegin();
        hit != halfedgesEnd(); ++hit)
    {
        if(hasFromVertexValue())    fromVertexValueNode(*hit)   = Node();
        if(hasToVertexValue())      toVertexValueNode(*hit)     = Node();
        if(hasEdgeValue())          edgeValueNode(*hit)         = Node();
        if(hasEdgeGradient())       edgeGradientNode(*hit)      = Node();
    }
    for(VertexIterator vit = verticesBegin();
        vit != verticesEnd(); ++vit)
    {
        if(isGradientConstraint(*vit))
        {
            removeGradientConstraint(*vit);
        }
    }

    for(unsigned pci = 0; pci < nPointConstraints(); ++pci)
    {
        PointConstraint pc(pci);
        Vertex vx = vertex(pc);
        assert(isValid(vx));

        Node vn;
        if(isValueConstraint(pc))
            vn = addNode(value(pc));

        HalfedgeAroundVertexCirculator hit = halfedges(vx);
        HalfedgeAroundVertexCirculator hend = hit;
        do {
            Halfedge opp = oppositeHalfedge(*hit);
            if(isValueConstraint(pc))
            {
                if(!isBoundary(*hit))
                    halfedgeNode(*hit, FROM_VERTEX_VALUE) = vn;
                if(!isBoundary(opp))
                    halfedgeOppositeNode(*hit, FROM_VERTEX_VALUE) = vn;
            }
            ++hit;
        } while(hit != hend);

        if(isGradientConstraint(pc))
        {
            setGradientConstraint(vertex(pc), gradient(pc));
        }
    }

    for(unsigned ci = 0; ci < nCurves(); ++ci)
    {
        Curve c(ci);

        Halfedge lh = firstHalfedge(c);
        if(!lh.isValid())
            continue;

        Node fromNode[2];
        addGradientNodes(fromNode, c, VALUE, fromCurvePos(lh));
        do {
            Node toNode[2];
            addGradientNodes(toNode, c, VALUE, toCurvePos(lh));

            Halfedge rh = oppositeHalfedge(lh);
            float midPos = (fromCurvePos(lh) + toCurvePos(lh)) / 2.f;

            bool lhnb = !isBoundary(lh);
            bool rhnb = !isBoundary(rh);

            if(hasFromVertexValue())
            {
                if(lhnb) fromVertexValueNode(lh) = fromNode[LEFT];
                if(rhnb) fromVertexValueNode(rh) =   toNode[RIGHT];
            }
            if(hasToVertexValue())
            {
                if(lhnb) toVertexValueNode(lh) =   toNode[LEFT];
                if(rhnb) toVertexValueNode(rh) = fromNode[RIGHT];
            }

            if(hasEdgeValue())
            {
                Node midNode[2];
                addGradientNodes(midNode, c, VALUE, midPos);
                if(lhnb) edgeValueNode(lh) = midNode[LEFT];
                if(rhnb) edgeValueNode(rh) = midNode[RIGHT];
            }

            if(hasEdgeGradient())
            {
                Node gNode[2];
                addGradientNodes(gNode, c, GRADIENT, midPos);
                if(lhnb) edgeGradientNode(lh) = gNode[LEFT];
                if(rhnb) edgeGradientNode(rh) = gNode[RIGHT];

                if(halfedgeOrientation(lh) && lhnb)
                    value(gNode[LEFT]) *= -1;
                if(gNode[LEFT] != gNode[RIGHT] && halfedgeOrientation(lh) && rhnb)
                    value(gNode[RIGHT]) *= -1;

            }

            fromNode[0] = toNode[0];
            fromNode[1] = toNode[1];
            lh = nextCurveHalfedge(lh);
        } while(lh.isValid());
    }

//    if(flags_ & FLAT_BOUNDARY)
//    {
//        Halfedge h;
//        for(EdgeIterator eit = edgesBegin();
//            eit != edgesEnd(); ++eit)
//        {
//            if(!isBoundary(*eit))
//                continue;
//            for(int i = 0; i < 2; ++i)
//            {
//                h = halfedge(*eit, i);
//                if(!isBoundary(h))
//                    edgeGradientNode(h) = addNode(Value::Constant(0));
//            }
//        }
//    }
}


VGMeshWithCurves::Value VGMeshWithCurves::evalValueFunction(
        Curve c, unsigned which, float pos) const
{
    return valueFunction(c, which)(pos);
}


void VGMeshWithCurves::copyVGMeshWithCurvesMembers(const VGMeshWithCurves& other)
{
    m_halfedgeCurveConn = halfedgeProperty<HalfedgeCurveConnectivity>("h:curveConnectivity");

    m_pointConstraints = other.m_pointConstraints;
    m_curves = other.m_curves;
}


void VGMeshWithCurves::addGradientNodes(
        Node nodes[2], Curve c, unsigned gType, float pos)
{
    bool tear = (gType == VALUE)? valueTear(c): gradientTear(c);

    nodes[LEFT] = valueFunction(c, gType | LEFT).empty()?
                addNode():
                addNode(evalValueFunction(c, gType | LEFT, pos));
    nodes[RIGHT] =
            (!tear)?
                nodes[LEFT]:
                valueFunction(c, gType | RIGHT).empty()?
                            addNode():
                            addNode(evalValueFunction(c, gType | RIGHT, pos));
}
