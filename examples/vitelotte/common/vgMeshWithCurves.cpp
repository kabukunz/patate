#include "vgMeshWithCurves.h"


VGMeshWithCurves::VGMeshWithCurves()
{

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

    CurveInfo ci = m_curves[c.idx()];
    if(flags & VALUE_TEAR && !(ci.flags & VALUE_TEAR))
        ci.gradient[VALUE_RIGHT] = ci.gradient[VALUE_LEFT];
    if(flags & GRADIENT_TEAR && !(ci.flags & GRADIENT_TEAR))
        ci.gradient[GRADIENT_RIGHT] = ci.gradient[GRADIENT_LEFT];
    ci.flags = flags;
}


const VGMeshWithCurves::ValueGradient&
VGMeshWithCurves::valueGradient(Curve c, unsigned which) const
{
    return const_cast<VGMeshWithCurves*>(this)->valueGradient(c, which);
}


VGMeshWithCurves::ValueGradient&
VGMeshWithCurves::valueGradient(Curve c, unsigned which)
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
