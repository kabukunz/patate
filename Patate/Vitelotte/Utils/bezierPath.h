/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_BEZIER_PATH_
#define _VITELOTTE_BEZIER_PATH_


#include <cassert>
#include <vector>

#include <Eigen/Geometry>


namespace Vitelotte
{


template < typename _Vector >
class BezierPath
{
public:
    typedef _Vector Vector;

    enum SegmentType
    {
        LINEAR    = 2,
        QUADRATIC = 3,
        CUBIC     = 4
    };

public:
    static unsigned size(SegmentType type) { return unsigned(type); }

public:
    inline BezierPath() {}

    inline unsigned nPoints()   const { return m_points.size(); }
    inline unsigned nSegments() const { return m_segments.size(); }

    inline SegmentType type(unsigned si) const { return m_segments.at(si).type; }
    inline unsigned nPoints(unsigned si) const { return size(type(si)); }

    inline const Vector& point(unsigned pi) const { return m_points.at(pi); }
    inline const Vector& point(unsigned si, unsigned pi) const
    {
        assert(si < nSegments() && pi < nPoints(si));
        return m_points.at(m_segments[si].firstPoint + pi);
    }

    inline void setFirstPoint(const Vector& point)
    {
        assert(nPoints() == 0);
        m_points.push_back(point);
    }

    unsigned addSegment(SegmentType type, const Vector* points)
    {
        assert(nPoints() != 0);
        unsigned si = nSegments();
        Segment s;
        s.type       = type;
        s.firstPoint = nPoints() - 1;
        m_segments.push_back(s);

        for(unsigned i = 0; i < nPoints(si) - 1; ++i)
        {
            m_points.push_back(points[i]);
        }

        return si;
    }

private:
    struct Segment {
        SegmentType type;
        unsigned    firstPoint;
    };

    typedef std::vector<Vector>  PointList;
    typedef std::vector<Segment> SegmentList;

private:
    PointList   m_points;
    SegmentList m_segments;
};


}

#endif
