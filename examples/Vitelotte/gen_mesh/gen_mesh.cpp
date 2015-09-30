#include <fstream>
#include <iostream>
#include <vector>
#include <deque>
#include <iomanip>
#include <cmath>

#include <Patate/vitelotte.h>


typedef double REAL;

#include "triangle.h"

#include "pugixml.hpp"


using namespace Vitelotte;


typedef DCMesh<float, 2, 4> Mesh;

typedef Mesh::Scalar Scalar;
typedef Mesh::Vector Vector;
typedef Mesh::Value  Value;

typedef Mesh::Vertex Vertex;
typedef Mesh::Halfedge Halfedge;
typedef Mesh::Edge Edge;
typedef Mesh::Face Face;
typedef Mesh::PointConstraint PointConstraint;
typedef Mesh::Curve Curve;

typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

typedef BezierSegment<Vector> BSegment;

typedef std::vector<Vector, Eigen::aligned_allocator<Vector> > VectorVector;


Scalar lerp(Scalar a, Scalar from, Scalar to) {
    return (1-a) * from + a * to;
}

Scalar ilerp(const Vector& p, const Vector& p0, const Vector& p1) {
    Vector u = p1 - p0;
    Vector v = p - p0;
    return u.dot(v) / u.squaredNorm();
}


void freeTri(triangulateio* tri) {
    trifree((int*)tri->pointlist);
    trifree((int*)tri->pointattributelist);
    trifree((int*)tri->pointmarkerlist);
    trifree((int*)tri->trianglelist);
    trifree((int*)tri->triangleattributelist);
    trifree((int*)tri->trianglearealist);
    trifree((int*)tri->neighborlist);
    trifree((int*)tri->segmentlist);
    trifree((int*)tri->segmentmarkerlist);
    trifree((int*)tri->holelist);
    trifree((int*)tri->regionlist);
    trifree((int*)tri->edgelist);
    trifree((int*)tri->edgemarkerlist);
    trifree((int*)tri->normlist);
}


class BezierArrangement {
public:
    BezierArrangement() {
    }

    unsigned nPoints() const { return _points.size(); }
    unsigned nSegments() const { return _segments.size(); }
    unsigned nPointConstraints() const { return _pcs.size(); }

    const Vector& point(unsigned i) const { return _points.at(i); }
//    Vector& point(unsigned i) { return _points.at(i); }

    unsigned segmentFrom(unsigned i) const { return _segments.at(i).p0; }
    unsigned segmentTo(unsigned i) const { return _segments.at(i).p1; }

    BSegment segment(unsigned i) const {
        const Segment& c = _segments.at(i);

        BSegment bs;
        bs.setType(c.type);
        bs.point(0) = point(c.p0);
        bs.point(c.type - 1) = point(c.p1);

        switch(c.type) {
        case BEZIER_CUBIC:
            bs.point(2) = c.pb;
            // fall-through
        case BEZIER_QUADRATIC:
            bs.point(1) = c.pa;
            break;
        default:
            break;
        }

        return bs;
    }

    unsigned pointConstraint(unsigned i) const { return _pcs.at(i); }

    unsigned addPoint(const Vector& p) {
        _points.push_back(p);
        return _points.size() - 1;
    }

    unsigned addSegment(unsigned p0, unsigned p1) {
        _segments.push_back(Segment(p0, p1));
        return _segments.size() - 1;
    }

    unsigned addSegment(unsigned p0, unsigned p1, const Vector& pa) {
        _segments.push_back(Segment(p0, p1, pa));
        return _segments.size() - 1;
    }

    unsigned addSegment(unsigned p0, unsigned p1, const Vector& pa, const Vector& pb) {
        _segments.push_back(Segment(p0, p1, pa, pb));
        return _segments.size() - 1;
    }

    unsigned addPointConstraint(unsigned pi) {
        _pcs.push_back(pi);
        return _pcs.size() - 1;
    }


private:
    typedef VectorVector PointList;

    struct Segment {
        unsigned p0;
        unsigned p1;
        BezierSegmentType type;
        Vector pa;
        Vector pb;

        Segment(unsigned p0, unsigned p1) : p0(p0), p1(p1), type(BEZIER_LINEAR) {}
        Segment(unsigned p0, unsigned p1, const Vector& pa)
            : p0(p0), p1(p1), type(BEZIER_QUADRATIC), pa(pa) {}
        Segment(unsigned p0, unsigned p1, const Vector& pa, const Vector& pb)
            : p0(p0), p1(p1), type(BEZIER_CUBIC), pa(pa), pb(pb) {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<Segment, Eigen::aligned_allocator<Segment> > SegmentList;
    typedef std::vector<unsigned> PCList;

private:
    PointList   _points;
    SegmentList _segments;
    PCList      _pcs;
};


struct SvgWalker : public pugi::xml_tree_walker {
    typedef Eigen::Transform<Scalar, 2, Eigen::Affine> Transform;
    typedef std::vector<Transform, Eigen::aligned_allocator<Transform> > TransformStack;

    SvgWalker(BezierArrangement& arr, Scalar snapDist)
        : _arr(arr),
          _snapDist(snapDist),
          _trans() {
        _trans.push_back(Transform::Identity());
        _trans.back().scale(Vector(1, -1));
    }

    unsigned addPoint(const Vector& p) {
        Vector p2 = _trans.back() * p;
        if(_snapDist != 0) {
            Scalar d2 = _snapDist * _snapDist;
            for(unsigned i = 0; i < _arr.nPoints(); ++i) {
                if((_arr.point(i) - p2).squaredNorm() < d2) {
                    return i;
                }
            }
        }
        unsigned i = _arr.addPoint(p2);
//        std::cout << "Add point " << i << ": " << p2.transpose() << "\n";
        return i;
    }

    unsigned addSegment(unsigned p0, unsigned p1) {
//        unsigned i0 = (p0 < p1)? p0: p1;
//        unsigned i1 = (p0 < p1)? p1: p0;
//        for(unsigned i = 0; i < _arr.nSegments(); ++i) {
//            unsigned s0 = _arr.segmentFrom(i);
//            unsigned s1 = _arr.segmentTo(i);
//            if(s1 < s0) std::swap(s0, s1);
//            if(i0 == s0 && i1 == s1) {
//                return i;
//            }
//        }
//        std::cout << "Add segment " << p0 << ", " << p1 << "\n";
        return _arr.addSegment(p0, p1);
    }

    unsigned addSegment(unsigned p0, unsigned p1, const Vector& pa) {
//        std::cout << "Add segment " << p0 << ", " << p1
//                  << ", " << (_trans.back() * pa).transpose() << "\n";
        return _arr.addSegment(p0, p1, _trans.back() * pa);
    }

    unsigned addSegment(unsigned p0, unsigned p1, const Vector& pa, const Vector& pb) {
//        std::cout << "Add segment " << p0 << ", " << p1
//                  << ", " << (_trans.back() * pa).transpose()
//                  << ", " << (_trans.back() * pb).transpose() << "\n";
        return _arr.addSegment(p0, p1, _trans.back() * pa, _trans.back() * pb);
    }

    // Also eat comma.
    void eatWhitespaces(std::istream& in) {
        int c = in.peek();
        while(std::isspace(c) || c == ',') {
            in.get();
            c = in.peek();
        }
    }

    char parsePathType(std::istream& in) {
        eatWhitespaces(in);

        char type = 0;
        if(std::isalpha(in.peek())) {
            type = in.get();
        }

        eatWhitespaces(in);

        return type;
    }

    Scalar parsePathNumber(std::istream& in) {
        Scalar f;
        in >> f;
        if(!in) {
            std::cerr << "Error while parsing path definition.\n";
        }
        eatWhitespaces(in);
        return f;
    }

    Vector parsePathVector(std::istream& in) {
        Vector v;
        in >> v(0);
        eatWhitespaces(in);
        in >> v(1);
        if(!in) {
            std::cerr << "Error while parsing path definition.\n";
        }
        eatWhitespaces(in);
        return v;
    }

    virtual bool for_each(pugi::xml_node& node) {
        using namespace pugi;

        if(node.type() != node_element) {
            return true;
        }

        if(strcmp(node.name(), "svg") == 0) {
            const xml_attribute& vba = node.attribute("viewBox");
            if(vba) {
                Vector tl;
                Scalar w;
                Scalar h;

                std::istringstream in(vba.as_string());
                in >> tl[0] >> tl[1] >> w >> h;

                unsigned p0 = addPoint(tl + Vector(0, 0));
                unsigned p1 = addPoint(tl + Vector(w, 0));
                unsigned p2 = addPoint(tl + Vector(w, h));
                unsigned p3 = addPoint(tl + Vector(0, h));

                addSegment(p0, p1);
                addSegment(p1, p2);
                addSegment(p2, p3);
                addSegment(p3, p0);
            }
        }
        else if(strcmp(node.name(), "polygon") == 0) {
            std::string pts = node.attribute("points").as_string();
            for(unsigned i = 0; i < pts.size(); ++i) if(pts[i] == ',') pts[i] = ' ';

            std::istringstream in(pts);
            in >> std::ws;
            int first = -1;
            int last = -1;
            while(in.good()) {
                Vector p;
                in >> p(0) >> p(1) >> std::ws;
                int pi = addPoint(p);
                if(first == -1) {
                    first = pi;
                } else {
                    addSegment(last, pi);
                }
                last = pi;
            }
            if(first != last) {
                addSegment(last, first);
            }
        }
        else if(strcmp(node.name(), "circle") == 0) {
            Vector c(node.attribute("cx").as_float(), node.attribute("cy").as_float());
            Scalar r = node.attribute("r").as_float();

            Vector o[4] = {
                Vector( 0,  r),
                Vector( r,  0),
                Vector( 0, -r),
                Vector(-r,  0)
            };

            Vector p[4];
            unsigned v[4];
            for(unsigned i = 0; i < 4; ++i) {
                p[i] = c + o[i];
                v[i] = addPoint(p[i]);
            }

            Scalar alpha = (3./4.) * (std::sqrt(3) - 1);
            for(unsigned i0 = 0; i0 < 4; ++i0) {
                unsigned i1 = (i0 + 1) % 4;
                addSegment(v[i0], v[i1], p[i0] + alpha*o[i1], p[i1] + alpha*o[i0]);
            }
        }
        else if(strcmp(node.name(), "rect") == 0) {
            Vector tl(node.attribute("x").as_float(), node.attribute("y").as_float());
            Scalar w = node.attribute("width").as_float();
            Scalar h = node.attribute("height").as_float();

            unsigned p0 = addPoint(tl + Vector(0, 0));
            unsigned p1 = addPoint(tl + Vector(w, 0));
            unsigned p2 = addPoint(tl + Vector(w, h));
            unsigned p3 = addPoint(tl + Vector(0, h));

            addSegment(p0, p1);
            addSegment(p1, p2);
            addSegment(p2, p3);
            addSegment(p3, p0);
        }
        else if(strcmp(node.name(), "path") == 0) {
            std::string d = node.attribute("d").as_string();
//            std::cout << "Parsing path: " << d << "\n";
            std::istringstream in(d);

            char type = 'm';
            char prevType = 'm';
            bool absolute = false;
            Vector p(0, 0);
            int pi = -1;
            Vector first = p;
            int firsti = -1;
            Vector prevHandle;
            while(in.good()) {
                char token = parsePathType(in);

                if(token) {
                    type = std::tolower(token);
                    absolute = std::isupper(token);

                    // z do not take a parameter, so we need to handle it here.
                    if(type == 'z') {
//                        std::cout << "z\n";
                        if(firsti != pi) {
//                            std::cout << "  l " << first.transpose() << "\n";
                            addSegment(pi, firsti);
                        }
                        p = first;
                        pi = firsti;

                        // l is implied after z
                        type = 'l';
                    }
                } else {
                    // Add the first point of the subsegment if required
                    if(pi == -1) {
                        switch(type) {
                        case 'l':
                        case 'h':
                        case 'v':
                        case 'c':
                        case 's':
                        case 'q':
                        case 't':
//                            std::cout << "  first point: " << p.transpose() << "\n";
                            pi = addPoint(p);
                            first = p;
                            firsti = pi;
                            break;
                        }
                    }

                    switch(type) {
                    case 'm':
                        p = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "m " << p.transpose() << "\n";
                        pi = -1;
                        first = p;
                        firsti = -1;
                        // Line-to is implied
                        type = 'l';
                        break;
                    case 'z':
                        break;
                    case 'l': {
                        Vector dst = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "l " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i);
                        p = dst;
                        pi = i;
                        break;
                    }
                    case 'h': {
                        Vector dst(parsePathNumber(in) + (absolute? 0: p.x()), p.y());
//                        std::cout << "h " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i);
                        p = dst;
                        pi = i;
                        break;
                    }
                    case 'v': {
                        Vector dst(p.x(), parsePathNumber(in) + (absolute? 0: p.y()));
//                        std::cout << "v " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i);
                        p = dst;
                        pi = i;
                        break;
                    }
                    case 'c': {
                        Vector h0 = parsePathVector(in) + (absolute? Vector(0, 0): p);
                        Vector h1 = parsePathVector(in) + (absolute? Vector(0, 0): p);
                        Vector dst = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "c " << h0.transpose()
//                                  << " - " << h1.transpose()
//                                  << " - " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i, h0, h1);
                        p = dst;
                        pi = i;
                        prevHandle = h1;
                        break;
                    }
                    case 's': {
                        Vector h0 = (prevType == 'c' || prevType == 's')?
                                    2 * p - prevHandle: p;
                        Vector h1 = parsePathVector(in) + (absolute? Vector(0, 0): p);
                        Vector dst = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "s (" << h0.transpose()
//                                  << ") - " << h1.transpose()
//                                  << " - " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i, h0, h1);
                        p = dst;
                        pi = i;
                        prevHandle = h1;
                        break;
                    }
                    case 'q': {
                        Vector h0 = parsePathVector(in) + (absolute? Vector(0, 0): p);
                        Vector dst = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "q " << h0.transpose()
//                                  << " - " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i, h0);
                        p = dst;
                        pi = i;
                        prevHandle = h0;
                        break;
                    }
                    case 't': {
                        Vector h0 = (prevType == 'q' || prevType == 't')?
                                    2 * p - prevHandle: p;
                        Vector dst = parsePathVector(in) + (absolute? Vector(0, 0): p);
//                        std::cout << "t (" << h0.transpose()
//                                  << ") - " << dst.transpose() << "\n";
                        unsigned i = addPoint(dst);
                        addSegment(pi, i, h0);
                        p = dst;
                        pi = i;
                        prevHandle = h0;
                        break;
                    }
//                    case 'a':
//                        break;
                    default:
                        std::cerr << "Unknown path command '" << type << "'.\n";
                        return true;
                    }

                    prevType = type;
                }
            }
        }


        return true;
    }
private:
    BezierArrangement& _arr;
    Scalar _snapDist;
    TransformStack _trans;
};

bool readSvg(BezierArrangement& arr, const char* svgFilename, Scalar snapDist) {
    using namespace pugi;

    xml_document doc;
    xml_parse_result result = doc.load_file(svgFilename);
    if(!result) {
        std::cerr << "Failed to parse SVG: " << result.description() << "\n";
        return false;
    }

    SvgWalker walker(arr, snapDist);
    doc.traverse(walker);

    return true;
}

//struct VecCmp {
//    VecCmp(const Mesh& arr) : _mesh(arr) {}
//    bool operator()(Vertex v0, Vertex v1) {
//        return (   _mesh.position(v0)(0) == _mesh.position(v1)(0)
//                && _mesh.position(v0)(1) < _mesh.position(v1)(1))
//            || _mesh.position(v0)(0) < _mesh.position(v1)(0);
//    }
//    const Mesh& _mesh;
//};

// Compare the nth coefficient of a pair of vectors.
template < typename Vec >
struct CmpCoeff {
    CmpCoeff(int coeff) : _coeff(coeff) {}
    bool operator()(const Vec& lhs, const Vec& rhs) {
        return lhs(_coeff) < rhs(_coeff);
    }
    int _coeff;
};

class VGMeshBuilder {
public:
    typedef Eigen::Matrix<REAL, 2, 1> MVector;
    typedef std::vector<MVector, Eigen::aligned_allocator<MVector> > MVectorVector;

    struct CSeg {
        int segId;
        int from;
        int to;
        Scalar pFrom;
        Scalar pTo;
    };

    typedef std::vector<CSeg> CSegVector;
    typedef std::pair<int, int> PointPair;
    typedef std::map<PointPair, int> CSegMap;

public:
    VGMeshBuilder(const BezierArrangement& arrangement, Scalar maxCurveEdgeLen)
        : _arrangement(&arrangement),
          _maxCurveEdgeLen(maxCurveEdgeLen),
          _quadraticThreshold2(1. / (4. * 4.)),
          _cubicThreshold2(1. / (6. * 6. * 8.)) {
        std::memset(&_tmesh, 0, sizeof(triangulateio));

        VectorVector points;

        // Add arrangment points:
        for(unsigned i = 0; i < arrangement.nPoints(); ++i) {
            points.push_back(arrangement.point(i));
        }

        // Add a dummy CSeg, because triangle seems to not like index 0 and
        // replace it by 1.
        addCSeg(-1, -1, -1, 0, 0);

        // Subdivide and add segments:
        for(unsigned i = 0; i < arrangement.nSegments(); ++i) {
            subdivideSegment(
                        points,
                        arrangement.segment(i),
                        arrangement.segmentFrom(i),
                        arrangement.segmentTo(i),
                        i, 0, 1, 8);
//            std::cout << "subdivide " << i << ": " << getSubSegId(i) << "\n";
        }


        _tmesh.numberofpoints = points.size();
        _tmesh.pointlist = (REAL*)malloc(sizeof(REAL) * 2 * points.size());
        for(unsigned i = 0; i < points.size(); ++i) {
            _tmesh.pointlist[i*2 + 0] = points[i].x();
            _tmesh.pointlist[i*2 + 1] = points[i].y();
        }

        _tmesh.numberofsegments = _csegs.size();
        _tmesh.segmentlist = (int*)malloc(sizeof(int) * 2 * _csegs.size());
        _tmesh.segmentmarkerlist = (int*)malloc(sizeof(int) * _csegs.size());
        for(unsigned i = 0; i < _csegs.size(); ++i) {
            _tmesh.segmentlist[i*2 + 0] = _csegs[i].from;
            _tmesh.segmentlist[i*2 + 1] = _csegs[i].to;
            _tmesh.segmentmarkerlist[i] = i;
//            std::cout << "Add segment: " << _tmesh.segmentmarkerlist[i] << "\n";
        }
    }

    const Mesh& mesh() {
        _mesh.clear();

        for(int i = 0; i < _tmesh.numberofpoints; ++i) {
            _mesh.addVertex(Vector(
                                _tmesh.pointlist[i*2 + 0],
                                _tmesh.pointlist[i*2 + 1]));
        }

        for(int i = 0; i < _tmesh.numberoftriangles; ++i) {
            _mesh.addTriangle(
                        Vertex(_tmesh.trianglelist[i*3 + 0]),
                        Vertex(_tmesh.trianglelist[i*3 + 1]),
                        Vertex(_tmesh.trianglelist[i*3 + 2]));
        }

        for(unsigned i = 0; i < _arrangement->nPointConstraints(); ++i) {
            Mesh::PointConstraint pc =
                    _mesh.addPointConstraint(Vertex(_arrangement->pointConstraint(i)));
            _mesh.value(pc) = Mesh::Value(.9, .9, .9, 1);
//            _mesh.gradient(pc) = Mesh::Gradient::Constant(0);
        }

        // Now for the hard part: we need to find in the refined triangulation
        // which edge belong to which curve.

        // 1. Fill an array containing all constrained edges an sort it by
        // cseg index.
        typedef Eigen::Vector3i Edge;
        typedef std::vector<Edge, Eigen::aligned_allocator<Edge> > EdgeVector;
        EdgeVector edgeVector;
        for(int i = 0; i < _tmesh.numberofsegments; ++i) {
            Edge ss(_tmesh.segmentlist[i*2 + 0],
                    _tmesh.segmentlist[i*2 + 1],
                    _tmesh.segmentmarkerlist[i]);
            edgeVector.push_back(ss);
//            std::cout << "Edge " << i
//                      << ": " << _tmesh.segmentlist[i*2 + 0]
//                      << " - " << _tmesh.segmentlist[i*2 + 1]
//                      << ": " << _tmesh.segmentmarkerlist[i] << "\n";
        }

        std::sort(edgeVector.begin(), edgeVector.end(), CmpCoeff<Edge>(2));

        // map a vertex on a subsegment to its (up to 2) neighbors
        typedef std::map<int, Eigen::Vector2i> VxMap;
        VxMap vxMap;

        // 2. Build each segment one by one. Subsegments in edgeVector are in
        // order, but edges of each subsegment are not.
        // for each segment
        EdgeVector::const_iterator eIt = edgeVector.begin();
        for(unsigned segId = 0; segId < _arrangement->nSegments(); ++segId) {
            int vx = _arrangement->segmentFrom(segId);
            Curve curve = _mesh.addCurve(Mesh::VALUE_TEAR | Mesh::GRADIENT_TEAR);
//            _mesh.valueFunction(curve, Mesh::VALUE)
//                    .add(0, Mesh::Value(0, 0, 0, 1));
            _mesh.valueFunction(curve, Mesh::VALUE_LEFT)
                    .add(1, Mesh::Value(0, 0, 0, 1));
            _mesh.valueFunction(curve, Mesh::VALUE_RIGHT)
                    .add(1, Mesh::Value(1, 1, 1, 1));
//            Scalar g = 0.025;
//            _mesh.valueFunction(curve, Mesh::GRADIENT_LEFT).add(0, Mesh::Value(g, g, g, 0));
//            _mesh.valueFunction(curve, Mesh::GRADIENT_RIGHT).add(0, Mesh::Value(-g, -g, -g, 0));
            _mesh.bezierPath(curve).addSegment(_arrangement->segment(segId));

//            std::cout << "Segment " << segId << ":\n";

            // For each sub-segment (CSeg)
            while(eIt != edgeVector.end()) {
                int csegId = (*eIt)(2);
                CSeg cseg = _csegs[csegId];

                // Stop if we changed segment
                if(cseg.segId != int(segId)) {
                    break;
                }

//                std::cout << "  Cseg " << csegId << ": "
//                          << cseg.from << ", " << cseg.to << "\n";

                Vector csp0 = _mesh.position(Vertex(cseg.from));
                Vector csp1 = _mesh.position(Vertex(cseg.to));

                // Compute neighborhood of each vertex of the subsegment
                while(eIt != edgeVector.end() && (*eIt)(2) == csegId) {
                    Edge e = *eIt;
//                    std::cout << "    Edge: " << e(0)
//                              << " - " << e(1)
//                              << ": " << e(2) << "\n";
                    for(int vi = 0; vi < 2; ++vi) {
                        VxMap::iterator vit = vxMap.insert(
                                    std::make_pair(e(vi), Eigen::Vector2i(-1, -1))).first;
                        int i = (vit->second(0) == -1)? 0:
                                (vit->second(1) == -1)? 1: 2;
                        assert(i != 2);
                        vit->second(i) = e(!vi);
                    }
                    ++eIt;
                }

                // Starting from vx, find and add all subsegment vectices in order
                VxMap::iterator fromIt = vxMap.find(vx);
                while(vxMap.size() > 1) {
//                    std::cout << "    Vertex " << vx
//                              << ": " << fromIt->second.transpose() << "\n";
                    assert(fromIt != vxMap.end() && fromIt->second(0) != -1
                                                 && fromIt->second(1) == -1);
                    int next = fromIt->second(0);

                    // Compute the coordinate of the extremities in parameter
                    // space
                    Vector ep0 = Eigen::Map<MVector>(&_tmesh.pointlist[2*vx]).cast<Scalar>();
                    Vector ep1 = Eigen::Map<MVector>(&_tmesh.pointlist[2*next]).cast<Scalar>();
                    Scalar pFrom = lerp(ilerp(ep0, csp0, csp1), cseg.pFrom, cseg.pTo);
                    Scalar pTo   = lerp(ilerp(ep1, csp0, csp1), cseg.pFrom, cseg.pTo);
                    _mesh.addHalfedgeToCurve(curve,
                                             _mesh.findHalfedge(Vertex(vx), Vertex(next)),
                                             pFrom, pTo);

                    // Move points added by triangle's refinement to their real positions.
                    // Might produce overlap in extreme cases ?
                    _mesh.position(Vertex(next)) = _arrangement->segment(segId).eval(pTo);

                    // Processed vertices are removed from the map to simplify
                    // subsequent steps.
                    vxMap.erase(fromIt);
                    fromIt = vxMap.find(next);
                    if(fromIt->second(0) == vx) {
                        std::swap(fromIt->second(0), fromIt->second(1));
                    }
                    assert(fromIt->second(1) == vx);
                    fromIt->second(1) = -1;

                    vx = next;
                }
                assert(fromIt != vxMap.end() && fromIt->second(0) == -1
                                             && fromIt->second(1) == -1);
                // remove the last vertex from the map.
                vxMap.clear();
            }
        }

        return _mesh;
    }

    bool needSplit(const BSegment& segment) {
        if((segment.point(0) - segment.point(segment.type()-1)).norm() > _maxCurveEdgeLen) {
            return true;
        }
        switch(segment.type()) {
        case BEZIER_LINEAR:
            // Linear segment never need to be split.
            return false;
        case BEZIER_QUADRATIC: {
            // Split if the handle is too far away from the midpoint.
            Scalar l_2 = (segment.point(2) - segment.point(0)).squaredNorm();
            Vector mid = (segment.point(0) + segment.point(2)) / 2;
            Scalar d_2 = (segment.point(1) - mid).squaredNorm();
            return d_2 > (l_2 / _quadraticThreshold2);
        }
        case BEZIER_CUBIC: {
            // Split if the handles are to far from the ideal linear configuration.
            Scalar l_2  = (segment.point(3) - segment.point(0)).squaredNorm();
            Vector mid1 = (segment.point(0) * 2 + segment.point(3)) / 3;
            Vector mid2 = (segment.point(0) + segment.point(3) * 2) / 3;
            Scalar d1_2 = (segment.point(1) - mid1).squaredNorm();
            Scalar d2_2 = (segment.point(2) - mid2).squaredNorm();
            return d1_2 > (l_2 * _cubicThreshold2)
                || d2_2 > (l_2 * _cubicThreshold2);
        }
        default:
            abort();
        }
        return false;
    }

    void subdivideSegment(VectorVector& points,
                          const BSegment& segment,
                          int firstIndex,
                          int lastIndex,
                          int segId,
                          Scalar pFrom,
                          Scalar pTo,
                          int maxDepth) {
        if(maxDepth && needSplit(segment)) {
            BSegment head;
            BSegment tail;
            segment.split(.5, head, tail);
            int mid = points.size();
            points.push_back(tail.point(0));
            subdivideSegment(points, head, firstIndex, mid, segId,
                             pFrom, (pFrom + pTo) / 2, maxDepth - 1);
            subdivideSegment(points, tail, mid, lastIndex, segId,
                             (pFrom + pTo) / 2, pTo, maxDepth - 1);
        } else {
            addCSeg(firstIndex, lastIndex, segId, pFrom, pTo);
        }
    }

    void refineCurves(Scalar refine) {
        triangulateio delaunay;
        std::memset(&delaunay, 0, sizeof(triangulateio));

        // Conforming delaunay triangulation
        // z: first index is 0
        // p: input is a PSLG
        // D: produce a conforming Delaunay triangulation
        // n: output neighbors of each triangles
        // j: Delete unused vertices
        // Q: quiet, remove output
        triangulate((char*)"zpnjQ", &_tmesh, &delaunay, NULL);

        // Add constraints around curves extremities / point constraints
        std::vector<bool> mark(_arrangement->nPoints(), false);
        std::vector<MVector> points;
        for(int fi = 0; fi < delaunay.numberoftriangles; ++fi) {
            for(int fvi = 0; fvi < 3; ++fvi) {
                int vi = delaunay.trianglelist[fi*3 + fvi];
                // Points of interest == points in the arrangement.
                if(vi < int(_arrangement->nPoints()) && !mark[vi]) {
                    mark[vi] = true;
                    addConstraintsAroundVertex(points, delaunay, vi, fi);
                }
            }
        }
        size_t pointlistSize = sizeof(REAL) * 2 * delaunay.numberofpoints;
        size_t newPointsSize = sizeof(REAL) * 2 * points.size();
        delaunay.pointlist = (REAL*)realloc(delaunay.pointlist,
                                            pointlistSize + newPointsSize);
        memcpy(delaunay.pointlist + 2 * delaunay.numberofpoints,
               &points[0], newPointsSize);
        delaunay.numberofpoints = delaunay.numberofpoints + points.size();

#define FREE_PTR(_ptr) if(_ptr) { free(_ptr); _ptr = NULL; }
        FREE_PTR(delaunay.pointattributelist)
        FREE_PTR(delaunay.pointmarkerlist)
        FREE_PTR(delaunay.triangleattributelist)
        FREE_PTR(delaunay.trianglearealist)
        FREE_PTR(delaunay.neighborlist)
        FREE_PTR(delaunay.holelist)
        FREE_PTR(delaunay.regionlist)
        FREE_PTR(delaunay.edgelist)
        FREE_PTR(delaunay.edgemarkerlist)
        FREE_PTR(delaunay.normlist)
        delaunay.numberofpointattributes = 0;
        delaunay.numberoftriangleattributes = 0;
        delaunay.numberofholes = 0;
        delaunay.numberofregions = 0;
        delaunay.numberofedges = 0;

        triangulateio mesh;
        std::memset(&mesh, 0, sizeof(triangulateio));

        // Refine the previous mesh
        // z: first index is 0
        // p: take segments into account
        // (not used due to the previous step) r: input is a mesh
        // q: Delaunay refinment
        // j: Delete unused vertices
        // Q: quiet, remove output
        char buff[128];
        sprintf(buff, "zpq%fjQ", double(refine));
        triangulate(buff, &delaunay, &mesh, NULL);

        std::swap(_tmesh, mesh);
//        std::swap(_tmesh, delaunay);
        freeTri(&delaunay);
        freeTri(&mesh);
    }

    void addConstraintsAroundVertex(std::vector<MVector>& points,
                                    const triangulateio& mesh,
                                    int vi, int fi) {
        // Move clockwise around vi until a border or a complete loop.
        // Find the right stating point in case of boundary vertex.
        int fBegin = fi;
        int lastFi = fi;
        do {
            // Find vi index in face
            int fvi = findInnerIndex(mesh, vi, fi);
            // Next face clockwise.
            lastFi = fi;
            fi = mesh.neighborlist[fi*3 + (fvi+2)%3];
        } while(fi != -1 && fi != fBegin);
        bool boundary = fi == -1;
        fBegin = lastFi;
        fi = fBegin;

//        std::cout << "Vx " << vi << ": "
//                  << Eigen::Map<MVector>(&mesh.pointlist[vi*2]).transpose() << "\n";

        // Compute closest constaint distance and fill dirVector.
        // Rotate counterclockwise. Should go over all the faces even in case
        // of boundary because we took care of moving backward just before.
        _dirVector.clear();
        REAL minDist = std::numeric_limits<REAL>::infinity();
        do {
            // Find vi index in face
            int fvi = findInnerIndex(mesh, vi, fi);

            // Update distance
            int v[3];
            MVector p[3];
            for(int i = 0; i < 3; ++i) {
                v[i] = mesh.trianglelist[fi*3 + (fvi+i)%3];
                p[i] = Eigen::Map<MVector>(&mesh.pointlist[v[i]*2]);
            }
            minDist = std::min(minDist, segDist(p[0], p[1], p[2]));
//            if(boundary && fi == fBegin)
//                std::cout << "  edge " << (p[1] - p[0]).transpose() << "\n";
//            std::cout << "  edge " << (p[2] - p[0]).transpose() << "\n";

            if(boundary && fi == fBegin && isCSeg(v[0], v[1])) {
                _dirVector.push_back((p[1] - p[0]).normalized());
//                std::cout << "  dir " << _dirVector.back().transpose() << "\n";
            }
            if(isCSeg(v[0], v[2])) {
                _dirVector.push_back((p[2] - p[0]).normalized());
//                std::cout << "  dir " << _dirVector.back().transpose() << "\n";
            }

            // Next face counterclockwise.
            fi = mesh.neighborlist[fi*3 + (fvi+1)%3];
        } while(fi != -1 && fi != fBegin);

//        std::cout << "  Min dist: " << minDist << "\n";

        bool point = false;
        if(_dirVector.empty()) {
            _dirVector.push_back(MVector::UnitX());
            point = true;
            // Point constaints need much higher tesselation.
            minDist /= 8;
        }
        if(!boundary) {
            _dirVector.push_back(_dirVector.front());
        }

        Scalar epsilon = 1.e-4;
        MVector p = Eigen::Map<MVector>(&mesh.pointlist[vi*2]);
        for(unsigned i = 0; i < _dirVector.size() - 1; ++i) {
            Scalar a0 = std::atan2(_dirVector[i+0].y(), _dirVector[i+0].x());
            Scalar a1 = std::atan2(_dirVector[i+1].y(), _dirVector[i+1].x());
            if(a1 < a0 + epsilon) a1 += Scalar(2 * M_PI);
            Scalar b = a1 - a0;
            int nSplit = std::floor((b + epsilon) / Scalar(M_PI / 3) + Scalar(.5));
//            std::cout << "  Arc " << _dirVector[i+0].transpose()
//                      << " - " << _dirVector[i+1].transpose()
//                      << ": " << b / M_PI * 180
//                      << " (" << nSplit
//                      << ", " << (b + epsilon) / (M_PI / 3) + .5 << ")\n";
            for(int split = 1; split < nSplit + point; ++split) {
                Scalar c = lerp(Scalar(split) / nSplit, a0, a1);
                points.push_back(p + MVector(cos(c), sin(c)) * minDist / 3);
//                std::cout << "    Add point: " << points.back().transpose()
//                          << " (" << c / M_PI * 180 << ")\n";
            }
        }
    }

    int findInnerIndex(const triangulateio& mesh, int vi, int fi) {
        int fvi = 0;
        while(fvi != 3 && mesh.trianglelist[fi*3 + fvi] != vi) ++fvi;
        assert(fvi != 3);
        return fvi;
    }

    REAL segDist(const MVector& p, const MVector& p1, const MVector& p2) {
        MVector u = p2 - p1;

        MVector v1 = p1 - p;
        REAL dot1 = u.dot(v1);
        if(dot1 <= 0) return v1.norm();

        MVector v2 = p2 - p;
        REAL dot2 = u.dot(v1);
        if(dot2 <= 0) return v2.norm();

        return std::abs(det2(u, v1)) / u.norm();
    }


    void addCSeg(int from, int to, int segId, Scalar pFrom, Scalar pTo) {
        if(to < from) {
            std::swap(from, to);
            std::swap(pFrom, pTo);
        }
        _csegMap.insert(std::make_pair(PointPair(from, to), _csegs.size()));
        CSeg cseg;
        cseg.segId = segId;
        cseg.from  = from;
        cseg.to    = to;
        cseg.pFrom = pFrom;
        cseg.pTo   = pTo;
        _csegs.push_back(cseg);
    }

    bool isCSeg(int from, int to) {
        if(to < from) {
            std::swap(from, to);
        }
        return _csegMap.find(PointPair(from, to)) != _csegMap.end();
    }

    CSeg getCSeg(int from, int to) {
        assert(isCSeg(from, to));
        int i;
        if(from < to) {
            i = _csegMap.find(PointPair(from, to))->second;
        } else {
            i = _csegMap.find(PointPair(to, from))->second;
        }
        CSeg cseg = _csegs[i];
        if(to < from) {
            std::swap(cseg.from, cseg.to);
            std::swap(cseg.pFrom, cseg.pTo);
        }
        return cseg;
    }

private:
    const BezierArrangement* _arrangement;

    triangulateio _tmesh;
    Mesh _mesh;

    CSegVector _csegs;
    CSegMap  _csegMap;
    MVectorVector _dirVector;

    Scalar _maxCurveEdgeLen;

    Scalar _quadraticThreshold2;
    Scalar _cubicThreshold2;
};



void addRandomPoints(BezierArrangement& arr, unsigned nPoints) {
    for(unsigned i = 0; i < nPoints; ++i) {
        arr.addPoint(Vector::Random());
    }
}


void usage(const char* progName) {
    std::cout << "Usage: " << progName << " [-m MAX_LEN] [-r ANGLE] [-s SNAP_DIST] IN OUT\n";
    exit(1);
}

int main(int argc, char** argv) {
    BezierArrangement arr;

    const char* inFilename = NULL;
    const char* outFilename = NULL;
    Scalar refine = 20;
    Scalar maxCurveEdgeLen = std::numeric_limits<Scalar>::infinity();
    Scalar snapDist = 0;

    for(int argi = 1; argi < argc; ++argi) {
        const char* arg = argv[argi];
        if(strcmp(arg, "-m") == 0) {
            if(++argi == argc) usage(argv[0]);
            maxCurveEdgeLen = std::atof(argv[argi]);
        } else if(strcmp(arg, "-r") == 0) {
            if(++argi == argc) usage(argv[0]);
            refine = std::atof(argv[argi]);
        } else if(strcmp(arg, "-s") == 0) {
            if(++argi == argc) usage(argv[0]);
            snapDist = std::atof(argv[argi]);
        } else {
            if(!inFilename) {
                inFilename = arg;
            } else if(!outFilename) {
                outFilename = arg;
            } else {
                usage(argv[0]);
            }
        }
    }

    if(!inFilename || !outFilename) {
        usage(argv[0]);
    }

    readSvg(arr, inFilename, snapDist);

    VGMeshBuilder builder(arr, maxCurveEdgeLen);
    builder.refineCurves(refine);

    Mesh mesh = builder.mesh();
//    mesh.setAttributes(Mesh::LINEAR_FLAGS);
//    mesh.setNodesFromCurves();
//    mesh.finalize();

    std::ofstream out(outFilename);
    MVGWithCurvesWriter<Mesh>().write(out, mesh);

    return 0;
}
