#include <fstream>
#include <iostream>
#include <vector>
#include <deque>
#include <iomanip>
#include <cmath>

#include <Patate/vitelotte.h>


typedef double REAL;

#include "triangle.h"


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

private:
    PointList _points;
    SegmentList _segments;
};

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
    VGMeshBuilder(const BezierArrangement& arrangement)
        : _arrangement(&arrangement),
          _quadraticThreshold2(1. / (4. * 4.)),
          _cubicThreshold2(1. / (6. * 6. * 8.)) {
        std::memset(&_tmesh, 0, sizeof(triangulateio));

        VectorVector points;

        // Add arrangment points:
        for(int i = 0; i < arrangement.nPoints(); ++i) {
            points.push_back(arrangement.point(i));
        }

        // Add a dummy CSeg, because triangle seems to not like index 0 and
        // replace it by 1.
        addCSeg(-1, -1, -1, 0, 0);

        // Subdivide and add segments:
        for(int i = 0; i < arrangement.nSegments(); ++i) {
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
        for(int i = 0; i < points.size(); ++i) {
            _tmesh.pointlist[i*2 + 0] = points[i].x();
            _tmesh.pointlist[i*2 + 1] = points[i].y();
        }

        _tmesh.numberofsegments = _csegs.size();
        _tmesh.segmentlist = (int*)malloc(sizeof(int) * 2 * _csegs.size());
        _tmesh.segmentmarkerlist = (int*)malloc(sizeof(int) * _csegs.size());
        for(int i = 0; i < _csegs.size(); ++i) {
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

        // for each segment
        EdgeVector::const_iterator eIt = edgeVector.begin();
        for(int segId = 0; segId < _arrangement->nSegments(); ++segId) {
            int vx = _arrangement->segmentFrom(segId);
            Curve curve = _mesh.addCurve(0);
            _mesh.valueFunction(curve, Mesh::VALUE)
                    .add(0, Mesh::Value(Eigen::internal::random(0., 1.),
                                        Eigen::internal::random(0., 1.),
                                        Eigen::internal::random(0., 1.),
                                        1));
            _mesh.valueFunction(curve, Mesh::VALUE)
                    .add(1, Mesh::Value(Eigen::internal::random(0., 1.),
                                        Eigen::internal::random(0., 1.),
                                        Eigen::internal::random(0., 1.),
                                        1));
            _mesh.bezierPath(curve).addSegment(_arrangement->segment(segId));

//            std::cout << "Segment " << segId << ":\n";

            // For each sub-segment (CSeg)
            while(eIt != edgeVector.end()) {
                int csegId = (*eIt)(2);
                CSeg cseg = _csegs[csegId];

                // Stop if we changed segment
                if(cseg.segId != segId) {
                    break;
                }

//                std::cout << "  Cseg " << csegId << ": "
//                          << cseg.from << ", " << cseg.to << "\n";

                Vector csp0 = _mesh.position(Vertex(cseg.from));
                Vector csp1 = _mesh.position(Vertex(cseg.to));

                // Add all edges belonging to the same subsegment to a map
                //vxMap.clear();
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

                // Starting from vx, find and add all subsegment in order
                VxMap::iterator fromIt = vxMap.find(vx);
                while(vxMap.size() > 1) {
//                    std::cout << "    Vertex " << vx
//                              << ": " << fromIt->second.transpose() << "\n";
                    assert(fromIt != vxMap.end() && fromIt->second(0) != -1
                                                 && fromIt->second(1) == -1);
                    int next = fromIt->second(0);

                    // TODO: param values
                    Vector ep0 = Eigen::Map<MVector>(&_tmesh.pointlist[2*vx]).cast<Scalar>();
                    Vector ep1 = Eigen::Map<MVector>(&_tmesh.pointlist[2*next]).cast<Scalar>();
                    Scalar pFrom = lerp(ilerp(ep0, csp0, csp1), cseg.pFrom, cseg.pTo);
                    Scalar pTo   = lerp(ilerp(ep1, csp0, csp1), cseg.pFrom, cseg.pTo);
                    _mesh.addHalfedgeToCurve(curve,
                                             _mesh.findHalfedge(Vertex(vx), Vertex(next)),
                                             pFrom, pTo);

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
                vxMap.clear();
            }
        }

        return _mesh;
    }

    bool needSplit(const BSegment& segment) {
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
                          float pFrom,
                          float pTo,
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

    void refineCurves() {
        triangulateio delaunay;
        std::memset(&delaunay, 0, sizeof(triangulateio));

        // Conforming delaunay triangulation
        // z: first index is 0
        // p: input is a PSLG
        // D: produce a conforming Delaunay triangulation
        // n: output neighbors of each triangles
        triangulate((char*)"zpn", &_tmesh, &delaunay, NULL);

        // Add constraints around curves extremities / point constraints
        std::vector<bool> mark(_arrangement->nPoints(), false);
        std::vector<MVector> points;
        for(int fi = 0; fi < delaunay.numberoftriangles; ++fi) {
            for(int fvi = 0; fvi < 3; ++fvi) {
                int vi = delaunay.trianglelist[fi*3 + fvi];
                // Points of interest == points in the arrangement.
                if(vi < _arrangement->nPoints() && !mark[vi]) {
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
        triangulate((char*)"zpq", &delaunay, &mesh, NULL);

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
        for(int i = 0; i < _dirVector.size() - 1; ++i) {
            Scalar a0 = std::atan2(_dirVector[i+0].y(), _dirVector[i+0].x());
            Scalar a1 = std::atan2(_dirVector[i+1].y(), _dirVector[i+1].x());
            if(a1 < a0 + epsilon) a1 += 2 * M_PI;
            Scalar b = a1 - a0;
            int nSplit = std::floor((b + epsilon) / (M_PI / 3) + .5);
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
        _csegs.push_back(CSeg { segId, from, to, pFrom, pTo });
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

    Scalar _quadraticThreshold2;
    Scalar _cubicThreshold2;
};



void addRandomPoints(BezierArrangement& arr, unsigned nPoints) {
    for(unsigned i = 0; i < nPoints; ++i) {
        arr.addPoint(Vector::Random());
    }
}

int main(int argc, char** argv) {
    BezierArrangement arr;

    unsigned pb0 = arr.addPoint(Vector(0, 0));
    unsigned pb1 = arr.addPoint(Vector(100, 0));
    unsigned pb2 = arr.addPoint(Vector(100, 100));
    unsigned pb3 = arr.addPoint(Vector(0, 100));
    unsigned pc0 = arr.addPoint(Vector(10, 10));
    unsigned pc1 = arr.addPoint(Vector(50, 50));
    unsigned pc2 = arr.addPoint(Vector(20, 70));
    unsigned pc3 = arr.addPoint(Vector(80, 20));
    unsigned p0  = arr.addPoint(Vector(50, 10));

    unsigned sb0 = arr.addSegment(pb0, pb1);
    unsigned sb1 = arr.addSegment(pb1, pb2);
    unsigned sb2 = arr.addSegment(pb2, pb3);
    unsigned sb3 = arr.addSegment(pb3, pb0);

    unsigned s0 = arr.addSegment(pc0, pc1, Vector(30, 10), Vector(40, 40));
    unsigned s1 = arr.addSegment(pc2, pc3, Vector(120, 110), Vector(50, 50));

//    addRandomPoints(arr, 8);
//    arr.addSegment(4, 7);

    VGMeshBuilder builder(arr);
    builder.refineCurves();

    Mesh mesh = builder.mesh();
//    mesh.setAttributes(Mesh::LINEAR_FLAGS);
//    mesh.setNodesFromCurves();
//    mesh.finalize();

    std::ofstream out("refine.mvg");
    MVGWithCurvesWriter<Mesh>().write(out, mesh);

    return 0;
}
