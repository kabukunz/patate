#include <fstream>
#include <iostream>
#include <vector>
#include <deque>
#include <iomanip>

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


//bool convexIntersectionHelper(const VectorVector& poly0, const VectorVector& poly1,
//                        Scalar epsilon) {
//    Vector from = poly0.back();
//    for(VectorVector::const_iterator toIt = poly0.begin();
//        toIt != poly0.end(); ++toIt) {

//        Vector edge = *toIt - from;
//        VectorVector::const_iterator pIt = poly1.begin();
//        for(; pIt != poly1.end(); ++pIt) {
//            if(det2(edge, *pIt - from) > -epsilon) {
//                break;
//            }
//        }

//        // Loop didn't break, so there is no vertex on the wrong side of the
//        // edge, so there is no collision.
//        if(pIt == poly1.end()) {
//            return false;
//        }

//        from = *toIt;
//    }
//    return true;
//}

//bool convexIntersection(const VectorVector& poly0, const VectorVector& poly1,
//                        Scalar epsilon) {
//    return convexIntersectionHelper(poly0, poly1, epsilon)
//        && convexIntersectionHelper(poly1, poly0, epsilon);
//}


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
    typedef Eigen::Vector3i Segment;
    typedef std::vector<Segment, Eigen::aligned_allocator<Segment> > SegmentVector;

public:
    VGMeshBuilder(const BezierArrangement& arrangement)
        : _arrangement(&arrangement),
          _quadraticThreshold2(1. / (4. * 4.)),
          _cubicThreshold2(1. / (6. * 6. * 8.)) {
        std::memset(&_tmesh, 0, sizeof(triangulateio));

        VectorVector points;
        SegmentVector segments;

        // Add arrangment points:
        for(int i = 0; i < arrangement.nPoints(); ++i) {
            points.push_back(arrangement.point(i));
        }

        int test = getSubSegId(123, 846);
        assert(getSegId(test) == 123 && getSubId(test) == 846);
        // Subdivide and add segments:
        for(int i = 0; i < arrangement.nSegments(); ++i) {
            subdivideSegment(
                        points,
                        segments,
                        arrangement.segment(i),
                        arrangement.segmentFrom(i),
                        arrangement.segmentTo(i),
                        getSubSegId(i),
                        8);
            std::cout << "subdivide " << i << ": " << getSubSegId(i) << "\n";
        }


        _tmesh.numberofpoints = points.size();
        _tmesh.pointlist = (REAL*)malloc(sizeof(REAL) * 2 * points.size());
        for(int i = 0; i < points.size(); ++i) {
            _tmesh.pointlist[i*2 + 0] = points[i].x();
            _tmesh.pointlist[i*2 + 1] = points[i].y();
        }

        _tmesh.numberofsegments = segments.size();
        _tmesh.segmentlist = (int*)malloc(sizeof(int) * 2 * segments.size());
        _tmesh.segmentmarkerlist = (int*)malloc(sizeof(int) * segments.size());
        for(int i = 0; i < segments.size(); ++i) {
            _tmesh.segmentlist[i*2 + 0] = segments[i](0);
            _tmesh.segmentlist[i*2 + 1] = segments[i](1);
            _tmesh.segmentmarkerlist[i] = segments[i](2);
            std::cout << "Add segment: " << _tmesh.segmentmarkerlist[i] << "\n";
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

        // TODO: Create curves.
        typedef std::vector<Segment> SegVector;
        SegVector segVector;
        for(int i = 0; i < _tmesh.numberofsegments; ++i) {
            Segment seg(
                    _tmesh.segmentlist[i*2 + 0],
                    _tmesh.segmentlist[i*2 + 1],
                    _tmesh.segmentmarkerlist[i]);
            segVector.push_back(seg);
            std::cout << "Segment " << i
                      << ": " << _tmesh.segmentlist[i*2 + 0]
                      << " - " << _tmesh.segmentlist[i*2 + 1]
                      << ": " << getSegId(_tmesh.segmentmarkerlist[i])
                      << ", " << getSubId(_tmesh.segmentmarkerlist[i]) << "\n";
        }

        std::sort(segVector.begin(), segVector.end(), CmpCoeff<Segment>(2));

        // map a vertex of a subsegment to its (up to 2) neighbors
        typedef std::map<int, Eigen::Vector2i> VxMap;
        VxMap vxMap;
        int prevSeg = -1;
        int vx = -1;
        Curve curve;

        // Iterate segments
        SegVector::const_iterator segIt = segVector.begin();
        while(segIt != segVector.end()) {

            int id = (*segIt)(2);
            int segId = getSegId(id);

            // If we start to process an new segment, update the current vertex
            if(prevSeg != segId) {
                // Arrangement points have the same index in the triangulation
                std::cout << "Start segment " << segId << "\n";
                vx = _arrangement->segmentFrom(segId);
                curve = _mesh.addCurve(0);
                _mesh.valueFunction(curve, Mesh::VALUE)
                        .add(0, Mesh::Value(Eigen::internal::random(0., 1.),
                                            Eigen::internal::random(0., 1.),
                                            Eigen::internal::random(0., 1.),
                                            1));
            }

            std::cout << "  Subegment: " << getSubId(id) << "\n";

            // Add all edges belonging to the same subsegment to a map
            //vxMap.clear();
            while(segIt != segVector.end() && (*segIt)(2) == id) {
                Segment seg = *segIt;
                std::cout << "    Edge: " << seg(0)
                          << " - " << seg(1)
                          << ": " << getSegId(seg(2))
                          << ", " << getSubId(seg(2)) << "\n";
                for(int vi = 0; vi < 2; ++vi) {
                    VxMap::iterator vit = vxMap.insert(
                                std::make_pair(seg(vi), Eigen::Vector2i(-1, -1))).first;
                    int i = (vit->second(0) == -1)? 0:
                            (vit->second(1) == -1)? 1: 2;
                    assert(i != 2);
                    vit->second(i) = seg(!vi);
                }
                ++segIt;
            }

            // Starting from vx, find and add all subsegment in order
            VxMap::iterator fromIt = vxMap.find(vx);
            while(vxMap.size() > 1) {
                std::cout << "    Vertex " << vx
                          << ": " << fromIt->second.transpose() << "\n";
                assert(fromIt != vxMap.end() && fromIt->second(0) != -1
                                             && fromIt->second(1) == -1);
                int next = fromIt->second(0);
                // TODO: param values
                _mesh.addHalfedgeToCurve(curve,
                                         _mesh.findHalfedge(Vertex(vx), Vertex(next)),
                                         0, 0);
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

            prevSeg = segId;
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

    int subdivideSegment(VectorVector& points,
                          SegmentVector& segments,
                          const BSegment& segment,
                          int firstIndex,
                          int lastIndex,
                          int segSubId,
                          int maxDepth) {
        if(maxDepth && needSplit(segment)) {
            BSegment head;
            BSegment tail;
            segment.split(.5, head, tail);
            int mid = points.size();
            points.push_back(tail.point(0));
            segSubId = subdivideSegment(points, segments, head, firstIndex, mid,
                                        segSubId, maxDepth - 1);
            segSubId = subdivideSegment(points, segments, tail, mid, lastIndex,
                                        segSubId, maxDepth - 1);
        } else {
            segments.push_back(Segment(firstIndex, lastIndex, segSubId));
            ++segSubId;
        }
        return segSubId;
    }

    void refineCurves() {
        triangulateio delaunay;
        std::memset(&delaunay, 0, sizeof(triangulateio));

        // Conforming delaunay triangulation
        // z: first index is 0
        // p: input is a PSLG
        // D: produce a conforming Delaunay triangulation
        triangulate((char*)"zpD", &_tmesh, &delaunay, NULL);

        // TODO: add constraints around curves extremities / point constraints

        triangulateio mesh;
        std::memset(&mesh, 0, sizeof(triangulateio));

        // Refine the previous mesh
        // z: first index is 0
        // p: take segments into account
        // r: input is a mesh
        // q: Delaunay refinment
        triangulate((char*)"zprq", &delaunay, &mesh, NULL);

        std::swap(_tmesh, mesh);
        freeTri(&delaunay);
        freeTri(&mesh);
    }

    int getSubSegId(int segId, int subId = 0) {
        assert(segId >= 0 && subId >= 0);
        assert(segId < (1<<16) && subId < (1<<16));
        return (segId << 16) | subId;
    }

    int getSegId(int subSegId) {
        return subSegId >> 16;
    }

    int getSubId(int subSegId) {
        return subSegId & 0xffff;
    }

//    bool isConstrained(Halfedge h) const {
//        return _mesh.isBoundary(_mesh.edge(h)) || _mesh.curve(h).isValid();
//    }

//    bool isRefinable(Face f) const {
//        unsigned nCons;
//        Mesh::HalfedgeAroundFaceCirculator hit = _mesh.halfedges(f);
//        Mesh::HalfedgeAroundFaceCirculator hEnd = hit;
//        do {
//            if(isConstrained(*hit)) {
//                ++nCons;
//            }
//            if(nCons == 2) return false;
//            ++hit;
//        } while(hit != hEnd);
//        return true;
//    }

//    unsigned refineTriangles(float arThresold) {
//        FaceDeque toRefine;
//        for(Mesh::FaceIterator fit = _mesh.facesBegin();
//            fit != _mesh.facesEnd(); ++fit) {
//            toRefine.push_back(*fit);
//        }

//        unsigned count = 0;
//        while(!toRefine.empty()) {
//            Face f = toRefine.front();
//            toRefine.pop_front();

//            if(!isRefinable(f)) {
//                continue;
//            }

//            // Aspect ratio
//            Scalar l[3] = {
//                edgeVector(f, 0).norm(),
//                edgeVector(f, 1).norm(),
//                edgeVector(f, 2).norm()
//            };
//            Scalar s = (l[0] + l[1] + l[2]) / 2;
//            Scalar ar = l[0] * l[1] * l[2]
//                      / (8 * (s - l[0]) * (s - l[1]) * (s - l[2]));

//            if(ar > arThresold) {
//                // Circumcenter
//                Vector v1 = point(f, 1) - point(f, 0);
//                Vector v2 = point(f, 2) - point(f, 0);
//                Vector p(v2.y() * v1.squaredNorm() - v1.y() * v2.squaredNorm(),
//                         v1.x() * v2.squaredNorm() - v2.x() * v1.squaredNorm());
//                p /= 2 * (v1.x() * v2.y() - v1.y() * v2.x());
//                p += point(f, 0);

//                assert(abs((point(f, 0) - p).squaredNorm()
//                         - (point(f, 1) - p).squaredNorm()) < _epsilon);
//                assert(abs((point(f, 0) - p).squaredNorm()
//                         - (point(f, 2) - p).squaredNorm()) < _epsilon);

//                // Find the triangle / edge to refine
//                Halfedge h = _mesh.halfedge(f);
//                Halfedge hEnd = h;
//                Vertex vx;
//                do {
//                    Vector p0 = _mesh.position(_mesh.fromVertex(h));
//                    Vector p1 = _mesh.position(_mesh.toVertex(h));
//                    Scalar det = det2(p1 - p0, p - p0);
//                    if(det < _epsilon) {
//                        if(isConstrained(h) || det > -_epsilon) {
//                            if(_mesh.curve(h).isValid()) {
//                                vx = _mesh.splitCurvedEdge(h, .5);
//                            } else {
//                                vx = _mesh.addVertex((p0 + p1) / 2);
//                                _mesh.PatateCommon::SurfaceMesh::split(_mesh.edge(h), vx);
//                            }
//                            break;
//                        }
//                        h = _mesh.oppositeHalfedge(h);
//                        hEnd = h;
//                    }
//                    h = _mesh.nextHalfedge(h);
//                } while(!vx.isValid() && h != hEnd);

//                // If we did not insert a vertex on an edge.
//                if(!vx.isValid()) {
//                    vx = _mesh.addVertex(p);
//                    _mesh.split(_mesh.face(h), vx);
//                }

//                ++count;

//                Mesh::FaceAroundVertexCirculator afit = _mesh.faces(vx);
//                Mesh::FaceAroundVertexCirculator afEnd = afit;
//                do {
//                    addDirtyFace(*afit);
//                    ++afit;
//                } while(afit != afEnd);

//                delaunay();
//            }
//        }

//        return count;
//    }

//    bool isFixed(Vertex vx) const {
//        Mesh::HalfedgeAroundVertexCirculator hit = _mesh.halfedges(vx);
//        Mesh::HalfedgeAroundVertexCirculator hEnd = hit;
//        do {
//            if(isConstrained(*hit)) {
//                return true;
//            }
//            ++hit;
//        } while(hit != hEnd);

//        return false;
//    }

//    void smooth(unsigned nPass) {
//        Mesh::VectorMatrix pos;
//        for(unsigned pass = 0; pass < nPass; ++pass) {
//            pos = Mesh::VectorMatrix::Zero(_mesh._positionMatrix().rows(),
//                                           _mesh._positionMatrix().cols());

//            for(Mesh::VertexIterator vit = _mesh.verticesBegin();
//                vit != _mesh.verticesEnd(); ++vit) {
//                if(isFixed(*vit)) {
//                    pos.col((*vit).idx()) = _mesh.position(*vit);
//                    continue;
//                }

//                Mesh::VertexAroundVertexCirculator nit = _mesh.vertices(*vit);
//                Mesh::VertexAroundVertexCirculator nEnd = nit;
//                unsigned count = 0;
//                do {
//                    pos.col((*vit).idx()) += _mesh.position(*nit);
//                    ++count;
//                    ++nit;
//                } while(nit != nEnd);

//                pos.col((*vit).idx()) /= count;
//            }
//            _mesh._positionMatrix().swap(pos);

//            for(Mesh::FaceIterator fit = _mesh.facesBegin();
//                fit != _mesh.facesEnd(); ++fit) {
//                addDirtyFace(*fit);
//            }
//            delaunay();
//        }
//    }

private:
    const BezierArrangement* _arrangement;

    triangulateio _tmesh;
    Mesh _mesh;

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
    mesh.setAttributes(Mesh::LINEAR_FLAGS);
    mesh.setNodesFromCurves();

    std::ofstream out("refine.mvg");
    MVGWithCurvesWriter<Mesh>().write(out, mesh);

    return 0;
}
