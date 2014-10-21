#include <cstdio>
#include <iostream>
#include <fstream>

#include <Patate/Vitelotte/Core/quadraticMesh.h>
#include <Patate/Vitelotte/Utils/qvgWriter.h>
#include <Patate/Vitelotte/Utils/qvgReader.h>


typedef double Scalar;
typedef Eigen::Matrix<Scalar, 2, 2> Matrix;
typedef Vitelotte::QuadraticMesh<Scalar> Mesh;

typedef Mesh::Vector Vector;
typedef Mesh::NodeValue NodeValue;
typedef Mesh::NodeID NodeID;

typedef Mesh::Vertex Vertex;
typedef Mesh::Halfedge Halfedge;
typedef Mesh::Edge Edge;
typedef Mesh::Face Face;

typedef Mesh::VertexIterator VertexIterator;
typedef Mesh::EdgeIterator EdgeIterator;
typedef Mesh::FaceIterator FaceIterator;

typedef Vitelotte::QVGWriter<Mesh> Writer;
typedef Vitelotte::QVGReader<Mesh> Reader;

void printMesh(const Mesh& mesh, std::ostream& out)
{
    out << "Nodes (" << mesh.nNodes() << "):\n";
    for(int i = 0; i < mesh.nNodes(); ++i)
    {
        out << "  n" << i
            << ": " << mesh.nodeValue(i).transpose()
            << " - " << (mesh.isConstraint(i)? "C": "N")
            << "\n";
    }

    out << "Vertices (" << mesh.nVertices() << "):\n";
    size_t count = 0;
    for(VertexIterator vIt = mesh.verticesBegin();
        vIt != mesh.verticesEnd(); ++vIt, ++count)
    {
        out << "  v" << count
            << ": " << mesh.position(*vIt).transpose()
            << "\n";
    }

    out << "Faces (" << mesh.nFaces() << "):\n";
    count = 0;
    for(FaceIterator fIt = mesh.facesBegin();
        fIt != mesh.facesEnd(); ++fIt, ++count)
    {
        assert(mesh.valence(*fIt) == 3);
        Halfedge h0 = mesh.halfedge(*fIt);
        Halfedge h1 = mesh.nextHalfedge(h0);
        Halfedge h2 = mesh.nextHalfedge(h1);
        out << "  f" << count << ":\n"
            << "    v0: " << mesh.position(mesh.toVertex(h0)).transpose() << "\n"
            << "        " << mesh.toNode(h0) << " - " << mesh.fromNode(h1) << "\n"
            << "    v1: " << mesh.position(mesh.toVertex(h1)).transpose() << "\n"
            << "        " << mesh.toNode(h1) << " - " << mesh.fromNode(h2) << "\n"
            << "    v2: " << mesh.position(mesh.toVertex(h2)).transpose() << "\n"
            << "        " << mesh.toNode(h2) << " - " << mesh.fromNode(h0) << "\n"
            << "    v0-v1: " << mesh.midNode(h1) << "\n"
            << "    v1-v2: " << mesh.midNode(h2) << "\n"
            << "    v2-v0: " << mesh.midNode(h0) << "\n";
    }

}


int main(int argc, char** argv)
{
    Mesh mesh;

    Matrix t;
    t << 1, -0.5,
         0, std::sqrt(3) / 2;

    unsigned sx = 4;
    unsigned sy = 4;
    for(unsigned j = 0; j < sy; ++j)
    {
        for(unsigned i = 0; i < sx; ++i)
        {
            mesh.addVertex(t * Vector(i, j));
        }
    }

    for(unsigned j = 0; j < sy-1; ++j)
    {
        for(unsigned i = 0; i < sx-1; ++i)
        {
            mesh.addTriangle(
                        Vertex(i+1 + (j+1)*sx),
                        Vertex(i   + (j+1)*sx),
                        Vertex(i   +  j   *sx));
            mesh.addTriangle(
                        Vertex(i   +  j   *sx),
                        Vertex(i+1 +  j   *sx),
                        Vertex(i+1 + (j+1)*sx));
        }
    }

    mesh.initializeUnconstrained();

    for(EdgeIterator eit = mesh.edgesBegin();
        eit != mesh.edgesEnd(); ++eit)\
    {
        Halfedge h = mesh.halfedge(*eit, 0);
        if(mesh.isConstraint(*eit))
        {
            std::cerr << mesh.fromVertex(h).idx() << " - " << mesh.toVertex(h).idx() << (mesh.isBoundary(h)? " B": "")
                      << " = " << mesh.fromNode(h) << (mesh.isConstraint(mesh.fromNode(h))? " C": " U")
                      << ", " << mesh.midNode(h) << (mesh.isConstraint(mesh.midNode(h))? " C": " U")
                      << ", " << mesh.toNode(h) << (mesh.isConstraint(mesh.toNode(h))? " C": " U")
                      << "\n";
            h = mesh.halfedge(*eit, 1);
            std::cerr << mesh.fromVertex(h).idx() << " - " << mesh.toVertex(h).idx() << (mesh.isBoundary(h)? " B": "")
                      << " = " << mesh.fromNode(h) << (mesh.isConstraint(mesh.fromNode(h))? " C": " U")
                      << ", " << mesh.midNode(h) << (mesh.isConstraint(mesh.midNode(h))? " C": " U")
                      << ", " << mesh.toNode(h) << (mesh.isConstraint(mesh.toNode(h))? " C": " U")
                      << "\n";
            abort();
        }
    }

    NodeID na = mesh.addNode();
    NodeID nb = mesh.addNode();
    NodeID nc = mesh.addNode();
    NodeID nd = mesh.addNode();

    NodeID n1l = mesh.addNode(NodeValue(1, 0, 0, 1));
    NodeID n2l = mesh.addNode(NodeValue(0, 1, 0, 1));
    NodeID n1r = mesh.addNode(NodeValue(0, 0, 1, 1));
    NodeID n2r = mesh.addNode(NodeValue(1, 1, 0, 1));

    Halfedge h1 = mesh.findHalfedge(Vertex(5), Vertex(6));
    mesh.setConstraint(h1, n1l, na, nb);
    mesh.setConstraint(mesh.oppositeHalfedge(h1), nb, na, n1r);

    Halfedge h2 = mesh.findHalfedge(Vertex(5), Vertex(9));
    mesh.setConstraint(h2, n2r, nc, nd);
    mesh.setConstraint(mesh.oppositeHalfedge(h2), nd, nc, n2l);

//    Halfedge h = mesh.findHalfedge(Vertex(5), Vertex(6));
//    mesh.setContinuousConstraint(h,
//        mesh.addNode(NodeValue(1, 0, 0, 1)),
//        mesh.addNode(NodeValue(0, 1, 0, 1)),
//        mesh.addNode(NodeValue(0, 0, 1, 1)));

//    Halfedge h2 = mesh.findHalfedge(Vertex(5), Vertex(9));
//    mesh.setContinuousConstraint(h2,
//        mesh.fromNode(h),
//        mesh.addNode(NodeValue(1, .5, 0, 1)),
//        mesh.addNode(NodeValue(1, 1, 0, 1)));
//    mesh.setConstraint(mesh.oppositeHalfedge(h2),
//        mesh.addNode(NodeValue(1, 0, 1, 1)),
//        mesh.addNode(NodeValue(1, 0, .5, 1)),
//        mesh.fromNode(h));

    mesh.propagateConstraints();

    std::ofstream out("test.qvg");
    Writer writer(mesh);
    writer.write(out);
    out.close();

//    std::cout << "UnconstrainedNode = "
//              << Mesh::UnconstrainedNode.transpose()
//              << "\n";

//    mesh.reserve(4, 5, 2, 20);

//    Vertex v0 = mesh.addVertex(Vector(0., 0.));
//    Vertex v1 = mesh.addVertex(Vector(1., 0.));
//    Vertex v2 = mesh.addVertex(Vector(1., 1.));
//    Vertex v3 = mesh.addVertex(Vector(0., 1.));

//    Face f0 = mesh.addTriangle(v0, v1, v2);
//    Face f1 = mesh.addTriangle(v2, v3, v0);

//    mesh.addNode();
//    NodeID n00 = mesh.addNode(NodeValue(1., 0., 0., 1.));
//    NodeID n01 = mesh.addNode(NodeValue(0., 1., 0., 1.));
//    NodeID n02 = mesh.addNode(NodeValue(0., 0., 1., 1.));
//    NodeID n03 = mesh.addNode();
//    NodeID n04 = mesh.addNode();
//    mesh.addNode(NodeValue(1., 1., 1., 1.));
//    mesh.addNode();
//    NodeID n05 = mesh.addNode();
//    NodeID n06 = mesh.addNode(NodeValue(1., 1., 0., 1.));
//    mesh.addNode(NodeValue(1., 1., 1., 1.));
//    NodeID n07 = mesh.addNode(NodeValue(0., 1., 1., 1.));
//    NodeID n08 = mesh.addNode(NodeValue(1., 0., 1., 1.));
//    NodeID n09 = mesh.addNode();
//    mesh.addNode();

//    Halfedge h = mesh.halfedge(f0);
//    mesh.toNode(h) = n00;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n00;
//    mesh.midNode(h) = n01;
//    mesh.toNode(h) = n02;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n02;
//    mesh.midNode(h) = n03;
//    mesh.toNode(h) = n04;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n04;
//    mesh.midNode(h) = n09;

//    h = mesh.halfedge(f1);
//    mesh.toNode(h) = n04;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n04;
//    mesh.midNode(h) = n05;
//    mesh.toNode(h) = n06;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n06;
//    mesh.midNode(h) = n07;
//    mesh.toNode(h) = n08;
//    h = mesh.nextHalfedge(h);
//    mesh.fromNode(h) = n00;
//    mesh.midNode(h) = n09;

//    std::cout << "Original mesh:\n";
//    printMesh(mesh, std::cout);

//    Mesh copy(mesh);

//    mesh.sortAndCompactNodes();

//    std::cout << "Clean mesh:\n";
//    printMesh(mesh, std::cout);

//    std::ofstream out("test.qvg");
//    Writer writer(mesh);
//    writer.write(out);
//    out.close();

//    std::cout << "Original copy:\n";
//    printMesh(copy, std::cout);

//    std::ifstream in("test.qvg");
//    Reader reader(mesh);
//    reader.read(in);
//    in.close();

//    std::cout << "Read:\n";
//    printMesh(mesh, std::cout);

    return EXIT_SUCCESS;
}
