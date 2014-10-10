#include <cstdio>
#include <iostream>

#include <Patate/Vitelotte/Core/fvMesh.h>


typedef double Scalar;
typedef Vitelotte::FVMesh<Scalar> Mesh;

typedef Mesh::Vector Vector;
typedef Mesh::NodeValue NodeValue;
typedef Mesh::NodeID NodeID;

typedef Mesh::Vertex Vertex;
typedef Mesh::Halfedge Halfedge;
typedef Mesh::Edge Edge;
typedef Mesh::Face Face;

typedef Mesh::VertexIterator VertexIterator;
typedef Mesh::FaceIterator FaceIterator;

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
            << " - " << mesh.hasFlatGradient(*vIt)
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
            << "    v0-v1: " << mesh.midNode(h1) << " - " << mesh.gradientNode(h1) << "\n"
            << "    v1-v2: " << mesh.midNode(h2) << " - " << mesh.gradientNode(h2) << "\n"
            << "    v2-v0: " << mesh.midNode(h0) << " - " << mesh.gradientNode(h0) << "\n";
    }

}


int main(int argc, char** argv)
{
    Mesh mesh;

    std::cout << "UnconstrainedNode = "
              << Mesh::UnconstrainedNode.transpose()
              << "\n";

    mesh.reserve(4, 5, 2, 20);

    Vertex v0 = mesh.addVertex(Vector(0., 0.));
    Vertex v1 = mesh.addVertex(Vector(1., 0.));
    Vertex v2 = mesh.addVertex(Vector(1., 1.));
    mesh.setFlatGradient(v2, true);
    Vertex v3 = mesh.addVertex(Vector(0., 1.));

    Face f0 = mesh.addTriangle(v0, v1, v2);
    Face f1 = mesh.addTriangle(v2, v3, v0);

    mesh.addNode();
    NodeID n00 = mesh.addNode(NodeValue(1., 0., 0., 1.));
    NodeID n01 = mesh.addNode(NodeValue(0., 1., 0., 1.));
    NodeID n02 = mesh.addNode(NodeValue(0., 0., 1., 1.));
    NodeID n03 = mesh.addNode();
    NodeID n04 = mesh.addNode();
    mesh.addNode();
    NodeID n05 = mesh.addNode();
    NodeID n06 = mesh.addNode(NodeValue(1., 1., 0., 1.));
    NodeID n07 = mesh.addNode(NodeValue(0., 1., 1., 1.));
    NodeID n08 = mesh.addNode(NodeValue(1., 0., 1., 1.));
    NodeID n09 = mesh.addNode();
    mesh.addNode(NodeValue(1., 1., 1., 1.));
    NodeID n10 = mesh.addNode(NodeValue(0., 0., 0., 0.));
    NodeID n11 = mesh.addNode();
    NodeID n12 = mesh.addNode();
    mesh.addNode(NodeValue(1., 1., 1., 1.));
    NodeID n13 = mesh.addNode();
    NodeID n14 = mesh.addNode();
    mesh.addNode();

    Halfedge h = mesh.halfedge(f0);
    mesh.toNode(h) = n00;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n00;
    mesh.midNode(h) = n01;
    mesh.gradientNode(h) = n10;
    mesh.toNode(h) = n02;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n02;
    mesh.midNode(h) = n03;
    mesh.gradientNode(h) = n11;
    mesh.toNode(h) = n04;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n04;
    mesh.midNode(h) = n09;
    mesh.gradientNode(h) = n14;

    h = mesh.halfedge(f1);
    mesh.toNode(h) = n04;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n04;
    mesh.midNode(h) = n05;
    mesh.gradientNode(h) = n12;
    mesh.toNode(h) = n06;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n06;
    mesh.midNode(h) = n07;
    mesh.gradientNode(h) = n13;
    mesh.toNode(h) = n08;
    h = mesh.nextHalfedge(h);
    mesh.fromNode(h) = n00;
    mesh.midNode(h) = n09;
    mesh.gradientNode(h) = n14;

    std::cout << "Original mesh:\n";
    printMesh(mesh, std::cout);

    Mesh copy(mesh);

    mesh.sortAndCompactNodes();

    std::cout << "Clean mesh:\n";
    printMesh(mesh, std::cout);

    std::cout << "Original copy:\n";
    printMesh(copy, std::cout);

    return EXIT_SUCCESS;
}
