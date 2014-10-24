#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include "Patate/vitelotte.h"
#include "Patate/Vitelotte/Utils/qvgReader.h"
#include "Patate/Vitelotte/Utils/qvgWriter.h"


void usage(char* progName)
{
    std::cout << "usage: " << progName << " mesh attributes out\n";
    exit(1);
}


void solveHarmonic(const std::string& meshFilename,
                   const std::string& outFilename)
{
    typedef Vitelotte::QuadraticMesh<float> Mesh;
    typedef Vitelotte::QVGReader<Mesh> Reader;
    typedef Vitelotte::QVGWriter<Mesh> Writer;

    //typedef Vitelotte::QuadraticElement<Mesh, double> Element;
    typedef Vitelotte::QuadraticElement<Mesh, double> QuadraticElement;
    typedef Vitelotte::SingularElementDecorator<QuadraticElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    Mesh mesh;

    std::ifstream inMeshFile(meshFilename.c_str());
    Reader reader(mesh);
    reader.read(inMeshFile);

    //inMesh.loadConstraintMap(attrFilename);
    //inMesh.buildElements(true, true);

    Solver solver(&mesh);
    solver.build();
    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }

    std::ofstream outMeshFile(outFilename.c_str());
    Writer writer(mesh);
    writer.write(outMeshFile);
}


void solveBiharmonic(const std::string& meshFilename,
                     const std::string& outFilename)
{
    typedef Vitelotte::FVMesh<float> Mesh;
    typedef Vitelotte::QVGReader<Mesh> Reader;
    typedef Vitelotte::QVGWriter<Mesh> Writer;

    typedef Vitelotte::FVElementBuilder<Mesh, double> Element;
//    typedef Vitelotte::FVElement<Mesh, double> FVElement;
//    typedef Vitelotte::SingularElementDecorator<FVElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    Mesh mesh;

    std::ifstream inMeshFile(meshFilename.c_str());
    Reader reader(mesh);
    reader.read(inMeshFile);

    mesh.initializeGradientConstraints();

    Mesh::Halfedge h = mesh.findHalfedge(Mesh::Vertex(0), Mesh::Vertex(1));
    mesh.nodeValue(mesh.gradientNode(h)) = Mesh::NodeValue(-1, -1, -1, 0);

    //inMesh.loadConstraintMap(attrFilename);
    //inMesh.buildElements(true, true);

    Solver solver(&mesh);
    solver.build();
    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }

    std::ofstream outMeshFile(outFilename.c_str());
    Writer writer(mesh);
    writer.write(outMeshFile);
}


int main(int argc, char** argv)
{
    std::string meshFilename;
    std::string attrFilename;
    std::string outFilename;

    if(argc == 4)
    {
        meshFilename = argv[1];
        attrFilename = argv[2];
        outFilename = argv[3];
    }
    else
        usage(argv[0]);

//    solveHarmonic(meshFilename, outFilename);
    solveBiharmonic(meshFilename, outFilename);

    return EXIT_SUCCESS;
}
