#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include "Patate/vitelotte.h"
#include "Patate/Vitelotte/Utils/qvgReader.h"
#include "Patate/Vitelotte/Utils/qvgWriter.h"


typedef Vitelotte::QuadraticMesh<float> Mesh;
typedef Vitelotte::QVGReader<Mesh> Reader;
typedef Vitelotte::QVGWriter<Mesh> Writer;

typedef Vitelotte::QuadraticElement<Mesh, double> Element;
typedef Vitelotte::FemSolver<Mesh, Element> Solver;

void usage(char* progName)
{
    std::cout << "usage: " << progName << " mesh attributes out\n";
    exit(1);
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

    return EXIT_SUCCESS;
}
