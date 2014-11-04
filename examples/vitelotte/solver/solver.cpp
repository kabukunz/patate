#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include "Patate/vitelotte.h"
#include "Patate/Vitelotte/Utils/mvgReader.h"
#include "Patate/Vitelotte/Utils/mvgWriter.h"


typedef Vitelotte::FemMesh<float> Mesh;
typedef Vitelotte::MVGReader<Mesh> Reader;
typedef Vitelotte::MVGWriter<Mesh> Writer;


void usage(char* progName)
{
    std::cout << "usage: " << progName << " mesh output\n";
    exit(1);
}


void solveHarmonic(Mesh& mesh)
{
    //typedef Vitelotte::QuadraticElement<Mesh, double> Element;
    typedef Vitelotte::QuadraticElement<Mesh, double> QuadraticElement;
    typedef Vitelotte::SingularElementDecorator<QuadraticElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    Solver solver(&mesh);
    solver.build();
    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }
}


void solveBiharmonic(Mesh& mesh)
{
//    typedef Vitelotte::FVElementBuilder<Mesh, double> Element;
    typedef Vitelotte::FVElementBuilder<Mesh, double> FVElement;
    typedef Vitelotte::SingularElementDecorator<FVElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    Solver solver(&mesh);
    solver.build();
    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }
}


int main(int argc, char** argv)
{
    std::string meshFilename;
    std::string outFilename;

    if(argc == 3)
    {
        meshFilename = argv[1];
        outFilename = argv[2];
    }
    else
        usage(argv[0]);

    Mesh mesh;

    Vitelotte::readMvgFromFile(meshFilename, mesh);

    switch(mesh.getAttributes())
    {
    case Mesh::Quadratic:
        solveHarmonic(mesh);
        break;
    case Mesh::FV:
        solveBiharmonic(mesh);
        break;
    default:
        std::cerr << "Mesh type not supported.\n";
        return 3;
    }

    Vitelotte::writeMvgToFile(outFilename, mesh);

    return EXIT_SUCCESS;
}
