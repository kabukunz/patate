#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include "Patate/vitelotte.h"
#include "Patate/Vitelotte/Utils/mvgReader.h"
#include "Patate/Vitelotte/Utils/mvgWriter.h"


typedef Vitelotte::VGMesh<float> Mesh;
typedef Vitelotte::MVGReader<Mesh> Reader;
typedef Vitelotte::MVGWriter<Mesh> Writer;


void usage(char* progName)
{
    std::cout << "usage: " << progName << " mesh output\n";
    exit(1);
}


template <typename Solver>
void solveGeneric(Mesh& mesh)
{
    Solver solver(&mesh);
    solver.build();
    solver.sort();
    solver.factorize();

    if(solver.error().status() == Vitelotte::SolverError::STATUS_WARNING)
    {
        std::cerr << "Warning: " << solver.error().message() << "\n";
    }
    else if(solver.error().status() == Vitelotte::SolverError::STATUS_ERROR)
    {
        std::cerr << "Error: " << solver.error().message() << "\n";
        return;
    }

    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }
}


void solveHarmonicLinear(Mesh& mesh)
{
    typedef Vitelotte::LinearElementBuilder<Mesh, double> LinearElement;
    typedef Vitelotte::SingularElementDecorator<LinearElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    std::cout << "Harmonic linear diffusion.\n";
    solveGeneric<Solver>(mesh);
}


void solveHarmonicQuadratic(Mesh& mesh)
{
    typedef Vitelotte::QuadraticElementBuilder<Mesh, double> QuadraticElement;
    typedef Vitelotte::SingularElementDecorator<QuadraticElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    std::cout << "Harmonic quadratic diffusion.\n";
    solveGeneric<Solver>(mesh);
}


void solveBiharmonicLinear(Mesh& mesh)
{
    typedef Vitelotte::MorleyElementBuilder<Mesh, double> MorleyElement;
    typedef Vitelotte::SingularElementDecorator<MorleyElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    std::cout << "Biharmonic linear diffusion.\n";
    solveGeneric<Solver>(mesh);
}


void solveBiharmonicQuadratic(Mesh& mesh)
{
    typedef Vitelotte::FVElementBuilder<Mesh, double> FVElement;
    typedef Vitelotte::SingularElementDecorator<FVElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    std::cout << "Biharmonic quadratic diffusion.\n";
    solveGeneric<Solver>(mesh);
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
    case Mesh::LINEAR_FLAGS:
        solveHarmonicLinear(mesh);
        break;
    case Mesh::QUADRATIC_FLAGS:
        solveHarmonicQuadratic(mesh);
        break;
    case Mesh::MORLEY_FLAGS:
        solveBiharmonicLinear(mesh);
        break;
    case Mesh::FV_FLAGS:
        solveBiharmonicQuadratic(mesh);
        break;
    default:
        std::cerr << "Mesh type not supported.\n";
        return 3;
    }

    Vitelotte::writeMvgToFile(outFilename, mesh);

    return EXIT_SUCCESS;
}
