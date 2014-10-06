#include <cstdlib>
#include <iostream>
#include <string>

#include "Patate/vitelotte.h"


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

    Vitelotte::FemInMesh inMesh(Vitelotte::FemInMesh::Fraeijs);
    inMesh.read(meshFilename);
    inMesh.loadConstraintMap(attrFilename);
    inMesh.buildElements(true, true);

    Vitelotte::FemSolver solver(&inMesh);
    solver.buildMatrices(true);
    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        exit(2);
    }

    inMesh.saveTriangulationIntoQvg(outFilename);

    return EXIT_SUCCESS;
}
