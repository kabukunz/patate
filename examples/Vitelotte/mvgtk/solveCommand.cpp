/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "solveCommand.h"


typedef double SolverScalar;

template <typename Solver>
bool solveGeneric(Mesh& mesh)
{
    Solver solver(&mesh);
    solver.build();

    if(solver.error().status() == Vitelotte::SolverError::STATUS_WARNING)
    {
        std::cerr << "Warning: " << solver.error().message() << "\n";
    }
    else if(solver.error().status() == Vitelotte::SolverError::STATUS_ERROR)
    {
        std::cerr << "Error: " << solver.error().message() << "\n";
        return false;
    }

    solver.solve();

    if(!solver.isSolved())
    {
        std::cerr << "Failed to solve the diffusion.\n";
        return false;
    }
    return true;
}


bool solveHarmonicLinear(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::LinearElementBuilder<Mesh, SolverScalar> LinearElement;
    typedef Vitelotte::SingularElementDecorator<LinearElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Harmonic linear diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveHarmonicQuadratic(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::QuadraticElementBuilder<Mesh, SolverScalar> QuadraticElement;
    typedef Vitelotte::SingularElementDecorator<QuadraticElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Harmonic quadratic diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveBiharmonicLinear(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::MorleyElementBuilder<Mesh, SolverScalar> MorleyElement;
    typedef Vitelotte::SingularElementDecorator<MorleyElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Biharmonic linear diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveBiharmonicQuadratic(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::FVElementBuilder<Mesh, SolverScalar> FVElement;
    typedef Vitelotte::SingularElementDecorator<FVElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Biharmonic quadratic diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool SolveCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Solve...\n";
    switch(mesh.getAttributes())
    {
    case Mesh::LINEAR_FLAGS:
        return solveHarmonicLinear(mesh, opts);
    case Mesh::QUADRATIC_FLAGS:
        return solveHarmonicQuadratic(mesh, opts);
    case Mesh::MORLEY_FLAGS:
        return solveBiharmonicLinear(mesh, opts);
    case Mesh::FV_FLAGS:
        return solveBiharmonicQuadratic(mesh, opts);
    }
    std::cerr << "Mesh type not supported.\n";
    return false;
}


const char* SolveCommand::cmdOptions()
{
    return "";
}


const char* SolveCommand::cmdDesc()
{
    return "Solve the diffusion problem, thus assigning a value to each node. "
           "The type of diffusion (harmonic or biharmonic) depend on the mesh "
           "attributes. The input mesh should not have invalid nodes. You can "
           "use the finalize command to produce an acceptable input mesh.";
}
