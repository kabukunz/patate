#include "solveCommand.h"


template <typename Solver>
bool solveGeneric(Mesh& mesh)
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
    typedef Vitelotte::LinearElementBuilder<Mesh, double> LinearElement;
    typedef Vitelotte::SingularElementDecorator<LinearElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Harmonic linear diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveHarmonicQuadratic(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::QuadraticElementBuilder<Mesh, double> QuadraticElement;
    typedef Vitelotte::SingularElementDecorator<QuadraticElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Harmonic quadratic diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveBiharmonicLinear(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::MorleyElementBuilder<Mesh, double> MorleyElement;
    typedef Vitelotte::SingularElementDecorator<MorleyElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> Solver;

    if(opts && opts->verbose) std::cout << "Biharmonic linear diffusion.\n";
    return solveGeneric<Solver>(mesh);
}


bool solveBiharmonicQuadratic(Mesh& mesh, const GlobalOptions* opts)
{
    typedef Vitelotte::FVElementBuilder<Mesh, double> FVElement;
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
