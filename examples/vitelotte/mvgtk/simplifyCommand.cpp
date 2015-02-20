#include "simplifyCommand.h"


bool SimplifyCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Simplify...\n";
    mesh.simplifyConstraints();
    mesh.compactNodes();
    return true;
}


