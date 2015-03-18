#include "simplifyCommand.h"


bool SimplifyCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Simplify...\n";
    mesh.simplifyConstraints();
    mesh.deleteUnusedNodes();
    mesh.garbageCollection();
    return true;
}


const char* SimplifyCommand::cmdOptions()
{
    return "";
}


const char* SimplifyCommand::cmdDesc()
{
    return "Simplify the mesh by removing unused nodes and useless "
           "unconstrained nodes.";
}
