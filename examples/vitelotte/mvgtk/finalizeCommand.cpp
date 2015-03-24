#include "finalizeCommand.h"


bool FinalizeCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Finalize...\n";

    mesh.finalize();

    return true;
}


const char* FinalizeCommand::cmdOptions()
{
    return "";
}


const char* FinalizeCommand::cmdDesc()
{
    return "Finalize the mesh by adding nodes where needed.";
}
