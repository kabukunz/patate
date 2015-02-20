#include "finalizeCommand.h"


bool FinalizeCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Finalize...\n";
    mesh.finalize();
    return true;
}


