#include "compactCommand.h"


bool CompactCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Compact...\n";
    mesh.compactNodes();
    return true;
}


const char* CompactCommand::cmdOptions()
{
    return "";
}


const char* CompactCommand::cmdDesc()
{
    return "Remove unused nodes.";
}
