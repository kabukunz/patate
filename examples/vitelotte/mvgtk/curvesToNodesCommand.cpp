#include "curvesToNodesCommand.h"


bool CurvesToNodesCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Set nodes from curves...\n";
    mesh.setNodesFromCurves();
    return true;
}
