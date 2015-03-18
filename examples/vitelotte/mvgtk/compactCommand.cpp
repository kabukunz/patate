#include "compactCommand.h"


bool CompactCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Compact...\n";
    mesh.deleteUnusedNodes();
    if(opts && opts->verbose)
    {
        int nv = mesh.verticesSize() - mesh.nVertices();
        int ne = mesh.edgesSize()    - mesh.nEdges();
        int nf = mesh.facesSize()    - mesh.nFaces();
        int nn = mesh.nodesSize()    - mesh.nNodes();
        if(nv) std::cout << "Removing " << nv << " vertices.\n";
        if(ne) std::cout << "Removing " << ne << " edges.\n";
        if(nf) std::cout << "Removing " << nf << " faces.\n";
        if(nn) std::cout << "Removing " << nn << " nodes.\n";
    }
    mesh.garbageCollection();
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
