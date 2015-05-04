/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
