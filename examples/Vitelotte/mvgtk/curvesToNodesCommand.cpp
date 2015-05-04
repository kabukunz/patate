/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "curvesToNodesCommand.h"


bool CurvesToNodesCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Set nodes from curves...\n";

    mesh.setNodesFromCurves();

    return true;
}


const char* CurvesToNodesCommand::cmdOptions()
{
    return "";
}


const char* CurvesToNodesCommand::cmdDesc()
{
    return "Clear all nodes then set constrained nodes with respect to curves "
           "and point constraints. You likely want to call finalize after.";
}
