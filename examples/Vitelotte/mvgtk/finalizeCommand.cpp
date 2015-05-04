/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
