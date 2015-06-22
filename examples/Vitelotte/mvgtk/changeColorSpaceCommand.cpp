/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <iomanip>

#include "changeColorSpaceCommand.h"


ChangeColorSpaceCommand::ChangeColorSpaceCommand()
    : m_colorSpace(PatateCommon::COLOR_NONE)
{
}


bool ChangeColorSpaceCommand::parseArgs(int argc, char** argv, int& argi)
{
    if(argi < argc)
    {
        bool ok = false;
        m_colorSpace = PatateCommon::colorSpaceFromName(argv[argi++], &ok);
        if(!ok) {
            std::cerr << "Invalid color space specification.\n";
            return false;
        }
        return true;
    }
    return false;
}


bool ChangeColorSpaceCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Set color space to "
                                        << PatateCommon::getColorSpaceName(m_colorSpace) << "...\n";

    mesh.setColorSpace(m_colorSpace);

    return true;
}


const char* ChangeColorSpaceCommand::cmdOptions()
{
    return "COLOR_SPACE";
}


const char* ChangeColorSpaceCommand::cmdDesc()
{
    return "Set mesh color space to COLOR_SPACE. Accepted values are none, "
           "srgb, linear_srgb, cie_xyz and cie_lab. This does not modify node "
           "values; use convert_color_space for this.";
}
