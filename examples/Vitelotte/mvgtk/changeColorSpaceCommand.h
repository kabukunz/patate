/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _MVGTK_CHANGE_COLOR_SPACE_COMMAND_
#define _MVGTK_CHANGE_COLOR_SPACE_COMMAND_


#include "mvgtk.h"


class ChangeColorSpaceCommand : public MvgtkCommand
{
public:
    ChangeColorSpaceCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();

private:
    Vitelotte::ColorSpace m_colorSpace;
};


#endif
