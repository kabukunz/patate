#ifndef _MVGTK_CURVES_TO_NODES_COMMAND_
#define _MVGTK_CURVES_TO_NODES_COMMAND_


#include "mvgtk.h"


class CurvesToNodesCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();
};


#endif
