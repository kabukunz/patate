#ifndef _VITELOTTE_EXAMPLE_MVGTK_CURVES_TO_NODES_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_CURVES_TO_NODES_COMMAND_


#include "mvgtk.h"


class CurvesToNodesCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);
};


#endif
