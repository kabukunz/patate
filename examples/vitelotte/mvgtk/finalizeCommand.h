#ifndef _VITELOTTE_EXAMPLE_MVGTK_FINALIZE_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_FINALIZE_COMMAND_


#include "mvgtk.h"


class FinalizeCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);
};


#endif
