#ifndef _VITELOTTE_EXAMPLE_MVGTK_SIMPLIFY_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_SIMPLIFY_COMMAND_


#include "mvgtk.h"


class SimplifyCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);
};


#endif
