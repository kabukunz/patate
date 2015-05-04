#ifndef _VITELOTTE_EXAMPLE_MVGTK_SOLVE_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_SOLVE  _COMMAND_


#include "mvgtk.h"


class SolveCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();
};


#endif
