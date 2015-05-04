#ifndef _MVGTK_SIMPLIFY_COMMAND_
#define _MVGTK_SIMPLIFY_COMMAND_


#include "mvgtk.h"


class SimplifyCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();
};


#endif
