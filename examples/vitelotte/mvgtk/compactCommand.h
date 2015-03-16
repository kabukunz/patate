#ifndef _VITELOTTE_EXAMPLE_MVGTK_COMPACT_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_COMPACT_COMMAND_


#include "mvgtk.h"


class CompactCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();
};


#endif
