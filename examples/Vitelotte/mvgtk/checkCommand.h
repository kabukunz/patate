#ifndef _MVGTK_CHECK_COMMAND_
#define _MVGTK_CHECK_COMMAND_


#include "mvgtk.h"


class CheckCommand : public MvgtkCommand
{
public:
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();
};


#endif
