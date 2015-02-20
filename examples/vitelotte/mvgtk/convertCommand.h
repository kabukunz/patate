#ifndef _VITELOTTE_EXAMPLE_MVGTK_CONVERT_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_CONVERT_COMMAND_


#include "mvgtk.h"


class ConvertCommand : public MvgtkCommand
{
public:
    ConvertCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

private:
    int m_attributes;
};


#endif
