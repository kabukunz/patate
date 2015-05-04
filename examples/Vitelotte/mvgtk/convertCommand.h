#ifndef _MVGTK_CONVERT_COMMAND_
#define _MVGTK_CONVERT_COMMAND_


#include "mvgtk.h"


class ConvertCommand : public MvgtkCommand
{
public:
    ConvertCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

    static const char* cmdOptions();
    static const char* cmdDesc();

private:
    int m_attributes;
};


#endif
