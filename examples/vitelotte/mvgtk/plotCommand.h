#ifndef _VITELOTTE_EXAMPLE_MVGTK_PLOT_COMMAND_
#define _VITELOTTE_EXAMPLE_MVGTK_PLOT_COMMAND_


#include "mvgtk.h"


class PlotCommand : public MvgtkCommand
{
public:
    PlotCommand();

    virtual bool parseArgs(int argc, char** argv, int& argi);
    virtual bool execute(Mesh& mesh, const GlobalOptions* opts = 0);

private:
    unsigned m_layer;
    unsigned m_nSubdiv;
    std::string m_outFilename;
};


#endif
