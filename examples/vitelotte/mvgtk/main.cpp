#include <cstdlib>
#include <iostream>

#include "mvgtk.h"
#include "checkCommand.h"
#include "compactCommand.h"
#include "convertCommand.h"
#include "curvesToNodesCommand.h"
#include "finalizeCommand.h"
#include "plotCommand.h"
#include "simplifyCommand.h"
#include "solveCommand.h"


// Main -----------------------------------------------------------------------


int main(int argc, char** argv)
{
    Mvgtk mvgtk;

    mvgtk.registerCommand<CheckCommand>("check");
    mvgtk.registerCommand<CompactCommand>("compact");
    mvgtk.registerCommand<ConvertCommand>("convert");
    mvgtk.registerCommand<CurvesToNodesCommand>("curves-to-nodes");
    mvgtk.registerCommand<FinalizeCommand>("finalize");
    mvgtk.registerCommand<OutputCommand>("output");
    mvgtk.registerCommand<PlotCommand>("plot");
    mvgtk.registerCommand<SimplifyCommand>("simplify");
    mvgtk.registerCommand<SolveCommand>("solve");

    mvgtk.addCommandAlias("output", "out");
    mvgtk.addCommandAlias("convert", "conv");
    mvgtk.addCommandAlias("curves-to-nodes", "c2n");
    mvgtk.addCommandAlias("simplify", "simp");

    mvgtk.parseArgs(argc, argv);
    return mvgtk.executeCommands()? EXIT_SUCCESS: EXIT_FAILURE;
}
