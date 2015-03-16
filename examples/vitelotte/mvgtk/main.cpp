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

//    ArgMap commands;
//    addOpt(commands, "check", Check);
//    addOpt(commands, "convert", Convert);
//    addOpt(commands, "finalize", Finalize);
//    addOpt(commands, "simplify", Simplify);

//    ArgMap globalOptions;
//    addOpt(globalOptions, "-h", GlobalHelp);
//    addOpt(globalOptions, "--help", GlobalHelp);
//    addOpt(globalOptions, "-v", GlobalVerbose);

//    GlobalOptions opts;
//    opts.verbose = false;

//    std::string inFilename;

//    int argi = 1;
//    for(; argi < argc; ++argi)
//    {
//        std::string arg(argv[argi]);

//        if(arg.empty()) usage();
//        else if(arg[0] == '-')
//        {
//            ArgMap::iterator opt = globalOptions.find(arg);
//            if(opt == globalOptions.end())
//                usage();

//            switch((*opt).second)
//            {
//            case GlobalHelp:
//                usage(0);
//                break;
//            case GlobalVerbose:
//                opts.verbose = true;
//                break;
//            default:
//                assert(false);
//            }
//        }
//        else
//        {
//            if(inFilename.empty())
//            {
//                inFilename = arg;
//            }
//            else
//            {
//                break;
//            }
//        }
//    }

//    if(inFilename.empty()) usage();

//    std::list<MvgtkCommand*> cmdList;
//    while(argi < argc)
//    {
//        ArgMap::iterator cmd = commands.find(arg);
//        if(cmd == commands.end())
//            usage();

//        MvgtkCommand* cmd =
//    }

//    command = (*cmd).second;
//    if(command < 0)
//        usage();

//    switch(command)
//    {
//    case Check:
//        return check(argc - argi, argv + argi, opts);
//    case Convert:
//        return convert(argc - argi, argv + argi);
//    case Finalize:
//        return finalize(argc - argi, argv + argi);
//    case Simplify:
//        return simplify(argc - argi, argv + argi);
//    default:
//        assert(false);
//    }
}
