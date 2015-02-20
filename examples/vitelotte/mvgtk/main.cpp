#include <cstdlib>
#include <iostream>

#include "mvgtk.h"
#include "checkCommand.h"
#include "convertCommand.h"
#include "curvesToNodesCommand.h"
#include "finalizeCommand.h"
#include "plotCommand.h"
#include "simplifyCommand.h"
#include "solveCommand.h"


char* progName;
void usage(int returnValue = 127)
{
    std::cout << "usage: " << progName << " [OPTIONS] COMMAND [CMD-OPTIONS]\n"
"Mvg toolkit: a set of tools to manipulate mvg files.\n"
"\n"
"Commands:\n"
"  check IN      Check IN mvg to ensure everything is right.\n"
"  convert [-a ATTRS | --attrib ATTRS] IN OUT\n"
"                Convert a mesh IN from a supported format to an mvg file OUT,\n"
"                with attributes ATTRS. If ATTRS is not provided, keep the\n"
"                default attributes if IN is a mvg file or default to fv.\n"
"  finalize IN OUT\n"
"                Finalize the mvg by setting nodes on all faces.\n"
"\n"
"Options:\n"
"  -h, --help    Display this message.\n"
"  -v            Verbose. Print extra informations.\n"
"\n";
    exit(returnValue);
}






// Main -----------------------------------------------------------------------


int main(int argc, char** argv)
{
    progName = argv[0];

    Mvgtk mvgtk;

    mvgtk.registerCommand<OutputCommand>("out");
    mvgtk.registerCommand<OutputCommand>("output");

    mvgtk.registerCommand<CheckCommand>("check");
    mvgtk.registerCommand<ConvertCommand>("conv");
    mvgtk.registerCommand<ConvertCommand>("convert");
    mvgtk.registerCommand<FinalizeCommand>("finalize");
    mvgtk.registerCommand<CurvesToNodesCommand>("c2n");
    mvgtk.registerCommand<CurvesToNodesCommand>("curves-to-nodes");
    mvgtk.registerCommand<PlotCommand>("plot");
    mvgtk.registerCommand<SimplifyCommand>("simp");
    mvgtk.registerCommand<SimplifyCommand>("simplify");
    mvgtk.registerCommand<SolveCommand>("solve");

    if(!mvgtk.parseArgs(argc, argv)) usage();

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
