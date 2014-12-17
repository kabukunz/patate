#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <map>

#include "Patate/common/surface_mesh/objReader.h"

#include "Patate/vitelotte.h"
#include "Patate/Vitelotte/Utils/mvgReader.h"
#include "Patate/Vitelotte/Utils/mvgWriter.h"


// Usage and options stuff ----------------------------------------------------

char* progName;
void usage(int returnValue = 127)
{
    std::cout << "usage: " << progName << " [OPTIONS] COMMAND [CMD-OPTIONS]\n"
    "Mvg toolkit: a set of tools to manipulate mvg files.\n"
    "\n"
    "Commands:\n"
    "  convert       Generate a mvg from a mesh in another format.\n"
    "  finalize      Finalize the mvg by setting nodes on all faces.\n"
    "\n"
    "Options:\n"
    "  -h, --help    Display this message.\n"
    "  -v            Verbose. Print extra informations.\n"
    "\n";
    exit(returnValue);
}


typedef std::map<std::string, int> ArgMap;

enum
{
    Convert,
    Finalize
};

enum
{
    GlobalHelp,
    GlobalVerbose
};

struct GlobalOptions
{
    bool verbose;
};

void addOpt(ArgMap& map, const std::string& key, int value)
{
    map.insert(std::make_pair(key, value));
}


// Global stuff ---------------------------------------------------------------


typedef Vitelotte::VGMesh<float> Mesh;
typedef Vitelotte::MVGReader<Mesh> Reader;
typedef Vitelotte::MVGWriter<Mesh> Writer;


int parseAttribSet(const std::string& attr)
{
    if(attr == "linear")
        return Mesh::Linear;
    if(attr == "quadratic")
        return Mesh::Quadratic;
    if(attr == "morley")
        return Mesh::Morley;
    if(attr == "fv")
        return Mesh::FV;
    return -1;
}

// Convert command ------------------------------------------------------------


int convert(int argc, char** argv)
{
    if(argc != 3 && argc != 4)
        usage();

    int argi = 1;

    int attribs = Mesh::FV;
    if(argc == 4)
        attribs = parseAttribSet(argv[argi++]);
    std::string meshFilename(argv[argi++]);
    std::string outFilename(argv[argi++]);

    if(attribs < 0)
        usage();

    Mesh mesh;

    std::ifstream in(meshFilename.c_str());
    PatateCommon::OBJReader<Mesh::Vector> reader(mesh, mesh.positionProperty());
    reader.read(in);

    mesh.setAttributes(attribs);

    Vitelotte::writeMvgToFile(outFilename, mesh);

    return 0;
}


// Finalize command -----------------------------------------------------------


int finalize(int argc, char** argv)
{
    if(argc != 3)
        usage();

    std::string meshFilename(argv[1]);
    std::string outFilename(argv[2]);

    Mesh mesh;

    Vitelotte::readMvgFromFile(meshFilename, mesh);

    mesh.finalize();

    Vitelotte::writeMvgToFile(outFilename, mesh);

    return 0;
}


// Main -----------------------------------------------------------------------


int main(int argc, char** argv)
{
    progName = argv[0];

    ArgMap commands;
    addOpt(commands, "convert", Convert);
    addOpt(commands, "finalize", Finalize);

    ArgMap globalOptions;
    addOpt(globalOptions, "-h", GlobalHelp);
    addOpt(globalOptions, "--help", GlobalHelp);
    addOpt(globalOptions, "-v", GlobalVerbose);

    GlobalOptions opts;
    opts.verbose = false;

    int command = -1;

    int argi = 1;
    for(; argi < argc; ++argi)
    {
        std::string arg(argv[argi]);

        if(arg.empty()) usage();
        else if(arg[0] == '-')
        {
            ArgMap::iterator opt = globalOptions.find(arg);
            if(opt == globalOptions.end())
                usage();

            switch((*opt).second)
            {
            case GlobalHelp:
                usage(0);
                break;
            case GlobalVerbose:
                opts.verbose = true;
                break;
            default:
                assert(false);
            }
        }
        else
        {
            ArgMap::iterator cmd = commands.find(arg);
            if(cmd == commands.end())
                usage();

            command = (*cmd).second;
            break;
        }
    }

    if(command < 0)
        usage();

    switch(command)
    {
    case Convert:
        return convert(argc - argi, argv + argi);
    case Finalize:
        return finalize(argc - argi, argv + argi);
    default:
        assert(false);
    }

    return EXIT_SUCCESS;
}
