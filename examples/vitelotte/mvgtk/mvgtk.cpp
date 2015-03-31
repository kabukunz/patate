#include <fstream>
#include <iostream>

#include <Patate/common/surface_mesh/objReader.h>
#include <Patate/vitelotte_io.h>

#include "../common/textFormatter.h"
#include "../common/vgMeshWithCurvesReader.h"
#include "../common/vgMeshWithCurvesWriter.h"

#include "mvgtk.h"


int parseAttribSet(const std::string& attr)
{
    if(attr == "none")
        return 0;
    if(attr == "linear")
        return Mesh::LINEAR_FLAGS;
    if(attr == "quadratic")
        return Mesh::QUADRATIC_FLAGS;
    if(attr == "morley")
        return Mesh::MORLEY_FLAGS;
    if(attr == "fv")
        return Mesh::FV_FLAGS;
    return -1;
}


// ////////////////////////////////////////////////////////////////////////////


MvgtkCommand::~MvgtkCommand()
{
}


bool MvgtkCommand::parseArgs(int /*argc*/, char** /*argv*/, int& /*argi*/)
{
    return true;
}


// ////////////////////////////////////////////////////////////////////////////


bool OutputCommand::parseArgs(int argc, char** argv, int& argi)
{
    if(argi < argc) {
        m_outFilename = argv[argi++];
        return true;
    }
    return false;
}


bool OutputCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts->verbose) std::cout << "Output \"" << m_outFilename << "\"...\n";
    std::ofstream out(m_outFilename.c_str());
    if(!out.good())
    {
        std::cerr << "Unable to open \"" << m_outFilename << "\".";
        return false;
    }
    VGMeshWithCurveWriter().write(out, mesh);
    return true;
}


const char* OutputCommand::cmdOptions()
{
    return "MVG_FILE";
}


const char* OutputCommand::cmdDesc()
{
    return "Output the mesh in MVG_FILE.";
}


// ////////////////////////////////////////////////////////////////////////////


Mvgtk::Mvgtk()
{
    opts.verbose = false;

    m_argMap.insert(std::make_pair("-h", GlobalHelp));
    m_argMap.insert(std::make_pair("--help", GlobalHelp));
    m_argMap.insert(std::make_pair("-v", GlobalVerbose));
    m_argMap.insert(std::make_pair("--verbose", GlobalVerbose));
}


Mvgtk::~Mvgtk()
{
    for(FactoryList::iterator it = m_factories.begin();
        it != m_factories.end(); ++it)
    {
        delete *it;
    }
    for(CommandList::iterator it = m_commands.begin();
        it != m_commands.end(); ++it)
    {
        delete *it;
    }
}


void Mvgtk::addCommandAlias(const std::string& command, const std::string& alias)
{
    FactoryMap::iterator it = m_factoryMap.find(command);
    assert(it != m_factoryMap.end());
    bool isNew = m_factoryMap.insert(std::make_pair(alias, it->second)).second;
    assert(isNew);
}


void Mvgtk::printUsage(std::ostream& out, int exitCode) const
{
    // TODO: add a simple, more or less portable way to get terminal width.
    unsigned width = 70;

    std::ostringstream tmpOut;
    tmpOut << "Usage: " << m_progName << " [OPTIONS] INPUT_FILE COMMAND [CMD-OPTION] [COMMAND [CMD-OPTION] [...]]\n";
    format(out, tmpOut.str().c_str(), width, 8, 0);
    format(out,
           "Mvg toolkit: a set of tools to manipulate mvg files.\n"
           "\n"
           "INPUT_FILE can be an .mvg file or a .obj. The mesh is then "
           "processed successively by commands in order. Each command take as "
           "input the mesh produced by the previous one. A mesh can be writen "
           "with the output command.",
           width, 0);
    out << "\n";
    out << "Commands:\n";

    for(FactoryList::const_iterator fit = m_factories.begin();
        fit != m_factories.end(); ++fit)
    {
        tmpOut.str(std::string());
        tmpOut.clear();
        // Crappy complexity, but who cares ?
        std::vector<std::string> cmds;
        for(FactoryMap::const_iterator lfit = m_factoryMap.begin();
            lfit != m_factoryMap.end(); ++lfit)
        {
            if(lfit->second == *fit) cmds.push_back(lfit->first);
        }
        for(unsigned i = 0; i < cmds.size() - 1; ++i)
            format(out, cmds[i].c_str(), width, 10, 2);
        tmpOut << cmds.back() << " " << (*fit)->cmdOptions();
        format(out, tmpOut.str().c_str(), width, 10, 2);
        format(out, (*fit)->cmdDesc(), width, 6);
        out << "\n";
    }

    std::exit(exitCode);
}


bool Mvgtk::parseArgs(int argc, char** argv)
{
    m_progName = argv[0];
    int argi = 1;
    std::string arg;
    for(; argi < argc; ++argi)
    {
        arg = argv[argi];
        if(arg.empty()) printUsage(std::cerr);
        else if(arg[0] == '-')
        {
            ArgMap::iterator opt = m_argMap.find(arg);
            if(opt == m_argMap.end()) return false;

            switch((*opt).second)
            {
            case GlobalHelp:
                printUsage(std::cout, 0);
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
            if(m_inFilename.empty())
            {
                m_inFilename = arg;
            }
            else
            {
                break;
            }
        }
    }
    while(argi < argc)
    {
        arg = argv[argi++];
        FactoryMap::iterator it = m_factoryMap.find(arg);
        if(it == m_factoryMap.end()) printUsage(std::cerr);

        MvgtkCommand* cmd = it->second->create();
        m_commands.push_back(cmd);
        if(!cmd->parseArgs(argc, argv, argi)) printUsage(std::cerr);
    }

    return true;
}


bool Mvgtk::executeCommands()
{
    Mesh mesh;

    bool readFailed = false;
    if(m_inFilename.rfind(".obj") == m_inFilename.length() - 4)
    {
        if(opts.verbose) std::cout << "Load obj \"" << m_inFilename << "\"...\n";
        std::ifstream in(m_inFilename.c_str());
        PatateCommon::OBJReader<Mesh> reader;
        readFailed = reader.read(in, mesh);
    }
    else if(m_inFilename.rfind(".mvg") == m_inFilename.length() - 4)
    {
        if(opts.verbose) std::cout << "Load mvg \"" << m_inFilename << "\"...\n";
        VGMeshWithCurveReader reader;
        std::ifstream in(m_inFilename.c_str());
        readFailed = reader.read(in, mesh);
    }
    else
    {
        std::cerr << "Unrecoginzed file type, aborting.\n";
        return false;
    }

    if(readFailed)
    {
        return false;
    }

    for(CommandList::iterator it = m_commands.begin();
        it != m_commands.end(); ++it)
    {
        if(!(*it)->execute(mesh, &opts))
            return false;
    }

    return true;
}


Mvgtk::CommandFactory::~CommandFactory()
{
}

