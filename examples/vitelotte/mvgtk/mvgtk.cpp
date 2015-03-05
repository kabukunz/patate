#include <fstream>

#include <Patate/common/surface_mesh/objReader.h>
#include <Patate/vitelotte_io.h>

#include "../common/vgMeshWithCurvesReader.h"

#include "mvgtk.h"


int parseAttribSet(const std::string& attr)
{
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


MvgtkCommand::~MvgtkCommand()
{
}


bool MvgtkCommand::parseArgs(int /*argc*/, char** /*argv*/, int& /*argi*/)
{
    return true;
}


Mvgtk::Mvgtk()
{
    opts.verbose = false;

    m_argMap.insert(std::make_pair("-h", GlobalHelp));
    m_argMap.insert(std::make_pair("--help", GlobalHelp));
    m_argMap.insert(std::make_pair("-v", GlobalVerbose));
    m_argMap.insert(std::make_pair("--verbose", GlobalVerbose));
}


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
    Vitelotte::writeMvgToFile(m_outFilename, mesh);
    return true;
}


Mvgtk::~Mvgtk()
{
    for(FactoryMap::iterator it = m_factories.begin();
        it != m_factories.end(); ++it)
        delete it->second;
    for(CommandList::iterator it = m_commands.begin();
        it != m_commands.end(); ++it)
        delete *it;
}


bool Mvgtk::parseArgs(int argc, char** argv)
{
    int argi = 1;
    std::string arg;
    for(; argi < argc; ++argi)
    {
        arg = argv[argi];
        if(arg.empty()) return false;
        else if(arg[0] == '-')
        {
            ArgMap::iterator opt = m_argMap.find(arg);
            if(opt == m_argMap.end()) return false;

            switch((*opt).second)
            {
            case GlobalHelp:
                return false;
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
        FactoryMap::iterator it = m_factories.find(arg);
        if(it == m_factories.end()) return false;

        MvgtkCommand* cmd = it->second->create();
        m_commands.push_back(cmd);
        if(!cmd->parseArgs(argc, argv, argi)) return false;
    }

    return true;
}


bool Mvgtk::executeCommands()
{
    Mesh mesh;

    if(m_inFilename.rfind(".obj") == m_inFilename.length() - 4)
    {
        if(opts.verbose) std::cout << "Load obj \"" << m_inFilename << "\"...\n";
        std::ifstream in(m_inFilename.c_str());
        PatateCommon::OBJReader<Mesh> reader;
        reader.read(in, mesh);
    }
    else if(m_inFilename.rfind(".mvg") == m_inFilename.length() - 4)
    {
        if(opts.verbose) std::cout << "Load mvg \"" << m_inFilename << "\"...\n";
        VGMeshWithCurveReader reader;
        std::ifstream in(m_inFilename.c_str());
        reader.read(in, mesh);
    }
    else
    {
        std::cerr << "Unrecoginzed file type, aborting.\n";
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

