#include "convertCommand.h"


ConvertCommand::ConvertCommand()
    : m_attributes(-1)
{
}


bool ConvertCommand::parseArgs(int argc, char** argv, int& argi)
{
    if(argi < argc)
    {
        m_attributes = parseAttribSet(argv[argi++]);
        return true;
    }
    return false;
}


bool ConvertCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Set attributes to " << m_attributes << "...\n";
    mesh.setAttributes(m_attributes);
    return true;
}
