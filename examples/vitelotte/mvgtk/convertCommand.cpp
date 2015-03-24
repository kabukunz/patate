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
        if(m_attributes == -1)
        {
            std::cerr << "Invalid attributes specification.\n";
            return false;
        }
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


const char* ConvertCommand::cmdOptions()
{
    return "ATTRIBUTES_DESC";
}


const char* ConvertCommand::cmdDesc()
{
    return "Set mesh attribute to ATTRIBUTE_DESC. Accepted values are none, "
           "linear, quadratic, morley and fv.";
}
