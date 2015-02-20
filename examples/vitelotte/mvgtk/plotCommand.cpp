#include "../common/plotObj.h"

#include "plotCommand.h"


PlotCommand::PlotCommand()
    : m_layer(0),
      m_nSubdiv(4),
      m_outFilename()
{
}


bool PlotCommand::parseArgs(int argc, char** argv, int& argi)
{
    if(argi+2 < argc)
    {
        // TODO: proper argument parsing.
        m_layer = std::atoi(argv[argi++]);
        m_nSubdiv = std::atoi(argv[argi++]);
        m_outFilename = argv[argi++];
        return true;
    }
    return false;
}


bool PlotCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Output plot \"" << m_outFilename << "\"...\n";
    switch(mesh.getAttributes())
    {
    case Mesh::LINEAR_FLAGS:
        exportPlot<Mesh, Vitelotte::LinearElement<float> >(
                    mesh, m_outFilename, m_layer, m_nSubdiv);
        break;
    case Mesh::QUADRATIC_FLAGS:
        exportPlot<Mesh, Vitelotte::QuadraticElement<float> >(
                    mesh, m_outFilename, m_layer, m_nSubdiv);
        break;
    case Mesh::MORLEY_FLAGS:
        exportPlot<Mesh, Vitelotte::MorleyElement<float> >(
                    mesh, m_outFilename, m_layer, m_nSubdiv);
        break;
    case Mesh::FV_FLAGS:
        exportPlot<Mesh, Vitelotte::FVElement<float> >(
                    mesh, m_outFilename, m_layer, m_nSubdiv);
        break;
    default:
        std::cerr << "Element type not supported.\n";
        return false;
    }
    return true;
}

