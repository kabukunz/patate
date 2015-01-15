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


typedef std::map<std::string, int> ArgMap;

enum
{
    Check,
    Convert,
    Finalize,
    Simplify
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
        return Mesh::LINEAR_FLAGS;
    if(attr == "quadratic")
        return Mesh::QUADRATIC_FLAGS;
    if(attr == "morley")
        return Mesh::MORLEY_FLAGS;
    if(attr == "fv")
        return Mesh::FV_FLAGS;
    return -1;
}

// Check command --------------------------------------------------------------


int check(int argc, char** argv, const GlobalOptions& opts)
{
    if(argc != 2)
        usage();

    Mesh mesh;
    Vitelotte::readMvgFromFile(argv[1], mesh);

    bool nError = 0;
    for(Mesh::FaceIterator fit = mesh.facesBegin();
        fit != mesh.facesEnd(); ++fit)
    {
        if(mesh.valence(*fit) != 3)
        {
            std::cout << "face " << (*fit).idx() << " is not triangular (valence "
                      << mesh.valence(*fit) << ")\n";
            ++nError;
        }
        else
        {
            Mesh::Halfedge h = mesh.halfedge(*fit);
            Mesh::Vector p0 = mesh.position(mesh.toVertex(h));
            h = mesh.nextHalfedge(h);
            Mesh::Vector p1 = mesh.position(mesh.toVertex(h));
            h = mesh.nextHalfedge(h);
            Mesh::Vector p2 = mesh.position(mesh.toVertex(h));
            Eigen::Matrix2f m; m << (p1-p0), (p2-p1);
            float det = m.determinant();
            if(det <= 0)
            {
                std::cout << "face " << (*fit).idx() << " is degenerate or oriented clockwise.\n";
                ++nError;
                if(opts.verbose)
                {
                    std::cout << "  Area: " << det / 2. << "\n";

                    Mesh::Halfedge hend = h;
                    do
                    {
                        std::cout << "  v" << mesh.toVertex(h).idx() << ": "
                                  << mesh.position(mesh.toVertex(h)).transpose() << "\n";
                        h = mesh.nextHalfedge(h);
                    } while(h != hend);
                }
            }
        }

        Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(*fit);
        Mesh::HalfedgeAroundFaceCirculator hend = hit;
        int singularCount = 0;
        do
        {
            if(mesh.hasToVertexValue() && mesh.hasFromVertexValue())
            {
                Mesh::Node n0 = mesh.toVertexValueNode(*hit);
                Mesh::Node n1 = mesh.fromVertexValueNode(mesh.nextHalfedge(*hit));
                bool n0c = n0.isValid() && mesh.isConstraint(n0);
                bool n1c = n1.isValid() && mesh.isConstraint(n1);

                if(n0c && n1c && n0 != n1)
                {
                    ++singularCount;
                }
            }

            if(mesh.hasToVertexValue())
            {
                Mesh::Node n = mesh.toVertexValueNode(*hit);
                if(n.isValid())
                {
                    if(!mesh.isValid(n))
                    {
                        std::cout << "face " << (*fit).idx()
                                  << " has an out-of-bound to-vertex value node.\n";
                        ++nError;
                    }
                }
            }
            if(mesh.hasFromVertexValue())
            {
                Mesh::Node n = mesh.fromVertexValueNode(*hit);
                if(n.isValid())
                {
                    if(!mesh.isValid(n))
                    {
                        std::cout << "face " << (*fit).idx()
                                  << " has an out-of-bound from-vertex value node.\n";
                        ++nError;
                    }
                }
            }
            if(mesh.hasEdgeValue())
            {
                Mesh::Node n = mesh.edgeValueNode(*hit);
                if(n.isValid())
                {
                    if(!mesh.isValid(n))
                    {
                        std::cout << "face " << (*fit).idx()
                                  << " has an out-of-bound edge value node.\n";
                        ++nError;
                    }
                }
            }
            if(mesh.hasEdgeGradient())
            {
                Mesh::Node n = mesh.edgeGradientNode(*hit);
                if(n.isValid())
                {
                    if(!mesh.isValid(n))
                    {
                        std::cout << "face " << (*fit).idx()
                                  << " has an out-of-bound edge gradient node.\n";
                        ++nError;
                    }
                }
            }
            ++hit;
        } while(hit != hend);

        if(singularCount > 1)
        {
            std::cout << "face " << (*fit).idx() << " has " << singularCount
                      << " singular faces (maximum supported by solvers and viewer is 1).\n";
            ++nError;
        }
    }

    return nError > 0;
}


// Convert command ------------------------------------------------------------


enum MeshType
{
    MeshObj,
    MeshMvg
};

int convert(int argc, char** argv)
{
    int attribs = -1;
    std::string meshFilename;
    std::string outFilename;

    int argi = 1;
    for(; argi < argc; ++argi)
    {
        std::string arg(argv[argi]);
        if(arg[0] == '-')
        {
            if(arg == "-a" || arg == "--attributes")
            {
                attribs = parseAttribSet(argv[++argi]);
                if(attribs < 0)
                    usage();
            }
            else
                usage();
        }
        else if(meshFilename.empty())
            meshFilename = arg;
        else if(outFilename.empty())
            outFilename = arg;
        else
            usage();
    }

    MeshType meshType;
    if(meshFilename.rfind(".obj") == meshFilename.length() - 4)
    {
        meshType = MeshObj;
    }
    else if(meshFilename.rfind(".mvg") == meshFilename.length() - 4)
    {
        meshType = MeshMvg;
    }
    else
    {
        std::cerr << "Unrecoginzed file type, aborting.\n";
        exit(1);
    }

    if(attribs < 0 && meshType != MeshMvg)
        attribs = Mesh::FV_FLAGS;

    Mesh mesh;

    switch(meshType)
    {
    case MeshObj:
    {
        std::ifstream in(meshFilename.c_str());
        PatateCommon::OBJReader<Mesh> reader;
        reader.read(in, mesh);

        break;
    }
    case MeshMvg:
    {
        Vitelotte::readMvgFromFile(meshFilename, mesh);
        break;
    }
    }

    if(attribs != -1)
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


// Simplify command -----------------------------------------------------------


int simplify(int argc, char** argv)
{
    if(argc != 3)
        usage();

    std::string meshFilename(argv[1]);
    std::string outFilename(argv[2]);

    Mesh mesh;

    Vitelotte::readMvgFromFile(meshFilename, mesh);

    mesh.simplifyConstraints();
    mesh.compactNodes();

    Vitelotte::writeMvgToFile(outFilename, mesh);

    return 0;
}


// Main -----------------------------------------------------------------------


int main(int argc, char** argv)
{
    progName = argv[0];

    ArgMap commands;
    addOpt(commands, "check", Check);
    addOpt(commands, "convert", Convert);
    addOpt(commands, "finalize", Finalize);
    addOpt(commands, "simplify", Simplify);

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
    case Check:
        return check(argc - argi, argv + argi, opts);
    case Convert:
        return convert(argc - argi, argv + argi);
    case Finalize:
        return finalize(argc - argi, argv + argi);
    case Simplify:
        return simplify(argc - argi, argv + argi);
    default:
        assert(false);
    }

    return EXIT_SUCCESS;
}
