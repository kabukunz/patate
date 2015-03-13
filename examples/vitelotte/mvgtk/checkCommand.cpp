#include "checkCommand.h"


bool CheckCommand::execute(Mesh& mesh, const GlobalOptions* opts)
{
    if(opts && opts->verbose) std::cout << "Check mesh...\n";
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
                if(opts && opts->verbose)
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

    return nError == 0;
}


const char* CheckCommand::cmdOptions()
{
    return "";
}


const char* CheckCommand::cmdDesc()
{
    return "Does some sanity check on the input mesh. Print diagnostic "
           "messages in case of problem and exit with a non-zero error code.";
}
