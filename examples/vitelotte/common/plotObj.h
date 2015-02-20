#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_PLOT_OBJ_
#define _EXAMPLES_VITELOTTE_COMMON_VG_PLOT_OBJ_


#include <fstream>

#include <Eigen/Core>


template <typename Elem>
void _printPlotElementVertices(std::ostream& out, const Elem& elem,
                              const Eigen::Matrix<float, 2, 3>& points,
                              const Eigen::VectorXf& nodeValues,
                              unsigned nSubdiv) {
    for(unsigned i = 0; i < nSubdiv+1; ++i)
    {
        for(unsigned j = 0; j < (i + 1); ++j)
        {
            Eigen::Vector3f bc;
            bc(0) = 1.f - float(i) / float(nSubdiv);
            bc(2) = float(j) / float(nSubdiv);
            bc(1) = 1.f - bc(0) - bc(2);

            float value = nodeValues.dot(elem.eval(bc));
            out << "v " << (points * bc).transpose() << " " << value << "\n";
        }
    }
}


template <typename Mesh, typename Elem>
void exportPlot(const Mesh& mesh, const std::string& filename,
                unsigned layer, unsigned nSubdiv)
{
    std::ofstream out(filename.c_str());

    unsigned vxPerEdge = nSubdiv + 1;
    unsigned vxPerFace = (vxPerEdge * (vxPerEdge + 1)) / 2;

    unsigned nNodesPerElem = 0;
    int toVertexValueOffset = -1;
    int edgeValueOffset     = -1;
    int edgeGradientOffset  = -1;
    if(mesh.hasToVertexValue()) {
        toVertexValueOffset = nNodesPerElem;
        nNodesPerElem += 3;
    }
    if(mesh.hasEdgeValue()) {
        edgeValueOffset = nNodesPerElem;
        nNodesPerElem += 3;
    }
    if(mesh.hasEdgeGradient()) {
        edgeGradientOffset = nNodesPerElem;
        nNodesPerElem += 3;
    }
    Eigen::VectorXf nodeValues(nNodesPerElem);

    for(typename Mesh::FaceIterator fit = mesh.facesBegin();
        fit != mesh.facesEnd(); ++fit)
    {
        Eigen::Matrix<float, 2, 3> points;

        typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(*fit);
        typename Mesh::HalfedgeAroundFaceCirculator hend = hit;

        for(unsigned i = 0; i < 3; ++i)
        {
            if(edgeValueOffset >= 0)
            {
                nodeValues(i + edgeValueOffset) =
                        mesh.nodeValue(mesh.edgeValueNode(*hit))(layer);
            }
            if(edgeGradientOffset >= 0)
            {
                nodeValues(i + edgeGradientOffset) =
                        mesh.nodeValue(mesh.edgeGradientNode(*hit))(layer);
                if(!mesh.halfedgeOrientation(*hit))
                    nodeValues(i + edgeGradientOffset) *= -1;
            }
            ++hit;
            points.col(i) = mesh.position(mesh.toVertex(*hit));
            if(toVertexValueOffset >= 0)
            {
                nodeValues(i + toVertexValueOffset) =
                        mesh.nodeValue(mesh.toVertexValueNode(*hit))(layer);
            }
        }

        _printPlotElementVertices(out, Elem(points.col(0), points.col(1), points.col(2)),
                                  points, nodeValues, nSubdiv);
    }

    unsigned count = 0;
    for(typename Mesh::FaceIterator fit = mesh.facesBegin();
        fit != mesh.facesEnd(); ++fit)
    {
        unsigned first = count * vxPerFace + 1; // first vertex is 1 in .obj
        unsigned v0 = first;
        unsigned v1 = first + 1;
        for(int i = 0; i < nSubdiv; ++i)
        {
            for(int j = 0; j < i; ++j)
            {
                out << "f " << v0 << " " << v1 << " " << v1 + 1 << "\n";
                out << "f " << v0 << " " << v1 + 1 << " " << v0 + 1 << "\n";
                ++v0;
                ++v1;
            }
            out << "f " << v0 << " " << v1 << " " << v1 + 1 << "\n";
            ++v0;
            v1 += 2;
        }
        ++count;
    }
}


#endif

