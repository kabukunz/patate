#include "mvgWriter.h"


namespace Vitelotte {

template < typename _Mesh >
void
MVGWriter<_Mesh>::write(std::ostream& _out) const
{
    typedef typename Mesh::VertexIterator VertexIterator;
    typedef typename Mesh::FaceIterator FaceIterator;
    typedef typename Mesh::HalfedgeAroundFaceCirculator
            HalfedgeAroundFaceCirculator;

    assert(m_version == Version1_0);

    // Ensure that the stream we read encode numbers using the C locale
    _out.imbue(std::locale::classic());

    int iOffset = 0;

    _out << "mvg 1.0\n";
    _out << "dim " << Mesh::Dim << "\n";
    _out << "parameters " << Mesh::Chan << "\n";

    if(m_mesh.getAttributes() == Mesh::Linear)
        _out << "linear\n";
    else if(m_mesh.getAttributes() == Mesh::Quadratic)
        _out << "quadratic\n";
    else if(m_mesh.getAttributes() == Mesh::Morley)
        _out << "morley\n";
    else if(m_mesh.getAttributes() == Mesh::FV)
        _out << "fv\n";
    else
        _out << "mesh " << m_mesh.getAttributes() << "\n";

    _out << "vertices " << m_mesh.nVertices() << "\n";
    _out << "nodes " << m_mesh.nNodes() << "\n";
    _out << "faces " << m_mesh.nFaces() << "\n";

    for(VertexIterator vit = m_mesh.verticesBegin();
        vit != m_mesh.verticesEnd(); ++vit)
    {
        _out << "v " << m_mesh.position(*vit).transpose() << "\n";
    }

    for(unsigned i = 0; i < m_mesh.nNodes(); ++i)
    {
        if(m_mesh.isConstraint(Node(i)))
            _out << "n " << m_mesh.nodeValue(Node(i)).transpose() << "\n";
        else
            _out << "n void\n";
    }

    for(FaceIterator fit = m_mesh.facesBegin();
         fit != m_mesh.facesEnd(); ++fit)
    {
        _out << (m_mesh.isSingular(*fit)? "fs": "f");

        HalfedgeAroundFaceCirculator
                hit  = m_mesh.halfedges(*fit),
                hend = hit;
        do
        {
            _out << " " << m_mesh.toVertex(*hit).idx() + iOffset;

            if(m_mesh.hasVertexValue())
            {
                Node vn = m_mesh.vertexValueNode(*hit);
                _out << "/";
                printNode(_out, vn);

                // VertexFromValue only makes sense if vertexValue in enable.
                if(m_mesh.hasVertexFromValue())
                {
                    Node fn = m_mesh.vertexFromValueNode(m_mesh.nextHalfedge(*hit));
                    if(vn != fn)
                    {
                        _out << "/";
                        printNode(_out, fn);
                    }
                }
            }
        }
        while(++hit != hend);

        if(m_mesh.hasEdgeValue() || m_mesh.hasEdgeGradient())
        {
            _out << " -";

            --hit; hend = hit;
            do
            {
                char sep = ' ';
                if(m_mesh.hasEdgeValue())
                {
                    _out << sep;
                    sep = '/';
                    printNode(_out, m_mesh.edgeValueNode(*hit));
                }
                if(m_mesh.hasEdgeGradient())
                {
                    _out << sep;
                    sep = '/';
                    printNode(_out, m_mesh.edgeGradientNode(*hit));
                }
            }
            while(++hit != hend);
        }

        _out << "\n";
    }
}

template < typename _Mesh >
void
MVGWriter<_Mesh>::printNode(std::ostream& _out, Node n) const
{
    if(n.isValid())
        _out << n.idx();
    else
        _out << "x";
}


template < typename Mesh >
void writeMvg(std::ostream& out, const Mesh& mesh,
              typename MVGWriter<Mesh>::Version version)
{
    MVGWriter<Mesh> writer(mesh, version);
    writer.write(out);
}

template < typename Mesh >
void writeMvgToFile(const std::string& filename, const Mesh& mesh,
                    typename MVGWriter<Mesh>::Version version)
{
    std::ofstream out(filename.c_str());
    writeMvg(out, mesh, version);
}


}  // namespace Vitelotte
