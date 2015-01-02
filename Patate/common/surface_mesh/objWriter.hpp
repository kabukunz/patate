#include "objWriter.h"


namespace PatateCommon
{


template < typename _Point >
OBJWriter<_Point>::OBJWriter(const SurfaceMesh& mesh,
                             const SurfaceMesh::VertexProperty<Point>& positions)
    : m_mesh(mesh), m_vPos(positions)
{
}

template < typename _Point >
void
OBJWriter<_Point>::write(std::ostream& out)
{
    out.imbue(std::locale::classic());

    //vertices
    for (SurfaceMesh::VertexIterator vit = m_mesh.verticesBegin();
         vit != m_mesh.verticesEnd(); ++vit)
    {
        out << "v " << m_vPos[*vit].transpose() << "\n";
    }

    //faces
    for (SurfaceMesh::FaceIterator fit = m_mesh.facesBegin();
         fit != m_mesh.facesEnd(); ++fit)
    {
        out << "f";
        SurfaceMesh::VertexAroundFaceCirculator
                fvit  = m_mesh.vertices(*fit),
                fvend = fvit;
        do
        {
            out << " " << (*fvit).idx()+1;
        }
        while (++fvit != fvend);
        out << "\n";
    }
}


}
