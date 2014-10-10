#ifndef _OBJWRITER_H_
#define _OBJWRITER_H_


#include <stdexcept>
#include <ostream>
#include <fstream>

#include "surfaceMesh.h"


namespace Patate
{


template < typename _Point >
class OBJWriter
{
public:
    typedef _Point Point;

public:
    inline OBJWriter(const SurfaceMesh& mesh,
                     const SurfaceMesh::VertexProperty<Point>& positions);

    void write(std::ostream& out);

private:
    const SurfaceMesh& m_mesh;
    const SurfaceMesh::VertexProperty<Point>& m_vPos;
};


#include "objWriter.hpp"

}  // namespace Patate

#endif  // _OBJWRITER_H_
