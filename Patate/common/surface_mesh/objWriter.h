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
    inline OBJWriter(SurfaceMesh& mesh,
                     SurfaceMesh::VertexProperty<Point> positions);

    void write(std::ostream& out);

private:
    SurfaceMesh& m_mesh;
    SurfaceMesh::VertexProperty<Point> m_vPos;
};


#include "objWriter.hpp"

}  // namespace Patate

#endif  // _OBJWRITER_H_
