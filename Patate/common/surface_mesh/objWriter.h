#ifndef _OBJWRITER_H_
#define _OBJWRITER_H_


#include <stdexcept>
#include <ostream>
#include <fstream>

#include "surfaceMesh.h"


namespace PatateCommon
{


template < typename _Mesh >
class OBJWriter
{
public:
    typedef _Mesh Mesh;

    typedef typename Mesh::Vector Vector;

public:
    inline OBJWriter();

    void write(std::ostream& out, const Mesh& mesh);
};


}  // namespace Patate

#include "objWriter.hpp"


#endif  // _OBJWRITER_H_
