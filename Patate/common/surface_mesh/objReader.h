#ifndef _OBJREADER_H_
#define _OBJREADER_H_


#include <stdexcept>
#include <string>
#include <istream>
#include <fstream>
#include <sstream>
#include <vector>

#include "surfaceMesh.h"


namespace Patate
{


class OBJBaseReader
{
public:
    inline OBJBaseReader() {}
    virtual ~OBJBaseReader() {}

    void read(std::istream& in);

protected:
    virtual void parseHeader(std::istream& /*in*/) {}
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def) = 0;

    void parseIndiceList(const std::string& _list,
                         std::vector<int>& _indices);

    void error(const std::string& msg);

private:
    unsigned m_lineNb;
    std::string m_line;
    std::istringstream m_indicesStream;
};


template < typename _Point >
class OBJReader: public OBJBaseReader
{
public:
    typedef _Point Point;

public:
    inline OBJReader(SurfaceMesh& mesh,
                     SurfaceMesh::VertexProperty<Point> positions);

protected:
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def);

protected:
    SurfaceMesh& m_mesh;
    SurfaceMesh::VertexProperty<Point> m_vPos;

    std::vector<SurfaceMesh::Vertex>  m_fVertices;

};


#include "objReader.hpp"

}  // namespace Patate

#endif  // _OBJREADER_H_
