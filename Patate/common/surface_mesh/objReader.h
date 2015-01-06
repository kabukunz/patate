#ifndef _OBJREADER_H_
#define _OBJREADER_H_


#include <stdexcept>
#include <string>
#include <istream>
#include <fstream>
#include <sstream>
#include <vector>

#include "surfaceMesh.h"


namespace PatateCommon
{


inline bool defaultErrorCallback(const std::string& msg, void* ptr);


class OBJBaseReader
{
public:
    typedef bool (*ErrorCallback)(const std::string& msg, void* ptr);

public:
    inline OBJBaseReader()
        : m_error(false),
          m_errorCallback(defaultErrorCallback),
          m_warningCallback(0),
          m_errorCallbackPtr(0) {}
    virtual ~OBJBaseReader() {}

    void setErrorCallback(ErrorCallback error, ErrorCallback warning, void* ptr);

    bool read(std::istream& in);

protected:
    virtual void parseHeader(std::istream& /*in*/) {}
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def) = 0;

    bool readLine(std::istream& in);
    void parseIndiceList(const std::string& _list,
                         std::vector<unsigned>& _indices);

    void error(const std::string& msg);
    void warning(const std::string& msg);

protected:
    unsigned m_lineNb;
    bool m_error;

    std::string m_line;
    std::istringstream m_lineStream;
    std::istringstream m_indicesStream;

    ErrorCallback m_errorCallback;
    ErrorCallback m_warningCallback;
    void* m_errorCallbackPtr;
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


}  // namespace PatateCommon

#include "objReader.hpp"


#endif  // _OBJREADER_H_
