#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_READER_
#define _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_READER_


#include <Patate/vitelotte_io.h>

#include "vgMeshWithCurves.h"


class VGMeshWithCurveReader: public Vitelotte::MVGReader<VGMeshWithCurves>
{
public:
    typedef Vitelotte::MVGReader<VGMeshWithCurves> Base;

    typedef VGMeshWithCurves Mesh;

    typedef Mesh::Vector Vector;
    typedef Mesh::Value Value;
    typedef Mesh::ValueFunction ValueFunction;

    typedef Mesh::Vertex Vertex;
    typedef Mesh::Halfedge Halfedge;
    typedef Mesh::PointConstraint PointConstraint;
    typedef Mesh::Curve Curve;

public:
    VGMeshWithCurveReader();


protected:
    enum
    {
        CONS_LEFT = 1,
        CONS_RIGHT = 2,
        TEAR = 4
    };

protected:
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def, Mesh& mesh);

//    void parsePointConstraint(std::istream& def);
//    void parseDc(std::istream& def);
//    int parseCurveType(std::istream& in);
//    bool parseGradient(std::istream& def, Mesh::ValueFunction& g);
//    bool parseCurveVertices(std::istream& def, Mesh::Curve curve);

protected:
    std::string m_part;
    std::string m_token;
    std::istringstream m_in;

};


#endif
