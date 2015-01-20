#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_READER_
#define _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_READER_


#include <Patate/vitelotte_io.h>

#include "vgMeshWithCurves.h"


class VGMeshWithCurveReader: public Vitelotte::MVGReader<VGMeshWithCurves>
{
public:
    typedef Vitelotte::MVGReader<VGMeshWithCurves> Base;

    typedef VGMeshWithCurves Mesh;

    typedef typename Mesh::Vector Vector;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::NodeValue NodeValue;

public:
    VGMeshWithCurveReader();

protected:
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def);

    void parseDc(std::istream& def);

protected:
};


#endif
