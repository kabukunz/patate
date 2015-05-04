/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_WRITER_
#define _EXAMPLES_VITELOTTE_COMMON_VG_MESH_WITH_CURVES_WRITER_


#include <Patate/vitelotte_io.h>

#include "vgMeshWithCurves.h"


class VGMeshWithCurveWriter: public Vitelotte::MVGWriter<VGMeshWithCurves>
{
public:
    typedef Vitelotte::MVGWriter<VGMeshWithCurves> Base;

    typedef VGMeshWithCurves Mesh;

    typedef Mesh::Vector Vector;
    typedef Mesh::Vertex Vertex;
    typedef Mesh::Value Value;
    typedef Mesh::ValueFunction ValueFunction;

public:
    VGMeshWithCurveWriter(Version version=LATEST_VERSION);

    void write(std::ostream& _out, const Mesh& mesh);

protected:
    void writeValueFunction(std::ostream& out, const ValueFunction& vg) const;
};


#endif
