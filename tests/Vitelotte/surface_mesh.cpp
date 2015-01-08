#include <fstream>
#include <iostream>

#include <Eigen/Core>

#include <Patate/common/surface_mesh/surfaceMesh.h>
#include <Patate/common/surface_mesh/objReader.h>
#include <Patate/common/surface_mesh/objWriter.h>


class Mesh : public PatateCommon::SurfaceMesh
{
public:
    typedef Eigen::Vector3f Vector;

public:
    Mesh() : m_vPos(addVertexProperty<Vector>("v:position")) {}

    Vertex addVertex(const Vector& v = Vector())
    {
        Vertex vx = addVertex();
        m_vPos[vx] = v;
        return vx;
    }

    Vector& position(Vertex vx) { return m_vPos[vx]; }
    const Vector& position(Vertex vx) const { return m_vPos[vx]; }

private:
    VertexProperty<Vector> m_vPos;
};


int main(int argc, char** argv)
{
    Mesh mesh;

    std::cout << "Test surface mesh...\n";

    std::ifstream in("test.obj");
    PatateCommon::OBJReader<Mesh> reader;
    reader.read(in, mesh);

    std::ofstream out("out.obj");
    PatateCommon::OBJWriter<Mesh> writer;
    writer.write(out, mesh);
}
