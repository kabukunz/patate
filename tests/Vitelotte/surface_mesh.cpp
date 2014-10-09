#include <fstream>
#include <iostream>

#include <Eigen/Core>

#include <Patate/common/surface_mesh/surfaceMesh.h>
#include <Patate/common/surface_mesh/objReader.h>
#include <Patate/common/surface_mesh/objWriter.h>


typedef Eigen::Vector3f Vector;

int main(int argc, char** argv)
{
    Patate::SurfaceMesh mesh;
    Patate::SurfaceMesh::VertexProperty<Vector> positions =
            mesh.addVertexProperty<Vector>("v:position");

    std::cout << "Test surface mesh...\n";

    std::ifstream in("test.obj");
    Patate::OBJReader<Vector> reader(mesh, positions);
    reader.read(in);

    std::ofstream out("out.obj");
    Patate::OBJWriter<Vector> writer(mesh, positions);
    writer.write(out);
}
