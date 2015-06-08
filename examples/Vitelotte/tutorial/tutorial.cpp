/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <cstdlib>
#include <iostream>
#include <fstream>

// Include the obj reader
#include <Patate/common/surface_mesh/objReader.h>

// Include the core of vitelotte: VGMesh and the solver
#include <Patate/vitelotte.h>

// Include the class to deal with mvg input/output
#include <Patate/vitelotte_io.h>


/// [Declare the Mesh class]

// A mesh using floats to represent coefficients and with a dynamic number of
// dimensions and coefficients.
typedef Vitelotte::VGMesh<float, Vitelotte::Dynamic, Vitelotte::Dynamic> Mesh;

/// [Declare the Mesh class]


int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " MESH\n";
        return EXIT_FAILURE;
    }

    const char* meshFilename = argv[1];

    srand(time(NULL));
    srand(9);

    /// [Create the mesh]

    // Our mesh, initalized to be 3D with 3 coefficients with FV attributes
    Mesh mesh(3, 3, Mesh::FV_FLAGS);

    /// [Create the mesh]

    /// [Load a .obj]

    // Open the file and parse the mesh
    std::ifstream meshFile(meshFilename);
    bool ok = PatateCommon::OBJReader<Mesh>().read(meshFile, mesh);

    /// [Load a .obj]

    // Some error handling
    if(!ok)
    {
        std::cerr << "Failed to load \"" << meshFilename << "\".\n";
        return EXIT_FAILURE;
    }

    /// [Convert the mesh in 2D]

    // Discard the z coordinate
    mesh.setNDims(2);

    /// [Convert the mesh in 2D]

    /// [Color dots and finalize]

    // Loop over each vertex
    for(Mesh::VertexIterator vx = mesh.verticesBegin();
        vx != mesh.verticesEnd(); ++vx)
    {
        // Select some random vertices
        if(rand() > RAND_MAX / 10) {
            continue;
        }

        // Add a node with random value.
        Mesh::Node n = mesh.addNode(Mesh::Value::Random(3) / 2 + Mesh::Value::Constant(3, .5));

        // Set the node to a single arbitrary outward halfedge
        mesh.fromVertexValueNode(mesh.halfedge(*vx)) = n;
    }

    // Finalize will propagate our constraints around the constrained
    // vertices and set unknowns to have a smooth diffusion everywhere else.
    mesh.finalize();

    /// [Color dots and finalize]

    /// [Create the solver]

    // The element builder type
    typedef Vitelotte::FVElementBuilder<Mesh, double> BaseBuilder;
    typedef Vitelotte::SingularElementDecorator<BaseBuilder> ElementBuilder;

    // The solver type
    typedef Vitelotte::FemSolver<Mesh, ElementBuilder> Solver;

    Solver solver;

    /// [Create the solver]

    /// [Solve the diffusion]

    // Build the internal matrix and factorize it
    solver.build(mesh);

    // Solve the diffusion
    solver.solve(mesh);

    // Check for errors
    if(solver.error().status() != Vitelotte::SolverError::STATUS_OK) {
        bool error = solver.error().status() == Vitelotte::SolverError::STATUS_ERROR;
        std::cerr << "Solver "
                  << (error? "error": "warning")
                  << ": " << solver.error().message() << "\n";
    }

    /// [Solve the diffusion]

    /// [Write the mesh]

    // Discard the EDGE_GRADIENT attribute which is useless for rendering
    mesh.setAttributes(Mesh::QUADRATIC_FLAGS);

    // Write the mesh to the standard output
    Vitelotte::MVGWriter<Mesh>().write(std::cout, mesh);

    /// [Write the mesh]

}
