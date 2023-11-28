#include "PySurfaceMesh.hh"

pybind11::array_t< Vertex > getVertexData(SurfaceMesh* surfaceMesh)
{
    pybind11::capsule cleanup(surfaceMesh->vertex, [](void *f) {});

    return pybind11::array_t< Vertex >(
       { surfaceMesh->numberVertices }, // Shape
       { sizeof(Vertex) },              // Stride
       surfaceMesh->vertex,             // Pointer to data
       cleanup                          // Garbage collection callback
    );
}

pybind11::array_t< Triangle > getFaceData(SurfaceMesh* surfaceMesh)
{
    pybind11::capsule cleanup(surfaceMesh->face, [](void *f) {});

    return pybind11::array_t< Triangle >(
       { surfaceMesh->numberFaces },    // Shape
       { sizeof(Triangle) },            // Stride
       surfaceMesh->face,               // Pointer to data
       cleanup                          // Garbage collection callback
    );
}

