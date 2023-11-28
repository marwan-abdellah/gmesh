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

