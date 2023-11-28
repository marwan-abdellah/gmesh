#include "SurfaceMesh.hh"

/**
 * @brief createSurfaceMesh (SurfaceMesh_ctor)
 * @param numberVertices
 * @param numberFaces
 * @return
 */
SurfaceMesh* createSurfaceMesh(const size_t& numberVertices, const size_t& numberFaces)
{
    // Allocation
    SurfaceMesh* surfaceMesh = new SurfaceMesh();

    // Update the vertices
    surfaceMesh->numberVertices = numberVertices;
    if (numberVertices > 0)
        surfaceMesh->vertex = new Vertex[numberVertices];
    else
        surfaceMesh->vertex = nullptr;

    // Update the triangular faces
    surfaceMesh->numberFaces = numberFaces;
    if (numberFaces > 0)
        surfaceMesh->face = new Triangle[numberFaces];
    else
        surfaceMesh->face = nullptr;

    // Neighbours
    surfaceMesh->neighbor = nullptr;
    surfaceMesh->neighborList = nullptr;
    surfaceMesh->averageLength = 0.f;

    // Bounds
    surfaceMesh->pMin->x = 0.f; surfaceMesh->pMin->y = 0.f; surfaceMesh->pMin->z = 0.f;
    surfaceMesh->pMax->x = 0.f; surfaceMesh->pMax->y = 0.f; surfaceMesh->pMax->z = 0.f;

    // Initialize SurfaceMesh structures
    for (size_t i = 0; i < numberVertices; ++i)
    {
        Vertex& vert = surfaceMesh->vertex[i];
        vert.x = vert.y = vert.z = vert.marker = 0;
        vert.selected = true;
    }

    for (size_t i = 0; i < numberFaces; ++i)
    {
        Triangle& face = surfaceMesh->face[i];
        face.v1 = face.v2 = face.v3 = face.marker = 0;
        face.selected = true;
    }

    // Initialize domain data
    surfaceMesh->closed = true;
    surfaceMesh->marker = 1;
    surfaceMesh->volumeConstraint = 100;
    surfaceMesh->useVolumeConstraint = false;
    surfaceMesh->asHole = false;

    return surfaceMesh;
}
