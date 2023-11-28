#include "SurfaceMesh.hpp"
#include "BMesh.hpp"

// SurfaceMesh_ctor
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

SurfaceMesh* createSurfaceMeshFromBlenderData(const BVertices& vertices,
                                              const BTriangles& triangles)
{
    // Allocate and initialize the surface mesh
    SurfaceMesh* surfaceMesh = createSurfaceMesh(vertices.size(), triangles.size());

    // Fill the vertices
#pragma omp parallel for
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        auto vertex = vertices[i];
        surfaceMesh->vertex[i].x = vertex[0];
        surfaceMesh->vertex[i].y = vertex[1];
        surfaceMesh->vertex[i].z = vertex[2];
    }

    // Fill the faces
#pragma omp parallel for
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        auto triangle = triangles[i];
        surfaceMesh->face[i].v1 = triangle[0];
        surfaceMesh->face[i].v2 = triangle[1];
        surfaceMesh->face[i].v3 = triangle[2];
    }

    // Return the created surface mesh
    return surfaceMesh;
}

void scaleMeshUniformly(SurfaceMesh* surfaceMesh, const float& scaleFactor)
{
    // Fill the vertices
#pragma omp parallel for
    for (size_t i = 0; i < surfaceMesh->numberVertices; ++i)
    {
        surfaceMesh->vertex[i].x *= scaleFactor;
        surfaceMesh->vertex[i].y *= scaleFactor;
        surfaceMesh->vertex[i].z *= scaleFactor;
    }
}
