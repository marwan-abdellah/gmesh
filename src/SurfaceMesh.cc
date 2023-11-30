#include "SurfaceMesh.hpp"
#include "SurfaceMesh.hh"
#include "BMesh.hpp"
#include <stdlib.h>
#include <stdio.h>

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
#pragma omp parallel for
    for (size_t i = 0; i < numberVertices; ++i)
    {
        Vertex& vert = surfaceMesh->vertex[i];
        vert.x = vert.y = vert.z = vert.marker = 0;
        vert.selected = true;
    }

#pragma omp parallel for
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

// SurfaceMesh_dtor
void destructSurfaceMesh(SurfaceMesh* surfaceMesh)
{
    // Relase data memory
    releaseSurfaceMeshData(surfaceMesh);

    // Release memory for the SurfaceMesh object
    delete surfaceMesh;
}

// SurfaceMesh_releaseData
void releaseSurfaceMeshData(SurfaceMesh* surfaceMesh)
{
    // Free allocated memory
    if (surfaceMesh->vertex)
        delete [] surfaceMesh->vertex;

    if (surfaceMesh->face)
        delete [] surfaceMesh->face;

    // Destroy neighbourList
    destroyNeighborlist(surfaceMesh);
}

void removeUnconnectedVertices(SurfaceMesh* surfaceMesh)
{
    // Collect statistics
    size_t numberRemovedVertices = 0;

    std::vector< size_t > verticesToRemove(surfaceMesh->numberVertices);

#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        if (surfaceMesh->vertex[n].marker < 0)
        {
            numberRemovedVertices++;
        }

        verticesToRemove[n] = numberRemovedVertices;
    }

    printf("Removing %ld vertices.\n", numberRemovedVertices);

    // Move vertices forward
#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        // If a vertex is to be removed
        if ((n == 0 && verticesToRemove[n] != 0) ||
                (n != 0 && verticesToRemove[n-1] != verticesToRemove[n]))
            continue;

        // Move vertices forward
        surfaceMesh->vertex[n - verticesToRemove[n]].x = surfaceMesh->vertex[n].x;
        surfaceMesh->vertex[n - verticesToRemove[n]].y = surfaceMesh->vertex[n].y;
        surfaceMesh->vertex[n - verticesToRemove[n]].z = surfaceMesh->vertex[n].z;
        surfaceMesh->vertex[n - verticesToRemove[n]].selected = surfaceMesh->vertex[n].selected;
        surfaceMesh->vertex[n - verticesToRemove[n]].marker = surfaceMesh->vertex[n].marker;
    }

    // Fix face offset
#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        surfaceMesh->face[n].v1 =
                surfaceMesh->face[n].v1 - verticesToRemove[surfaceMesh->face[n].v1];
        surfaceMesh->face[n].v2 =
                surfaceMesh->face[n].v2 - verticesToRemove[surfaceMesh->face[n].v2];
        surfaceMesh->face[n].v3 =
                surfaceMesh->face[n].v3 - verticesToRemove[surfaceMesh->face[n].v3];
    }

    // Adjust num_vertices
    surfaceMesh->numberVertices -= numberRemovedVertices;

}

// SurfaceMesh_createNeighborlist
void createNeighborlist(SurfaceMesh* surfaceMesh)
{
    // Destroy any exsisting neighborlist
    destroyNeighborlist(surfaceMesh);

    // Create an array of NeighborPoint3Ptr used to store
    // TODO: Use new
    NeighborPoint3Ptr* neighbourList = (NeighborPoint3Ptr*) malloc(
                sizeof(NeighborPoint3Ptr) *  surfaceMesh->numberVertices);

    // Initialize the neighbor list
#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        neighbourList[n] = nullptr;

        // By default, mark all vertices for deletion
        surfaceMesh->vertex[n].marker = -1;
    }

    // Iterate over the faces and collect line segments (a, b) and its connection to a face (c).
    // Save the line segment so it forms a counter clockwise triangle with the origin vertex
    NPNT3 *firstNeighbour, *secondNeighbour, *auxiliaryNeighbour, *lastNeighbour;
    int numberConnected = 0;
    for (size_t n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        size_t a = surfaceMesh->face[n].v1;
        size_t b = surfaceMesh->face[n].v2;
        size_t c = surfaceMesh->face[n].v3;

        if (a == b || b == c || a == c)
        {
            printf("Face  %ld include vertices with same indices (%ld, %ld, %ld).\n", n, a, b, c);
        }

        firstNeighbour = (NPNT3*)malloc(sizeof(NPNT3));
        firstNeighbour->a = b;
        firstNeighbour->b = c;
        firstNeighbour->c = n;
        firstNeighbour->next = neighbourList[a];
        neighbourList[a] = firstNeighbour;

        // Mark vertex as connected
        if (surfaceMesh->vertex[a].marker < 0)
        {
            surfaceMesh->vertex[a].marker = 0;
            numberConnected += 1;
        }

        firstNeighbour = (NPNT3*) malloc(sizeof(NPNT3));
        firstNeighbour->a = c;
        firstNeighbour->b = a;
        firstNeighbour->c = n;
        firstNeighbour->next = neighbourList[b];
        neighbourList[b] = firstNeighbour;

        // Mark vertex as connected
        if (surfaceMesh->vertex[b].marker < 0)
        {
            surfaceMesh->vertex[b].marker = 0;
            numberConnected += 1;
        }

        firstNeighbour = (NPNT3*) malloc(sizeof(NPNT3));
        firstNeighbour->a = a;
        firstNeighbour->b = b;
        firstNeighbour->c = n;
        firstNeighbour->next = neighbourList[c];
        neighbourList[c] = firstNeighbour;

        // Mark vertex as connected
        if (surfaceMesh->vertex[c].marker < 0)
        {
            surfaceMesh->vertex[c].marker = 0;
            numberConnected += 1;
        }

    }

    // Check if there are vertices which are not connect to any face
    if (numberConnected < surfaceMesh->numberVertices)
    {
        // Attach the neighborlist to the surfaceMesh and destroy it
        surfaceMesh->neighborList = neighbourList;
        destroyNeighborlist(surfaceMesh);

        // Remove unconnected vertices
        removeUnconnectedVertices(surfaceMesh);

        // Re-create neighbors
        createNeighborlist(surfaceMesh);

        return;
    }

    // Order the neighbors so they are connected counter clockwise
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        int a0 = -1;
        int b0 = -1;

        firstNeighbour = neighbourList[n];

        int c = firstNeighbour->a;
        int d = firstNeighbour->b;

        while (firstNeighbour != nullptr)
        {
            int a = firstNeighbour->a;
            int b = firstNeighbour->b;

            secondNeighbour = firstNeighbour->next;
            while (secondNeighbour != nullptr)
            {
                a0 = secondNeighbour->a;
                b0 = secondNeighbour->b;
                if (a0 == b && b0 != a)
                {
                    auxiliaryNeighbour = firstNeighbour;
                    while (auxiliaryNeighbour != nullptr)
                    {
                        if (auxiliaryNeighbour->next == secondNeighbour)
                        {
                            auxiliaryNeighbour->next = secondNeighbour->next;
                            break;
                        }
                        auxiliaryNeighbour = auxiliaryNeighbour->next;
                    }
                    auxiliaryNeighbour = firstNeighbour->next;
                    firstNeighbour->next = secondNeighbour;
                    secondNeighbour->next = auxiliaryNeighbour;
                    break;
                }

                secondNeighbour = secondNeighbour->next;

            }

            firstNeighbour = firstNeighbour->next;

        }

        // Check that the neighbor list is connected
        auxiliaryNeighbour = neighbourList[n];

        bool closed = true;
        while (auxiliaryNeighbour->next != nullptr)
        {
            // Check that we are connected
            if (auxiliaryNeighbour->b != auxiliaryNeighbour->next->a)
            {
                if (closed)
                {
                    printf("Polygons connected to vertex %ld are not closed (interupted):"
                           " (%.2f, %.2f, %.2f)\n", n, surfaceMesh->vertex[n].x,
                           surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z);
                }

                // Do not bail, just register the vertex to not be done anything with
                surfaceMesh->vertex[n].selected = false;

                closed = false;
            }

            // Step one face forward
            auxiliaryNeighbour = auxiliaryNeighbour->next;
        }

        // Check if the list forms a closed ring
        if (closed && b0 != c)
        {
            printf("Polygons connected to vertex %ld are not closed (not closed):"
                   " (%.2f, %.2f, %.2f)\n", n, surfaceMesh->vertex[n].x,
                   surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z);

            // Do not bail, just register the vertex to not be done anything with
            surfaceMesh->vertex[n].selected = false;

            closed = false;
        }

        if (!closed)
        {
            surfaceMesh->closed = false;

        }
    }

    // Attach the neighborlist to the surfaceMesh
    surfaceMesh->neighborList = neighbourList;
}

// SurfaceMesh_destroyNeighborlist
void destroyNeighborlist(SurfaceMesh* surfaceMesh)
{
    NeighborPoint3Ptr firstNeighbour = nullptr;
    NeighborPoint3Ptr auxiliaryNeighbour = nullptr;

    // The neighbor list must exist before deleting it
    if (surfaceMesh->neighborList != nullptr)
    {
        // Release the single neighbors
#pragma omp parallel for
        for (size_t i = 0; i < surfaceMesh->numberVertices; ++i)
        {
            firstNeighbour = surfaceMesh->neighborList[i];
            while (firstNeighbour != nullptr)
            {
                auxiliaryNeighbour = firstNeighbour->next;
                free(firstNeighbour);
                firstNeighbour = auxiliaryNeighbour;
            }
        }

        // Free the array of pointers
        free(surfaceMesh->neighborList);
        surfaceMesh->neighborList = nullptr;
    }
}

void deleteFaces(SurfaceMesh* surfaceMesh)
{
    // Iterate over vertices and mark all for deletion
#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
        surfaceMesh->vertex[n].marker = -1;

    // Delete faces connected to vertices
    int numberRemovedFaces = 0;
    for (int n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        // Check for removal of face
        if (surfaceMesh->face[n].marker < 0)
            numberRemovedFaces += 1;
        else
        {
            // If any previous face has been marked for deletion
            if (numberRemovedFaces > 0)
            {
                // Copy the face to a previous face
                surfaceMesh->face[n-numberRemovedFaces].v1 = surfaceMesh->face[n].v1;
                surfaceMesh->face[n-numberRemovedFaces].v2 = surfaceMesh->face[n].v2;
                surfaceMesh->face[n-numberRemovedFaces].v3 = surfaceMesh->face[n].v3;
                surfaceMesh->face[n-numberRemovedFaces].marker = surfaceMesh->face[n].marker;
                surfaceMesh->face[n-numberRemovedFaces].selected = surfaceMesh->face[n].selected;
            }

            // Un mark vertex for deletion
            surfaceMesh->vertex[surfaceMesh->face[n].v1].marker = 0;
            surfaceMesh->vertex[surfaceMesh->face[n].v2].marker = 0;
            surfaceMesh->vertex[surfaceMesh->face[n].v3].marker = 0;
        }
    }

    // Update the number of faces
    surfaceMesh->numberFaces -= numberRemovedFaces;
    removeUnconnectedVertices(surfaceMesh);
}

void deleteVertices(SurfaceMesh* surfaceMesh)
{
    // Mark faces connected to vertices for deletion
#pragma omp parallel for
    for (size_t n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        if (surfaceMesh->vertex[surfaceMesh->face[n].v1].marker < 0 ||
            surfaceMesh->vertex[surfaceMesh->face[n].v2].marker < 0 ||
            surfaceMesh->vertex[surfaceMesh->face[n].v3].marker < 0 )
        {
            surfaceMesh->face[n].marker = -1;
        }
    }

    // Delete marked faces
    deleteFaces(surfaceMesh);
}

void translateMesh(SurfaceMesh* surfaceMesh, const float& dx, const float& dy, const float& dz)
{
#pragma omp parallel for
    for (size_t i = 0; i < surfaceMesh->numberVertices; ++i)
    {
        surfaceMesh->vertex[i].x += dx;
        surfaceMesh->vertex[i].y += dy;
        surfaceMesh->vertex[i].z += dz;
    }
}

void scaleMesh(SurfaceMesh* surfaceMesh, const float& xScale, const float& yScale, const float& zScale)
{
#pragma omp parallel for
    for (size_t i = 0; i < surfaceMesh->numberVertices; ++i)
    {
        surfaceMesh->vertex[i].x *= xScale;
        surfaceMesh->vertex[i].y *= yScale;
        surfaceMesh->vertex[i].z *= zScale;
    }
}

void scaleMeshUniformly(SurfaceMesh* surfaceMesh, const float& scaleFactor)
{
    scaleMesh(surfaceMesh, scaleFactor, scaleFactor, scaleFactor);
}

