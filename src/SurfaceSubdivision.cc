#include "SurfaceMesh.hh"
#include <cstdlib>

void refine(SurfaceMesh* surfaceMesh)
{
    size_t a, b, c;
    size_t local_vertices[3], local_additional_vertices[3];

    NPNT3* ngr;
    float ax, ay, az;
    float nx, ny, nz;

    // Check if neighborlist is created, otherwise create it
    if (surfaceMesh->neighborList == nullptr)
        createNeighborlist(surfaceMesh);

    NPNT3** neighbourList = surfaceMesh->neighborList;

    // Store the number of vertices in the original mesh
    size_t initialNumberVertices = surfaceMesh->numberVertices;

    // Create an array with the number of edges associated with each vertex
    size_t* numberEdges = (size_t*) malloc(sizeof(size_t) * initialNumberVertices);

    // Create an array with the offsets into the vertex2edge array for each vertex
    size_t* offsets = (size_t*) malloc(sizeof(size_t) * initialNumberVertices);

    // Iterate over all vertices and collect edges
    size_t totalNumberEdges = 0;
    size_t localNumberEdges = 0;
    for (size_t n = 0; n < initialNumberVertices; ++n)
    {
        offsets[n] = totalNumberEdges;
        localNumberEdges = 0;

        ngr = neighbourList[n];
        while (ngr != nullptr)
        {
            // If n is smaller than ngr->a we have an edge
            if (int(n) < ngr->a)
            {
                totalNumberEdges++;
                localNumberEdges++;
            }
            ngr = ngr->next;
        }
        numberEdges[n] = localNumberEdges;

    }

    // Create memory for the refined mesh
    SurfaceMesh* refinedSurfaceMesh = createSurfaceMesh(initialNumberVertices + totalNumberEdges,
                                           surfaceMesh->numberFaces * 4);
    refinedSurfaceMesh->numberVertices = initialNumberVertices;
    refinedSurfaceMesh->numberFaces = surfaceMesh->numberFaces;

    // Copy the original mesh to the new mesh
    for (size_t n = 0; n < initialNumberVertices; ++n)
    {
        refinedSurfaceMesh->vertex[n].x = surfaceMesh->vertex[n].x;
        refinedSurfaceMesh->vertex[n].y = surfaceMesh->vertex[n].y;
        refinedSurfaceMesh->vertex[n].z = surfaceMesh->vertex[n].z;
    }

    for (size_t n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        refinedSurfaceMesh->face[n].v1 = surfaceMesh->face[n].v1;
        refinedSurfaceMesh->face[n].v2 = surfaceMesh->face[n].v2;
        refinedSurfaceMesh->face[n].v3 = surfaceMesh->face[n].v3;
    }

    // Create the map from vertices to edges
    size_t* vertex2edge = (size_t*) malloc(sizeof(size_t) * totalNumberEdges);

    // Iterate over all vertices and split edges
    size_t edgeNumber = 0;
    for (size_t n = 0; n < initialNumberVertices; ++n)
    {
        // Get the coordinates of vertex n
        nx = refinedSurfaceMesh->vertex[n].x;
        ny = refinedSurfaceMesh->vertex[n].y;
        nz = refinedSurfaceMesh->vertex[n].z;

        ngr = neighbourList[n];
        while (ngr != nullptr)
        {
            // If n is smaller than ngr->a we have an edge
            if (int(n) < ngr->a)
            {
                // Add the value of the opposite vertex to the map
                vertex2edge[edgeNumber] = ngr->a;

                // Get the coordinates of vertex ngr->a
                ax = refinedSurfaceMesh->vertex[ngr->a].x;
                ay = refinedSurfaceMesh->vertex[ngr->a].y;
                az = refinedSurfaceMesh->vertex[ngr->a].z;

                // Add the new vertex coordinates of the splitted edge
                refinedSurfaceMesh->vertex[initialNumberVertices + edgeNumber].x = 0.5*(ax + nx);
                refinedSurfaceMesh->vertex[initialNumberVertices + edgeNumber].y = 0.5*(ay + ny);
                refinedSurfaceMesh->vertex[initialNumberVertices + edgeNumber].z = 0.5*(az + nz);

                // Increase the edge number
                edgeNumber++;
            }
            ngr = ngr->next;
        }
    }

    // A counter for adding new faces
    size_t faceNumber = refinedSurfaceMesh->numberFaces;

    // Iterate over faces and add information of the refined face
    for (size_t n = 0; n < refinedSurfaceMesh->numberFaces; ++n)
    {
        local_vertices[0] = refinedSurfaceMesh->face[n].v1;
        local_vertices[1] = refinedSurfaceMesh->face[n].v2;
        local_vertices[2] = refinedSurfaceMesh->face[n].v3;

        // Iterate over the vertices and find the edges
        for (size_t m = 0; m < 3; ++m)
        {
            size_t min_vertex_num = std::min(local_vertices[m], local_vertices[(m + 1) % 3]);
            size_t max_vertex_num = std::max(local_vertices[m], local_vertices[(m + 1) % 3]);

            // Find the edge number that fit the pair of vertices
            size_t k = 0;
            for (k = 0; k < numberEdges[min_vertex_num]; ++k)
                if (vertex2edge[offsets[min_vertex_num] + k] == max_vertex_num)
                    break;

            // The edge number represents the number of the added vertex plus the
            // number of original vertices
            local_additional_vertices[m] = initialNumberVertices + offsets[min_vertex_num] + k;

        }

        // Add information of the four new faces

        // First the mid face
        refinedSurfaceMesh->face[n].v1 = local_additional_vertices[0];
        refinedSurfaceMesh->face[n].v2 = local_additional_vertices[1];
        refinedSurfaceMesh->face[n].v3 = local_additional_vertices[2];

        // Then the three corner faces
        for (size_t m = 0; m < 3; m++)
        {
            refinedSurfaceMesh->face[faceNumber].v1 = local_vertices[m];
            refinedSurfaceMesh->face[faceNumber].v2 = local_additional_vertices[m];
            refinedSurfaceMesh->face[faceNumber].v3 = local_additional_vertices[(m + 2) % 3];
            faceNumber++;
        }
    }

    // Release memory
    free(numberEdges);
    free(offsets);
    free(vertex2edge);

    // Update number information
    refinedSurfaceMesh->numberVertices += totalNumberEdges;
    refinedSurfaceMesh->numberFaces *= 4;

    // Release old data
    releaseSurfaceMeshData(surfaceMesh);

    // Assign the refined mesh to the passed
    surfaceMesh->numberVertices = refinedSurfaceMesh->numberVertices;
    surfaceMesh->numberFaces = refinedSurfaceMesh->numberFaces;
    surfaceMesh->vertex = refinedSurfaceMesh->vertex;
    surfaceMesh->face = refinedSurfaceMesh->face;

    // Free memory of refined surface mesh struct
    delete refinedSurfaceMesh;

    // Recreate the neigborlist
    createNeighborlist(surfaceMesh);

}
