#include "SurfaceMesh.hpp"
#include "SurfaceMesh.hh"
#include <stdio.h>

bool smooth(SurfaceMesh *surfaceMesh,
            const size_t& maxMinAngle,
            const size_t& minaMaxAngle,
            const size_t& maximumIterations,
            const bool& preserveRidges)
{
    float min_angle, max_angle;
    size_t num_small, num_large;
    size_t i, n;
    bool smoothed = false;

    // Check if neighborlist is created
    if (surfaceMesh->neighborList == nullptr)
        createNeighborlist(surfaceMesh);

    if (surfaceMesh->neighborList == nullptr)
    {
        printf("Could not create neigbor list some polygons might not be closed \nor you need to harmanize the normals of the mesh.\n");
        printf("Bailing out...\n");
        return false;
    }

    i = 0;

    getMinMaxAngles(surfaceMesh, &min_angle, &max_angle, &num_small,
                                &num_large, maxMinAngle, minaMaxAngle);

    // Print the initial quality only when doing 1 or more iterations
    if (maximumIterations > 1)
    {
        printf("Min Max angles\n");
        printf("%2d: min_angle: %f - max_angle: %f - "
               "smaller-than-%d: %d - larger-than-%d: %d\n",
               i, min_angle, max_angle, maxMinAngle, num_small,
               minaMaxAngle, num_large);

    }

    // Check if the mesh is smoothed
    smoothed = min_angle > maxMinAngle && max_angle < minaMaxAngle;
    while (!smoothed && i < maximumIterations)
    {

        i++;

        // Smooth all vertices
        for (n = 0; n < surfaceMesh->numberVertices; n++)
        {

            // If we have a vertex wich is not selected we continue
            if (!surfaceMesh->vertex[n].selected)
                continue;

            moveVerticesAlongSurface(surfaceMesh, n);
            edgeFlipping(surfaceMesh, n, preserveRidges);
        }

        // Calculate and print quality after surface smooth
        getMinMaxAngles(surfaceMesh, &min_angle, &max_angle, &num_small,
                                    &num_large, maxMinAngle, minaMaxAngle);

        // Print the iteration number only when doing 1 or more iterations
        if (maximumIterations != 1)
            printf("%2d: min_angle: %f - max_angle: %f - "
                   "smaller-than-%d: %d - larger-than-%d: %d\n",
                   i, min_angle, max_angle, maxMinAngle, num_small,
                   minaMaxAngle, num_large);
        else
            printf("    min_angle: %f - max_angle: %f - "
                   "smaller-than-%d: %d - larger-than-%d: %d\n",
                   min_angle, max_angle, maxMinAngle, num_small,
                   minaMaxAngle, num_large);

        // Check if the mesh is smoothed
        smoothed = min_angle > maxMinAngle && max_angle < minaMaxAngle;
    }

    return smoothed;
}

void smoothNormals(SurfaceMesh *surfaceMesh,
                   float& minAngle, float& maxAngle,
                   size_t& numberSmallerAngles, size_t& numberGreaterAngles,
                   const float& maxMinAngle, const float& minMaxAngle,
                   const bool &verbose)
{
    // Check if neighbour list is created, otherwise create it
    if (!surfaceMesh->neighborList)
        createNeighborlist(surfaceMesh);

    // If it is still not created, then some polygons are not closed
    if (surfaceMesh->neighborList == NULL)
    {
        printf("@smoothNormals: Could not create neigbor list. "
               "Some polygons might not be closed. Operation not done!\n");
        return ;
    }

    // Normal smooth all vertices
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        // The vertex must be selected to smooth its normal
        if (!surfaceMesh->vertex[n].selected) { continue; }

        // Smooth the normal of the vertex
        smoothNormal(surfaceMesh, n);
    }

    // Compute the angles
    getMinMaxAngles(surfaceMesh,
                    &minAngle, &maxAngle, &numberSmallerAngles, &numberGreaterAngles,
                    maxMinAngle, minMaxAngle);

    if (verbose)
    {
        printf("\t* Min. Angle: [%f], Max. Angle: [%f], "
               "Number of angles smaller than %f: [%ld], "
               "Number of angles greater than %f: [%ld]\n",
               minAngle, maxAngle,
               maxMinAngle, numberSmallerAngles,
               minMaxAngle, numberGreaterAngles);
    }
}
