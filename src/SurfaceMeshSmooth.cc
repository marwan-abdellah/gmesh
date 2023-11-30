#include "SurfaceMesh.hpp"
#include "SurfaceMesh.hh"
#include "Common.hh"
#include <stdio.h>


bool smooth(SurfaceMesh *surfaceMesh,
            const size_t& maxMinAngle,
            const size_t& minaMaxAngle,
            const size_t& maximumIterations,
            const bool& preserveRidges,
            const bool& verbose)
{
    // Check if neighbour list is created, otherwise create it
    if (!surfaceMesh->neighborList)
        createNeighborlist(surfaceMesh);

    // If it is still not created, then some polygons are not closed
    if (surfaceMesh->neighborList == nullptr)
    {
        printf(LIB_STRING "ERROR @smooth: Could not create neigbor list. "
               "Some polygons might not be closed. Operation not done!\n");
        return 0;
    }

    // Compute the distribution of the angles
    float minAngle, maxAngle;
    size_t numberSmallerAngles, numberLargerAngles;
    getMinMaxAngles(surfaceMesh, &minAngle, &maxAngle, &numberSmallerAngles, &numberLargerAngles,
                    maxMinAngle, minaMaxAngle);

    // Print the initial quality only when doing 1 or more iterations
    size_t i = 0;
    if (verbose && maximumIterations > 1)
    {
        printf(LIB_STRING "Angles:\n");
        printf(LIB_STRING "%3ld: Min θ, Max θ [%.5f, %.5f] "
               "θ < %ld, θ > %ld [%ld, %ld]\n",
               i, minAngle, maxAngle,
               maxMinAngle, minaMaxAngle,
               numberSmallerAngles, numberLargerAngles);
    }

    // Check if the mesh is smoothed or not
    bool smoothed = minAngle > maxMinAngle && maxAngle < minaMaxAngle;
    while (!smoothed && i < maximumIterations)
    {
        i++;

        // Smooth all vertices
        for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
        {
            // If we have a vertex wich is not selected we continue
            if (!surfaceMesh->vertex[n].selected)
                continue;

            // Move the vertex along the surface of the mesh
            moveVerticesAlongSurface(surfaceMesh, n);

            // Flip the edge
            edgeFlipping(surfaceMesh, n, preserveRidges);
        }

        // Calculate and print quality after surface smooth
        getMinMaxAngles(surfaceMesh, &minAngle, &maxAngle, &numberSmallerAngles,
                        &numberLargerAngles, maxMinAngle, minaMaxAngle);

        // Print the iteration number only when doing 1 or more iterations
        if (maximumIterations != 1 && verbose)
        {
            printf(LIB_STRING "%3ld: Min θ, Max θ [%.5f, %.5f] "
                   "θ < %ld, θ > %ld [%ld, %ld]\n",
                   i, minAngle, maxAngle,
                   maxMinAngle, minaMaxAngle,
                   numberSmallerAngles, numberLargerAngles);
        }
        else
        {
            if (verbose)
            {
                printf(LIB_STRING "%3ld: Min θ, Max θ [%.5f, %.5f] "
                       "θ < %ld, θ > %ld [%ld, %ld]\n",
                       i, minAngle, maxAngle,
                       maxMinAngle, minaMaxAngle,
                       numberSmallerAngles, numberLargerAngles);
            }
        }

        // Check if the mesh is smoothed or not
        smoothed = minAngle > maxMinAngle && maxAngle < minaMaxAngle;
    }

    return smoothed;
}

void smoothNormals(SurfaceMesh *surfaceMesh,
                   const float& maxMinAngle, const float& minMaxAngle,
                   const bool &verbose)
{
    // Check if neighbour list is created, otherwise create it
    if (!surfaceMesh->neighborList)
        createNeighborlist(surfaceMesh);

    // If it is still not created, then some polygons are not closed
    if (surfaceMesh->neighborList == nullptr)
    {
        printf("ERROR @smoothNormals: Could not create neigbor list. "
               "Some polygons might not be closed. Operation not done!\n");
        return;
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
    float minAngle, maxAngle;
    size_t numberSmallerAngles, numberGreaterAngles;
    getMinMaxAngles(surfaceMesh,
                    &minAngle, &maxAngle, &numberSmallerAngles, &numberGreaterAngles,
                    maxMinAngle, minMaxAngle);

    if (verbose)
    {
        printf(LIB_STRING "Min θ, Max θ [%.5f, %.5f] "
               "θ < %f, θ > %f [%ld, %ld]\n",
               minAngle, maxAngle,
               maxMinAngle, minMaxAngle,
               numberSmallerAngles, numberGreaterAngles);
    }
}
