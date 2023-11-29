#include "SurfaceMesh.hpp"
#include "SurfaceMesh.hh"
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
        printf("@smooth: Could not create neigbor list. "
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
        printf("\tMin/Max angles:\n");
        printf("\t [%ld]: Min θ: [%f], Max θ: [%f], "
               "< θ = %ld: [%ld], "
               "> θ = %ld: [%ld]\n",
               i, minAngle, maxAngle,
               maxMinAngle, numberSmallerAngles,
               minaMaxAngle, numberLargerAngles);
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
            printf("\t%ld: Min θ: [%f] Max θ: [%f] "
                   "θ < %ld: [%ld], "
                   "θ > %ld: [%ld]\r",
                   i, minAngle, maxAngle,
                   maxMinAngle, numberSmallerAngles,
                   minaMaxAngle, numberLargerAngles);
        }
        else
        {
            if (verbose)
            {
                printf("\t%ld: Min θ: [%f] Max θ: [%f] "
                       "θ < %ld: [%ld], "
                       "θ > %ld: [%ld]\r",
                       i, minAngle, maxAngle,
                       maxMinAngle, numberSmallerAngles,
                       minaMaxAngle, numberLargerAngles);
            }
        }

        // Check if the mesh is smoothed or not
        smoothed = minAngle > maxMinAngle && maxAngle < minaMaxAngle;
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
    if (surfaceMesh->neighborList == nullptr)
    {
        printf("\tERROR @smoothNormals: Could not create neigbor list. "
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
    getMinMaxAngles(surfaceMesh,
                    &minAngle, &maxAngle, &numberSmallerAngles, &numberGreaterAngles,
                    maxMinAngle, minMaxAngle);

    if (verbose)
    {
        printf("\t* Min θ: [%f], Max θ: [%f], "
               "< θ = %f: [%ld], "
               "> θ = %f: [%ld]\n",
               minAngle, maxAngle,
               maxMinAngle, numberSmallerAngles,
               minMaxAngle, numberGreaterAngles);
    }
}
