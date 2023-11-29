#include "SurfaceMesh.hh"
#include "SurfaceMesh.hpp"
#include "EigenValue.hh"
#include "EigenVector.hpp"
#include <stdio.h>
#include <math.h>

char coarse(SurfaceMesh* surfaceMesh,
            float coarsenessRate,
            float flatnessRate, float densenessWeight,
            float maxNormalAngle,
            const bool &verbose)
{
    // Check if neighbour list is created, otherwise create it
    if (!surfaceMesh->neighborList)
        createNeighborlist(surfaceMesh);


    // If it is still not created, then some polygons are not closed
    if (surfaceMesh->neighborList == nullptr)
    {
        printf("\tERROR @coarse: Could not create neigbor list. "
               "Some polygons might not be closed. Operation not done!\n");
        return 0;
    }

    NPNT3** neighbourList = surfaceMesh->neighborList;
    size_t* vertexIndexArray = (size_t *) malloc(sizeof(size_t) * surfaceMesh->numberVertices);
    size_t* faceIndexArray = (size_t *) malloc(sizeof(size_t) * surfaceMesh->numberFaces);

    if (verbose)
    {
        printf("\tThe original mesh has [%ld] vertices and [%ld] faces.\n",
               surfaceMesh->numberVertices, surfaceMesh->numberFaces);
    }

    size_t inputNumberVertices = surfaceMesh->numberVertices;

    char stop = 0;

    // If using sparseness weight, calculate the average segment length of the mesh
    if (densenessWeight > 0.0)
    {
        float averageLength = 0.f;
        for (size_t n = 0; n < surfaceMesh->numberFaces; n++)
        {
            int a = surfaceMesh->face[n].v1;
            int b = surfaceMesh->face[n].v2;
            int c = surfaceMesh->face[n].v3;

            float nx = std::sqrt((surfaceMesh->vertex[a].x - surfaceMesh->vertex[b].x) *
                                 (surfaceMesh->vertex[a].x - surfaceMesh->vertex[b].x) +
                                 (surfaceMesh->vertex[a].y - surfaceMesh->vertex[b].y) *
                                 (surfaceMesh->vertex[a].y - surfaceMesh->vertex[b].y) +
                                 (surfaceMesh->vertex[a].z - surfaceMesh->vertex[b].z) *
                                 (surfaceMesh->vertex[a].z - surfaceMesh->vertex[b].z));

            float ny = std::sqrt((surfaceMesh->vertex[a].x - surfaceMesh->vertex[c].x) *
                                 (surfaceMesh->vertex[a].x - surfaceMesh->vertex[c].x) +
                                 (surfaceMesh->vertex[a].y - surfaceMesh->vertex[c].y) *
                                 (surfaceMesh->vertex[a].y - surfaceMesh->vertex[c].y) +
                                 (surfaceMesh->vertex[a].z - surfaceMesh->vertex[c].z) *
                                 (surfaceMesh->vertex[a].z - surfaceMesh->vertex[c].z));

            float nz = std::sqrt((surfaceMesh->vertex[c].x - surfaceMesh->vertex[b].x) *
                                 (surfaceMesh->vertex[c].x - surfaceMesh->vertex[b].x) +
                                 (surfaceMesh->vertex[c].y - surfaceMesh->vertex[b].y) *
                                 (surfaceMesh->vertex[c].y - surfaceMesh->vertex[b].y) +
                                 (surfaceMesh->vertex[c].z - surfaceMesh->vertex[b].z) *
                                 (surfaceMesh->vertex[c].z - surfaceMesh->vertex[b].z));

            averageLength += (nx + ny + nz) / 3.0f;
        }

        if (surfaceMesh->numberFaces == 0)
        {
            printf("\tERROR @coarse: Zero degree on a vertex.\n");
            return 0;
        }
        else
        {
            surfaceMesh->averageLength = averageLength / (float)(surfaceMesh->numberFaces);
        }
    }

    float ratio1 = 1.0, ratio2 = 1.0;
    int maxLength = 0;
    int auxNumber1, auxNumber2;
    int faceAvailableList[64], faceAvailableIndex;
    int neighborAuxList[64];

    // The main loop over all vertices
    for (size_t n = 0; n < surfaceMesh->numberVertices; n++)
    {
        // Status report
        if (((n+1) % 888) == 0 || (n+1) == surfaceMesh->numberVertices)
        {
            const float percentage = 100.0 * (n + 1) / (float)surfaceMesh->numberVertices;
            printf("\tProgress: %2.2f %% done (%ld) \r", percentage, n + 1);
            fflush(stdout);
        }
        fflush(stdout);

        // If the vertex have been flagged to not be removed
        if (!surfaceMesh->vertex[n].selected)
        {
            continue;
        }

        // Check if the vertex has enough neigborgs to be deleted
        char deleteFlag = 1;
        NPNT3* firstNeighbour = neighbourList[n];
        while (firstNeighbour != nullptr)
        {
            int a = firstNeighbour->a;
            auxNumber1 = 0;
            auxNumber2 = 0;

            NPNT3* secondNeighbour = neighbourList[a];
            while (secondNeighbour != nullptr)
            {
                fflush(stdout);
                int b = secondNeighbour->a;
                NPNT3* auxNeighbour1 = neighbourList[n];
                while (auxNeighbour1 != nullptr)
                {
                    if (auxNeighbour1->a == b)
                        auxNumber2++;
                    auxNeighbour1 = auxNeighbour1->next;
                }

                auxNumber1++;
                secondNeighbour = secondNeighbour->next;
            }

            if (auxNumber1 <= 3 || auxNumber2 > 2)
                deleteFlag = 0;

            firstNeighbour = firstNeighbour->next;
        }

        if (deleteFlag)
        {
            float x = surfaceMesh->vertex[n].x;
            float y = surfaceMesh->vertex[n].y;
            float z = surfaceMesh->vertex[n].z;

            maxLength = -1;
            firstNeighbour = neighbourList[n];

            // If using sparseness as a criteria for coarsening calculate the maximal segment length
            if (densenessWeight > 0.0)
            {
                while (firstNeighbour != nullptr)
                {
                    int a = firstNeighbour->a;
                    int b = firstNeighbour->b;

                    float nx = std::sqrt((x - surfaceMesh->vertex[a].x) *
                                         (x - surfaceMesh->vertex[a].x) +
                                         (y - surfaceMesh->vertex[a].y) *
                                         (y - surfaceMesh->vertex[a].y) +
                                         (z - surfaceMesh->vertex[a].z) *
                                         (z - surfaceMesh->vertex[a].z));
                    float ny = std::sqrt((x - surfaceMesh->vertex[b].x) *
                                         (x - surfaceMesh->vertex[b].x) +
                                         (y - surfaceMesh->vertex[b].y) *
                                         (y - surfaceMesh->vertex[b].y) +
                                         (z - surfaceMesh->vertex[b].z) *
                                         (z - surfaceMesh->vertex[b].z));

                    if (nx > maxLength)
                        maxLength = nx;

                    if (ny > maxLength)
                        maxLength = ny;

                    firstNeighbour = firstNeighbour->next;
                }

                // Max segment length over the average segment length of the mesh
                ratio2 = maxLength / surfaceMesh->averageLength;
                ratio2 = pow(ratio2, densenessWeight);
            }

            // If using curvatory as a coarsening criteria calculate the local structure tensor
            float maxAngle;
            if (flatnessRate > 0.0)
            {
                EigenValue eigenValue;
                EigenVector eigenVector = getEigenVector(surfaceMesh, n, &eigenValue, &maxAngle);

                if (eigenValue.x == 0)
                {
                    printf("\tERROR @coarse: Max EigenValue is zero!\n");
                    return 0;
                }
                else
                {
                    // printf("\t @coarse: EigenValues: [%ld], "
                    //  "(%.3f, %.3f, %.3f)\n", n, eigenValue.x, eigenValue.y, eigenValue.z);


                    ratio1 = fabs((eigenValue.y)/(eigenValue.x));
                    ratio1 = pow(ratio1, flatnessRate);
                    // ratio1 = (1.0 - maxAngle) * fabs((eigenValue.y) / (eigenValue.x));
                }
            }

            // Compare the two coarseness criterias against the given coarsenessRate
            bool deleteVertex = ratio1 * ratio2 < coarsenessRate;

            // Use maximal angle between vertex normal as a complementary coarse criteria
            if (maxNormalAngle > 0)
                deleteVertex = deleteVertex && maxAngle > maxNormalAngle;

            // Deleting a vertex and retrianglulate the hole
            if (deleteVertex)
            {
                inputNumberVertices--;

                // delete vertex n
                surfaceMesh->vertex[n].x = -99999;
                surfaceMesh->vertex[n].y = -99999;
                surfaceMesh->vertex[n].z = -99999;

                int neighborNumber = 0;
                firstNeighbour = neighbourList[n];
                int face_marker;
                while (firstNeighbour != nullptr)
                {
                    int a = firstNeighbour->a;
                    int c = firstNeighbour->c;
                    faceAvailableList[neighborNumber] = c;
                    neighborAuxList[neighborNumber] = a;
                    neighborNumber++;

                    // Get face marker
                    face_marker = surfaceMesh->face[c].marker;

                    /* delete faces associated with vertex n */
                    surfaceMesh->face[c].v1 = -1;
                    surfaceMesh->face[c].v2 = -1;
                    surfaceMesh->face[c].v3 = -1;
                    surfaceMesh->face[c].marker = -1;

                    // Delete neighbors associated with vertex n
                    NPNT3* secondNeighbour = neighbourList[a];
                    NPNT3* auxNeighbour1 = secondNeighbour;
                    while (secondNeighbour != nullptr)
                    {
                        if (secondNeighbour->a == n || secondNeighbour->b == n)
                        {
                            if (secondNeighbour == neighbourList[a])
                            {
                                neighbourList[a] = secondNeighbour->next;
                                free(secondNeighbour);
                                secondNeighbour = neighbourList[a];
                                auxNeighbour1 = secondNeighbour;
                            }
                            else
                            {
                                auxNeighbour1->next = secondNeighbour->next;
                                free(secondNeighbour);
                                secondNeighbour = auxNeighbour1->next;
                            }
                        }
                        else
                        {
                            if (secondNeighbour == neighbourList[a])
                            {
                                secondNeighbour = secondNeighbour->next;
                            }
                            else
                            {
                                auxNeighbour1 = secondNeighbour;
                                secondNeighbour = secondNeighbour->next;
                            }
                        }
                    }

                    auxNumber1 = 0;
                    secondNeighbour = neighbourList[a];
                    while (secondNeighbour != nullptr)
                    {
                        auxNumber1++;
                        secondNeighbour = secondNeighbour->next;
                    }
                    firstNeighbour->b = auxNumber1;
                    firstNeighbour = firstNeighbour->next;
                }

                firstNeighbour = neighbourList[n];
                while (firstNeighbour->next != nullptr)
                    firstNeighbour = firstNeighbour->next;
                firstNeighbour->next = neighbourList[n];

                faceAvailableIndex = 0;
                subdividePolygon(surfaceMesh, neighbourList[n], faceAvailableList,
                                 &faceAvailableIndex, face_marker);

                // Order the neighbors
                for (size_t m = 0; m < neighborNumber; m++)
                {
                    firstNeighbour = neighbourList[neighborAuxList[m]];
                    int c = firstNeighbour->a;
                    while (firstNeighbour != nullptr)
                    {
                        int a = firstNeighbour->a;
                        int b = firstNeighbour->b;

                        NPNT3* secondNeighbour = firstNeighbour->next;
                        while (secondNeighbour != nullptr)
                        {
                            int a0 = secondNeighbour->a;
                            int b0 = secondNeighbour->b;

                            // Assume counter clockwise orientation
                            if (a0==b && b0!=a)
                            {
                                NPNT3* auxNeighbour1 = firstNeighbour;
                                while (auxNeighbour1 != nullptr)
                                {
                                    if (auxNeighbour1->next == secondNeighbour)
                                    {
                                        auxNeighbour1->next = secondNeighbour->next;
                                        break;
                                    }
                                    auxNeighbour1 = auxNeighbour1->next;
                                }
                                auxNeighbour1 = firstNeighbour->next;
                                firstNeighbour->next = secondNeighbour;
                                secondNeighbour->next = auxNeighbour1;
                                break;
                            }

                            secondNeighbour = secondNeighbour->next;
                        }

                        if (firstNeighbour->next == nullptr)
                        {
                            if (firstNeighbour->b != c)
                            {
                                printf("\tERROR @coarse: Some polygons are not closed @[%ld] \n", n);
                            }
                        }

                        firstNeighbour = firstNeighbour->next;
                    }
                }

                // Smooth the neighbors
                for (size_t m = 0; m < neighborNumber; m++)
                {
                    if (!surfaceMesh->vertex[auxNumber2].selected)
                        continue;

                    auxNumber2 = neighborAuxList[m];

                    x = surfaceMesh->vertex[auxNumber2].x;
                    y = surfaceMesh->vertex[auxNumber2].y;
                    z = surfaceMesh->vertex[auxNumber2].z;

                    float nx = 0;
                    float ny = 0;
                    float nz = 0;
                    float weight = 0;

                    firstNeighbour = neighbourList[auxNumber2];
                    while (firstNeighbour != nullptr)
                    {
                        int a = firstNeighbour->a;
                        int b = firstNeighbour->b;

                        NPNT3* secondNeighbour = firstNeighbour->next;
                        if (secondNeighbour == nullptr)
                            secondNeighbour = neighbourList[auxNumber2];

                        int c = secondNeighbour->b;

                        Vertex newPosition = getVertexPositionAlongSurface(x, y, z, b, a, c, surfaceMesh);
                        float angle = computeDotProduct(surfaceMesh, b, a, c);
                        angle += 1.0;
                        nx += angle * newPosition.x;
                        ny += angle * newPosition.y;
                        nz += angle * newPosition.z;

                        weight += angle;
                        firstNeighbour = firstNeighbour->next;
                    }

                    if (weight > 0)
                    {
                        nx /= weight;
                        ny /= weight;
                        nz /= weight;

                        EigenValue eigenValue;
                        EigenVector eigenVector = getEigenVector(surfaceMesh, auxNumber2,
                                                                 &eigenValue, &maxAngle);

                        if ((eigenVector.x1==0 && eigenVector.y1==0 && eigenVector.z1==0) ||
                            (eigenVector.x2==0 && eigenVector.y2==0 && eigenVector.z2==0) ||
                            (eigenVector.x3==0 && eigenVector.y3==0 && eigenVector.z3==0))
                        {
                            surfaceMesh->vertex[auxNumber2].x = nx;
                            surfaceMesh->vertex[auxNumber2].y = ny;
                            surfaceMesh->vertex[auxNumber2].z = nz;
                        }
                        else
                        {
                            nx -= x; ny -= y; nz -= z;

                            float w1 = (nx * eigenVector.x1 + ny * eigenVector.y1 + nz * eigenVector.z1) /
                                    (1.0 + eigenValue.x);
                            float w2 = (nx * eigenVector.x2 + ny * eigenVector.y2 + nz * eigenVector.z2) /
                                    (1.0 + eigenValue.y);
                            float w3 = (nx *eigenVector.x3 + ny * eigenVector.y3 + nz * eigenVector.z3) /
                                    (1.0 + eigenValue.z);

                            surfaceMesh->vertex[auxNumber2].x =
                                    w1 * eigenVector.x1 + w2 * eigenVector.x2 + w3 * eigenVector.x3 + x;
                            surfaceMesh->vertex[auxNumber2].y =
                                    w1 * eigenVector.y1 + w2 * eigenVector.y2 + w3 * eigenVector.y3 + y;
                            surfaceMesh->vertex[auxNumber2].z =
                                    w1 * eigenVector.z1 + w2 * eigenVector.z2 + w3 * eigenVector.z3 + z;
                        }
                    }
                }
            }
        }

        // if (inputNumberVertices < MeshSizeUpperLimit) { stop = 1; break; }
    }

    // Clean the lists of nodes and faces
    int startIndex = 0;
    for (size_t n = 0; n < surfaceMesh->numberVertices; ++n)
    {
        if (surfaceMesh->vertex[n].x != -99999 &&
            surfaceMesh->vertex[n].y != -99999 &&
            surfaceMesh->vertex[n].z != -99999)
        {
            if (startIndex != n)
            {
                surfaceMesh->vertex[startIndex].x = surfaceMesh->vertex[n].x;
                surfaceMesh->vertex[startIndex].y = surfaceMesh->vertex[n].y;
                surfaceMesh->vertex[startIndex].z = surfaceMesh->vertex[n].z;
                surfaceMesh->vertex[startIndex].marker = surfaceMesh->vertex[n].marker;
                surfaceMesh->vertex[startIndex].selected = surfaceMesh->vertex[n].selected;
                neighbourList[startIndex] = neighbourList[n];
            }

            vertexIndexArray[n] = startIndex;
            startIndex++;
        }
        else
        {
            vertexIndexArray[n] = -1;
        }
    }

    surfaceMesh->numberVertices = startIndex;

    startIndex = 0;
    for (size_t n = 0; n < surfaceMesh->numberFaces; n++)
    {
        int a = surfaceMesh->face[n].v1;
        int b = surfaceMesh->face[n].v2;
        int c = surfaceMesh->face[n].v3;
        int face_marker = surfaceMesh->face[n].marker;
        if (a >= 0 && b >= 0 && c >= 0 &&
            vertexIndexArray[a] >= 0 && vertexIndexArray[b] >= 0 && vertexIndexArray[c] >= 0)
        {
            surfaceMesh->face[startIndex].v1 = vertexIndexArray[a];
            surfaceMesh->face[startIndex].v2 = vertexIndexArray[b];
            surfaceMesh->face[startIndex].v3 = vertexIndexArray[c];
            surfaceMesh->face[startIndex].marker = face_marker;
            faceIndexArray[n] = startIndex;
            startIndex++;
        }
        else
        {
            faceIndexArray[n] = -1;
        }
    }
    surfaceMesh->numberFaces = startIndex;

    for (size_t n = 0; n < surfaceMesh->numberVertices; n++)
    {
        NPNT3* firstNeighbour = neighbourList[n];
        while (firstNeighbour != nullptr)
        {
            int a = firstNeighbour->a;
            int b = firstNeighbour->b;
            int c = firstNeighbour->c;

            firstNeighbour->a = vertexIndexArray[a];
            firstNeighbour->b = vertexIndexArray[b];
            firstNeighbour->c = faceIndexArray[c];

            firstNeighbour = firstNeighbour->next;
        }
    }

    free(vertexIndexArray);
    free(faceIndexArray);

    return(stop);
}


void coarseDense(SurfaceMesh* surfaceMesh,
                 const float& denseRate, const size_t &iterations, const bool verbose)
{
    for (int64_t i = 0; i < iterations; ++i)
        if (!coarse(surfaceMesh, denseRate, 0, 10, -1, verbose)) break;
}

void coarseFlat(SurfaceMesh* surfaceMesh,
                 const float& flatnessRate, const size_t &iterations, const bool verbose)
{
    for (int64_t i = 0; i < iterations; ++i)
        if (!coarse(surfaceMesh, flatnessRate, 1, 0, -1, verbose)) break;
}


