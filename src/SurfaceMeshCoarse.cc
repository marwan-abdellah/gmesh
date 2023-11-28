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


    int m, a0, b0;
    int a, b, c, face_marker;
    float x, y, z;
    NPNT3 *first_ngr, *second_ngr, *tmp_ngr, *tmp_ngr1;
    int number, neighbor_number, num;

    EigenVector eigen_vect;
    EigenValue eigen_value;
    // float averageLength, max_len;
    float ratio1 = 1.0, ratio2 = 1.0;
    int face_available_list[64], face_available_index;
    int neighbor_tmp_list[64];
    float weight, angle;

    //int start_index, *vertexIndexArray, *faceIndexArray;

    Vertex pos_vect;
    float w1, w2, w3;
    char delete_flag;
    float max_angle;

    bool delete_vertex;

    // Check if neighbour list is created, otherwise create it and reset the vertex markers
    if (!surfaceMesh->neighborList)
        createNeighborlist(surfaceMesh);

    if (surfaceMesh->neighborList == nullptr)
    {
        printf("@coarse: Could not create neigbor list. "
               "Some polygons might not be closed. Operation not done!\n");
        return 0;
    }

    NPNT3** neighbourList = surfaceMesh->neighborList;

    size_t* vertexIndexArray = (size_t *) malloc(sizeof(size_t) * surfaceMesh->numberVertices);
    size_t* faceIndexArray = (size_t *) malloc(sizeof(size_t) * surfaceMesh->numberFaces);

    if (verbose)
    {
        printf("\t The mesh has [%ld] vertices and [%ld] faces\n",
               surfaceMesh->numberVertices, surfaceMesh->numberFaces);
    }

    size_t vertex_num = surfaceMesh->numberVertices;

    char stop = 0;

    // If using sparseness weight, calculate the average segment length of the mesh
    if (densenessWeight > 0.0)
    {
        float averageLength = 0.f;
        for (size_t n = 0; n < surfaceMesh->numberFaces; n++)
        {
            a = surfaceMesh->face[n].v1;
            b = surfaceMesh->face[n].v2;
            c = surfaceMesh->face[n].v3;

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
            printf("ERROR: Zero degree on a vertex.\n");
            return 0;
        }
        else
        {
            surfaceMesh->averageLength = averageLength / (float)(surfaceMesh->numberFaces);
        }
    }

    // The main loop over all vertices
    int max_len = 0;
    for (size_t n = 0; n < surfaceMesh->numberVertices; n++)
    {

        // Status report
        if (((n+1) % 888) == 0 || (n+1) == surfaceMesh->numberVertices)
        {
            printf("%2.2f%% done (%08d)          \r", 100.0*(n+1)/
                   (float)surfaceMesh->numberVertices, n+1);
            fflush(stdout);
        }

        // If the vertex have been flagged to not be removed
        if (!surfaceMesh->vertex[n].selected)
        {
            //printf("Do not remove vertex %d\n", n);
            continue;
        }

        // Check if the vertex has enough neigborgs to be deleted
        delete_flag = 1;
        first_ngr = neighbourList[n];
        fflush(stdout);
        while (first_ngr != nullptr)
        {
            a = first_ngr->a;
            number = 0;
            num = 0;
            second_ngr = neighbourList[a];
            while (second_ngr != nullptr)
            {
                fflush(stdout);
                b = second_ngr->a;
                tmp_ngr = neighbourList[n];
                while (tmp_ngr != nullptr) {
                    if (tmp_ngr->a == b)
                        num++;
                    tmp_ngr = tmp_ngr->next;
                }
                number++;
                second_ngr = second_ngr->next;
            }

            if (number <= 3 || num > 2)
                delete_flag = 0;
            first_ngr = first_ngr->next;
        }

        if (delete_flag)
        {
            x = surfaceMesh->vertex[n].x;
            y = surfaceMesh->vertex[n].y;
            z = surfaceMesh->vertex[n].z;

            max_len = -1;
            first_ngr = neighbourList[n];

            // If using sparseness as a criteria for coarsening
            // calculate the maximal segment length
            if (densenessWeight > 0.0)
            {
                while (first_ngr != nullptr)
                {
                    a = first_ngr->a;
                    b = first_ngr->b;

                    float nx = std::sqrt((x - surfaceMesh->vertex[a].x)*(x - surfaceMesh->vertex[a].x)+
                                         (y - surfaceMesh->vertex[a].y)*(y - surfaceMesh->vertex[a].y)+
                                         (z - surfaceMesh->vertex[a].z)*(z - surfaceMesh->vertex[a].z));
                    float ny = std::sqrt((x - surfaceMesh->vertex[b].x)*(x - surfaceMesh->vertex[b].x)+
                                         (y - surfaceMesh->vertex[b].y)*(y - surfaceMesh->vertex[b].y)+
                                         (z - surfaceMesh->vertex[b].z)*(z - surfaceMesh->vertex[b].z));

                    if (nx > max_len)
                        max_len = nx;
                    if (ny > max_len)
                        max_len = ny;

                    first_ngr = first_ngr->next;
                }

                // Max segment length over the average segment length of the mesh
                ratio2 = max_len/surfaceMesh->averageLength;
                ratio2 = pow(ratio2, densenessWeight);
            }

            // If using curvatory as a coarsening criteria
            // calculate the local structure tensor
            if (flatnessRate > 0.0)
            {
                eigen_vect = getEigenVector(surfaceMesh, n, &eigen_value, &max_angle);

                if (eigen_value.x == 0)
                {
                    printf("max eigen_value is zero.... \n");
                    exit(0);
                }
                else
                {
                    //printf("Eigenvalues: %d, (%.3f, %.3f, %.3f)\n", n,
                    //	 eigen_value.x, eigen_value.y, eigen_value.z);
                    ratio1 = fabs((eigen_value.y)/(eigen_value.x));
                    ratio1 = pow(ratio1, flatnessRate);
                    //ratio1 = (1.0-max_angle)*fabs((eigen_value.y)/(eigen_value.x));
                }
            }

            // Compare the two coarseness criterias against the given coarsenessRate
            delete_vertex = ratio1*ratio2 < coarsenessRate;

            // Use maximal angle between vertex normal as a complementary coarse criteria
            if (maxNormalAngle > 0)
                delete_vertex = delete_vertex && max_angle > maxNormalAngle;

            // Deleting a vertex and retrianglulate the hole
            if (delete_vertex)
            {
                vertex_num--;

                /* delete vertex n */
                surfaceMesh->vertex[n].x = -99999;
                surfaceMesh->vertex[n].y = -99999;
                surfaceMesh->vertex[n].z = -99999;

                neighbor_number = 0;
                first_ngr = neighbourList[n];
                while (first_ngr != nullptr)
                {
                    a = first_ngr->a;
                    c = first_ngr->c;
                    face_available_list[neighbor_number] = c;
                    neighbor_tmp_list[neighbor_number] = a;
                    neighbor_number++;

                    // Get face marker
                    face_marker = surfaceMesh->face[c].marker;

                    /* delete faces associated with vertex n */
                    surfaceMesh->face[c].v1 = -1;
                    surfaceMesh->face[c].v2 = -1;
                    surfaceMesh->face[c].v3 = -1;
                    surfaceMesh->face[c].marker = -1;

                    /* delete neighbors associated with vertex n */
                    second_ngr = neighbourList[a];
                    tmp_ngr = second_ngr;
                    while (second_ngr != nullptr)
                    {
                        if (second_ngr->a == n || second_ngr->b == n)
                        {
                            if (second_ngr == neighbourList[a])
                            {
                                neighbourList[a] = second_ngr->next;
                                free(second_ngr);
                                second_ngr = neighbourList[a];
                                tmp_ngr = second_ngr;
                            }
                            else
                            {
                                tmp_ngr->next = second_ngr->next;
                                free(second_ngr);
                                second_ngr = tmp_ngr->next;
                            }
                        }
                        else
                        {
                            if (second_ngr == neighbourList[a])
                            {
                                second_ngr = second_ngr->next;
                            }
                            else
                            {
                                tmp_ngr = second_ngr;
                                second_ngr = second_ngr->next;
                            }
                        }
                    }

                    number = 0;
                    second_ngr = neighbourList[a];
                    while (second_ngr != nullptr)
                    {
                        number++;
                        second_ngr = second_ngr->next;
                    }
                    first_ngr->b = number;
                    first_ngr = first_ngr->next;
                }

                first_ngr = neighbourList[n];
                while (first_ngr->next != nullptr)
                    first_ngr = first_ngr->next;
                first_ngr->next = neighbourList[n];

                face_available_index = 0;
                subdividePolygon(surfaceMesh, neighbourList[n], face_available_list,
                                   &face_available_index, face_marker);

                /* order the neighbors */
                for (m = 0; m < neighbor_number; m++)
                {
                    first_ngr = neighbourList[neighbor_tmp_list[m]];
                    c = first_ngr->a;
                    while (first_ngr != nullptr)
                    {
                        a = first_ngr->a;
                        b = first_ngr->b;

                        second_ngr = first_ngr->next;
                        while (second_ngr != nullptr)
                        {
                            a0 = second_ngr->a;
                            b0 = second_ngr->b;

                            // Assume counter clockwise orientation
                            if (a0==b && b0!=a)
                            {
                                tmp_ngr = first_ngr;
                                while (tmp_ngr != nullptr)
                                {
                                    if (tmp_ngr->next == second_ngr)
                                    {
                                        tmp_ngr->next = second_ngr->next;
                                        break;
                                    }
                                    tmp_ngr = tmp_ngr->next;
                                }
                                tmp_ngr = first_ngr->next;
                                first_ngr->next = second_ngr;
                                second_ngr->next = tmp_ngr;
                                break;
                            }

                            second_ngr = second_ngr->next;
                        }
                        if (first_ngr->next == nullptr)
                        {
                            if (first_ngr->b != c)
                            {
                                printf("some polygons are not closed: %d \n", n);
                                // exit(0);
                            }
                        }

                        first_ngr = first_ngr->next;
                    }
                }

                /* Smooth the neighbors */
                for (m = 0; m < neighbor_number; m++)
                {
                    if (!surfaceMesh->vertex[num].selected)
                        continue;
                    num = neighbor_tmp_list[m];
                    x = surfaceMesh->vertex[num].x;
                    y = surfaceMesh->vertex[num].y;
                    z = surfaceMesh->vertex[num].z;
                    float nx = 0;
                    float ny = 0;
                    float nz = 0;

                    weight = 0;
                    first_ngr = neighbourList[num];
                    while (first_ngr != nullptr)
                    {
                        a = first_ngr->a;
                        b = first_ngr->b;
                        second_ngr = first_ngr->next;
                        if (second_ngr == nullptr)
                            second_ngr = neighbourList[num];
                        c = second_ngr->b;
                        pos_vect = getVertexPositionAlongSurface(x, y, z, b, a, c, surfaceMesh);
                        angle = computeDotProduct(surfaceMesh, b, a, c);
                        angle += 1.0;
                        nx += angle*pos_vect.x;
                        ny += angle*pos_vect.y;
                        nz += angle*pos_vect.z;

                        weight += angle;
                        first_ngr = first_ngr->next;
                    }

                    if (weight > 0)
                    {
                        nx /= weight;
                        ny /= weight;
                        nz /= weight;

                        eigen_vect = getEigenVector(surfaceMesh, num, &eigen_value, &max_angle);
                        if ((eigen_vect.x1==0 && eigen_vect.y1==0 && eigen_vect.z1==0) ||
                                (eigen_vect.x2==0 && eigen_vect.y2==0 && eigen_vect.z2==0) ||
                                (eigen_vect.x3==0 && eigen_vect.y3==0 && eigen_vect.z3==0))
                        {
                            surfaceMesh->vertex[num].x = nx;
                            surfaceMesh->vertex[num].y = ny;
                            surfaceMesh->vertex[num].z = nz;
                        }
                        else
                        {
                            nx -= x;
                            ny -= y;
                            nz -= z;
                            w1 = (nx*eigen_vect.x1+ny*eigen_vect.y1+nz*eigen_vect.z1)/
                                    (1.0+eigen_value.x);
                            w2 = (nx*eigen_vect.x2+ny*eigen_vect.y2+nz*eigen_vect.z2)/
                                    (1.0+eigen_value.y);
                            w3 = (nx*eigen_vect.x3+ny*eigen_vect.y3+nz*eigen_vect.z3)/
                                    (1.0+eigen_value.z);
                            surfaceMesh->vertex[num].x = w1*eigen_vect.x1+w2*eigen_vect.x2+
                                    w3*eigen_vect.x3 + x;
                            surfaceMesh->vertex[num].y = w1*eigen_vect.y1+w2*eigen_vect.y2+
                                    w3*eigen_vect.y3 + y;
                            surfaceMesh->vertex[num].z = w1*eigen_vect.z1+w2*eigen_vect.z2+
                                    w3*eigen_vect.z3 + z;
                        }
                    }
                }
            }
        }
        /*
      if (vertex_num < MeshSizeUpperLimit) {
      stop = 1;
      break;
      }
    */
    }

    /* Clean the lists of nodes and faces */
    int start_index = 0;
    for (size_t n = 0; n < surfaceMesh->numberVertices; n++)
    {
        if (surfaceMesh->vertex[n].x != -99999 &&
                surfaceMesh->vertex[n].y != -99999 &&
                surfaceMesh->vertex[n].z != -99999)
        {
            if (start_index != n)
            {
                surfaceMesh->vertex[start_index].x = surfaceMesh->vertex[n].x;
                surfaceMesh->vertex[start_index].y = surfaceMesh->vertex[n].y;
                surfaceMesh->vertex[start_index].z = surfaceMesh->vertex[n].z;
                surfaceMesh->vertex[start_index].marker = surfaceMesh->vertex[n].marker;
                surfaceMesh->vertex[start_index].selected = surfaceMesh->vertex[n].selected;
                neighbourList[start_index] = neighbourList[n];
            }

            vertexIndexArray[n] = start_index;
            start_index++;
        }
        else
        {
            vertexIndexArray[n] = -1;
        }
    }

    surfaceMesh->numberVertices = start_index;

    start_index = 0;
    for (size_t n = 0; n < surfaceMesh->numberFaces; n++)
    {
        a = surfaceMesh->face[n].v1;
        b = surfaceMesh->face[n].v2;
        c = surfaceMesh->face[n].v3;
        face_marker = surfaceMesh->face[n].marker;
        if (a >= 0 && b >= 0 && c >= 0 &&
                vertexIndexArray[a] >= 0 && vertexIndexArray[b] >= 0 && vertexIndexArray[c] >= 0)
        {
            surfaceMesh->face[start_index].v1 = vertexIndexArray[a];
            surfaceMesh->face[start_index].v2 = vertexIndexArray[b];
            surfaceMesh->face[start_index].v3 = vertexIndexArray[c];
            surfaceMesh->face[start_index].marker = face_marker;
            faceIndexArray[n] = start_index;
            start_index++;
        }
        else
        {
            faceIndexArray[n] = -1;
        }
    }
    surfaceMesh->numberFaces = start_index;

    for (size_t n = 0; n < surfaceMesh->numberVertices; n++)
    {
        first_ngr = neighbourList[n];
        while (first_ngr != nullptr)
        {
            a = first_ngr->a;
            b = first_ngr->b;
            c = first_ngr->c;

            first_ngr->a = vertexIndexArray[a];
            first_ngr->b = vertexIndexArray[b];
            first_ngr->c = faceIndexArray[c];

            first_ngr = first_ngr->next;
        }
    }

    free(vertexIndexArray);
    free(faceIndexArray);
    printf("\n");
    return(stop);
}
