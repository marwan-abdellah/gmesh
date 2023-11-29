#include "SurfaceMesh.hh"
#include "EigenValue.hh"
#include "EigenVector.hpp"
#include "Normal.hpp"
#include <math.h>
#include <stdio.h>

#define PIE              3.14159265358979f
#define DIM_SCALE         1.99

Vertex getVertexPositionAlongSurface(const float& x, const float& y, const float& z,
                                     const size_t& a, const size_t& b, const size_t& c,
                                     SurfaceMesh *surfaceMesh)
{
    float ax = surfaceMesh->vertex[a].x;
    float ay = surfaceMesh->vertex[a].y;
    float az = surfaceMesh->vertex[a].z;

    float bx = surfaceMesh->vertex[b].x;
    float by = surfaceMesh->vertex[b].y;
    float bz = surfaceMesh->vertex[b].z;

    float cx = surfaceMesh->vertex[c].x;
    float cy = surfaceMesh->vertex[c].y;
    float cz = surfaceMesh->vertex[c].z;

    bx -= ax; by -= ay; bz -= az;
    float distance = std::sqrt(bx * bx + by * by + bz * bz);
    if (distance > 0) { bx /= distance; by /= distance; bz /= distance; }

    cx -= ax; cy -= ay; cz -= az;
    distance = std::sqrt(cx * cx +cy * cy + cz * cz);
    if (distance > 0) { cx /= distance; cy /= distance; cz /= distance; }

    float tx = 0.5 * (cx + bx);
    float ty = 0.5 * (cy + by);
    float tz = 0.5 * (cz + bz);
    distance = sqrt(tx * tx + ty * ty + tz * tz);
    if (distance > 0) { tx /= distance; ty /= distance; tz /= distance; }

    float xx = by * cz - bz * cy;
    float yy = bz * cx - bx * cz;
    float zz = bx * cy - by *cx;
    distance = sqrt(xx * xx + yy * yy + zz * zz);
    if (distance > 0) { xx /= distance; yy /= distance; zz /= distance; }

    bx = xx; by = yy; bz = zz;
    distance = tx * (x - ax) + ty * (y - ay) + tz * (z - az);

    xx = distance * tx + ax;
    yy = distance * ty + ay;
    zz = distance * tz + az;
    distance = bx * (x - xx) + by * (y - yy) + bz * (z - zz);

    // Return the new vertex
    Vertex newVertex;
    newVertex.x = distance * bx + xx;
    newVertex.y = distance * by + yy;
    newVertex.z = distance * bz + zz;
    return newVertex;
}

float getAngleBetweenVertices(SurfaceMesh *surfaceMesh,
                              const size_t& a, const size_t& b, const size_t& c)
{
    float ax = surfaceMesh->vertex[a].x;
    float ay = surfaceMesh->vertex[a].y;
    float az = surfaceMesh->vertex[a].z;

    float bx = surfaceMesh->vertex[b].x;
    float by = surfaceMesh->vertex[b].y;
    float bz = surfaceMesh->vertex[b].z;

    float cx = surfaceMesh->vertex[c].x;
    float cy = surfaceMesh->vertex[c].y;
    float cz = surfaceMesh->vertex[c].z;

    float length1 = (ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz);
    float length2 = (ax - cx) * (ax - cx) + (ay - cy) * (ay - cy) + (az - cz) * (az - cz);
    float length3 = (bx - cx) * (bx - cx) + (by - cy) * (by - cy) + (bz - cz) * (bz - cz);

    float angle;
    if (length1 == 0 || length2 == 0) { angle = -999; }
    else
    {
        angle = 0.5 * (length1 + length2 - length3) / std::sqrt(length1 * length2);
        angle = std::acos(angle) * 180.0 / PIE;
    }

    return angle;
}

Normal getVertexNormal(SurfaceMesh *surfaceMesh, const size_t& n)
{
    float x = surfaceMesh->vertex[n].x;
    float y = surfaceMesh->vertex[n].y;
    float z = surfaceMesh->vertex[n].z;

    Normal normal; normal.x = 0; normal.y = 0; normal.z = 0;
    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour = neighbourList[n];
    int numberIterations = 0;
    while (firstNeighbour != nullptr)
    {
        int a = firstNeighbour->a;
        int b = firstNeighbour->b;

        float ax = surfaceMesh->vertex[a].x - x;
        float ay = surfaceMesh->vertex[a].y - y;
        float az = surfaceMesh->vertex[a].z - z;
        float length = std::sqrt(ax * ax + ay * ay + az * az);
        if (length > 0) { ax /= length; ay /= length; az /= length; }

        float bx = surfaceMesh->vertex[b].x - x;
        float by = surfaceMesh->vertex[b].y - y;
        float bz = surfaceMesh->vertex[b].z - z;
        length = std::sqrt(bx * bx + by * by + bz * bz);
        if (length > 0) { bx /= length; by /= length; bz /= length; }

        float gx = ay * bz - az * by;
        float gy = az * bx - ax * bz;
        float gz = ax * by - ay * bx;
        length = std::sqrt(gx * gx + gy * gy + gz * gz);
        if (length > 0) { gx /= length; gy /= length; gz /= length; }

        length = normal.x * gx + normal.y * gy + normal.z * gz;
        if (length < 0) { gx = -gx; gy = -gy; gz = -gz; }

        normal.x += gx;
        normal.y += gy;
        normal.z += gz;

        numberIterations++;
        firstNeighbour = firstNeighbour->next;
    }

    if (numberIterations > 0)
    {
        normal.x /= (float)numberIterations;
        normal.y /= (float)numberIterations;
        normal.z /= (float)numberIterations;

        float length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        if (length > 0) { normal.x /= length; normal.y /= length; normal.z /= length; }
    }
    else { normal.x = 0; normal.y = 0; normal.z = 0; }

    return normal;
}

EigenVector getEigenVector(SurfaceMesh *surfaceMesh,
                           const size_t& vertexIndex, EigenValue* eigenValue,
                           float *computedMaxAngle,
                           const bool &verbose)
{
    Normal vertexNormal = getVertexNormal(surfaceMesh, vertexIndex);
    if (verbose)
    {
        printf("Normal@ [%ld]: (%.2f, %.2f, %.2f)\n", vertexIndex,
               vertexNormal.x, vertexNormal.y, vertexNormal.z);
    }

    double A[3][3];
    A[0][0] = vertexNormal.x * vertexNormal.x;
    A[0][1] = vertexNormal.x * vertexNormal.y;
    A[0][2] = vertexNormal.x * vertexNormal.z;
    A[1][1] = vertexNormal.y * vertexNormal.y;
    A[1][2] = vertexNormal.y * vertexNormal.z;
    A[2][2] = vertexNormal.z * vertexNormal.z;

    size_t startPointer = 0;
    size_t endPointer = 1;
    size_t indexArray[333];
    size_t distArray[333];
    indexArray[startPointer] = vertexIndex;
    distArray[startPointer] = 0;

    float maxAngle = 99999.0;

    Normal currentNormal;
    currentNormal.x = vertexNormal.x;
    currentNormal.y = vertexNormal.y;
    currentNormal.z = vertexNormal.z;

    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour;

    int visited;
    while (startPointer < endPointer)
    {
        size_t index = indexArray[startPointer];
        size_t dist = distArray[startPointer];
        startPointer ++;

        if (dist < ((DIM_SCALE > 2) ? (3):(2)))
        {
            firstNeighbour = neighbourList[index];
            while (firstNeighbour != nullptr)
            {
                size_t m = firstNeighbour->a;
                visited = 0;
                for (size_t n = 0; n < endPointer; ++n)
                {
                    if (indexArray[n] == m)
                    {
                        visited = 1;
                        break;
                    }
                }

                if (visited == 0)
                {
                    vertexNormal = getVertexNormal(surfaceMesh, m);

                    float angle = currentNormal.x * vertexNormal.x +
                                  currentNormal.y * vertexNormal.y +
                                  currentNormal.z * vertexNormal.z;

                    if (angle < 0)
                        angle = -angle;

                    if (angle < maxAngle)
                        maxAngle = angle;

                    A[0][0] += vertexNormal.x * vertexNormal.x;
                    A[0][1] += vertexNormal.x * vertexNormal.y;
                    A[0][2] += vertexNormal.x * vertexNormal.z;
                    A[1][1] += vertexNormal.y * vertexNormal.y;
                    A[1][2] += vertexNormal.y * vertexNormal.z;
                    A[2][2] += vertexNormal.z * vertexNormal.z;

                    indexArray[endPointer] = m;
                    distArray[endPointer] = dist + 1;
                    endPointer++;
                }

                firstNeighbour = firstNeighbour->next;
            }
        }
    }

    // Update the maximum angle
    *computedMaxAngle = maxAngle;

    A[1][0] = A[0][1];
    A[2][0] = A[0][2];
    A[2][1] = A[1][2];

    double c0 = A[0][0] * A[1][1] * A[2][2] +
            2 * A[0][1] * A[0][2 ]* A[1][2] -
                A[0][0] * A[1][2] * A[1][2] -
                A[1][1] * A[0][2] * A[0][2] -
                A[2][2] * A[0][1] * A[0][1];

    double c1 = A[0][0] * A[1][1] - A[0][1] * A[0][1] + A[0][0] * A[2][2]-
                A[0][2] * A[0][2] + A[1][1] * A[2][2] - A[1][2] * A[1][2];
    double c2 = A[0][0] + A[1][1] + A[2][2];

    double a = (3.0 * c1 - c2 * c2) / 3.0;
    double b = (-2.0 * c2 * c2 * c2 + 9.0 * c1 * c2-27.0 * c0) / 27.0;
    double q = b * b / 4.0 + a * a * a / 27.0;

    double theta = std::atan2(sqrt(-q), -0.5 * b);
    double p = std::sqrt(0.25 * b * b - q);

    double x1 = c2 / 3.0 + 2.0 *
            std::pow(p, 1.0 / 3.0) * std::cos(theta / 3.0);
    double x2 = c2 / 3.0 - std::pow(p, 1.0 / 3.0) *
            (std::cos(theta / 3.0) + std::sqrt(3.0) * std::sin(theta / 3.0));
    double x3 = c2 / 3.0 - std::pow(p, 1.0 / 3.0) *
            (std::cos(theta / 3.0) - std::sqrt(3.0) * std::sin(theta / 3.0));

    EigenVector auxiliaryEigenVector;

    // If we have a perfect flat area inline of one of the x, y, z axis
    if (std::isnan(x1) || std::isnan(x2) || std::isnan(x3))
    {
        // printf("\t@getEigenVector: nan@ [%ld]\n", vertexIndex);

        eigenValue->x = c2;
        eigenValue->y = 0;
        eigenValue->z = 0;

        auxiliaryEigenVector.x1 = 1;
        auxiliaryEigenVector.y1 = 0;
        auxiliaryEigenVector.z1 = 0;

        auxiliaryEigenVector.x2 = 0;
        auxiliaryEigenVector.y2 = 1;
        auxiliaryEigenVector.z2 = 0;

        auxiliaryEigenVector.x3 = 0;
        auxiliaryEigenVector.y3 = 0;
        auxiliaryEigenVector.z3 = 1;

        return auxiliaryEigenVector;
    }

    double tx = std::max(x1, std::max(x2, x3));
    double ty, tz;
    if (tx == x1)
    {
        if (x2 >= x3) { ty = x2; tz = x3; }
        else { ty = x3; tz = x2; }
    }
    else if (tx == x2)
    {
        if (x1 >= x3) { ty = x1; tz = x3; }
        else { ty = x3; tz = x1; }
    }
    else if (tx == x3)
    {
        if (x1 >= x2) { ty = x1; tz = x2; }
        else { ty = x2; tz = x1; }
    }

    x1 = tx; x2 = ty; x3 = tz;

    eigenValue->x = tx;
    eigenValue->y = ty;
    eigenValue->z = tz;

    if (x1 > 99999 || x1 < -99999 || x2 > 99999 || x2 < -99999 || x3 > 99999 || x3 < -99999)
    {
        printf("ERROR @getEigenVector: [%f %f %f]\n", x1, x2, x3);
        exit(0);
    }

    A[0][0] -= x1;
    A[1][1] -= x1;
    A[2][2] -= x1;

    double B[6];
    B[0] = A[1][1] * A[2][2] - A[1][2] * A[1][2];
    B[1] = A[0][2] * A[1][2] - A[0][1] * A[2][2];
    B[2] = A[0][0] * A[2][2] - A[0][2] * A[0][2];
    B[3] = A[0][1] * A[1][2] - A[0][2] * A[1][1];
    B[4] = A[0][1] * A[0][2] - A[1][2] * A[0][0];
    B[5] = A[0][0] * A[1][1] - A[0][1] * A[0][1];

    c0 = B[0] * B[0] + B[1] * B[1] + B[3] * B[3];
    c1 = B[1] * B[1] + B[2] * B[2] + B[4] * B[4];
    c2 = B[3] * B[3] + B[4] * B[4] + B[5] * B[5];

    if (c0 >= c1 && c0 >= c2)
    {
        tx = B[0]; ty = B[1]; tz = B[3];
    }
    else if (c1 >= c0 && c1 >= c2)
    {
        tx = B[1]; ty = B[2]; tz = B[4];
    }
    else if (c2 >= c0 && c2 >= c1)
    {
        tx = B[3]; ty = B[4]; tz = B[5];
    }

    p = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (p > 0) { tx /= p; ty /= p; tz /= p; }

    auxiliaryEigenVector.x1 = tx;
    auxiliaryEigenVector.y1 = ty;
    auxiliaryEigenVector.z1 = tz;

    A[0][0] += x1;
    A[1][1] += x1;
    A[2][2] += x1;

    A[0][0] -= x2;
    A[1][1] -= x2;
    A[2][2] -= x2;

    B[0] = A[1][1] * A[2][2] - A[1][2] * A[1][2];
    B[1] = A[0][2] * A[1][2] - A[0][1] * A[2][2];
    B[2] = A[0][0] * A[2][2] - A[0][2] * A[0][2];
    B[3] = A[0][1] * A[1][2] - A[0][2] * A[1][1];
    B[4] = A[0][1] * A[0][2] - A[1][2] * A[0][0];
    B[5] = A[0][0] * A[1][1] - A[0][1] * A[0][1];

    c0 = B[0] * B[0]+ B[1] * B[1] + B[3] * B[3];
    c1 = B[1] * B[1]+ B[2] * B[2] + B[4] * B[4];
    c2 = B[3] * B[3]+ B[4] * B[4] + B[5] * B[5];
    if (c0 >= c1 && c0 >= c2)
    {
        tx = B[0]; ty = B[1]; tz = B[3];
    }
    else if (c1 >= c0 && c1 >= c2)
    {
        tx = B[1]; ty = B[2]; tz = B[4];
    }
    else if (c2 >= c0 && c2 >= c1)
    {
        tx = B[3]; ty = B[4]; tz = B[5];
    }
    p = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (p > 0) { tx /= p; ty /= p; tz /= p; }

    auxiliaryEigenVector.x2 = tx;
    auxiliaryEigenVector.y2 = ty;
    auxiliaryEigenVector.z2 = tz;

    auxiliaryEigenVector.x3 = auxiliaryEigenVector.y1 * tz-auxiliaryEigenVector.z1 * ty;
    auxiliaryEigenVector.y3 = auxiliaryEigenVector.z1 * tx-auxiliaryEigenVector.x1 * tz;
    auxiliaryEigenVector.z3 = auxiliaryEigenVector.x1 * ty-auxiliaryEigenVector.y1 * tx;

    return auxiliaryEigenVector;
}

float computeDotProduct(SurfaceMesh *surfaceMesh,
                        const size_t& a, const size_t& b, const size_t& c)
{
    float bx = surfaceMesh->vertex[b].x - surfaceMesh->vertex[a].x;
    float by = surfaceMesh->vertex[b].y - surfaceMesh->vertex[a].y;
    float bz = surfaceMesh->vertex[b].z - surfaceMesh->vertex[a].z;
    float length = std::sqrt(bx * bx + by * by + bz * bz);
    if (length > 0) { bx /= length; by /= length; bz /= length; }

    float cx = surfaceMesh->vertex[c].x - surfaceMesh->vertex[a].x;
    float cy = surfaceMesh->vertex[c].y - surfaceMesh->vertex[a].y;
    float cz = surfaceMesh->vertex[c].z - surfaceMesh->vertex[a].z;
    length = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (length > 0) { cx /= length; cy /= length; cz /= length; }

    length = bx * cx + by * cy + bz * cz;
    return length;
}

Normal computeCrossProduct(SurfaceMesh *surfaceMesh,
                           const size_t& a, const size_t& b, const size_t& c)
{
    float bx = surfaceMesh->vertex[b].x - surfaceMesh->vertex[a].x;
    float by = surfaceMesh->vertex[b].y - surfaceMesh->vertex[a].y;
    float bz = surfaceMesh->vertex[b].z - surfaceMesh->vertex[a].z;
    float length = sqrt(bx * bx + by * by + bz * bz);
    if (length > 0) { bx /= length; by /= length; bz /= length; }

    float cx = surfaceMesh->vertex[c].x - surfaceMesh->vertex[a].x;
    float cy = surfaceMesh->vertex[c].y - surfaceMesh->vertex[a].y;
    float cz = surfaceMesh->vertex[c].z - surfaceMesh->vertex[a].z;
    length = sqrt(cx * cx + cy * cy + cz * cz);
    if (length > 0) { cx /= length; cy /= length; cz /= length; }

    float gx = cy * bz - cz * by;
    float gy = cz * bx - cx * bz;
    float gz = cx * by - cy * bx;
    length = sqrt(gx * gx + gy * gy + gz * gz);
    if (length > 0) { gx /= length; gy /= length; gz /= length; }

    Normal value; value.x = gx; value.y = gy; value.z = gz;
    return value;
}

Normal rotate(const float& sx, const float& sy, const float& sz,
              const float& theta, const float& phi, const float& angle)
{
    float a[3][3], b[3][3];
    a[0][0] = (float)(std::cos(0.5 * PIE - phi) * std::cos(theta));
    a[0][1] = (float)(std::cos(0.5 * PIE - phi) * std::sin(theta));
    a[0][2] = (float)-std::sin(0.5 * PIE - phi);
    a[1][0] = (float)-std::sin(theta);
    a[1][1] = (float) std::cos(theta);
    a[1][2] = 0.f;
    a[2][0] = (float)(std::sin(0.5 * PIE - phi) * std::cos(theta));
    a[2][1] = (float)(std::sin(0.5 * PIE - phi) * std::sin(theta));
    a[2][2] = (float) std::cos(0.5 * PIE - phi);

    b[0][0] = (float)(std::cos(0.5 * PIE - phi) * std::cos(theta));
    b[0][1] = (float)-std::sin(theta);
    b[0][2] = (float)(std::sin(0.5 * PIE - phi) * std::cos(theta));
    b[1][0] = (float)(std::cos(0.5 * PIE - phi) * std::sin(theta));
    b[1][1] = (float)std::cos(theta);
    b[1][2] = (float)(std::sin(0.5 * PIE - phi) * std::sin(theta));
    b[2][0] = (float)-std::sin(0.5 * PIE - phi);
    b[2][1] = 0.f;
    b[2][2] = (float) std::cos(0.5 * PIE - phi);

    float x = a[0][0] * sx + a[0][1] * sy + a[0][2] * sz;
    float y = a[1][0] * sx + a[1][1] * sy + a[1][2] * sz;
    float z = a[2][0] * sx + a[2][1] * sy + a[2][2] * sz;

    float xx = (float)(std::cos(angle) * x - std::sin(angle) * y);
    float yy = (float)(std::sin(angle) * x + std::cos(angle) * y);
    float zz = z;

    Normal rotationVector;
    rotationVector.x = b[0][0] * xx + b[0][1] * yy + b[0][2] * zz;
    rotationVector.y = b[1][0] * xx + b[1][1] * yy + b[1][2] * zz;
    rotationVector.z = b[2][0] * xx + b[2][1] * yy + b[2][2] * zz;
    return rotationVector;
}

char checkFlipAction(SurfaceMesh *surfaceMesh,
                     const size_t& a, const size_t& b, const size_t& c, const size_t& d,
                     const bool& preserveRidges)
{
    NPNT3 **neighbourList = surfaceMesh->neighborList;

    /// Smaller angle criterion
    float minAngle1 = -99999;
    float angle = computeDotProduct(surfaceMesh, a, b, c);
    if (angle > minAngle1)
        minAngle1 = angle;
    angle = computeDotProduct(surfaceMesh, a, b, d);
    if (angle > minAngle1)
        minAngle1 = angle;
    angle = computeDotProduct(surfaceMesh, b, a, c);
    if (angle > minAngle1)
        minAngle1 = angle;
    angle = computeDotProduct(surfaceMesh, b, a, d);
    if (angle > minAngle1)
        minAngle1 = angle;

    float minAngle2 = -99999;
    angle = computeDotProduct(surfaceMesh, c, a, d);
    if (angle > minAngle2)
        minAngle2 = angle;
    angle = computeDotProduct(surfaceMesh, c, b, d);
    if (angle > minAngle2)
        minAngle2 = angle;
    angle = computeDotProduct(surfaceMesh, d, a, c);
    if (angle > minAngle2)
        minAngle2 = angle;
    angle = computeDotProduct(surfaceMesh, d, b, c);
    if (angle > minAngle2)
        minAngle2 = angle;

    // Check which of the triangle combination has the smallest angle
    // minAngle1 is the minimal angle of the flipped configuration
    // minAngle2 is the minimal angle of the present configuration
    if (minAngle1 > minAngle2)
    {
        // Check if the angle between the normals of the two triangles are too small for a flip
        // action, for example if we are on a ridge
        Normal normal1 = computeCrossProduct(surfaceMesh, a, c, b);
        Normal normal2 = computeCrossProduct(surfaceMesh, a, b, d);

        // If we want to preserve the ridges the angle between the surface normals must be smaller
        // than cos(60deg) to flip the edges
        if (not preserveRidges ||
                normal1.x * normal2.x + normal1.y * normal2.y + normal1.z * normal2.z > 0.866)
            return 1;
    }

    return 0;
}

void getMinMaxAngles(SurfaceMesh *surfaceMesh,
                     float *computedMinangle, float *computedMaxangle,
                     size_t *computedNumberSmallerAngles, size_t *computedNumberLargerAngles,
                     const float& maxMinAngle, const float& minMaxAngle)
{
    float minAngle = 99999.0;
    float maxAngle = -99999.0;

    size_t numberSmallerAngles = 0;
    size_t numberLargerAngles = 0;

    for (size_t n = 0; n < surfaceMesh->numberFaces; n++)
    {
        size_t a = surfaceMesh->face[n].v1;
        size_t b = surfaceMesh->face[n].v2;
        size_t c = surfaceMesh->face[n].v3;

        float angle = getAngleBetweenVertices(surfaceMesh, a, b, c);
        if (angle != -999)
        {
            if (angle < minAngle)
                minAngle = angle;

            if (angle > maxAngle)
                maxAngle = angle;

            if (angle < maxMinAngle)
                numberSmallerAngles++;

            if (angle > minMaxAngle)
                numberLargerAngles++;
        }

        angle = getAngleBetweenVertices(surfaceMesh, b, a, c);
        if (angle != -999)
        {
            if (angle < minAngle)
                minAngle = angle;

            if (angle > maxAngle)
                maxAngle = angle;

            if (angle < maxMinAngle)
                numberSmallerAngles++;

            if (angle > minMaxAngle)
                numberLargerAngles++;
        }

        angle = getAngleBetweenVertices(surfaceMesh, c, a, b);
        if (angle != -999)
        {
            if (angle < minAngle)
                minAngle = angle;

            if (angle > maxAngle)
                maxAngle = angle;

            if (angle < maxMinAngle)
                numberSmallerAngles++;

            if (angle > minMaxAngle)
                numberLargerAngles++;
        }
    }

    *computedMinangle = minAngle;
    *computedMaxangle = maxAngle;
    *computedNumberSmallerAngles = numberSmallerAngles;
    *computedNumberLargerAngles = numberLargerAngles;
}

void edgeFlipping(SurfaceMesh *surfaceMesh, const size_t& n, const bool& preserveRidges)
{
    int a, b, c;
    int f1, f2;
    char flipFlag, flipCheck;

    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour = neighbourList[n];
    NPNT3 *auxNeighbour1, *auxNeighbour2;
    NPNT3 *secondNeighbour;
    size_t numberIterations;
    while (firstNeighbour != nullptr)
    {
        numberIterations = 0;
        NPNT3* auxNeighbour = neighbourList[n];
        while (auxNeighbour != nullptr)
        {
            numberIterations++;
            auxNeighbour = auxNeighbour->next;
        }

        if (numberIterations <= 3)
        {
            if (numberIterations > 0)
            {
                float ax = 0;
                float ay = 0;
                float az = 0;

                auxNeighbour = neighbourList[n];

                while (auxNeighbour != nullptr)
                {
                    a = auxNeighbour->a;
                    ax += surfaceMesh->vertex[a].x;
                    ay += surfaceMesh->vertex[a].y;
                    az += surfaceMesh->vertex[a].z;
                    auxNeighbour = auxNeighbour->next;
                }

                surfaceMesh->vertex[n].x = ax / (float) numberIterations;
                surfaceMesh->vertex[n].y = ay / (float) numberIterations;
                surfaceMesh->vertex[n].z = az / (float) numberIterations;
            }
            return;
        }

        a = firstNeighbour->a;
        b = firstNeighbour->b;

        secondNeighbour = firstNeighbour->next;
        if (secondNeighbour == nullptr)
            secondNeighbour = neighbourList[n];

        c = secondNeighbour->b;

        flipFlag = 1;
        numberIterations = 0;
        auxNeighbour = neighbourList[b];
        while (auxNeighbour != nullptr)
        {
            numberIterations++;
            auxNeighbour = auxNeighbour->next;
        }

        if (numberIterations <= 3)
            flipFlag = 0;

        auxNeighbour = neighbourList[a];
        while (auxNeighbour != nullptr)
        {
            if (auxNeighbour->a == c)
                flipFlag = 0;
            auxNeighbour = auxNeighbour->next;
        }

        auxNeighbour = neighbourList[c];
        while (auxNeighbour != nullptr)
        {
            if (auxNeighbour->a == a)
                flipFlag = 0;
            auxNeighbour = auxNeighbour->next;
        }

        if (flipFlag)
        {
            flipCheck = checkFlipAction(surfaceMesh, n, b, a, c, preserveRidges);
            if (flipCheck)
            {
                f1 = firstNeighbour->c;
                f2 = secondNeighbour->c;

                // Update face info
                surfaceMesh->face[f1].v1 = n;
                surfaceMesh->face[f1].v2 = a;
                surfaceMesh->face[f1].v3 = c;
                surfaceMesh->face[f2].v1 = b;
                surfaceMesh->face[f2].v2 = c; // Switch a and c to make the face normal outwards
                surfaceMesh->face[f2].v3 = a; // Switch a and c to make the face normal outwards

                // Delete the entries in neighbor lists
                firstNeighbour->b = c;
                if (firstNeighbour->next == nullptr)
                    neighbourList[n] = neighbourList[n]->next;
                else
                    firstNeighbour->next = secondNeighbour->next;
                auxNeighbour1 = secondNeighbour;

                auxNeighbour = neighbourList[b];
                while (auxNeighbour != nullptr)
                {
                    if (auxNeighbour->b == n)
                        break;
                    auxNeighbour = auxNeighbour->next;
                }

                if (auxNeighbour == nullptr)
                    printf("ERROR @edgeFlipping @ [%ld]\n", n);

                if (auxNeighbour->a == c)
                {
                    auxNeighbour->b = a;
                    auxNeighbour->c = f2;

                    if (auxNeighbour->next == nullptr)
                    {
                        secondNeighbour = neighbourList[b];
                        neighbourList[b] = secondNeighbour->next;
                    }
                    else
                    {
                        secondNeighbour = auxNeighbour->next;
                        auxNeighbour->next = secondNeighbour->next;
                    }
                    auxNeighbour2 = secondNeighbour;
                }
                else
                {
                    printf("DELETE ERROR @edgeFlipping [%ld : %d %d %d]\n", n, a, b, c);
                    printf("\t[%f, %f, %f]\n",
                           surfaceMesh->vertex[n].x,
                           surfaceMesh->vertex[n].y,
                           surfaceMesh->vertex[n].z);
                }

                // Add the entries in neighbor lists
                auxNeighbour = neighbourList[a];
                while (auxNeighbour != nullptr)
                {
                    if ((auxNeighbour->a == n && auxNeighbour->b == b) ||
                        (auxNeighbour->a == b && auxNeighbour->b == n))
                        break;

                    auxNeighbour = auxNeighbour->next;
                }

                // Assume neigbors are stored counter clockwise
                if (auxNeighbour->a == b && auxNeighbour->b == n)
                {
                    auxNeighbour->b = c;
                    auxNeighbour->c = f2;
                    auxNeighbour1->a = c;
                    auxNeighbour1->b = n;
                    auxNeighbour1->c = f1;
                    auxNeighbour1->next = auxNeighbour->next;
                    auxNeighbour->next = auxNeighbour1;
                }
                else
                {
                    printf("ERROR @edgeFlipping: auxNeighbour->a == b && auxNeighbour->b == n\n");
                }

                auxNeighbour = neighbourList[c];
                while (auxNeighbour != nullptr)
                {
                    if ((auxNeighbour->a == n && auxNeighbour->b == b) ||
                            (auxNeighbour->a == b && auxNeighbour->b == n))
                        break;
                    auxNeighbour = auxNeighbour->next;
                }

                // Assume neigbors are stored counter clockwise
                if (auxNeighbour->a == n && auxNeighbour->b == b)
                {
                    auxNeighbour->b = a;
                    auxNeighbour->c = f1;
                    auxNeighbour2->a = a;
                    auxNeighbour2->b = b;
                    auxNeighbour2->c = f2;
                    auxNeighbour2->next = auxNeighbour->next;
                    auxNeighbour->next = auxNeighbour2;
                }
                else
                {
                    printf("ERROR @edgeFlipping: auxNeighbour->a == n && auxNeighbour->b == b\n");
                }
            }
        }

        firstNeighbour = firstNeighbour->next;
    }
}

void moveVerticesAlongSurface(SurfaceMesh *surfaceMesh, const size_t& n)
{
    float x = surfaceMesh->vertex[n].x;
    float y = surfaceMesh->vertex[n].y;
    float z = surfaceMesh->vertex[n].z;

    float nx = 0;
    float ny = 0;
    float nz = 0;

    float weight = 0;
    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour = neighbourList[n];
    while (firstNeighbour != nullptr)
    {
        int a = firstNeighbour->a;
        int b = firstNeighbour->b;

        NPNT3* secondNeighbour = firstNeighbour->next;
        if (secondNeighbour == nullptr)
            secondNeighbour = neighbourList[n];

        int c = secondNeighbour->b;
        Vertex newVertexPosition = getVertexPositionAlongSurface(x, y, z, b, a, c, surfaceMesh);
        float angle = computeDotProduct(surfaceMesh, b, a, c);
        angle += 1.0;
        nx += angle * newVertexPosition.x;
        ny += angle * newVertexPosition.y;
        nz += angle * newVertexPosition.z;

        weight += angle;
        firstNeighbour = firstNeighbour->next;
    }

    if (weight > 0)
    {
        nx /= weight; ny /= weight; nz /= weight;

        EigenValue eigenValue;
        float maxAngle;
        EigenVector eigenVector = getEigenVector(surfaceMesh, n, &eigenValue, &maxAngle);

        if ((eigenVector.x1 == 0 && eigenVector.y1 == 0 && eigenVector.z1 == 0) ||
            (eigenVector.x2 == 0 && eigenVector.y2 == 0 && eigenVector.z2 == 0) ||
            (eigenVector.x3 == 0 && eigenVector.y3 == 0 && eigenVector.z3 == 0))
        {
            // printf("\t@moveVerticesAlongSurface: "
            //        "Old point [%0.2f, %0.2f, %0.2f] New point [%0.2f, %0.2f, %0.2f]\n",
            //        surfaceMesh->vertex[n].x, surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z,
            //        nx, ny, nz);

            surfaceMesh->vertex[n].x = nx;
            surfaceMesh->vertex[n].y = ny;
            surfaceMesh->vertex[n].z = nz;
        }
        else
        {
            nx -= x; ny -= y; nz -= z;

            float w1 = (nx * eigenVector.x1 + ny * eigenVector.y1 + nz * eigenVector.z1) /
                    ( 1.0 + eigenValue.x);
            float w2 = (nx * eigenVector.x2 + ny * eigenVector.y2 + nz * eigenVector.z2) /
                    ( 1.0 + eigenValue.y);
            float w3 = (nx * eigenVector.x3 + ny * eigenVector.y3 + nz * eigenVector.z3) /
                    ( 1.0 + eigenValue.z);

            nx = w1 * eigenVector.x1 + w2 * eigenVector.x2 + w3 * eigenVector.x3 + x;
            ny = w1 * eigenVector.y1 + w2 * eigenVector.y2 + w3 * eigenVector.y3 + y;
            nz = w1 * eigenVector.z1 + w2 * eigenVector.z2 + w3 * eigenVector.z3 + z;

            // printf("\t@moveVerticesAlongSurface: "
            //        "Old point [%0.2f, %0.2f, %0.2f] New point [%0.2f, %0.2f, %0.2f]\n",
            //        surfaceMesh->vertex[n].x, surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z,
            //        nx, ny, nz);

            surfaceMesh->vertex[n].x = nx;
            surfaceMesh->vertex[n].y = ny;
            surfaceMesh->vertex[n].z = nz;
        }
    }
}

void smoothNormal(SurfaceMesh *surfaceMesh, const size_t& n)
{
    int a, b, c, d, e;
    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour, *second_ngr, *third_ngr;
    NPNT3 *tmp_ngr;
    float bx, by, bz;
    float cx, cy, cz;
    float dx, dy, dz;
    float fx, fy, fz;
    float gx, gy, gz;
    float pos_x, pos_y, pos_z;
    int numberIterations, num;
    float theta, phi, alpha;
    float length;
    Normal normal, sv;


    numberIterations = 0;
    pos_x = 0;
    pos_y = 0;
    pos_z = 0;
    firstNeighbour = neighbourList[n];
    while (firstNeighbour != nullptr)
    {
        a = firstNeighbour->a;
        b = firstNeighbour->b;
        second_ngr = firstNeighbour->next;
        if (second_ngr == nullptr)
            second_ngr = neighbourList[n];
        c = second_ngr->b;
        third_ngr = second_ngr->next;
        if (third_ngr == nullptr)
            third_ngr = neighbourList[n];
        d = third_ngr->b;

        tmp_ngr = neighbourList[b];

        // If a vertex is neigbor with a non selected vertex continue
        if (!surfaceMesh->vertex[b].selected)
            return;

        while (tmp_ngr != nullptr)
        {
            if ((tmp_ngr->a == c && tmp_ngr->b != n) ||
                    (tmp_ngr->b == c && tmp_ngr->a != n))
                break;
            tmp_ngr = tmp_ngr->next;
        }
        if (tmp_ngr->a == c && tmp_ngr->b != n)
            e = tmp_ngr->b;
        else if (tmp_ngr->b == c && tmp_ngr->a != n)
            e = tmp_ngr->a;
        else
            printf("normal smoothing error...\n");

        normal = computeCrossProduct(surfaceMesh, n, b, c);
        gx = normal.x;
        gy = normal.y;
        gz = normal.z;
        dx = 0;
        dy = 0;
        dz = 0;

        num  = 0;
        normal = computeCrossProduct(surfaceMesh, n, a, b);
        length = normal.x*gx+normal.y*gy+normal.z*gz;
        if (length > 0)
        {
            num++;
            dx += length*normal.x;
            dy += length*normal.y;
            dz += length*normal.z;
        }
        normal = computeCrossProduct(surfaceMesh, n, c, d);
        length = normal.x*gx+normal.y*gy+normal.z*gz;
        if (length > 0)
        {
            num++;
            dx += length*normal.x;
            dy += length*normal.y;
            dz += length*normal.z;
        }
        normal = computeCrossProduct(surfaceMesh, b, e, c);
        length = normal.x*gx+normal.y*gy+normal.z*gz;
        if (length > 0)
        {
            num++;
            dx += length*normal.x;
            dy += length*normal.y;
            dz += length*normal.z;
        }

        length = std::sqrt(dx*dx+dy*dy+dz*dz);
        if (length > 0)
        {
            dx /= length;
            dy /= length;
            dz /= length;
            fx = gy*dz-gz*dy;
            fy = gz*dx-gx*dz;
            fz = gx*dy-gy*dx;
            cx = surfaceMesh->vertex[c].x;
            cy = surfaceMesh->vertex[c].y;
            cz = surfaceMesh->vertex[c].z;
            bx = surfaceMesh->vertex[b].x;
            by = surfaceMesh->vertex[b].y;
            bz = surfaceMesh->vertex[b].z;
            length = fx*(bx-cx)+fy*(by-cy)+fz*(bz-cz);
            if (length >= 0)
            {
                theta = (float) std::atan2(by-cy, bx-cx);
                phi = (float) std::atan2(bz-cz, sqrt((bx-cx)*(bx-cx)+(by-cy)*(by-cy)));
            }
            else
            {
                theta = (float) std::atan2(cy-by, cx-bx);
                phi = (float) std::atan2(cz-bz, sqrt((bx-cx)*(bx-cx)+(by-cy)*(by-cy)));
            }

            alpha = std::acos(dx*gx+dy*gy+dz*gz)/(float)(4.0-num);
            sv = rotate(surfaceMesh->vertex[n].x-cx, surfaceMesh->vertex[n].y-cy, surfaceMesh->vertex[n].z-cz, theta, phi, alpha);
            pos_x += sv.x+cx;
            pos_y += sv.y+cy;
            pos_z += sv.z+cz;

            numberIterations++;
        }

        firstNeighbour = firstNeighbour->next;
    }

    if (numberIterations > 0 && !std::isnan(pos_x) && !std::isnan(pos_y) && !std::isnan(pos_z))
    {
        surfaceMesh->vertex[n].x = pos_x/(float)numberIterations;
        surfaceMesh->vertex[n].y = pos_y/(float)numberIterations;
        surfaceMesh->vertex[n].z = pos_z/(float)numberIterations;
    }
}

void subdividePolygon(SurfaceMesh *surfaceMesh,
                        NPNT3 *start_ngr, int *face_available_list,
                        int *face_available_index, int face_marker)
{
    NPNT3 **neighbourList = surfaceMesh->neighborList;
    NPNT3 *firstNeighbour, *second_ngr;
    NPNT3 *tmp_ngr, *first_copy_ngr, *second_copy_ngr;
    int min_num, degree;
    int face_index, numberIterations;
    int a, b, c;


    numberIterations = 1;
    tmp_ngr = start_ngr;
    while (tmp_ngr->next != start_ngr)
    {
        numberIterations++;
        tmp_ngr = tmp_ngr->next;
    }

    if (numberIterations < 3)
    {
        printf("error: numberIterations of nodes less than 3 \n");
        exit(0);
    }

    if (numberIterations == 3)
    {
        a = start_ngr->a;
        tmp_ngr = start_ngr->next;
        free(start_ngr);
        start_ngr = tmp_ngr;

        b = start_ngr->a;
        tmp_ngr = start_ngr->next;
        free(start_ngr);
        start_ngr = tmp_ngr;

        c = start_ngr->a;
        tmp_ngr = start_ngr->next;
        free(start_ngr);
        start_ngr = tmp_ngr;

        face_index = face_available_list[*face_available_index];
        surfaceMesh->face[face_index].v1 = a;
        surfaceMesh->face[face_index].v2 = b;
        surfaceMesh->face[face_index].v3 = c;
        surfaceMesh->face[face_index].marker = face_marker;
        *face_available_index += 1;

        firstNeighbour = (NPNT3 *)malloc(sizeof(NPNT3));
        firstNeighbour->a = b;
        firstNeighbour->b = c;
        firstNeighbour->c = face_index;
        firstNeighbour->next = neighbourList[a];
        neighbourList[a] = firstNeighbour;

        firstNeighbour = (NPNT3 *)malloc(sizeof(NPNT3));
        firstNeighbour->a = c;
        firstNeighbour->b = a;
        firstNeighbour->c = face_index;
        firstNeighbour->next = neighbourList[b];
        neighbourList[b] = firstNeighbour;

        firstNeighbour = (NPNT3 *)malloc(sizeof(NPNT3));
        firstNeighbour->a = a;
        firstNeighbour->b = b;
        firstNeighbour->c = face_index;
        firstNeighbour->next = neighbourList[c];
        neighbourList[c] = firstNeighbour;

    }
    else
    {
        tmp_ngr = start_ngr;
        min_num = tmp_ngr->b;
        firstNeighbour = tmp_ngr;
        tmp_ngr = tmp_ngr->next;
        while (tmp_ngr != start_ngr)
        {
            degree = tmp_ngr->b;
            if (degree < min_num)
            {
                min_num = degree;
                firstNeighbour = tmp_ngr;
            }
            tmp_ngr = tmp_ngr->next;
        }

        min_num = 99999;
        tmp_ngr = start_ngr;
        if (tmp_ngr != firstNeighbour &&
                tmp_ngr != firstNeighbour->next &&
                tmp_ngr->next != firstNeighbour)
        {
            min_num = tmp_ngr->b;
            second_ngr = tmp_ngr;
        }

        tmp_ngr = tmp_ngr->next;
        while (tmp_ngr != start_ngr)
        {
            degree = tmp_ngr->b;
            if (tmp_ngr != firstNeighbour &&
                    tmp_ngr != firstNeighbour->next &&
                    tmp_ngr->next != firstNeighbour &&
                    degree < min_num)
            {
                min_num = degree;
                second_ngr = tmp_ngr;
            }
            tmp_ngr = tmp_ngr->next;
        }

        firstNeighbour->b += 1;
        second_ngr->b += 1;
        first_copy_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        first_copy_ngr->a = firstNeighbour->a;
        first_copy_ngr->b = firstNeighbour->b;
        second_copy_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        second_copy_ngr->a = second_ngr->a;
        second_copy_ngr->b = second_ngr->b;
        tmp_ngr = firstNeighbour;
        while (tmp_ngr->next != firstNeighbour)
            tmp_ngr = tmp_ngr->next;
        tmp_ngr->next = first_copy_ngr;
        first_copy_ngr->next = second_copy_ngr;
        second_copy_ngr->next = second_ngr->next;
        second_ngr->next = firstNeighbour;

        subdividePolygon(surfaceMesh, firstNeighbour, face_available_list,
                           face_available_index, face_marker);
        subdividePolygon(surfaceMesh, first_copy_ngr, face_available_list,
                           face_available_index, face_marker);
    }

    return;
}
