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

    bx -= ax;
    by -= ay;
    bz -= az;
    float distance = std::sqrt(bx * bx + by * by + bz * bz);
    if (distance > 0) { bx /= distance; by /= distance; bz /= distance; }

    cx -= ax;
    cy -= ay;
    cz -= az;
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

    bx = xx;
    by = yy;
    bz = zz;
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
    if (length1 == 0 || length2 == 0)
    {
        angle = -999;
    }
    else
    {
        angle = 0.5 * (length1 + length2 - length3) / std::sqrt(length1 * length2);
        angle = std::acos(angle) * 180.0 / PIE;
    }

    return angle;
}


void SurfaceMesh_getMinMaxAngles(SurfaceMesh *surfaceMesh,
                                 float *minAngle, float *maxAngle,
                                 int *smallestNumber, int *largestNumber,
                                 int maxMinAngle, int minMaxAngle)
{
    int num1 = 0; int num2 = 0;
    float minAngleLocal = 99999.0; float maxAngleLocal = -99999.0;

    for (size_t n = 0; n < surfaceMesh->numberFaces; ++n)
    {
        size_t a = surfaceMesh->face[n].v1;
        size_t b = surfaceMesh->face[n].v2;
        size_t c = surfaceMesh->face[n].v3;

        float angle = getAngleBetweenVertices(surfaceMesh, a, b, c);

        if (angle != -999)
        {
            if (angle < minAngleLocal)
                minAngleLocal = angle;

            if (angle > maxAngleLocal)
                maxAngleLocal = angle;

            if (angle < maxMinAngle)
                num1++;

            if (angle > minMaxAngle)
                num2++;
        }

        angle = getAngleBetweenVertices(surfaceMesh, b, a, c);
        if (angle != -999)
        {
            if (angle < minAngleLocal)
                minAngleLocal = angle;

            if (angle > maxAngleLocal)
                maxAngleLocal = angle;

            if (angle < maxMinAngle)
                num1++;

            if (angle > minMaxAngle)
                num2++;
        }

        angle = getAngleBetweenVertices(surfaceMesh, c, a, b);
        if (angle != -999)
        {
            if (angle < minAngleLocal)
                minAngleLocal = angle;

            if (angle > maxAngleLocal)
                maxAngleLocal = angle;

            if (angle < maxMinAngle)
                num1++;

            if (angle > minMaxAngle)
                num2++;
        }
    }

    *minAngle = minAngleLocal;
    *maxAngle = maxAngleLocal;
    *smallestNumber = num1;
    *largestNumber = num2;
}

Normal GetNormals(SurfaceMesh *surfaceMesh, int n)
{
    int a, b;
    float x, y, z;
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr;
    int number;
    Normal normal;
    float gx, gy, gz;
    float ax, ay, az;
    float bx, by, bz;
    float length;

    x = surfaceMesh->vertex[n].x;
    y = surfaceMesh->vertex[n].y;
    z = surfaceMesh->vertex[n].z;

    number = 0;
    normal.x = 0;
    normal.y = 0;
    normal.z = 0;
    first_ngr = neighbor_list[n];
    while (first_ngr != NULL)
    {
        a = first_ngr->a;
        b = first_ngr->b;

        ax = surfaceMesh->vertex[a].x-x;
        ay = surfaceMesh->vertex[a].y-y;
        az = surfaceMesh->vertex[a].z-z;
        length = std::sqrt(ax*ax+ay*ay+az*az);
        if (length > 0)
        {
            ax /= length;
            ay /= length;
            az /= length;
        }
        bx = surfaceMesh->vertex[b].x-x;
        by = surfaceMesh->vertex[b].y-y;
        bz = surfaceMesh->vertex[b].z-z;
        length = std::sqrt(bx*bx+by*by+bz*bz);
        if (length > 0)
        {
            bx /= length;
            by /= length;
            bz /= length;
        }
        gx = ay*bz-az*by;
        gy = az*bx-ax*bz;
        gz = ax*by-ay*bx;
        length = std::sqrt(gx*gx+gy*gy+gz*gz);
        if (length > 0)
        {
            gx /= length;
            gy /= length;
            gz /= length;
        }
        length = normal.x*gx+normal.y*gy+normal.z*gz;
        if (length < 0)
        {
            gx = -gx;
            gy = -gy;
            gz = -gz;
        }
        normal.x += gx;
        normal.y += gy;
        normal.z += gz;

        number ++;
        first_ngr = first_ngr->next;
    }

    if (number > 0)
    {
        normal.x /= (float)number;
        normal.y /= (float)number;
        normal.z /= (float)number;
        length = std::sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
        if (length > 0)
        {
            normal.x /= length;
            normal.y /= length;
            normal.z /= length;
        }
    }
    else
    {
        normal.x = 0;
        normal.y = 0;
        normal.z = 0;
    }

    return normal;
}


EigenVector getEigenVector(SurfaceMesh *surfaceMesh,
                         const size_t& index0, EigenValue* eigenValue,
                         float *max_ang)
{
    int index, dist;
    int n, m;
    double x1, x2, x3;
    double a, b, Q;
    double c0, c1, c2;
    double A[3][3];
    double B[6];
    double theta, p;
    double tx, ty, tz;
    EigenVector tmp;
    Normal normal, normal0;

    int IndexArray[333];
    int DistArray[333];
    int start_ptr, end_ptr;
    int visited;

    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr;
    float angle, max_angle;

    normal = GetNormals(surfaceMesh, index0);

    //printf("Normal: %d (%.2f, %.2f, %.2f)\n", index0, normal.x, normal.y, normal.z);
    A[0][0] = normal.x*normal.x;
    A[0][1] = normal.x*normal.y;
    A[0][2] = normal.x*normal.z;
    A[1][1] = normal.y*normal.y;
    A[1][2] = normal.y*normal.z;
    A[2][2] = normal.z*normal.z;

    start_ptr = 0;
    end_ptr = 1;
    IndexArray[start_ptr] = index0;
    DistArray[start_ptr] = 0;

    max_angle = 99999.0;
    normal0.x = normal.x;
    normal0.y = normal.y;
    normal0.z = normal.z;
    while (start_ptr < end_ptr)
    {
        index = IndexArray[start_ptr];
        dist = DistArray[start_ptr];
        start_ptr ++;

        if (dist < ((DIM_SCALE>2) ? (3):(2)))
        {
            first_ngr = neighbor_list[index];
            while (first_ngr != NULL)
            {
                m = first_ngr->a;
                visited = 0;
                for (n = 0; n < end_ptr; n++)
                {
                    if (IndexArray[n] == m)
                    {
                        visited = 1;
                        //printf("Visited!\n");
                        break;
                    }
                }
                if (visited == 0)
                {
                    normal = GetNormals(surfaceMesh, m);
                    angle = normal0.x*normal.x+normal0.y*normal.y+normal0.z*normal.z;
                    if (angle < 0)
                        angle = -angle;
                    if (angle < max_angle)
                        max_angle = angle;
                    A[0][0] += normal.x*normal.x;
                    A[0][1] += normal.x*normal.y;
                    A[0][2] += normal.x*normal.z;
                    A[1][1] += normal.y*normal.y;
                    A[1][2] += normal.y*normal.z;
                    A[2][2] += normal.z*normal.z;
                    IndexArray[end_ptr] = m;
                    DistArray[end_ptr] = dist+1;
                    end_ptr ++;
                }
                first_ngr = first_ngr->next;
            }
        }
    }
    *max_ang = max_angle;

    A[1][0] = A[0][1];
    A[2][0] = A[0][2];
    A[2][1] = A[1][2];

    c0 = A[0][0]*A[1][1]*A[2][2]+2*A[0][1]*A[0][2]*A[1][2]-A[0][0]*A[1][2]*A[1][2]
            -A[1][1]*A[0][2]*A[0][2]-A[2][2]*A[0][1]*A[0][1];
    c1 = A[0][0]*A[1][1]-A[0][1]*A[0][1]+A[0][0]*A[2][2]-
            A[0][2]*A[0][2]+A[1][1]*A[2][2]-A[1][2]*A[1][2];
    c2 = A[0][0]+A[1][1]+A[2][2];

    a = (3.0*c1-c2*c2)/3.0;
    b = (-2.0*c2*c2*c2+9.0*c1*c2-27.0*c0)/27.0;
    Q = b*b/4.0+a*a*a/27.0;

    theta = std::atan2(sqrt(-Q), -0.5*b);
    p = std::sqrt(0.25*b*b-Q);

    x1 = c2/3.0+2.0*std::pow(p, 1.0/3.0)*std::cos(theta/3.0);
    x2 = c2/3.0-std::pow(p, 1.0/3.0)*(std::cos(theta/3.0)+std::sqrt(3.0)*std::sin(theta/3.0));
    x3 = c2/3.0-std::pow(p, 1.0/3.0)*(std::cos(theta/3.0)-std::sqrt(3.0)*std::sin(theta/3.0));

    // If we have a perfect flat area inline of one of the x, y, z axis
    if (std::isnan(x1) || std::isnan(x2) || std::isnan(x3))
    {

        //printf("Got a NAN: %d\n", index0);
        eigenValue->x = c2;
        eigenValue->y = 0;
        eigenValue->z = 0;

        tmp.x1 = 1;
        tmp.y1 = 0;
        tmp.z1 = 0;

        tmp.x2 = 0;
        tmp.y2 = 1;
        tmp.z2 = 0;

        tmp.x3 = 0;
        tmp.y3 = 0;
        tmp.z3 = 1;

        return tmp;
    }

    tx = std::max(x1, std::max(x2, x3));
    if (tx == x1)
    {
        if (x2 >= x3)
        {
            ty = x2;
            tz = x3;
        }
        else
        {
            ty = x3;
            tz = x2;
        }
    }
    else if (tx == x2)
    {
        if (x1 >= x3)
        {
            ty = x1;
            tz = x3;
        }
        else {
            ty = x3;
            tz = x1;
        }
    }
    else if (tx == x3)
    {
        if (x1 >= x2)
        {
            ty = x1;
            tz = x2;
        }
        else
        {
            ty = x2;
            tz = x1;
        }
    }
    x1 = tx;
    x2 = ty;
    x3 = tz;
    eigenValue->x = tx;
    eigenValue->y = ty;
    eigenValue->z = tz;

    if (x1 > 99999 || x1 < -99999 ||
            x2 > 99999 || x2 < -99999 ||
            x3 > 99999 || x3 < -99999)
    {
        printf("dsadsadsad: %f %f %f\n", x1, x2, x3);
        exit(0);
    }


    A[0][0] -= x1;
    A[1][1] -= x1;
    A[2][2] -= x1;
    B[0] = A[1][1]*A[2][2]-A[1][2]*A[1][2];
    B[1] = A[0][2]*A[1][2]-A[0][1]*A[2][2];
    B[2] = A[0][0]*A[2][2]-A[0][2]*A[0][2];
    B[3] = A[0][1]*A[1][2]-A[0][2]*A[1][1];
    B[4] = A[0][1]*A[0][2]-A[1][2]*A[0][0];
    B[5] = A[0][0]*A[1][1]-A[0][1]*A[0][1];
    c0 = B[0]*B[0]+B[1]*B[1]+B[3]*B[3];
    c1 = B[1]*B[1]+B[2]*B[2]+B[4]*B[4];
    c2 = B[3]*B[3]+B[4]*B[4]+B[5]*B[5];
    if (c0 >= c1 && c0 >= c2)
    {
        tx = B[0];
        ty = B[1];
        tz = B[3];
    }
    else if (c1 >= c0 && c1 >= c2)
    {
        tx = B[1];
        ty = B[2];
        tz = B[4];
    }
    else if (c2 >= c0 && c2 >= c1)
    {
        tx = B[3];
        ty = B[4];
        tz = B[5];
    }
    p = sqrt(tx*tx+ty*ty+tz*tz);
    if (p > 0)
    {
        tx /= p;
        ty /= p;
        tz /= p;
    }
    tmp.x1 = tx;
    tmp.y1 = ty;
    tmp.z1 = tz;
    A[0][0] += x1;
    A[1][1] += x1;
    A[2][2] += x1;


    A[0][0] -= x2;
    A[1][1] -= x2;
    A[2][2] -= x2;
    B[0] = A[1][1]*A[2][2]-A[1][2]*A[1][2];
    B[1] = A[0][2]*A[1][2]-A[0][1]*A[2][2];
    B[2] = A[0][0]*A[2][2]-A[0][2]*A[0][2];
    B[3] = A[0][1]*A[1][2]-A[0][2]*A[1][1];
    B[4] = A[0][1]*A[0][2]-A[1][2]*A[0][0];
    B[5] = A[0][0]*A[1][1]-A[0][1]*A[0][1];
    c0 = B[0]*B[0]+B[1]*B[1]+B[3]*B[3];
    c1 = B[1]*B[1]+B[2]*B[2]+B[4]*B[4];
    c2 = B[3]*B[3]+B[4]*B[4]+B[5]*B[5];
    if (c0 >= c1 && c0 >= c2)
    {
        tx = B[0];
        ty = B[1];
        tz = B[3];
    }
    else if (c1 >= c0 && c1 >= c2)
    {
        tx = B[1];
        ty = B[2];
        tz = B[4];
    }
    else if (c2 >= c0 && c2 >= c1)
    {
        tx = B[3];
        ty = B[4];
        tz = B[5];
    }
    p = sqrt(tx*tx+ty*ty+tz*tz);
    if (p > 0)
    {
        tx /= p;
        ty /= p;
        tz /= p;
    }
    tmp.x2 = tx;
    tmp.y2 = ty;
    tmp.z2 = tz;

    tmp.x3 = tmp.y1*tz-tmp.z1*ty;
    tmp.y3 = tmp.z1*tx-tmp.x1*tz;
    tmp.z3 = tmp.x1*ty-tmp.y1*tx;

    return tmp;
}

float computeDotProduct(SurfaceMesh *surfaceMesh,
                        const size_t& a, const size_t& b, const size_t& c)
{
    float cx, cy, cz;
    float bx, by, bz;
    float length;


    bx = surfaceMesh->vertex[b].x-surfaceMesh->vertex[a].x;
    by = surfaceMesh->vertex[b].y-surfaceMesh->vertex[a].y;
    bz = surfaceMesh->vertex[b].z-surfaceMesh->vertex[a].z;
    length = std::sqrt(bx*bx+by*by+bz*bz);
    if (length > 0)
    {
        bx /= length;
        by /= length;
        bz /= length;
    }

    cx = surfaceMesh->vertex[c].x-surfaceMesh->vertex[a].x;
    cy = surfaceMesh->vertex[c].y-surfaceMesh->vertex[a].y;
    cz = surfaceMesh->vertex[c].z-surfaceMesh->vertex[a].z;
    length = std::sqrt(cx*cx+cy*cy+cz*cz);
    if (length > 0)
    {
        cx /= length;
        cy /= length;
        cz /= length;
    }

    length = bx*cx+by*cy+bz*cz;
    return length;
}

Normal computeCrossProduct(SurfaceMesh *surfaceMesh,
                           const size_t& a, const size_t& b, const size_t& c)
{
    float gx, gy, gz;
    float cx, cy, cz;
    float bx, by, bz;
    float length;
    Normal value;


    bx = surfaceMesh->vertex[b].x-surfaceMesh->vertex[a].x;
    by = surfaceMesh->vertex[b].y-surfaceMesh->vertex[a].y;
    bz = surfaceMesh->vertex[b].z-surfaceMesh->vertex[a].z;
    length = sqrt(bx*bx+by*by+bz*bz);
    if (length > 0)
    {
        bx /= length;
        by /= length;
        bz /= length;
    }
    cx = surfaceMesh->vertex[c].x-surfaceMesh->vertex[a].x;
    cy = surfaceMesh->vertex[c].y-surfaceMesh->vertex[a].y;
    cz = surfaceMesh->vertex[c].z-surfaceMesh->vertex[a].z;
    length = sqrt(cx*cx+cy*cy+cz*cz);
    if (length > 0)
    {
        cx /= length;
        cy /= length;
        cz /= length;
    }
    gx = cy*bz-cz*by;
    gy = cz*bx-cx*bz;
    gz = cx*by-cy*bx;
    length = sqrt(gx*gx+gy*gy+gz*gz);
    if (length > 0)
    {
        gx /= length;
        gy /= length;
        gz /= length;
    }

    value.x = gx;
    value.y = gy;
    value.z = gz;

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
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    float min_angle1, min_angle2, angle;
    Normal normal_a, normal_b;

    /* smaller angle criterion */
    min_angle1 = -99999;
    angle = computeDotProduct(surfaceMesh, a, b, c);
    if (angle > min_angle1)
        min_angle1 = angle;
    angle = computeDotProduct(surfaceMesh, a, b, d);
    if (angle > min_angle1)
        min_angle1 = angle;
    angle = computeDotProduct(surfaceMesh, b, a, c);
    if (angle > min_angle1)
        min_angle1 = angle;
    angle = computeDotProduct(surfaceMesh, b, a, d);
    if (angle > min_angle1)
        min_angle1 = angle;

    min_angle2 = -99999;
    angle = computeDotProduct(surfaceMesh, c, a, d);
    if (angle > min_angle2)
        min_angle2 = angle;
    angle = computeDotProduct(surfaceMesh, c, b, d);
    if (angle > min_angle2)
        min_angle2 = angle;
    angle = computeDotProduct(surfaceMesh, d, a, c);
    if (angle > min_angle2)
        min_angle2 = angle;
    angle = computeDotProduct(surfaceMesh, d, b, c);
    if (angle > min_angle2)
        min_angle2 = angle;

    // Check which of the triangle combination has the smallest angle
    // min_angle1 is the minimal angle of the flipped configuration
    // min_angle2 is the minimal angle of the present configuration
    if (min_angle1 > min_angle2)
    {
        // Check if the angle between the normals of the two triangles are
        // too small for a flip action, for example if we are on a ridge
        normal_a = computeCrossProduct(surfaceMesh, a, c, b);
        normal_b = computeCrossProduct(surfaceMesh, a, b, d);

        // If we want to preserve the ridges the angle between the surface
        // normals must be  smaller than cos(60deg) to flip the edges
        if (not preserveRidges ||
                normal_a.x*normal_b.x+normal_a.y*normal_b.y+normal_a.z*normal_b.z>0.866)
            return(1);
    }

    return 0;
}

void getMinMaxAngles(SurfaceMesh *surfaceMesh, float *minangle,
                     float *maxangle, size_t *num_small, size_t *num_large,
                     const float& max_min_angle, const float& min_max_angle)
{
    int n, num1, num2;
    int a, b, c;
    float min_angle, max_angle;
    float angle;


    min_angle = 99999.0;
    max_angle = -99999.0;
    num1 = 0;
    num2 = 0;
    for (n = 0; n < surfaceMesh->numberFaces; n++)
    {
        a = surfaceMesh->face[n].v1;
        b = surfaceMesh->face[n].v2;
        c = surfaceMesh->face[n].v3;

        angle = getAngleBetweenVertices(surfaceMesh, a, b, c);
        if (angle != -999)
        {
            if (angle < min_angle)
                min_angle = angle;
            if (angle > max_angle)
                max_angle = angle;
            if (angle < max_min_angle)
                num1++;
            if (angle > min_max_angle)
                num2++;
        }
        angle = getAngleBetweenVertices(surfaceMesh, b, a, c);
        if (angle != -999)
        {
            if (angle < min_angle)
                min_angle = angle;
            if (angle > max_angle)
                max_angle = angle;
            if (angle < max_min_angle)
                num1++;
            if (angle > min_max_angle)
                num2++;
        }
        angle = getAngleBetweenVertices(surfaceMesh, c, a, b);
        if (angle != -999)
        {
            if (angle < min_angle)
                min_angle = angle;
            if (angle > max_angle)
                max_angle = angle;
            if (angle < max_min_angle)
                num1++;
            if (angle > min_max_angle)
                num2++;
        }
    }

    *minangle = min_angle;
    *maxangle = max_angle;
    *num_small = num1;
    *num_large = num2;
}


void edgeFlipping(SurfaceMesh *surfaceMesh, const size_t& n, const bool& preserveRidges)
{
    int a, b, c;
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr, *second_ngr;
    NPNT3 *tmp_ngr1, *tmp_ngr2, *tmp_ngr;
    char flip_flag, flip_check;
    int f1, f2, number;
    float ax, ay, az;

    first_ngr = neighbor_list[n];
    while (first_ngr != NULL)
    {

        number = 0;
        tmp_ngr = neighbor_list[n];
        while (tmp_ngr != NULL)
        {
            number++;
            tmp_ngr = tmp_ngr->next;
        }
        if (number <= 3)
        {
            if (number > 0)
            {
                ax = 0;
                ay = 0;
                az = 0;
                tmp_ngr = neighbor_list[n];
                while (tmp_ngr != NULL)
                {
                    a = tmp_ngr->a;
                    ax += surfaceMesh->vertex[a].x;
                    ay += surfaceMesh->vertex[a].y;
                    az += surfaceMesh->vertex[a].z;
                    tmp_ngr = tmp_ngr->next;
                }

                surfaceMesh->vertex[n].x = ax/(float)number;
                surfaceMesh->vertex[n].y = ay/(float)number;
                surfaceMesh->vertex[n].z = az/(float)number;
            }
            return;
        }

        a = first_ngr->a;
        b = first_ngr->b;
        second_ngr = first_ngr->next;
        if (second_ngr == NULL)
            second_ngr = neighbor_list[n];
        c = second_ngr->b;

        flip_flag = 1;
        number = 0;
        tmp_ngr = neighbor_list[b];
        while (tmp_ngr != NULL)
        {
            number++;
            tmp_ngr = tmp_ngr->next;
        }
        if (number <= 3)
            flip_flag = 0;

        tmp_ngr = neighbor_list[a];
        while (tmp_ngr != NULL)
        {
            if (tmp_ngr->a == c)
                flip_flag = 0;
            tmp_ngr = tmp_ngr->next;
        }
        tmp_ngr = neighbor_list[c];
        while (tmp_ngr != NULL)
        {
            if (tmp_ngr->a == a)
                flip_flag = 0;
            tmp_ngr = tmp_ngr->next;
        }

        if (flip_flag) {

            flip_check = checkFlipAction(surfaceMesh, n, b, a, c, preserveRidges);

            if (flip_check) {
                f1 = first_ngr->c;
                f2 = second_ngr->c;

                /* Update face info */
                surfaceMesh->face[f1].v1 = n;
                surfaceMesh->face[f1].v2 = a;
                surfaceMesh->face[f1].v3 = c;
                surfaceMesh->face[f2].v1 = b;
                surfaceMesh->face[f2].v2 = c; // Switch a and c here to make the face
                surfaceMesh->face[f2].v3 = a; // normal point outward

                /* Delete the entries in neighbor lists */
                first_ngr->b = c;
                if (first_ngr->next == NULL)
                    neighbor_list[n] = neighbor_list[n]->next;
                else
                    first_ngr->next = second_ngr->next;
                tmp_ngr1 = second_ngr;

                tmp_ngr = neighbor_list[b];
                while (tmp_ngr != NULL)
                {
                    if (tmp_ngr->b == n)
                        break;
                    tmp_ngr = tmp_ngr->next;
                }
                if (tmp_ngr == NULL)
                    printf("my god ... %d\n", n);
                if (tmp_ngr->a == c)
                {
                    tmp_ngr->b = a;
                    tmp_ngr->c = f2;
                    if (tmp_ngr->next == NULL)
                    {
                        second_ngr = neighbor_list[b];
                        neighbor_list[b] = second_ngr->next;
                    }
                    else
                    {
                        second_ngr = tmp_ngr->next;
                        tmp_ngr->next = second_ngr->next;
                    }
                    tmp_ngr2 = second_ngr;
                }
                else
                {
                    printf("delete error!!! %d : %d %d %d\n", n, a, b, c);
                    printf("(%f, %f, %f)\n",
                           surfaceMesh->vertex[n].x,
                           surfaceMesh->vertex[n].y,
                           surfaceMesh->vertex[n].z);
                }

                /* Add the entries in neighbor lists */
                tmp_ngr = neighbor_list[a];
                while (tmp_ngr != NULL)
                {
                    if ((tmp_ngr->a == n && tmp_ngr->b == b) ||
                            (tmp_ngr->a == b && tmp_ngr->b == n))
                        break;
                    tmp_ngr = tmp_ngr->next;
                }

                // Assume neigbors are stored counter clockwise
                if (tmp_ngr->a == b && tmp_ngr->b == n)
                {
                    tmp_ngr->b = c;
                    tmp_ngr->c = f2;
                    tmp_ngr1->a = c;
                    tmp_ngr1->b = n;
                    tmp_ngr1->c = f1;
                    tmp_ngr1->next = tmp_ngr->next;
                    tmp_ngr->next = tmp_ngr1;
                }
                else
                    printf("add error 111\n");

                tmp_ngr = neighbor_list[c];
                while (tmp_ngr != NULL)
                {
                    if ((tmp_ngr->a == n && tmp_ngr->b == b) ||
                            (tmp_ngr->a == b && tmp_ngr->b == n))
                        break;
                    tmp_ngr = tmp_ngr->next;
                }

                // Assume neigbors are stored counter clockwise
                if (tmp_ngr->a == n && tmp_ngr->b == b)
                {
                    tmp_ngr->b = a;
                    tmp_ngr->c = f1;
                    tmp_ngr2->a = a;
                    tmp_ngr2->b = b;
                    tmp_ngr2->c = f2;
                    tmp_ngr2->next = tmp_ngr->next;
                    tmp_ngr->next = tmp_ngr2;
                }
                else
                    printf("add error 222\n");
            }
        }

        first_ngr = first_ngr->next;
    }
}

void moveVerticesAlongSurface(SurfaceMesh *surfaceMesh, const size_t& n)
{
    int a, b, c;
    float x, y, z;
    Vertex pos_vect;
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr, *second_ngr;
    float weight, angle;
    float nx, ny, nz;
    EigenVector eigen_vect;
    EigenValue eigen_value;
    float w1, w2, w3;
    float max_angle;


    x = surfaceMesh->vertex[n].x;
    y = surfaceMesh->vertex[n].y;
    z = surfaceMesh->vertex[n].z;

    nx = 0;
    ny = 0;
    nz = 0;

    weight = 0;
    first_ngr = neighbor_list[n];
    while (first_ngr != NULL)
    {
        a = first_ngr->a;
        b = first_ngr->b;
        second_ngr = first_ngr->next;
        if (second_ngr == NULL)
            second_ngr = neighbor_list[n];
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

        eigen_vect = getEigenVector(surfaceMesh, n, &eigen_value, &max_angle);
        if ((eigen_vect.x1==0 && eigen_vect.y1==0 && eigen_vect.z1==0) ||
                (eigen_vect.x2==0 && eigen_vect.y2==0 && eigen_vect.z2==0) ||
                (eigen_vect.x3==0 && eigen_vect.y3==0 && eigen_vect.z3==0))
        {
            //printf("old point (%0.2f, %0.2f, %0.2f), new point (%0.2f, %0.2f, %0.2f)\n",
            //	     surfaceMesh->vertex[n].x, surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z,
            //	     nx, ny, nz);
            surfaceMesh->vertex[n].x = nx;
            surfaceMesh->vertex[n].y = ny;
            surfaceMesh->vertex[n].z = nz;

        }
        else
        {
            nx -= x;
            ny -= y;
            nz -= z;
            w1 = (nx*eigen_vect.x1+ny*eigen_vect.y1+nz*eigen_vect.z1)/(1.0+eigen_value.x);
            w2 = (nx*eigen_vect.x2+ny*eigen_vect.y2+nz*eigen_vect.z2)/(1.0+eigen_value.y);
            w3 = (nx*eigen_vect.x3+ny*eigen_vect.y3+nz*eigen_vect.z3)/(1.0+eigen_value.z);
            nx = w1*eigen_vect.x1+w2*eigen_vect.x2+w3*eigen_vect.x3 + x;
            ny = w1*eigen_vect.y1+w2*eigen_vect.y2+w3*eigen_vect.y3 + y;
            nz = w1*eigen_vect.z1+w2*eigen_vect.z2+w3*eigen_vect.z3 + z;

            //printf("old point (%0.2f, %0.2f, %0.2f), new point (%0.2f, %0.2f, %0.2f)\n",
            //	     surfaceMesh->vertex[n].x, surfaceMesh->vertex[n].y, surfaceMesh->vertex[n].z,
            //	     nx, ny, nz);

            surfaceMesh->vertex[n].x = nx;
            surfaceMesh->vertex[n].y = ny;
            surfaceMesh->vertex[n].z = nz;
        }
    }
}

void smoothNormal(SurfaceMesh *surfaceMesh, const size_t& n)
{
    int a, b, c, d, e;
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr, *second_ngr, *third_ngr;
    NPNT3 *tmp_ngr;
    float bx, by, bz;
    float cx, cy, cz;
    float dx, dy, dz;
    float fx, fy, fz;
    float gx, gy, gz;
    float pos_x, pos_y, pos_z;
    int number, num;
    float theta, phi, alpha;
    float length;
    Normal normal, sv;


    number = 0;
    pos_x = 0;
    pos_y = 0;
    pos_z = 0;
    first_ngr = neighbor_list[n];
    while (first_ngr != NULL)
    {
        a = first_ngr->a;
        b = first_ngr->b;
        second_ngr = first_ngr->next;
        if (second_ngr == NULL)
            second_ngr = neighbor_list[n];
        c = second_ngr->b;
        third_ngr = second_ngr->next;
        if (third_ngr == NULL)
            third_ngr = neighbor_list[n];
        d = third_ngr->b;

        tmp_ngr = neighbor_list[b];

        // If a vertex is neigbor with a non selected vertex continue
        if (!surfaceMesh->vertex[b].selected)
            return;

        while (tmp_ngr != NULL)
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

            number++;
        }

        first_ngr = first_ngr->next;
    }

    if (number > 0 && !std::isnan(pos_x) && !std::isnan(pos_y) && !std::isnan(pos_z))
    {
        surfaceMesh->vertex[n].x = pos_x/(float)number;
        surfaceMesh->vertex[n].y = pos_y/(float)number;
        surfaceMesh->vertex[n].z = pos_z/(float)number;
    }
}

void subdividePolygon(SurfaceMesh *surfaceMesh,
                        NPNT3 *start_ngr, int *face_available_list,
                        int *face_available_index, int face_marker)
{
    NPNT3 **neighbor_list = surfaceMesh->neighborList;
    NPNT3 *first_ngr, *second_ngr;
    NPNT3 *tmp_ngr, *first_copy_ngr, *second_copy_ngr;
    int min_num, degree;
    int face_index, number;
    int a, b, c;


    number = 1;
    tmp_ngr = start_ngr;
    while (tmp_ngr->next != start_ngr)
    {
        number++;
        tmp_ngr = tmp_ngr->next;
    }

    if (number < 3)
    {
        printf("error: number of nodes less than 3 \n");
        exit(0);
    }

    if (number == 3)
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

        first_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        first_ngr->a = b;
        first_ngr->b = c;
        first_ngr->c = face_index;
        first_ngr->next = neighbor_list[a];
        neighbor_list[a] = first_ngr;

        first_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        first_ngr->a = c;
        first_ngr->b = a;
        first_ngr->c = face_index;
        first_ngr->next = neighbor_list[b];
        neighbor_list[b] = first_ngr;

        first_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        first_ngr->a = a;
        first_ngr->b = b;
        first_ngr->c = face_index;
        first_ngr->next = neighbor_list[c];
        neighbor_list[c] = first_ngr;

    }
    else
    {
        tmp_ngr = start_ngr;
        min_num = tmp_ngr->b;
        first_ngr = tmp_ngr;
        tmp_ngr = tmp_ngr->next;
        while (tmp_ngr != start_ngr)
        {
            degree = tmp_ngr->b;
            if (degree < min_num)
            {
                min_num = degree;
                first_ngr = tmp_ngr;
            }
            tmp_ngr = tmp_ngr->next;
        }

        min_num = 99999;
        tmp_ngr = start_ngr;
        if (tmp_ngr != first_ngr &&
                tmp_ngr != first_ngr->next &&
                tmp_ngr->next != first_ngr)
        {
            min_num = tmp_ngr->b;
            second_ngr = tmp_ngr;
        }

        tmp_ngr = tmp_ngr->next;
        while (tmp_ngr != start_ngr)
        {
            degree = tmp_ngr->b;
            if (tmp_ngr != first_ngr &&
                    tmp_ngr != first_ngr->next &&
                    tmp_ngr->next != first_ngr &&
                    degree < min_num)
            {
                min_num = degree;
                second_ngr = tmp_ngr;
            }
            tmp_ngr = tmp_ngr->next;
        }

        first_ngr->b += 1;
        second_ngr->b += 1;
        first_copy_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        first_copy_ngr->a = first_ngr->a;
        first_copy_ngr->b = first_ngr->b;
        second_copy_ngr = (NPNT3 *)malloc(sizeof(NPNT3));
        second_copy_ngr->a = second_ngr->a;
        second_copy_ngr->b = second_ngr->b;
        tmp_ngr = first_ngr;
        while (tmp_ngr->next != first_ngr)
            tmp_ngr = tmp_ngr->next;
        tmp_ngr->next = first_copy_ngr;
        first_copy_ngr->next = second_copy_ngr;
        second_copy_ngr->next = second_ngr->next;
        second_ngr->next = first_ngr;

        subdividePolygon(surfaceMesh, first_ngr, face_available_list,
                           face_available_index, face_marker);
        subdividePolygon(surfaceMesh, first_copy_ngr, face_available_list,
                           face_available_index, face_marker);
    }

    return;
}

