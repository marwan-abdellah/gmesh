#pragma once

#include "SurfaceMesh.hpp"
#include "Normal.hpp"
#include "EigenValue.hh"
#include "EigenVector.hpp"
#include "BMesh.hpp"

/**
 * @brief createSurfaceMesh
 * @param numberVertices
 * @param numberFaces
 * @return
 */
SurfaceMesh* createSurfaceMesh(const size_t& numberVertices, const size_t& numberFaces);

/**
 * @brief createSurfaceMeshFromBlenderData
 * @param vertices
 * @param triangles
 * @return
 */
SurfaceMesh* createSurfaceMeshFromBlenderData(const BVertices& vertices,
                                              const BTriangles& triangles);

/**
 * @brief releaseSurfaceMeshData
 * @param surfaceMesh
 */
void releaseSurfaceMeshData(SurfaceMesh* surfaceMesh);

/**
 * @brief destructSurfaceMesh
 * @param surfaceMesh
 */
void destructSurfaceMesh(SurfaceMesh* surfaceMesh);

/**
 * @brief createNeighborlist
 * @param surfaceMesh
 */
void createNeighborlist(SurfaceMesh* surfaceMesh);

/**
 * @brief destroyNeighborlist
 * @param surfaceMesh
 */
void destroyNeighborlist(SurfaceMesh* surfaceMesh);

/**
 * @brief removeUnconnectedVertices
 * @param surfaceMesh
 */
void removeUnconnectedVertices(SurfaceMesh* surfaceMesh);

/**
 * @brief deleteVertices
 * @param surfaceMesh
 */
void deleteVertices(SurfaceMesh* surfaceMesh);

/**
 * @brief deleteFaces
 * @param surfaceMesh
 */
void deleteFaces(SurfaceMesh* surfaceMesh);

/**
 * @brief translateMesh
 * @param surfaceMesh
 * @param dx
 * @param dy
 * @param dz
 */
void translateMesh(SurfaceMesh* surfaceMesh, const float& dx, const float& dy, const float& dz);

/**
 * @brief scaleMesh
 * @param surfaceMesh
 * @param xScale
 * @param yScale
 * @param zScale
 */
void scaleMesh(SurfaceMesh* surfaceMesh, const float& xScale, const float& yScale, const float& zScale);

/**
 * @brief scaleMeshUniformly
 * @param surfaceMesh
 * @param scaleFactor
 */
void scaleMeshUniformly(SurfaceMesh* surfaceMesh, const float& scaleFactor);

/**
 * @brief getEigenVector
 * @param surfaceMesh
 * @param vertexIndex
 * @param eigenValue
 * @param computedMaxAngle
 * @param verbose
 * @return
 */
EigenVector getEigenVector(SurfaceMesh *surfaceMesh,
                           const size_t& vertexIndex, EigenValue* eigenValue,
                           float *computedMaxAngle,
                           const bool &verbose = false);


/**
 * @brief getVertexPositionAlongSurface
 * @param x
 * @param y
 * @param z
 * @param a
 * @param b
 * @param c
 * @param surfaceMesh
 * @return
 */
Vertex getVertexPositionAlongSurface(const float& x, const float& y, const float& z,
                                     const size_t& a, const size_t& b, const size_t& c,
                                     SurfaceMesh *surfaceMesh);

/**
 * @brief getAngleBetweenVertices
 * @param surfaceMesh
 * @param a
 * @param b
 * @param c
 * @return
 */
float getAngleBetweenVertices(SurfaceMesh *surfaceMesh,
                              const size_t& a, const size_t& b, const size_t& c);


/**
 * @brief computeDotProduct
 * @param surfaceMesh
 * @param a
 * @param b
 * @param c
 * @return
 */
float computeDotProduct(SurfaceMesh *surfaceMesh,
                        const size_t& a, const size_t& b, const size_t& c);

/**
 * @brief computeCrossProduct
 * @param surfaceMesh
 * @param a
 * @param b
 * @param c
 * @return
 */
Normal computeCrossProduct(SurfaceMesh *surfaceMesh,
                           const size_t& a, const size_t& b, const size_t& c);

/**
 * @brief rotate
 * @param sx
 * @param sy
 * @param sz
 * @param theta
 * @param phi
 * @param angle
 * @return
 */
Normal rotate(const float& sx, const float& sy, const float& sz,
              const float& theta, const float& phi, const float& angle);

/**
 * @brief checkFlipAction
 * @param surfaceMesh
 * @param a
 * @param b
 * @param c
 * @param d
 * @param preserveRidges
 * @return
 */
char checkFlipAction(SurfaceMesh *surfaceMesh,
                     const size_t& a, const size_t& b, const size_t& c, const size_t& d,
                     const bool& preserveRidges);

/**
 * @brief getMinMaxAngles
 * @param surfaceMesh
 * @param computedMinangle
 * @param computedMaxangle
 * @param computedNumberSmallerAngles
 * @param computedNumberLargerAngles
 * @param maxMinAngle
 * @param minMaxAngle
 */
void getMinMaxAngles(SurfaceMesh *surfaceMesh,
                     float *computedMinangle, float *computedMaxangle,
                     size_t *computedNumberSmallerAngles, size_t *computedNumberLargerAngles,
                     const float &maxMinAngle, const float &minMaxAngle);

/**
 * @brief edgeFlipping
 * @param surfaceMesh
 * @param n
 * @param preserveRidges
 */
void edgeFlipping(SurfaceMesh *surfaceMesh,
                  const size_t& n, const bool& preserveRidges);

/**
 * @brief moveVerticesAlongSurface
 * @param surfaceMesh
 * @param n
 */
void moveVerticesAlongSurface(SurfaceMesh *surfaceMesh, const size_t& n);

/**
 * @brief subdividePolygon
 * @param surfaceMesh
 * @param startNeighbour
 * @param faceAvailableList
 * @param faceAvailableIndex
 * @param faceMarker
 */
void subdividePolygon(SurfaceMesh *surfaceMesh,
                      NPNT3 *startNeighbour, int *faceAvailableList,
                      int *faceAvailableIndex, int faceMarker);

/**
 * @brief smoothNormal
 * @param surfaceMesh
 * @param n
 */
void smoothNormal(SurfaceMesh *surfaceMesh, const size_t& n);

/**
 * @brief refine
 * @param surfaceMesh
 */
void refine(SurfaceMesh* surfaceMesh);

/**
 * @brief smooth
 * @param surfaceMesh
 * @param maxMinAngle
 * @param minaMaxAngle
 * @param maximumIterations
 * @param preserveRidges
 * @param verbose
 * @return
 */
bool smooth(SurfaceMesh *surfaceMesh,
            const size_t& maxMinAngle,
            const size_t& minaMaxAngle,
            const size_t& maximumIterations,
            const bool& preserveRidges,
            const bool &verbose = false);

/**
 * @brief smoothNormals
 * @param surfaceMesh
 * @param minAngle
 * @param maxAngle
 * @param numberSmallerAngles
 * @param numberGreaterAngles
 * @param maxMinAngle
 * @param minMaxAngle
 * @param verbose
 */
void smoothNormals(SurfaceMesh *surfaceMesh,
                   float& minAngle, float& maxAngle,
                   size_t &numberSmallerAngles, size_t &numberGreaterAngles,
                   const float& maxMinAngle = 15,
                   const float& minMaxAngle = 150,
                   const bool& verbose = false);

/**
 * @brief coarse
 * @param surfaceMesh
 * @param coarsenessRate
 * @param flatnessRate
 * @param densenessWeight
 * @param maxNormalAngle
 * @param verbose
 * @return
 */
char coarse(SurfaceMesh* surfaceMesh,
            float coarsenessRate, float flatnessRate, float densenessWeight,
            float maxNormalAngle,
            const bool& verbose = false);
