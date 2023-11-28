#pragma once

#include "SurfaceMesh.hpp"
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
 * @brief refine
 * @param surfaceMesh
 */
void refine(SurfaceMesh* surfaceMesh);
