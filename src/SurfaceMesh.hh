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
