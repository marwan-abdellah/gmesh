#pragma once

#include <vector>

/**
 * @brief BVertex
 * The BVertex is an std::vector of only 3 components (x, y, z).
 * @note This structure is used to parse vertex data coming from Blender (BVertex = Blender Vertex)
 */
typedef std::vector< float > BVertex;

/**
 * @brief BVertices
 * The BVertices is an std::vector of N elements, where every element is composed of 3 components.
 */
typedef std::vector< BVertex > BVertices;

/**
 * @brief BTriangle
 * The BTriangle is an std::vector of only 3 components (v1, v2, v3).
 * @note This structure is used to parse face data coming from Blender (BTriangle = Blender Triangle)
 */
typedef std::vector< int > BTriangle;

/**
 * @brief BTriangles
 * The BTriangles is an std::vector of N elements, where every element is composed of 3 components.
 */
typedef std::vector< BTriangle > BTriangles;
