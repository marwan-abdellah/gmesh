#pragma once

#include <vector>

/**
 * @brief The Vertex class
 */
class Vertex
{
public:
    /**
     * @brief x
     */
    float x;

    /**
     * @brief y
     */
    float y;

    /**
     * @brief z
     */
    float z;

    /**
     * @brief marker
     */
    int marker;

    /**
     * @brief selected
     */
    bool selected;
};

/**
 * @brief VertexPtr
 */
typedef Vertex* VertexPtr;

/**
 * @brief Vertices
 */
typedef std::vector< Vertex > Vertices;
