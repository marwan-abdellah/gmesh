#pragma once

#include <vector>

/**
 * @brief The Triangle class
 */
class Triangle
{
public:
    /**
     * @brief v1
     */
    size_t v1;

    /**
     * @brief v2
     */
    size_t v2;

    /**
     * @brief v3
     */
    size_t v3;

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
 * @brief TrianglePtr
 */
typedef Triangle* TrianglePtr;

/**
 * @brief Triangles
 */
typedef std::vector< Triangle > Triangles;
