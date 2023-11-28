#pragma once

#include <vector>

/**
 * @brief The Neighbour class
 */
class Neighbour
{
public:
    /**
     * @brief v1
     */
    size_t n1;

    /**
     * @brief v2
     */
    size_t n2;

    /**
     * @brief v3
     */
    size_t n3;

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
 * @brief NeighbourPtr
 */
typedef Neighbour* NeighbourPtr;

/**
 * @brief Neighbours
 */
typedef std::vector< Neighbour > Neighbours;
