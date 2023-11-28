#pragma once

#include <vector>

// Forward declaration
class NeighborPoint3;

/**
 * @brief The NeighborPoint3 class
 */
class NeighborPoint3
{
public:
    /**
     * @brief a
     * First integer
     */
    int a;

    /**
     * @brief b
     * Second integer
     */
    int b;

    /**
     * @brief c
     * Third integer
     */
    int c;

    /**
     * @brief next
     * Pointer to the next triangle
     */
    NeighborPoint3* next;
};

/**
 * @brief NeighborPoint3Ptr
 */
typedef NeighborPoint3* NeighborPoint3Ptr;

