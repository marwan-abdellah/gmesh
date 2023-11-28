#pragma once

#include "Vertex.hpp"
#include "Triangle.hpp"
#include "Neighbour.hpp"
#include "NeighborPoint3.hpp"

/**
 * @brief The SurfaceMesh class
 */
class SurfaceMesh
{
public:

    /**
     * @brief numberVertices
     * Number of vertices
     */
    size_t numberVertices;

    /**
     * @brief numberFaces
     * Number of triangles
     */
    size_t numberFaces;

    /**
     * @brief averageLength
     * Average edge length
     */
    float averageLength;

    /**
     * @brief pMin
     * Minimal coordinate of nodes
     */
    Vertex pMin[3];

    /**
     * @brief pMax
     * Maximal coordinate of nodes
     */
    Vertex pMax[3];

    /**
     * @brief vertex
     * Pointer to the vertices
     */
    VertexPtr vertex;

    /**
     * @brief face
     * Pointer to the triangles
     */
    TrianglePtr face;

    /**
     * @brief neighbor
     * Pointer to the neighbors (triangles)
     */
    NeighbourPtr neighbor;

    /**
     * @brief neighborList
     * Pointer to the neighbor list
     */
    NeighborPoint3** neighborList;

    /**
     * @brief closed
     * A flag to indicate if the surface mesh is closed or not
     */
    bool closed;

    /**
     * @brief marker
     * A doman marker, to be used when tetrahedralizing
     */
    int marker;

    /**
     * @brief volumeConstraint
     * Volume constraint of the tetrahedralized domain
     */
    float volumeConstraint;

    /**
     * @brief useVolumeConstraint
     * A flag that determines if the volume constraint is used or not
     */
    bool useVolumeConstraint;

    /**
     * @brief asHole
     * A flag that determines if the mesh is a hole or not
     */
    bool asHole;
};

/**
 * @brief SurfaceMeshPtr
 */
typedef SurfaceMesh* SurfaceMeshPtr;

/**
 * @brief SurfaceMeshes
 */
typedef std::vector< SurfaceMesh > SurfaceMeshes;

/**
 * @brief SurfaceMeshesPtrs
 */
typedef std::vector< SurfaceMeshPtr > SurfaceMeshesPtrs;

