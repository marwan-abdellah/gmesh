#pragma once

#include "SurfaceMesh.hpp"
#include "Vertex.hpp"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/complex.h>

/**
 * @brief getVertexData
 * @param surfaceMesh
 * @return
 */
pybind11::array_t< Vertex > getVertexData(SurfaceMesh* surfaceMesh);

/**
 * @brief getFaceData
 * @param surfaceMesh
 * @return
 */
pybind11::array_t< Triangle > getFaceData(SurfaceMesh* surfaceMesh);

