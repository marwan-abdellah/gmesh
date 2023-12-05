#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "virtual/Virtual.hh"
#include "Vertex.hpp"
#include "Triangle.hpp"
#include "SurfaceMesh.hpp"
#include "SurfaceMesh.hh"
#include "PySurfaceMesh.hh"
#include <omp.h>
#include <unistd.h>

namespace py = pybind11;

// Trampoline class for pybind to tell it how to deal with virtual functions.
class PyBase : public Base
{
public:
    using Base::Base;

protected:
    std::string virt() override
    {
        PYBIND11_OVERLOAD(std::string, Base, virt);
    }

    std::string pureVirtual() override
    {
        // We use PYBIND11_OVERLOAD_PURE_NAME instead of PYBIND11_OVERLOAD_PURE
        // since the method has a different name in Python.
        PYBIND11_OVERLOAD_PURE_NAME(
            std::string,    // C++ return type
            Base,           // Super class
            "pure_virtual", // Python method name
            pureVirtual     // C++ method name
        );
    }
};

// Helper class for pybind to expose protected methods publicly.
class PublishBase : public Base
{
public:
    using Base::virt;
    using Base::pureVirtual;
};

/**
 * @brief exposeClasses
 * Helper functiont to expose C++ things to Python.
 * @note In this function you would normally put this code directly in the PYBIND11_MODULE block,
 * but for larger libraries you will want to use functions in separate files to expose different
 * parts of your C++ library.
 * @param m
 */
void exposeClasses(py::module m)
{
    py::class_<Base, PyBase> base(m, "Base");
    base
        .def(py::init<>())
        // You normally does not use getter and setter functions, but use a property.
        .def("getA", &Base::getA)
        .def("setA", &Base::setA)
        .def_property("a", &Base::getA, &Base::setA)
        .def("virt", &PublishBase::virt)
        // Note that we use a different (PEP8-style) naming scheme in Python
        // to look more like a standard Python library.
        .def("pure_virtual", &PublishBase::pureVirtual)
        .def("call_pure_virtual", &Base::callPureVirtual)
        ;

    py::class_<Child>(m, "Child", base)
        .def(py::init<>())
        ;

    py::class_<SurfaceMesh>(m, "SurfaceMesh")
        .def(py::init<>());

    py::class_<Vertex>(m, "Vertex")
        .def(py::init<>())
        ;

    py::class_<Triangle>(m, "Triangle")
        .def(py::init<>())
        ;
}

void testOMP(int j)
{
    int* a = new int[j];
    int* b = new int[j];
    int* c = new int[j];

#pragma omp parallel for
    for (int i = 0; i < j; ++i)
    {
        a[i] = i * 3;
        b[i] = 2 + i;
    }

#pragma omp parallel for
    for (int i = 0; i < j; ++i)
    {
        c[i] = a[i] + b[i];
    }

    delete [] a;
    delete [] b;
    delete [] c;
}

void exposeFunctions(py::module m)
{

    // Creation of a SurfaceMesh from blender data, i.e. the input interface
    m.def("create_surface_mesh_from_blender_data", &createSurfaceMeshFromBlenderData,
          py::return_value_policy::copy);

    // Retrieval of the vertex data
    m.def("get_vertex_data", &getVertexData, py::return_value_policy::copy);

    // Retrieval of the face (triangles) data
    m.def("get_face_data", &getFaceData, py::return_value_policy::copy);

    // Just a testing function to scale the mesh uniformly
    m.def("scale_mesh_uniformly", &scaleMeshUniformly);

    // Refine the mesh
    m.def("refine", &refine, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    m.def("smooth", &smooth);

    m.def("smooth_normals", &smoothNormals);

    m.def("coarse", &coarse);

    m.def("coarse_dense", &coarseDense);

    m.def("coarse_flat", &coarseFlat);

    m.def("test_omp", &testOMP);
}

PYBIND11_MODULE(gmesh, m)
{
    m.doc() = "Python bindings for the GMesh (gmesh) optimization library";

    py::enum_<Base::Value>(m, "Value")
        .value("one", Base::one)
        .value("two", Base::two)
        .value("three", Base::three)
        ;

    exposeClasses(m);

    PYBIND11_NUMPY_DTYPE(Vertex, x, y, z);
    PYBIND11_NUMPY_DTYPE(Triangle, v1, v2, v3);

    exposeFunctions(m);
}
