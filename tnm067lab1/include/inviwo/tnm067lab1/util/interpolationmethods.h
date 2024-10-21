#pragma once

#include <inviwo/tnm067lab1/tnm067lab1moduledefine.h>
#include <inviwo/core/util/glm.h>


namespace inviwo {

template <typename T>
struct float_type {
    using type = double;
};

template <>
struct float_type<float> {
    using type = float;
};
template <>
struct float_type<vec3> {
    using type = float;
};
template <>
struct float_type<vec2> {
    using type = float;
};
template <>
struct float_type<vec4> {
    using type = float;
};

namespace TNM067 {
namespace Interpolation {

#define ENABLE_LINEAR_UNITTEST 1
template <typename T, typename F = double>
T linear(const T& a, const T& b, F x) {
    if (x <= 0) return a;
    if (x >= 1) return b;

    return  a * (1.0 - x) + b * x;
}

// clang-format off
    /*
     2------3
     |      |
    y|  •   |
     |      |
     0------1
        x
    */
    // clang format on
#define ENABLE_BILINEAR_UNITTEST 1
template<typename T, typename F = double> 
T bilinear(const std::array<T, 4> &v, F x, F y) {

    //interpolate along x-axis
    T bottom = linear(v[0],v[1],x);     
    T top = linear(v[2],v[3],x);        

    // interpolate along y-xis
    T res = linear(bottom,top,y);
    return res;
}


    // clang-format off
    /* 
    a--•----b------c
    0  x    1      2
    */
// clang-format on
#define ENABLE_QUADRATIC_UNITTEST 1
template <typename T, typename F = double>
T quadratic(const T& a, const T& b, const T& c, F x) {
    // Perform quadratic interpolation between a, b, and c using x
    return a * (1 - x) * (1 - 2 * x)    // Influence of the first point (a)
           + b * 4 * x * (1 - x)    // Influence of the middle point (b)
           + c * x * (2 * x - 1);   // Influence of the third point (c)
}

// clang-format off
    /* 
    6-------7-------8
    |       |       |
    |       |       |
    |       |       |
    3-------4-------5
    |       |       |
   y|  •    |       |
    |       |       |
    0-------1-------2
    0  x    1       2
    */
// clang-format on
#define ENABLE_BIQUADRATIC_UNITTEST 1
template <typename T, typename F = double>
T biQuadratic(const std::array<T, 9>& v, F x, F y) { 
    
    T bottom_x = quadratic(v[0], v[1], v[2], x);
    T middle_x = quadratic(v[3], v[4], v[5], x);
    T top_x = quadratic(v[6], v[7], v[8], x);

    T res = quadratic(bottom_x, middle_x, top_x, y);

    return res;
}

// clang-format off
    /*
     2---------3
     |'-.      |
     |   -,    |
   y |  •  -,  |
     |       -,|
     0---------1
        x
    */
// clang-format on
#define ENABLE_BARYCENTRIC_UNITTEST 1
template <typename T, typename F = double>
T barycentric(const std::array<T, 4>& v, F x, F y) {

    F alpha, beta, gamma;
    
    // Values at the vertices
    auto fA = v[0];
    auto fB = v[1];
    auto fC = v[2];

    // if the point is in the lower triangle
    if (x + y < 1) {
        alpha = 1 - x - y;
        beta = x;
        gamma = y;

        fA = v[0];

    } else {
        // upper triangle
        alpha = x + y - 1;
        beta = 1 - y;
        gamma = 1 - x;

        fA = v[3];

    }

    return alpha * fA + beta * fB + gamma * fC;


}

}  // namespace Interpolation
}  // namespace TNM067
}  // namespace inviwo
