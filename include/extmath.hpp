#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/fmt/bundled/format.h>

using namespace Eigen;

// fmt overload for Matrix4f
template<> struct fmt::formatter<Matrix4f> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const Matrix4f& input, FormatContext& ctx) -> decltype(ctx.out()) {
        return format_to(ctx.out(),
            "[{}, {}, {}, {}]\n[{}, {}, {}, {}]\n[{}, {}, {}, {}]\n[{}, {}, {}, {}]",
            input(0, 0), input(0, 1), input(0, 2), input(0, 3),
            input(1, 0), input(1, 1), input(1, 2), input(1, 3),
            input(2, 0), input(2, 1), input(2, 2), input(2, 3),
            input(3, 0), input(3, 1), input(3, 2), input(3, 3)
            );
    }
};

Matrix4f eulerRot(const Vector3f& euler);

/**
 * @brief Creates a view matrix from the given eye, center, and up vectors
 * 
 * @param eye A vector representing the camera's position
 * @param center A vector representing the point the camera is looking at
 * @param up A vector representing the camera's up direction
 * @return Transformation matrix
 */
Matrix4f lookAt(const Vector3f& eye, const Vector3f& center, const Vector3f& up);
Matrix4f perspective(float fov, float aspect, float near, float far);
Matrix4f orthographic(const Vector2f& size, float near, float far);