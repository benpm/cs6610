#include <light.hpp>
#include <camera.hpp>

Light::Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type)
    : pos(pos), color(color), intensity(intensity), type(type) {}

uLight Light::toStruct(const Camera& camera) const {
    return {
        .position = type == LightType::point ? camera.toView(pos) : -pos,
        .color = color,
        .intensity = intensity,
        .type = type
    };
}
