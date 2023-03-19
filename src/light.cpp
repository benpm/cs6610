#include <light.hpp>
#include <camera.hpp>

Light::Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type)
    : pos(pos), color(color), intensity(intensity), type(type) {}

uLight Light::toStruct(const Camera& camera) const {
    return {
        .position = this->pos,
        .color = this->color,
        .direction = this->dir,
        .intensity = this->intensity,
        .range = this->range,
        .spotAngle = cosf(this->spotAngle / 2.0f),
        .type = this->type
    };
}
