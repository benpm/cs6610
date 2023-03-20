#include <light.hpp>
#include <camera.hpp>
#include <gfx.hpp>
#include <spdlog/spdlog.h>

Light::Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type)
    : pos(pos), color(color), intensity(intensity), type(type) {}

uLight Light::toStruct(const Camera& camera, uint32_t& nextShadowLayer) const {
    Matrix4f transform = identityTransform().matrix();
    if (this->type == LightType::spot) {
        transform = camera.getProj({shadowMapSize, shadowMapSize}) * camera.getView();
    }
    return {
        .position = this->pos,
        .color = this->color,
        .direction = this->dir,
        .intensity = this->intensity,
        .range = this->range,
        .spotAngle = cosf(this->spotAngle / 2.0f),
        .far = camera.far,
        .type = this->type,
        .shadowMapLayer = this->castsShadows ? (int)(nextShadowLayer++) : -1,
        .transform = transform
    };
}

void Light::shadowCam(Camera &cam, size_t cubeFace) const {
    assert(cubeFace <= 6u);
    switch (this->type) {
        case LightType::point:
            cam.rot = gfx::cubeMapCameraRotations[cubeFace];
            cam.pos = this->pos;
            cam.fov = tau4;
            break;
        case LightType::spot:
            cam.pos = this->pos;
            cam.rot = dirToRot(this->dir).cwiseProduct(Vector3f(-1.0f, -1.0f, 1.0f));
            cam.fov = tau4;
            break;
        default:
            spdlog::error("Light::shadowCam: Unsupported light type");
            break;
    }
}
