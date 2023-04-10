#pragma once

#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <memory>
#include <vector>
#include <variant>
#include <optional>
#include <camera.hpp>
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#undef near
#undef far
#include <GLFW/glfw3.h>
#include <cyGL.h>
#include <model.hpp>
#include <mesh.hpp>
#include <physics.hpp>
#include <light.hpp>
#include <entt/entt.hpp>
#include <texture.hpp>
#include <cxxopts.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <compute_shader.hpp>

// Index into element counts and offsets arrays
struct ObjRef {
    size_t objID;
};


struct RenderTarget {
    enum class Type {
        texture,
        textureArray,
        cubemap,
        renderbuffer
    } type;
    GLuint id;
    GLenum attachment = GL_COLOR_ATTACHMENT0;
};

// Represents a render pass with a target framebuffer
struct RenderPass {
    enum class Type {
        reflection,
        cubemap,
        array,
        final,
        normal
    } type;
    GLuint fbo;
    // If not nullopt, the viewport will be set to this value, otherwise uses the window size
    std::optional<Vector2f> viewport = std::nullopt;
    std::vector<RenderTarget> targets;
    std::vector<entt::entity> objMask = {};
    RenderTarget arrayTarget;
};

class App
{
    private:
        ColliderInteriorBox box{Vector3f(-6.0f, 0.0f, -6.0f), vec3(6.0f)};
        // const size_t objectsToGen = this->box.volume() * 0.01f;
        const size_t objectsToGen = 5u;
        const float frameRate = 60.0f;
        const float framePeriod = 1.0f / this->frameRate;

        float t = 0.0f;
        float lastFrameTime = 0.0f;
        float simTimeStep = 0.25f;
        int simTimeIters = 1u;

        // Input
        Vector2f mousePos = {0.0f, 0.0f};
        Vector2f mouseClickStart = {0.0f, 0.0f};
        Vector2f mouseDeltaPos = {0.0f, 0.0f};
        bool mouseLeft = false;
        bool mouseRight = false;
        bool mouseMiddle = false;
        bool mouseMoved = false;
        std::unordered_set<int> pressedKeys;

        bool doDrawDebug = true;


        Vector2i windowSize = {1280, 720};
        Camera camera;
        CameraControl cameraControl;
        CameraControl lightControl;
        cyGLSLProgram meshProg;
        cyGLSLProgram wiresProg;
        cyGLSLProgram skyProg;
        cyGLSLProgram depthProg;
        cyGLSLProgram pointsProg;
        std::shared_ptr<Light> sunlight;

        entt::registry reg;
        entt::entity ePlane;
        entt::entity eSelectPoint;
        entt::entity eDragArrow;
        entt::entity eSpotLight;

        Vector3f selectPoint;
        entt::entity eSelected = entt::null;
        
        RNG rng{0u};

        ComputeShader csSurfaceNets;

        // ID of model transform matrices SSBO
        GLuint ssboModels;
        // ID of model materials SSBO
        GLuint ssboMaterials;
        // ID of lights SSBO
        GLuint ssboLights;

        GLuint vaoMeshes;
        MeshCollection meshes;

        GLuint vaoSky;
        GLuint vboSky;

        GLuint vaoWires;
        GLuint vboWires;
        GLuint vboPath;
        GLuint vboBox;
        // ID of arrow model transform matrices SSBO
        GLuint ssboArrows;
        // ID of arrow colors SSBO
        GLuint ssboArrowColors;

        TextureCollection skyTextures;

        GLuint fboReflections;
        GLuint rboReflections;
        GLuint texReflections;
        GLuint texCubeShadows;
        GLuint fboShadows;
        GLuint texSpotShadows;

        std::vector<RenderPass> renderPasses;

        // Element counts
        std::vector<GLsizei> vCounts;
        // Element offsets
        std::vector<size_t> vOffsets;

        struct UI {
            ImFont* font = nullptr;
        } ui;
    public:
        GLFWwindow* window;

        App(cxxopts::ParseResult& args);
        ~App();

        /**
         * @brief Handle key events from GLFW
         * 
         * @param key Some GLFW key code
         * @param pressed True if pressed, false if released
         */
        void onKey(int key, bool pressed);

        /**
         * @brief Handle mouse button events from GLFW
         * 
         * @param button 
         * @param pressed True if pressed, false if released
         */
        void onClick(int button, bool pressed);

        /**
         * @brief Called on window resize
         * 
         * @param width 
         * @param height 
         */
        void onResize(int width, int height);

        /**
         * @brief Called between frames if there is time to spare
         */
        void idle();

        /**
         * @brief Main entry
         */
        void run();

        /**
         * @brief Draw the scene
         */
        void draw(float dt);

        void buildShaders();

        // Draws the sky shader program to the current framebuffer
        void drawSky(const Camera& cam, const Vector2f& viewport);

        // Draws the meshes shader program to the current framebuffer
        void drawMeshes(const Camera& cam, const Vector2f& viewport);

        void drawMeshesDepth(const Camera& cam, const Vector2f& viewport);

        // Draws the debugging shader program to the current framebuffer
        void drawDebug(const Camera& cam, const Vector2f& viewport);

        // Updates SSBOs for all shader programs
        void updateBuffers();

        void simulate(float dt);

        void composeUI();

        void loadingScreen(const std::string& message);

        // Hides given object from rendering
        void hidden(entt::entity e, bool hidden);

        ObjRef makeObj(const MeshRef& mesh);

        entt::entity makeParticle();
        entt::entity makeModel(const std::string& name);
        entt::entity makeRigidBody(const std::string& name, const Vector3f& scale, const Vector3f& pos, const Vector3f& rot = {0.0f, 0.0f, 0.0f});
        entt::entity makeLight(const Vector3f& pos, const Vector3f& color, float intensity, float range, LightType type);
        entt::entity makeSpotLight(const Vector3f& pos, const Vector3f& dir, const Vector3f& color, float intensity, float spotAngle);
};