#pragma once

#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <memory>
#include <vector>
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

class App
{
    private:
        ColliderInteriorBox box{vec3(-25.0f), vec3(25.0f)};
        // const size_t objectsToGen = this->box.volume() * 0.01f;
        const size_t objectsToGen = 1000u;
        const float frameRate = 60.0f;
        const float framePeriod = 1.0f / this->frameRate;

        float t = 0.0f;
        float lastFrameTime = 0.0f;

        // Input
        Vector2f mousePos = {0.0f, 0.0f};
        Vector2f mouseClickStart = {0.0f, 0.0f};
        Vector2f mouseDeltaPos = {0.0f, 0.0f};
        bool mouseLeft = false;
        bool mouseRight = false;
        bool mouseMiddle = false;
        bool mouseMoved = false;
        std::unordered_set<int> pressedKeys;


        Vector2i windowSize = {1280, 720};
        Camera camera;
        Camera secondaryCamera;
        cyGLSLProgram meshProg;
        cyGLSLProgram wiresProg;
        std::shared_ptr<Light> sunlight;

        entt::registry reg;
        RNG rng{0u};

        // ID of model transform matrices SSBO
        GLuint ssboModels;
        // ID of model materials SSBO
        GLuint ssboMaterials;
        // ID of lights SSBO
        GLuint ssboLights;

        GLuint vaoMeshes;
        MeshCollection meshes;

        GLuint vaoWires;
        GLuint vboWires;
        GLuint vboPath;
        GLuint vboBox;
        // ID of arrow model transform matrices SSBO
        GLuint ssboArrows;
        // ID of arrow colors SSBO
        GLuint ssboArrowColors;

        // ID of main framebuffer
        GLuint fboMain;
        // ID of main framebuffer texture
        GLuint fTexMain;
        // ID of main framebuffer texture copy
        GLuint fTexMainCopy;

        // Element counts
        std::vector<GLsizei> vCounts;
        // Element offsets
        std::vector<size_t> vOffsets;
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

        void simulate(float dt);

        void composeUI();

        void loadingScreen(const std::string& message);

        entt::entity makeParticle();
        entt::entity makeModel(const std::string& name);
        entt::entity makeLight(const Vector3f& pos, const Vector3f& color, float intensity, LightType type);
};