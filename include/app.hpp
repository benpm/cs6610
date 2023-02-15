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
#include <light.hpp>
#include <entt/entt.hpp>
#include <texture.hpp>

class App
{
    private:
        AABB box{vec3(-25.0f), vec3(25.0f)};
        const size_t objectsToGen = this->box.volume() * 0.01f;
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

        enum class SimulationMethod {
            explicitEuler,
            implicitEuler,
            verlet,
            implicit,
            velocityOnly
        } simulationMethod = SimulationMethod::explicitEuler;
        const std::unordered_map<SimulationMethod, std::string> simMethodNames = {
            {SimulationMethod::explicitEuler, "Explicit Euler"},
            {SimulationMethod::implicitEuler, "Implicit Euler"},
            {SimulationMethod::verlet,        "Verlet"},
            {SimulationMethod::implicit,      "Constrained Implicit"},
            {SimulationMethod::velocityOnly,  "Velocity Only"}
        };

        // Element counts
        std::vector<GLsizei> vCounts;
        // Element offsets
        std::vector<size_t> vOffsets;
        // Lights
        std::vector<std::shared_ptr<Light>> lights;

        entt::entity particle;
        entt::entity interactArrow;

        std::vector<Vector3f> particlePath;

        TextureCollection tex;
    public:
        GLFWwindow* window;

        App();
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

        entt::entity makeParticle();
};