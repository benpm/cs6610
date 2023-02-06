#pragma once

#include <cmath>
#include <iostream>
#include <unordered_set>
#include <memory>
#include <vector>
#include <camera.hpp>
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cyGL.h>
#include <model.hpp>

class App
{
    private:
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
        std::unordered_set<int> pressedKeys;


        Vector2i windowSize = {1280, 720};
        Camera camera;
        cyGLSLProgram prog;
        std::vector<std::shared_ptr<Model>> models;
        std::shared_ptr<Model> teapot;

        // ID of model transform matrices SSBO
        GLuint ssboModels;
        // ID of model materials SSBO
        GLuint ssboMaterials;
        // ID of lights SSBO
        GLuint ssboLights;
        // Index of VBO for storing vertex data
        GLuint vertVBO;
        // Index of EBO for storing triangle element indices
        GLuint triEBO;

        // Flat, strided vertex array data (pos, color, normal)
        std::vector<Vector3f> arrVerts;
        // Flat element array data (vertex indices for triangles: v0,v1,v2)
        std::vector<uint32_t> arrElems;
        // Element counts
        std::vector<GLsizei> vCounts;
        // Element offsets
        std::vector<size_t> vOffsets;
        // Model transform matrices
        std::vector<Matrix4f> mTransforms;
        // Model materials
        std::vector<uMaterial> mMaterials;
        // Lights
        std::vector<uLight> lights;
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
};