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
        Vector2f mousePos = {0.0f, 0.0f};
        Vector2i windowSize = {1280, 720};
        Camera camera = {Vector3f::Zero(), Vector3f::Zero(), 0.1f, 100.0f, 45.0f};
        std::unordered_set<int> pressedKeys;
        cyGLSLProgram prog;
        std::vector<std::shared_ptr<Model>> models;
        std::shared_ptr<Model> teapot;
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