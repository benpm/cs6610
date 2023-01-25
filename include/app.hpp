#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <cyGL.h>

class App
{
    private:
        const float frameRate = 60.0f;
        const float framePeriod = 1.0f / this->frameRate;
        float t = 0.0f;
        float lastFrameTime = 0.0f;
        vec2 mousePos = vec2(0.0f, 0.0f);
        ivec2 windowSize = vec2(0, 0);
        std::unordered_set<int> pressedKeys;
        cyGLSLProgram prog;
    
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