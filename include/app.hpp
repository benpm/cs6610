#include <cmath>
#include <iostream>
#include <spdlog/spdlog.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

using namespace glm;

class App
{
    private:
        const float frameRate = 60.0f;
        const float framePeriod = 1.0f / this->frameRate;
        float t = 0.0f;
        float lastFrameTime = 0.0f;
    
    public:
        App();
        void display();
        void idle();
        void draw();
        void keyboard(unsigned char key, int x, int y);
};