#include <cmath>
#include <iostream>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

using namespace glm;

class App
{
    private:
        float t = 0.0f;
    
    public:
        App();
        void display();
        void idle();
        void draw();
};