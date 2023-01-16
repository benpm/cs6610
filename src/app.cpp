#include <functional>
#include <app.hpp>

App::App() {
    glutDisplayFuncUcall((void(*)(void*))(&App::display), this);
    glutIdleFuncUcall((void(*)(void*))(&App::idle), this);
}

void App::display() {
    this->draw();
}

void App::idle() {
    this->t += 0.01;
    this->draw();
}

void App::draw() {
    const vec4 clearColor(sin(this->t), sin(this->t + 2.0f), sin(this->t - 1.5f), 1.0f);
    glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
}