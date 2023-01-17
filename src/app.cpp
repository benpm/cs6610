#include <functional>
#include <app.hpp>

App::App() {}

void App::init() {

}

void App::display() {
    this->draw();
}

void App::idle() {
    // Wait until frame
    this->t = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    if (this->t - this->lastFrameTime < this->framePeriod) {
        return;
    }

    // Update
    glutPostRedisplay();
}

void App::draw() {
    const vec4 clearColor(sin(this->t), sin(this->t + 2.0f), sin(this->t - 1.5f), 1.0f);
    glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSolidTeapot(0.5f);
    glutSwapBuffers();
}

void App::keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:
            glutLeaveMainLoop();
            break;
    }
}