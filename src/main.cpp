#include <GL/freeglut.h>
#include <glm/glm.hpp>

using namespace glm;

int main(int argc, char const *argv[])
{
    glutInit(&argc, (char **)argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Hello World");
    glutDisplayFunc([]() {
        const vec4 clearColor(0.7f, 0.1f, 0.3f, 1.0f);
        glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
        glClear(GL_COLOR_BUFFER_BIT);
        glutSwapBuffers();
    });
    glutMainLoop();

    return 0;
}
