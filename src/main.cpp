#include <app.hpp>

int main(int argc, char const *argv[])
{
    glutInit(&argc, (char **)argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Hello World");
    App app;
    glutMainLoop();

    return 0;
}
