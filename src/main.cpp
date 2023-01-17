#include <app.hpp>

App app;

void display() {
    app.display();
}

void idle() {
    app.idle();
}

void keyboard(unsigned char key, int x, int y) {
    app.keyboard(key, x, y);
}

int main(int argc, char const *argv[])
{
    glutInit(&argc, (char **)argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Hello World");
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboard);
    app.init();
    glutMainLoop();

    return 0;
}
