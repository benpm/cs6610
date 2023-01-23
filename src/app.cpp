#include <cassert>
#include <functional>
#include <app.hpp>
#define GLEQ_IMPLEMENTATION
#include <gleq.h>

App::App() {
    // Initialize GLFW and Gleq
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    this->window = glfwCreateWindow(1280, 720, "CS6610", NULL, NULL);
    if (!this->window) {
        spdlog::error("Could not open GLFW window");
        exit(-1);
    }
    gleqTrackWindow(this->window);

    // Build and bind shader program
    bool built = this->prog.BuildFiles("shaders/basic.vert", "shaders/basic.frag");
    assert(built);
    this->prog.Bind();


    const vec3 vertices[] = {
        {-1.0f, -1.0f, 0.0f},
        {1.0f, -1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    const vec3 colors[] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Create VBO for vertices
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices) + sizeof(colors), NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(vertices), sizeof(colors), colors);

    // Attach VBO to vPos
    GLuint attrib_vPos = glGetAttribLocation(this->prog.GetID(), "vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)sizeof(vertices));
}

App::~App() {
    glfwTerminate();
}

void App::onKey(int key, bool pressed) {
    if (pressed) {
        this->pressedKeys.emplace(key);
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(this->window, GL_TRUE);
                break;
            default:
                break;
        }
    } else {
        this->pressedKeys.erase(key);
        switch (key) {
            default:
                break;
        }
    }
}

void App::onResize(int width, int height) {
    glViewport(0, 0, width, height);
    this->windowSize = {width, height};
    spdlog::debug("Resized window to {},{}", width, height);
}

void App::onClick(int button, bool pressed) {
}

void App::run() {
    while (!glfwWindowShouldClose(this->window)) {
        // Handle GLFW events (100 gleqs)
        GLEQevent event;
        while (gleqNextEvent(&event)) {
            switch (event.type) {
                case GLEQ_KEY_PRESSED:
                    this->onKey(event.keyboard.key, true);
                    break;
                case GLEQ_KEY_RELEASED:
                    this->onKey(event.keyboard.key, false);
                    break;
                case GLEQ_CURSOR_MOVED:
                    this->mousePos = {event.pos.x, event.pos.y};
                    break;
                case GLEQ_FRAMEBUFFER_RESIZED:
                    this->onResize(event.size.width, event.size.height);
                    break;
                case GLEQ_WINDOW_MOVED:
                    this->lastFrameTime = this->t;
                    break;
                case GLEQ_BUTTON_PRESSED:
                    this->onClick(event.mouse.button, true);
                    break;
                case GLEQ_BUTTON_RELEASED:
                    this->onClick(event.mouse.button, false);
                    break;
                default:
                    break;
            }

            gleqFreeEvent(&event);
        }
        glfwPollEvents();

        // Wait until next frame
        this->t = glfwGetTime();
        const float dt = this->t - this->lastFrameTime;
        this->lastFrameTime = glfwGetTime();

        // Draw the scene
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        this->draw(dt);
        this->idle();
        glfwSwapBuffers(this->window);
    }
}

void App::idle() {
}

void App::draw(float dt) {

    glDrawArrays(GL_TRIANGLES, 0, 3);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
}
