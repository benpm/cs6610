
#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif

#include <cassert>
#include <functional>
#include <algorithm>
#include <app.hpp>
#define GLEQ_IMPLEMENTATION
#include <gleq.h>

App::App() {
    // Initialize GLFW and Gleq
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    this->window = glfwCreateWindow(
        this->windowSize.x(),
        this->windowSize.y(), "CS6610", NULL, NULL);
    if (!this->window) {
        spdlog::error("Could not open GLFW window");
        exit(-1);
    }
    gleqTrackWindow(this->window);
    glfwMakeContextCurrent(this->window);

    // Init glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
    }

    glfwSwapInterval(1);

    // OpenGL config
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Build and bind shader program
    bool built = this->prog.BuildFiles(
        "resources/shaders/basic.vert",
        "resources/shaders/basic.frag");
    if (!built) {
        spdlog::error("Failed to build shader program");
    } else {
        this->prog.Bind();
    }
    
    // Create and bind VAO
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    this->teapot = std::make_shared<Model>("resources/models/teapot.obj", this->prog);
    this->models.push_back(this->teapot);
    this->teapot->scale = Vector3f::Ones() * 0.075f;
    this->teapot->pivot.x() = 0.0f;
    this->teapot->rot.x() = tau4 * 3.0f;

    {
        std::shared_ptr<Model> model = std::make_shared<Model>(*this->teapot.get());
        this->models.push_back(model);
        model->pos.x() = -2.5f;
        model->scale = Vector3f::Ones() * 0.1f;
        model->rot.y() = -tau4 * 0.5f;
    }

    {
        std::shared_ptr<Model> model = std::make_shared<Model>(*this->teapot.get());
        this->models.push_back(model);
        model->pos.x() = 2.5f;
        model->scale = Vector3f::Ones() * 0.05f;
        model->rot.y() = tau4 * 0.5f;
    }

    this->camera.orbitDist(2.0f);
    this->camera.orbitTarget(this->teapot->pos);
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
            case GLFW_KEY_F6: {
                bool built = this->prog.BuildFiles(
                    "resources/shaders/basic.vert",
                    "resources/shaders/basic.frag");
                if (!built) {
                    spdlog::error("Failed to build shader program");
                } else {
                    this->prog.Bind();
                    spdlog::info("Rebuilt shader program");
                }
            } break;
            case GLFW_KEY_P: {
                if (this->camera.projection == Camera::Projection::perspective) {
                    this->camera.projection = Camera::Projection::orthographic;
                    this->camera.zoom = 2000.0f;
                } else {
                    this->camera.projection = Camera::Projection::perspective;
                    this->camera.orbitDist(2.0f);
                }
            } break;
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
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
            this->mouseLeft = pressed;
            this->camera.orbitPanStart();
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            this->mouseRight = pressed;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            this->mouseMiddle = pressed;
            break;
        default:
            break;
    }
    if (pressed) {
        this->mouseClickStart = this->mousePos;
    } else {
        
    }
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
                    this->mouseDeltaPos = Vector2f(event.pos.x, event.pos.y) - this->mousePos;
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
                case GLEQ_SCROLLED: 
                    this->camera.universalZoom(-event.scroll.y * 0.1f);
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
    // Camera controls
    if (this->mouseLeft) {
        const float maxWinDim = (float)std::max(windowSize.x(), windowSize.y());
        const Vector2f panDelta = (this->mouseClickStart - this->mousePos) / maxWinDim * tau2;
        this->camera.orbitPan({panDelta.x(), -panDelta.y()});
    } else if (this->mouseRight) {
        this->camera.universalZoom(-(this->mouseDeltaPos.y() / this->windowSize.y()) * 10.0f);
    }

    // Set up transformation matrices
    const Matrix4f tProj = this->camera.getProj(this->windowSize.cast<float>());
    this->prog.SetUniformMatrix4("uTProj", tProj.data());
    const Matrix4f tView = this->camera.getView();
    this->prog.SetUniformMatrix4("uTView", tView.data());
    
    // Draw models in scene
    for (const std::shared_ptr<Model>& model : this->models) {

        model->draw(this->prog, this->camera);
    }
}
