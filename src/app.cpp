
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
    glfwWindowHint(GLFW_SAMPLES, 4);
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
    glEnable(GL_MULTISAMPLE);
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);
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

    RNG rng(0u);
    
    // Create models
    constexpr int bigness = 1000;

    Model modelTeapot("resources/models/teapot.obj");
    modelTeapot.normalize();
    Model modelSuzanne("resources/models/suzanne.obj");
    modelSuzanne.normalize();
    for (size_t i = 0; i < bigness; i++) {
        std::shared_ptr<Model> model = std::make_shared<Model>(rng.choose({modelTeapot, modelSuzanne}));
        model->scale = Vector3f::Ones() * rng.range(1.0f, 5.0f);
        model->pos = rng.vec(-Vector3f::Ones(), Vector3f::Ones()) * (float)bigness * 0.025f;
        model->rot = rng.rotation();
        float hue = rng.range(0.0f, 360.0f);
        model->mat.diffuseColor = hsvToRgb({hue, 0.8f, 0.7f});
        model->mat.specularColor = hsvToRgb({hue, 0.4f, 1.0f});
        this->models.push_back(model);
    }

    this->camera.orbitDist(50.0f);

    // Add models to world
    for (const std::shared_ptr<Model>& model : this->models) {
        model->addToWorld(
            this->arrVerts,
            this->arrElems,
            this->vCounts,
            this->vOffsets,
            this->mTransforms,
            this->mMaterials);
    }

    // Add lights
    this->lights.emplace_back(
        Vector3f(0.25f, 0.5f, 0.25f),
        Vector3f(1.0f, 1.0f, 0.9f),
        1.0f,
        LightType::directional);
    this->lights.emplace_back(
        Vector3f(0.15f, -0.5f, -0.45f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.10f,
        LightType::directional);
    for (size_t i = 0; i < 50; i++) {
        this->lights.emplace_back(
            rng.vec(-Vector3f::Ones(), Vector3f::Ones()) * (float)bigness * 0.025f,
            Vector3f(1.0f, 1.0f, 1.0f),
            rng.range(1.0f, 5.0f),
            LightType::point);
    }

    spdlog::info("{} models, {} vertices, {} triangles, {} lights",
        this->models.size(), this->arrVerts.size(), this->arrElems.size() / 3, this->lights.size());

    // Create and bind VAO
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Create and populate VBO
    glGenBuffers(1, &this->vertVBO);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    glBufferData(GL_ARRAY_BUFFER, this->arrVerts.size() * sizeof(Vector3f), this->arrVerts.data(), GL_STATIC_DRAW);

    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 0u));
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 1u));
    GLuint attrib_vNormal = prog.AttribLocation("vNormal");
    glEnableVertexAttribArray(attrib_vNormal);
    glVertexAttribPointer(attrib_vNormal, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 2u));

    // Create and populate triangles EBO
    glGenBuffers(1, &this->triEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->triEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->arrElems.size() * sizeof(uint32_t), this->arrElems.data(), GL_STATIC_DRAW);

    // Create and bind model transforms SSBO
    glGenBuffers(1, &this->ssboModels);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels);
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->mTransforms.size() * sizeof(Matrix4f), this->mTransforms.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Create and bind materials SSBO
    glGenBuffers(1, &this->ssboMaterials);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials);
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->mMaterials.size() * sizeof(uMaterial), this->mMaterials.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Create and bind lights SSBO
    glGenBuffers(1, &this->ssboLights);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights);
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->lights.size() * sizeof(uLight), this->lights.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->ssboLights);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    this->prog.SetUniform("nLights", (uint)this->lights.size());
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

    // Update lights
    std::vector<uLight> lightStructs(this->lights.size());
    for (size_t i = 0; i < this->lights.size(); i++) {
        lightStructs[i] = this->lights[i].toStruct(this->camera);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, lightStructs.size() * sizeof(uLight), lightStructs.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // // Update model transforms
    // for (size_t i = 0; i < this->models.size(); i++) {
    //     this->models[i]->rot.x() += dt * 0.5f;
    //     this->mTransforms[i] = this->models[i]->transform();
    // }
    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels);
    // glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, this->mTransforms.size() * sizeof(Matrix4f), this->mTransforms.data());
    // glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    
    // Draw models in scene
    glMultiDrawElements(
        GL_TRIANGLES,
        this->vCounts.data(),
        GL_UNSIGNED_INT,
        (const void**)this->vOffsets.data(),
        this->vCounts.size());
}
