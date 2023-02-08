
#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif

#include <cassert>
#include <functional>
#include <algorithm>
#include <app.hpp>
#define GLEQ_IMPLEMENTATION
#include <gleq.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <spdlog/formatter.h>

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

    // Initialize imgui
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(this->window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // OpenGL config
    // glEnable(GL_PROGRAM_POINT_SIZE);
    // glEnable(GL_POINT_SPRITE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_MULTISAMPLE);
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
    constexpr int bigness = 5000;

    Model modelTeapot("resources/models/teapot.obj");
    modelTeapot.normalize();
    Model modelSuzanne("resources/models/suzanne.obj");
    modelSuzanne.normalize();

    for (size_t i = 0; i < bigness; i++) {
        entt::entity e = this->reg.create();
        DynamicTransform& transform = this->reg.emplace<DynamicTransform>(e);
        Model& model = this->reg.emplace<Model>(e, rng.choose({modelTeapot, modelSuzanne}));
        uMaterial& mat = this->reg.emplace<uMaterial>(e);
        model.scale = Vector3f::Ones() * rng.range(1.0f, 5.0f);
        model.pos = rng.vec(-Vector3f::Ones(), Vector3f::Ones()) * (float)bigness * 0.01f;
        model.rot = rng.rotation();
        float hue = rng.range(0.0f, 360.0f);
        mat.diffuseColor = hsvToRgb({hue, 0.8f, 0.7f});
        mat.specularColor = hsvToRgb({hue, 0.4f, 1.0f});
        mat.shininess = 2000.0f;
        transform.transform = model.transform();
        model.addToWorld(
            this->arrVerts,
            this->arrElems,
            this->vCounts,
            this->vOffsets,
            this->mTransforms,
            this->mMaterials);
    }

    this->camera.orbitDist(50.0f);

    // Add lights
    this->sunlight = this->lights.emplace_back(std::make_shared<Light>(
        Vector3f(0.25f, 0.5f, 0.25f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.35f,
        LightType::directional));
    this->lights.emplace_back(std::make_shared<Light>(
        Vector3f(0.15f, -0.5f, -0.45f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.01f,
        LightType::directional));
    for (size_t i = 0; i < 5; i++) {
        this->lights.emplace_back(std::make_shared<Light>(
            rng.vec(-Vector3f::Ones(), Vector3f::Ones()) * (float)bigness * 0.01f,
            Vector3f(1.0f, 1.0f, 1.0f),
            rng.range(1.0f, 5.0f),
            LightType::point));
    }

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
    auto& transformsStorage = this->reg.view<DynamicTransform>().storage<DynamicTransform>();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        transformsStorage.size() * sizeof(Matrix4f),
        *transformsStorage.raw(), GL_DYNAMIC_DRAW);
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
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->lights.size() * sizeof(uLight), NULL, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->ssboLights);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    this->prog.SetUniform("nLights", (uint)this->lights.size());
}

App::~App() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
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
            case GLFW_KEY_1: {
                this->camera.mode = Camera::Mode::orbit;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            } break;
            case GLFW_KEY_2: {
                this->camera.mode = Camera::Mode::fly;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
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
        this->mouseDeltaPos = {0.0f, 0.0f};

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
    if (this->mouseLeft && this->pressedKeys.count(GLFW_KEY_LEFT_CONTROL)) {
        // Move light
        const Vector2f panDelta = this->mouseDeltaPos * 0.01f;
        this->sunlight->pos = identityTransform()
            .rotate(euler({panDelta.x(), panDelta.y(), 0.0f})) * this->sunlight->pos;
    } else if (this->mouseRight) {
        // Camera zoom
        this->camera.universalZoom(-(this->mouseDeltaPos.y() / this->windowSize.y()) * 10.0f);
    } else {
        // Camera controls
        const float maxWinDim = (float)std::max(windowSize.x(), windowSize.y());
        const Vector2f panDelta = (this->mouseClickStart - this->mousePos) / maxWinDim * tau2;
        const Vector2f keyboardDelta = {
            (float)this->pressedKeys.count(GLFW_KEY_D) - (float)this->pressedKeys.count(GLFW_KEY_A),
            (float)this->pressedKeys.count(GLFW_KEY_W) - (float)this->pressedKeys.count(GLFW_KEY_S)
        };
        this->camera.control(
            this->mouseDeltaPos * dt,
            this->mouseLeft ? Vector2f(panDelta * dt * 10.0f) : Vector2f(Vector2f::Zero()),
            keyboardDelta * dt * 10.0f);
    }

    // Set up transformation matrices
    const Matrix4f tProj = this->camera.getProj(this->windowSize.cast<float>());
    this->prog.SetUniformMatrix4("uTProj", tProj.data());
    const Matrix4f tView = this->camera.getView();
    this->prog.SetUniformMatrix4("uTView", tView.data());

    // Update lights
    std::vector<uLight> lightStructs(this->lights.size());
    for (size_t i = 0; i < this->lights.size(); i++) {
        lightStructs[i] = this->lights[i]->toStruct(this->camera);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, lightStructs.size() * sizeof(uLight), lightStructs.data());
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // Update model transforms
    for (auto e : this->reg.view<Model, DynamicTransform>()) {
        auto [model, transform] = this->reg.get<Model, DynamicTransform>(e);
        model.pos += Vector3f(sinf(model.pos.x() * 0.01f), 0.05f, 0.1f) * dt;
        model.rot += Vector3f(sinf(model.pos.x() * 0.05f), 0.1f, sinf(model.pos.x() * 0.01f)) * dt;
        transform.transform = model.transform();
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels);
    auto& transformsStorage = this->reg.view<DynamicTransform>().storage<DynamicTransform>();
    glBufferSubData(GL_SHADER_STORAGE_BUFFER,
        0, transformsStorage.size() * sizeof(Matrix4f),
        *transformsStorage.raw());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    
    // Draw models in scene
    glMultiDrawElements(
        GL_TRIANGLES,
        this->vCounts.data(),
        GL_UNSIGNED_INT,
        (const void**)this->vOffsets.data(),
        this->vCounts.size());

    this->composeUI();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void App::composeUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::Begin("Debug", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::End();
}
